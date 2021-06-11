// Copyright 2019 Project March.
#include "march_hardware/ethercat/ethercat_master.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller.h"

#include <chrono>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

#include <pthread.h>
#include <ros/ros.h>
#include <soem/ethercat.h>

namespace march {
EthercatMaster::EthercatMaster(
    std::string if_name, int max_slave_index, int cycle_time, int slave_timeout)
    : is_operational_(/*__i=*/false)
    , if_name_(std::move(if_name))
    , max_slave_index_(max_slave_index)
    , cycle_time_ms_(cycle_time)
    , slave_watchdog_timeout_(slave_timeout)
{
}

EthercatMaster::~EthercatMaster()
{
    this->stop();
}

bool EthercatMaster::isOperational() const
{
    return this->is_operational_;
}

int EthercatMaster::getCycleTime() const
{
    return this->cycle_time_ms_;
}

void EthercatMaster::waitForPdo()
{
    std::unique_lock<std::mutex> lock(this->wait_on_pdo_condition_mutex_);
    this->wait_on_pdo_condition_var_.wait(lock, [&] {
        return this->pdo_received_ || !this->is_operational_;
    });
    this->pdo_received_ = false;
}

std::exception_ptr EthercatMaster::getLastException() const noexcept
{
    return this->last_exception_;
}

bool EthercatMaster::start(std::vector<Joint>& joints)
{
    this->last_exception_ = nullptr;
    this->ethercatMasterInitiation();
    return this->ethercatSlaveInitiation(joints);
}

void EthercatMaster::ethercatMasterInitiation()
{
    ROS_INFO("Trying to start EtherCAT");
    if (!ec_init(this->if_name_.c_str())) {
        throw error::HardwareException(error::ErrorType::NO_SOCKET_CONNECTION,
            "No socket connection on %s", this->if_name_.c_str());
    }
    ROS_INFO("ec_init on %s succeeded", this->if_name_.c_str());

    const int slave_count = ec_config_init(FALSE);
    if (slave_count < this->max_slave_index_) {
        ec_close();
        throw error::HardwareException(error::ErrorType::NOT_ALL_SLAVES_FOUND,
            "%d slaves configured while soem only found %d slave(s)",
            this->max_slave_index_, slave_count);
    }
    ROS_INFO("%d slave(s) found and initialized.", slave_count);
}

int setSlaveWatchdogTimer(uint16 slave)
{
    uint16 configadr = ec_slave[slave].configadr;
    // Set the divider register of the WD
    ec_FPWRw(configadr, /*ADO=*/0x0400, MotorController::WATCHDOG_DIVIDER,
        EC_TIMEOUTRET);
    // Set the PDI watchdog = WD
    ec_FPWRw(configadr, /*ADO=*/0x0410, MotorController::WATCHDOG_TIME,
        EC_TIMEOUTRET);
    // Set the SM watchdog = WD
    ec_FPWRw(configadr, /*ADO=*/0x0420, MotorController::WATCHDOG_TIME,
        EC_TIMEOUTRET);
    return 1;
}

bool EthercatMaster::ethercatSlaveInitiation(std::vector<Joint>& joints)
{
    ROS_INFO("Request pre-operational state for all slaves");
    bool reset = false;
    ec_statecheck(/*slave=*/0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

    for (Joint& joint : joints) {
        ec_slave[joint.getMotorController()->getSlaveIndex()].PO2SOconfig
            = setSlaveWatchdogTimer;
        reset |= joint.initSdo(this->cycle_time_ms_);
    }

    ec_config_map(&this->io_map_);
    ec_configdc();

    ROS_INFO("Request safe-operational state for all slaves");
    ec_statecheck(/*slave=*/0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    this->expected_working_counter_
        = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ROS_INFO("Request operational state for all slaves");
    ec_writestate(/*slave=*/0);
    int chk = 40;

    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(/*slave=*/0, EC_STATE_OPERATIONAL, /*timeout=*/50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        ROS_INFO("Operational state reached for all slaves");
        this->is_operational_ = true;
        this->ethercat_thread_
            = std::thread(&EthercatMaster::ethercatLoop, this);
        this->setThreadPriority(EthercatMaster::THREAD_PRIORITY);
    } else {
        ec_readstate();
        std::ostringstream ss;
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                ss << std::endl
                   << "Slave " << i << " State=" << std::hex << std::showbase
                   << ec_slave[i].state
                   << " StatusCode=" << ec_slave[i].ALstatuscode << " ("
                   << ec_ALstatuscode2string(ec_slave[i].ALstatuscode) << ")";
            }
        }
        throw error::HardwareException(
            error::ErrorType::FAILED_TO_REACH_OPERATIONAL_STATE,
            "Not operational slaves: %s", ss.str().c_str());
    }
    return reset;
}

void EthercatMaster::ethercatLoop()
{
    size_t total_loops = 0;
    size_t not_achieved_count = 0;
    const size_t rate = 1000 / this->cycle_time_ms_;
    const std::chrono::milliseconds cycle_time(this->cycle_time_ms_);

    while (this->is_operational_) {
        const auto begin_time = std::chrono::high_resolution_clock::now();

        const bool pdo_received = this->sendReceivePdo();
        this->monitorSlaveConnection();

        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration
            = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - begin_time);

        {
            std::lock_guard<std::mutex> lock(
                this->wait_on_pdo_condition_mutex_);
            this->pdo_received_ = pdo_received;
        }
        this->wait_on_pdo_condition_var_.notify_one();

        if (duration > cycle_time) {
            not_achieved_count++;
        } else {
            std::this_thread::sleep_for(cycle_time - duration);
        }
        total_loops++;

        if (total_loops >= 10 * rate) {
            const double not_achieved_percentage
                = 100.0 * ((double)not_achieved_count / total_loops);
            if (not_achieved_percentage > 5.0) {
                ROS_WARN("EtherCAT rate of %d milliseconds per cycle was not "
                         "achieved for %f percent of all cycles",
                    this->cycle_time_ms_, not_achieved_percentage);
            }
            total_loops = 0;
            not_achieved_count = 0;
        }

        const auto delta_t = std::chrono::high_resolution_clock::now()
            - this->valid_slaves_timestamp_ms_;
        const auto slave_lost_duration
            = std::chrono::duration_cast<std::chrono::milliseconds>(delta_t);
        const std::chrono::milliseconds slave_watchdog_timeout(
            this->slave_watchdog_timeout_);

        if (slave_lost_duration > slave_watchdog_timeout) {
            this->last_exception_ = std::make_exception_ptr(
                error::HardwareException(error::ErrorType::SLAVE_LOST_TIMOUT,
                    "Slave connection lost for %i ms from slave %i and "
                    "onwards.",
                    this->slave_watchdog_timeout_, this->latest_lost_slave_));
            this->is_operational_ = false;
            this->wait_on_pdo_condition_var_.notify_one();

            this->closeEthercat();
        }
    }
}

bool EthercatMaster::sendReceivePdo()
{
    if (this->latest_lost_slave_ == -1) {
        ec_send_processdata();
        const int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < this->expected_working_counter_) {
            ROS_WARN_THROTTLE(1,
                "Working counter: %d  is lower than expected: %d", wkc,
                this->expected_working_counter_);
            return false;
        }
        return true;
    }
    return false;
}

void EthercatMaster::monitorSlaveConnection()
{
    ec_readstate();
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        if (ec_slave[slave].state != EC_STATE_OPERATIONAL) {
            ROS_WARN_THROTTLE(1,
                "EtherCAT train lost connection from slave %d onwards", slave);

            if (!this->attemptSlaveRecover(slave)) {
                this->latest_lost_slave_ = slave;
                return;
            }
        }
    }

    if (this->latest_lost_slave_ > -1) {
        ROS_INFO("All slaves returned to operational state.");
    }

    this->latest_lost_slave_ = -1;
    this->valid_slaves_timestamp_ms_
        = std::chrono::high_resolution_clock::now();
}

bool EthercatMaster::attemptSlaveRecover(int slave)
{
    if (ec_slave[slave].state != EC_STATE_OPERATIONAL) {
        if (ec_slave[slave].state == EC_STATE_PRE_OP) {
            ec_slave[slave].state = EC_STATE_SAFE_OP;
            ec_writestate(slave);
        }

        if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
        }

        if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, /*timeout=*/500)) {
                ec_slave[slave].islost = FALSE;
            }
        }

        if (!ec_slave[slave].islost) {
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
                ec_slave[slave].islost = TRUE;
                ROS_ERROR("Ethercat lost connection to slave %d", slave);
            }
        }
    }

    if (ec_slave[slave].islost) {
        if (ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, /*timeout=*/500)) {
                ec_slave[slave].islost = FALSE;
            }
        } else {
            ec_slave[slave].islost = FALSE;
        }
    }

    if (ec_slave[slave].state == EC_STATE_OPERATIONAL) {
        ROS_INFO("Slave %i resumed operational state", slave);
        return true;
    } else {
        return false;
    }
}

void EthercatMaster::closeEthercat()
{
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(/*slave=*/0);
    ec_close();
}

void EthercatMaster::stop()
{
    if (this->is_operational_) {
        ROS_INFO("Stopping EtherCAT");
        this->is_operational_ = false;
        this->ethercat_thread_.join();

        this->closeEthercat();
    }
}

void EthercatMaster::setThreadPriority(int priority)
{
    struct sched_param param = { priority };
    // SCHED_FIFO scheduling preempts other threads with lower priority as soon
    // as it becomes runnable. See
    // http://man7.org/linux/man-pages/man7/sched.7.html for more info.
    const int error = pthread_setschedparam(
        this->ethercat_thread_.native_handle(), SCHED_FIFO, &param);
    if (error != 0) {
        ROS_ERROR("Failed to set the ethercat thread priority to %d. (error "
                  "code: %d)",
            priority, error);
    } else {
        ROS_DEBUG("Set ethercat thread priority to %d", param.sched_priority);
    }
}
} // namespace march
