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
#include <soem/ethercat.h>

#define PDB_CHECKSUM_MISO_OFFSET 60

namespace march {
EthercatMaster::EthercatMaster(std::string network_interface_name, int max_slave_index, int cycle_time, int slave_timeout,
    std::shared_ptr<march_logger::BaseLogger> logger)
    : is_operational_(/*__i=*/false)
    , network_interface_name_(std::move(network_interface_name))
    , max_slave_index_(max_slave_index)
    , cycle_time_ms_(cycle_time)
    , logger_(std::move(logger))
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
    logger_->info("Trying to start EtherCAT");
    if (!ec_init(this->network_interface_name_.c_str())) {
        throw error::HardwareException(error::ErrorType::NO_SOCKET_CONNECTION,
            "No socket connection on %s. Check if the socket is active by typing `ifconfig` in a terminal "
            ", or this node is not executed as root.",
            this->network_interface_name_.c_str());
    }
    logger_->info(logger_->fstring("ec_init on %s succeeded", this->network_interface_name_.c_str()));

    const int slave_count = ec_config_init(FALSE);
    if (slave_count < this->max_slave_index_) {
        ec_close();
        throw error::HardwareException(error::ErrorType::NOT_ALL_SLAVES_FOUND,
            "%d slaves configured while soem only found %d slave(s)", this->max_slave_index_, slave_count);
    }
    logger_->info(logger_->fstring("%d slave(s) found and initialized.", slave_count));
}

int setSlaveWatchdogTimer(uint16 slave)
{
    uint16 configadr = ec_slave[slave].configadr;
    // Set the divider register of the WD
    ec_FPWRw(configadr, /*ADO=*/0x0400, MotorController::WATCHDOG_DIVIDER, EC_TIMEOUTRET);
    // Set the PDI watchdog = WD
    ec_FPWRw(configadr, /*ADO=*/0x0410, MotorController::WATCHDOG_TIME, EC_TIMEOUTRET);
    // Set the SM watchdog = WD
    ec_FPWRw(configadr, /*ADO=*/0x0420, MotorController::WATCHDOG_TIME, EC_TIMEOUTRET);
    return 1;
}

bool EthercatMaster::ethercatSlaveInitiation(std::vector<Joint>& joints)
{
    logger_->info("Request pre-operational state for all slaves");
    bool reset = false;
    ec_statecheck(/*slave=*/0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

    for (Joint& joint : joints) {
        ec_slave[joint.getMotorController()->getSlaveIndex()].PO2SOconfig = setSlaveWatchdogTimer;
        reset |= joint.initSdo(this->cycle_time_ms_);
    }

    ec_config_map(&this->io_map_);
    ec_configdc();

    logger_->info("Request safe-operational state for all slaves");
    ec_statecheck(/*slave=*/0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    this->expected_working_counter_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    logger_->info("Request operational state for all slaves");
    ec_writestate(/*slave=*/0);
    int chk = 40;

    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(/*slave=*/0, EC_STATE_OPERATIONAL, /*timeout=*/50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        logger_->info("Operational state reached for all slaves");
        this->is_operational_ = true;
        this->ethercat_thread_ = std::thread(&EthercatMaster::ethercatLoop, this);
        this->setThreadPriority(EthercatMaster::THREAD_PRIORITY);
    } else {
        ec_readstate();
        std::ostringstream ss;
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                ss << std::endl
                   << "Slave " << i << " State=" << std::hex << std::showbase << ec_slave[i].state
                   << " StatusCode=" << ec_slave[i].ALstatuscode << " ("
                   << ec_ALstatuscode2string(ec_slave[i].ALstatuscode) << ")";
            }
        }
        logger_->error(logger_->fstring("Not operational slaves: %s", ss.str().c_str()));
        throw error::HardwareException(
            error::ErrorType::FAILED_TO_REACH_OPERATIONAL_STATE, "Not operational slaves: %s", ss.str().c_str());
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
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time);

        {
            std::lock_guard<std::mutex> lock(this->wait_on_pdo_condition_mutex_);
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
            const double not_achieved_percentage = 100.0 * ((double)not_achieved_count / total_loops);
            if (not_achieved_percentage > 5.0) {
                logger_->warn(logger_->fstring("EtherCAT rate of %d milliseconds per cycle was not",
                    " achieved for %f percent of all cycles", this->cycle_time_ms_, not_achieved_percentage));
            }
            total_loops = 0;
            not_achieved_count = 0;
        }

        const auto delta_t = std::chrono::high_resolution_clock::now() - this->valid_slaves_timestamp_ms_;
        const auto slave_lost_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_t);
        const std::chrono::milliseconds slave_watchdog_timeout(this->slave_watchdog_timeout_);

        if (slave_lost_duration > slave_watchdog_timeout) {
            logger_->warn(logger_->fstring("Slave connection lost for %i ms from slave %i and onwards.",
                this->slave_watchdog_timeout_, this->latest_lost_slave_));
            this->last_exception_
                = std::make_exception_ptr(error::HardwareException(error::ErrorType::SLAVE_LOST_TIMOUT,
                    "Slave connection lost for %i ms from slave %i and "
                    "onwards.",
                    this->slave_watchdog_timeout_, this->latest_lost_slave_));
            this->is_operational_ = false;
            this->wait_on_pdo_condition_var_.notify_one();

            this->closeEthercat();
        }
    }
}

bool EthercatMaster::isCheckSumValid(uint16_t slave)
{
    uint32_t checkSumValue = 0;
    uint8_t miso_object_byte_offset;

    if (strcmp(ec_slave[slave].name, "PDBm9") == 0){
        miso_object_byte_offset = PDB_CHECKSUM_MISO_OFFSET;
    } else {
        miso_object_byte_offset = ODrivePDOmap::getMISOByteOffset(ODriveObjectName::CheckSumMISO, ODriveAxis::None);
    }

    for (int byte_offset = 0; byte_offset < miso_object_byte_offset; byte_offset++) {
        checkSumValue += pdo_interface_.read8(slave, byte_offset).ui;
    }

    uint32_t receivedCheckSum = pdo_interface_.read32(slave, miso_object_byte_offset).ui;

    if (receivedCheckSum != checkSumValue) {
        logger_->warn(logger_->fstring("Slave %d: Checksum value is not correct. Received checksum: %d, calculated checksum: %d", slave, receivedCheckSum, checkSumValue));
        return false;
    }
    return true;
}

bool EthercatMaster::checkSlaveName(uint16_t slave, const char* name)
{
    uint32_t slave_name_index = 0x1008;
    int name_length = 48;
    char slave_name[48] = {""};

    ec_SDOread(slave, slave_name_index, 0, FALSE, &name_length, &slave_name, EC_TIMEOUTRXM);
    if (strcmp(name, slave_name) != 0) {
        return false;
    }
    return true;
}

void EthercatMaster::writeChecksumMOSI()
{
    for (int slave = 1; slave <= max_slave_index_; slave++) {

        if (strcmp(ec_slave[slave].name, "MDrive") == 0) {
            uint32_t checkSumValue = 0;
            int8_t mosi_byte_offset = ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::CheckSumMOSI, ODriveAxis::One);
            for (int byte_offset = 0; byte_offset < mosi_byte_offset; byte_offset++) {
                checkSumValue += pdo_interface_.read8mosi(slave, byte_offset).ui;
            }

            uint32_t receivedCheckSumMOSIStatus = pdo_interface_.read32(slave, ODrivePDOmap::getMISOByteOffset(ODriveObjectName::CheckSumMOSIStatus, ODriveAxis::None)).ui;
        
            if (receivedCheckSumMOSIStatus != 0) {
                logger_->warn(logger_->fstring("Slave %d: Checksum MOSI status is not zero. Received checksum: %d", slave, receivedCheckSumMOSIStatus));
            }

            bit32 write_checksum = {};
            write_checksum.ui = checkSumValue;

            pdo_interface_.write32(slave, mosi_byte_offset, write_checksum);
        }
    }
}

bool EthercatMaster::sendReceivePdo()
{
    if (this->latest_lost_slave_ == -1) {
        writeChecksumMOSI();
        ec_send_processdata();
        const int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (!has_warned_about_worker_counter and wkc < this->expected_working_counter_) {
            has_warned_about_worker_counter = true;
            logger_->warn(logger_->fstring(
                "Working counter: %d  is lower than expected: %d", wkc, this->expected_working_counter_));
            return false;
        }
        // loop over max_slave_index_ to check for each slave if checksum is correct
        for (int slave = 1; slave <= max_slave_index_; slave++) {
            if (!this->isCheckSumValid(slave)) {
                return false;
            }
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
            if (!has_warned_about_ethercat_lost_connection) {
                has_warned_about_ethercat_lost_connection = true;
                logger_->warn(logger_->fstring("EtherCAT train lost connection from slave %d onwards", slave));
            }

            if (!this->attemptSlaveRecover(slave)) {
                this->latest_lost_slave_ = slave;
                return;
            }
        }
    }

    if (this->latest_lost_slave_ > -1) {
        logger_->info("All slaves returned to operational state.");
    }

    this->latest_lost_slave_ = -1;
    this->valid_slaves_timestamp_ms_ = std::chrono::high_resolution_clock::now();
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
                logger_->error(logger_->fstring("Ethercat lost connection to slave %i", slave));
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
        logger_->info(logger_->fstring("Slave %i resumed operational state", slave));
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
        logger_->info("Stopping EtherCAT");
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
    const int error = pthread_setschedparam(this->ethercat_thread_.native_handle(), SCHED_FIFO, &param);
    if (error != 0) {
        logger_->error(
            logger_->fstring("Failed to set the ethercat thread priority to %d. (error code: %d)", priority, error));
    } else {
        logger_->debug(logger_->fstring("Set ethercat thread priority to %d", param.sched_priority));
    }
}
} // namespace march
