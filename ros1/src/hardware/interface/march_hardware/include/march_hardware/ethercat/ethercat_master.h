// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#define MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <march_hardware/joint.h>

namespace march {
/**
 * Base class of the ethercat master supported with the SOEM library
 * @param if_name Network interface name, check ifconfig.
 * @param io_map Holds the mapping of the SOEM message.
 * @param expected_working_counter The expected working counter of the ethercat
 * train.
 * @param cycle_time_ms The ethercat cycle time.
 * @param max_slave_index The maximum amount of slaves connected to the train.
 */
class EthercatMaster {
public:
    EthercatMaster(std::string if_name, int max_slave_index, int cycle_time,
        int slave_timeout);
    ~EthercatMaster();

    /* Delete copy constructor/assignment since the member thread can not be
     * copied */
    EthercatMaster(const EthercatMaster&) = delete;
    EthercatMaster& operator=(const EthercatMaster&) = delete;

    /* Delete move constructor/assignment since atomic bool cannot be moved */
    EthercatMaster(EthercatMaster&&) = delete;
    EthercatMaster& operator=(EthercatMaster&&) = delete;

    bool isOperational() const;
    void waitForPdo();

    std::exception_ptr getLastException() const noexcept;

    /**
     * Returns the cycle time in milliseconds.
     */
    int getCycleTime() const;

    /**
     * Initializes the ethercat train and starts a thread for the loop.
     * @throws HardwareException If not the configured amount of slaves was
     * found or they did not all reach operational state
     */
    bool start(std::vector<Joint>& joints);

    /**
     * Stops the ethercat loop and joins the thread.
     */
    void stop();

    static const int THREAD_PRIORITY = 40;

private:
    /**
     * Opens the ethernet port with the given if_name and checks the amount of
     * slaves.
     */
    void ethercatMasterInitiation();

    /**
     * Configures the found slaves to operational state.
     */
    bool ethercatSlaveInitiation(std::vector<Joint>& joints);

    /**
     * The ethercat train PDO loop. If the working counter is lower than
     * expected 5% of the time, the program displays an error.
     */
    void ethercatLoop();

    /**
     * Sends the PDO and receives the working counter and check if this is lower
     * than expected.
     *
     * @returns true if and only if all PDOs have been successfully sent and
     * received, otherwise false.
     */
    bool sendReceivePdo();

    /**
     * Checks if all the slaves are connected and in operational state.
     */
    void monitorSlaveConnection();

    /**
     * Attempts to recover a slave to operational state.
     *
     * @returns true when recovery was successfull, otherwise false.
     */
    bool attemptSlaveRecover(int slave);

    /**
     * Sets ethercat state to INIT and closes port.
     */
    void closeEthercat();

    /**
     * Sets the ethercat thread priority and scheduling
     * to SCHED_FIFO using pthread.
     * Note: Only works on POSIX compliant systems.
     *
     * @param priority a pthread priority value between 1 and 99 for SCHED_FIFO
     * threads.
     */
    void setThreadPriority(int priority);

    std::atomic<bool> is_operational_;

    const std::string if_name_;
    const int max_slave_index_;
    const int cycle_time_ms_;

    std::mutex wait_on_pdo_condition_mutex_;
    std::condition_variable wait_on_pdo_condition_var_;
    bool pdo_received_ = false;

    std::array<char, 4096> io_map_ = { 0 };
    int expected_working_counter_ = 0;

    int latest_lost_slave_ = -1;
    const int slave_watchdog_timeout_;
    std::chrono::high_resolution_clock::time_point valid_slaves_timestamp_ms_;

    std::thread ethercat_thread_;
    std::exception_ptr last_exception_;
};

} // namespace march
#endif // MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
