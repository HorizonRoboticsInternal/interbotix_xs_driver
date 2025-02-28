#pragma once

#include <atomic>
#include <memory>
#include <thread>

namespace horizon::wx_armor
{

class GuardianThread;

/**
 * This is a thread for continuously checking the health of the dynamixel motors via pinging.
 * If the motors cannot be pinged, it will communicate this to the GuardianThread accordingly.
 */
class MotorHealthThread
{
  public:
    explicit MotorHealthThread(const std::shared_ptr<GuardianThread>& guardian_thread);
    ~MotorHealthThread();

  private:
    /**
     * @brief The worker function that continuously checks the health of the motors.
     */
    void CheckMotorHealth();

    // The GuardianThread that we forward errors to.
    std::shared_ptr<GuardianThread> guardian_thread_;

    // The worker thread.
    std::thread worker_thread_;

    // Thread-safe flags for setting ready and stop states.
    std::atomic<bool> thread_ready_;
    std::atomic<bool> stop_thread_;
};

}  // namespace horizon::wx_armor
