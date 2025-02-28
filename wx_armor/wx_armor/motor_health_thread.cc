#include "wx_armor/motor_health_thread.h"
#include "wx_armor/guardian_thread.h"
#include "wx_armor/wx_armor_driver.h"

#include "spdlog/spdlog.h"

namespace horizon::wx_armor
{

MotorHealthThread::MotorHealthThread(const std::shared_ptr<GuardianThread>& guardian)
    : thread_ready_(false), stop_thread_(false), guardian_thread_(guardian) {
    worker_thread_ = std::thread(&MotorHealthThread::CheckMotorHealth, this);

    // Block until the thread is ready.
    while (!thread_ready_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

MotorHealthThread::~MotorHealthThread() {
    stop_thread_.store(true);
    if (worker_thread_.joinable()) {
        worker_thread_.join();
        spdlog::info("MotorHealthThread has joined.");
    }
}

void MotorHealthThread::CheckMotorHealth() {
    while (!stop_thread_) {
        if (!Driver()->SafetyViolationTriggered() && !Driver()->MotorHealthCheck()) {
            const SensorData readings = guardian_thread_->GetCachedSensorData();
            Driver()->TriggerSafetyViolationMode();
            guardian_thread_->SetErrorCode(0, GuardianThread::kErrorMotorNotReachable);
            SlowDownToStop(readings);
        }
        thread_ready_.store(true);
    }
}

}  // namespace horizon::wx_armor
