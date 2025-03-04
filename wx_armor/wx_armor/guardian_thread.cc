#include "wx_armor/guardian_thread.h"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

using drogon::HttpRequestPtr;
using drogon::WebSocketConnectionPtr;
using drogon::WebSocketMessageType;

namespace horizon::wx_armor
{

GuardianThread::GuardianThread() {
    thread_ = std::jthread([this]() {
        std::vector<float> safety_velocity_limits = Driver()->GetSafetyVelocityLimits();
        std::vector<float> safety_current_limits = Driver()->GetSafetyCurrentLimits();
        bool logged = false;
        while (!shutdown_.load()) {
            // Read the sensor data
            std::optional<SensorData> sensor_data = Driver()->Read();

            // Publish the sensor data
            if (sensor_data.has_value() || Driver()->SafetyViolationTriggered()) {
                if (!sensor_data.has_value()) {
                    // If we have an error, we still want to publish the error
                    // codes
                    static const uint8_t num_joints = Driver()->Profile().joint_ids.size();
                    sensor_data = SensorData{.pos = std::vector<float>(num_joints),
                                             .vel = std::vector<float>(num_joints),
                                             .crt = std::vector<float>(num_joints),
                                             .err = error_codes_,
                                             .timestamp = -1};
                }
                else {
                    std::unique_lock<std::mutex> cache_lock{cache_mutex_};
                    // merge sensor_data's error codes with the existing ones
                    // with bitwise OR
                    error_codes_.resize(sensor_data.value().err.size());
                    for (size_t i = 0; i < sensor_data.value().err.size(); i++) {
                        // Do not call SetErrorCode() here, due to deadlocking
                        // of cache_mutex_
                        error_codes_[i] |= sensor_data.value().err[i];
                        if (!logged && sensor_data.value().err[i] != 0) {
                            spdlog::error("Guardian thread detected hardware error for "
                                          "motor {}. Error "
                                          "code: {}",
                                          i, sensor_data.value().err[i]);
                        }
                    }
                    sensor_data_cache_ = SensorData{.pos = sensor_data.value().pos,
                                                    .vel = sensor_data.value().vel,
                                                    .crt = sensor_data.value().crt,
                                                    .err = error_codes_};
                    sensor_data = sensor_data_cache_;
                    num_consecutive_read_errors_ = 0;
                }
                std::string message = nlohmann::json(sensor_data.value()).dump();
                std::lock_guard<std::mutex> lock{conns_mutex_};
                for (const WebSocketConnectionPtr& conn : conns_) {
                    conn->send(message);
                }

                // If a safety violation is triggered, Slow down to a stop.
                // This way, user will have to reset on client-side, but the
                // server will keep running. NOTE: We need to put this check
                // before all the triggers below, so that one last message
                // containing the error codes will be sent to the client before
                // we stop the arm here.
                if (Driver()->SafetyViolationTriggered()) {
                    if (!logged) {
                        spdlog::error("Guardian thread safety violation checking "
                                      "ignored. Safety "
                                      "violation is already triggered.");
                        SlowDownToStop(sensor_data.value());
                        logged = true;
                    }
                }
                else {
                    logged = false;
                }

                // if hardware error is detected, trigger safety violation mode
                if (sensor_data.value().err.size() > 0) {
                    for (const auto& error_code : sensor_data.value().err) {
                        if (error_code != 0) {
                            Driver()->TriggerSafetyViolationMode();
                            break;
                        }
                    }
                }

                // GuardianThread also has the dual role of monitoring for
                // safety e.g., checking for velocity limit violations
                std::vector<float> curr_velocities = sensor_data.value().vel;
                for (int i = 0; i < curr_velocities.size(); i++) {
                    float cv = fabs(curr_velocities[i]);
                    float limit = safety_velocity_limits[i];
                    if (cv > limit) {
                        Driver()->TriggerSafetyViolationMode();
                        SetErrorCode(i, kErrorVelocityLimitViolation);
                        if (!logged)
                            spdlog::error("Guardian thread detected velocity limit "
                                          "violation for motor "
                                          "{}. Current velocity: {}, Limit: {}",
                                          i, cv, limit);
                    }
                }

                // Check for current limit violations
                std::vector<float> curr_currents = sensor_data.value().crt;
                for (int i = 0; i < curr_currents.size(); i++) {
                    float cc = fabs(curr_currents[i]);
                    float limit = safety_current_limits[i];
                    if (cc > limit) {
                        Driver()->TriggerSafetyViolationMode();
                        // Set error_code_ to 1 for the motor that violated the
                        // limit
                        SetErrorCode(i, kErrorCurrentLimitViolation);
                        if (!logged)
                            spdlog::error("Guardian thread detected current limit "
                                          "violation for motor "
                                          "{}. Current current: {}, Limit: {}",
                                          i, cc, limit);
                    }
                }
            }
            else if (!Driver()->SafetyViolationTriggered()) {
                // When fail to read, accumulate the counter, check for
                // threshold and warn, and keep waiting. NOTE: no need to
                // trigger a power cycle here, given CheckAndSetPosition already
                // does a motor health check.
                ++num_consecutive_read_errors_;
                if (num_consecutive_read_errors_ > MAX_TOLERABLE_CONSECUTIVE_NUM_READ_ERRORS) {
                    if (!logged)
                        spdlog::critical("Encounter {} consecutive read errors, which is "
                                         "considered too "
                                         "many. Sleeping before retry..",
                                         num_consecutive_read_errors_);
                    Driver()->TriggerSafetyViolationMode();
                    SetErrorCode(0, kErrorMotorNotReachable);
                    num_consecutive_read_errors_ = 0;
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

const SensorData GuardianThread::GetCachedSensorData() {
    std::unique_lock<std::mutex> cache_lock{cache_mutex_};
    return sensor_data_cache_;
}

void GuardianThread::ResetErrorCodes() {
    std::unique_lock<std::mutex> cache_lock{cache_mutex_};
    for (int32_t& error_code : error_codes_) {
        error_code = 0;
    }
    Driver()->ResetSafetyViolationMode();
}

void GuardianThread::SetErrorCode(uint8_t motor_id, uint32_t error_code) {
    std::unique_lock<std::mutex> cache_lock{cache_mutex_};
    error_codes_[motor_id] |= error_code;
}

GuardianThread::~GuardianThread() {
    shutdown_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void GuardianThread::Subscribe(const WebSocketConnectionPtr& conn) {
    // Clear all safety violations only if there are no previous connections
    if (conns_.empty()) {
        Driver()->ResetSafetyViolationMode();
    }
    std::lock_guard<std::mutex> lock{conns_mutex_};
    conns_.emplace_back(conn);
}

void GuardianThread::Unsubscribe(const WebSocketConnectionPtr& conn) {
    std::lock_guard<std::mutex> lock{conns_mutex_};
    conns_.erase(std::remove(conns_.begin(), conns_.end(), conn), conns_.end());
}

void GuardianThread::KillConnections() {
    for (auto& conn : conns_) {
        // TODO(andrew): log which joint was responsible?
        conn->shutdown(drogon::CloseCode::kViolation, "Shutting down due to safety violation.");
    }
    conns_.clear();
}

}  // namespace horizon::wx_armor
