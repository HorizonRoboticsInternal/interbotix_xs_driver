#pragma once

#include "drogon/HttpAppFramework.h"
#include "drogon/PubSubService.h"
#include "drogon/WebSocketController.h"
#include "wx_armor/wx_armor_driver.h"

namespace horizon::wx_armor
{

static constexpr int MAX_TOLERABLE_CONSECUTIVE_NUM_READ_ERRORS = 6;

class GuardianThread
{
    /* The GuardianThread is responsible for 1) continuously (by running a
     * background thread) read the sensor data and publish it to the clients
     * that subscribe the sensor data and 2) monitoring for safety violations
     * and notifying WxArmorDriver accordingly to prevent system damage.
     *
     * Currently, the GuardianThread does several things, i.e. the continuous
     * reading (the thread part), the joint velocity safety check (the guardian
     * part), and sending the readings to the clients.
     *
     * However, there is additional safety checking for action delta in the
     * WxAarmorWebController.  This is not ideal.
     *
     * In the future, the Guardian should be moved to a separate class as a
     * wrapper for the WxArmorDriver.  It should contain the safety_violation_
     * related logic in the current WxArmorDriver.
     *
     * The WebController should be yet another layer on top of the Guardian
     * class, to handle sending the readings to the clients (via injecting a
     * callback to the Guardian's reading thread), and sending actions to the
     * WxArmorDriver.
     */
  public:
    GuardianThread();
    ~GuardianThread();

    // Add the client (identified by the connection) to the subscription
    // list.
    void Subscribe(const drogon::WebSocketConnectionPtr& conn);

    // Remove the client (identified by the connection) to the subscription
    // list.
    void Unsubscribe(const drogon::WebSocketConnectionPtr& conn);

    // Close and remove all client connections from subscription list
    void KillConnections();

    /**
     * @brief Returns the sensor data cached from the latest read.
     *
     * @return A copy of the sensor data.
     */
    const SensorData GetCachedSensorData();

    const static int32_t kErrorCommandDeltaTooLarge = 1 << 9;
    const static int32_t kErrorVelocityLimitViolation = 1 << 10;
    const static int32_t kErrorCurrentLimitViolation = 1 << 11;
    const static int32_t kErrorMotorNotReachable = 1 << 12;

    /**
     * @brief Resets the error codes for all motors.
     */
    void ResetErrorCodes();

    /**
     * @brief Sets the error code for a specific motor.
     *
     * Args:
     *     motor_idx: The motor index to set the error code for. Note that this is
     *         an index and NOT the motor ID, which usually starts from 1.
     *     error_code: The error code to set for the motor.
     *     is_joint_idx: If True, will consider the motor_idx, a joint_idx and map accordingly.
     */
    void SetErrorCode(uint8_t motor_idx, int32_t error_code, bool is_joint_idx = false);

  private:
    std::mutex conns_mutex_;  // protects conns_
    std::vector<drogon::WebSocketConnectionPtr> conns_{};

    std::atomic_bool shutdown_{false};
    std::jthread thread_{};

    // Book keeping of the number of read errors in a row. Reset to 0 as
    // soon as a successful read is seen. Server will crash as soon as this
    // number exceeds the threshold.
    int num_consecutive_read_errors_ = 0;

    SensorData sensor_data_cache_;
    std::mutex cache_mutex_;  // protects read/write access to sensor_data_cache.

    // Error codes for all motors, protected by cache_mutex_.
    // lower 8 bits are for dynamixel errors,
    // bit 9 is for command delta too large,
    // bit 10 is joint velocity limit violation,
    // bit 11 is joint over current limit violation,
    std::vector<int32_t> error_codes_{};
};

}  // namespace horizon::wx_armor
