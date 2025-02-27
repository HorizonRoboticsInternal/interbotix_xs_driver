#include <array>

#include "nlohmann/json.hpp"

namespace horizon
{
namespace widowx
{

struct GripperStatus
{
    float radian;
    float distance;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GripperStatus, radian, distance);
};

struct ArmStatus
{
    std::array<float, 7> joint_positions;
    std::array<float, 7> joint_velocities;
    std::array<float, 7> joint_efforts;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ArmStatus, joint_positions, joint_velocities, joint_efforts);
};

struct PositionBound
{
    PositionBound(float l, float u) : lower(l), upper(u) {
    }

    float lower;
    float upper;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PositionBound, lower, upper);
};

}  // namespace widowx
}  // namespace horizon
