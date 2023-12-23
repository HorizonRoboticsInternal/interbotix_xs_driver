#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

namespace horizon::wx_armor {

enum class IOType {
  READ = 0,
  WRITE = 1,
};

class DynamixelIO {
 public:
  DynamixelIO(IOType type,
              DynamixelWorkbench *dxl_wb,
              const std::vector<uint8_t> &joint_ids,
              const std::vector<std::string> &keys);

  inline const ControlItem &GetAddress(const std::string &key) const {
    return address_per_key_.at(key);
  }

 protected:
  DynamixelWorkbench *dxl_wb_;
  std::vector<uint8_t> joint_ids_;
  uint8_t num_joints_;

  uint8_t handler_index_ = 0;
  std::map<std::string, ControlItem> address_per_key_{};
};

class JointStateReader : protected DynamixelIO {
 public:
  JointStateReader(DynamixelWorkbench *dxl_wb,
                   const std::vector<uint8_t> &joint_ids);

  bool ReadTo(std::vector<float> *position,
              std::vector<float> *velocity,
              std::vector<float> *current);

 private:
  std::mutex handler_mutex_;
  std::mutex int_buffer_mutex_;
  ControlItem position_add_;
  std::vector<int32_t> position_int_;
  ControlItem velocity_add_;
  std::vector<int32_t> velocity_int_;
  ControlItem current_add_;
  std::vector<int32_t> current_int_;
};

}  // namespace horizon::wx_armor
