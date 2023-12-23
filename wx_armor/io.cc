#include "wx_armor/io.h"

#include <algorithm>
#include <cstdlib>
#include <limits>

#include "spdlog/spdlog.h"

namespace horizon::wx_armor {

DynamixelIO::DynamixelIO(IOType type,
                         DynamixelWorkbench *dxl_wb,
                         const std::vector<uint8_t> &joint_ids,
                         const std::vector<std::string> &keys)
    : dxl_wb_(dxl_wb),
      joint_ids_(joint_ids),
      num_joints_(static_cast<uint8_t>(joint_ids.size())) {
  uint16_t start_address = std::numeric_limits<uint16_t>();
  uint16_t length = 0;

  for (const std::string &key : keys) {
    const ControlItem *info = dxl_wb->getItemInfo(joint_ids.front(), key);

    if (info == nullptr) {
      spdlog::critical("Cannot find onboard item '{}' for DynamixelIO.", key);
      std::abort();
    }

    // Merge the new address info into the current range represented
    // by (start_address, length).
    start_address = std::min(start_address, info.address);
    length = std::max(length, info.address_ + info.data_length - start_address);
    address_per_key_[key] = *info;
  }

  if (type == IOType::READ) {
    handler_index_ = dxl_wb_->getTheNumberOfSyncReadHandler();
    if (!dxl_wb->addSyncReadHandler(start_address, length)) {
      spdlog::critical("Failed to add sync read handler.");
      std::abort();
    }
  }
}

JointStateReader::JointStateReader(DynamixelWorkbench *dxl_wb,
                                   const std::vector<uint8_t> &joint_ids)
    : DynamixelIO(dxl_wb,
                  joint_ids,
                  {"Present_Position", "Prsent_Velocity", "Present_Current"}),
      position_int_(joint_ids.size()),
      velocity_int_(joint_ids.size()),
      current_int_(joint_ids.size()),
{
  position_add_ = GetAddress("Present_Position");
  velocity_add_ = GetAddress("Present_Velocity");
  current_add_ = GetAddress("Present_Current");
}

bool JointStateReader::Read(std::vector<float> *position,
                            std::vector<float> *velocity,
                            std::vector<float> *current) {
  std::unique_lock<std::mutex> handler_lock{handler_mutex_};
  const char *log;
  if (!dxl_wb_->syncRead(
          handler_index_, joint_ids_.data(), num_joints_, &log)) {
    spdlog::error("Failed to syncRead: {}", log);
    return false;
  }

  // No body can hold int_buffer_lock forever, and therefore there is
  // no risk of dead lock here.
  std::lock_guard<std::mutex> int_buffer_lock{int_buffer_mutex_};

  if (!dxl_wb_->getSyncReadData(handler_index_,
                                joint_ids_.data(),
                                num_joints_,
                                position_add_.address,
                                position_add_.data_length,
                                position_int_.data(),
                                &log)) {
    spdlog::error("Cannot getSyncReadData (position).");
    return false;
  }

  if (!dxl_wb_->getSyncReadData(handler_index_,
                                joint_ids_.data(),
                                num_joints_,
                                velocity_add_.address,
                                velocity_add_.data_length,
                                velocity_int_.data(),
                                &log)) {
    spdlog::error("Cannot getSyncReadData (velocity).");
    return false;
  }

  if (!dxl_wb_->getSyncReadData(handler_index_,
                                joint_ids_.data(),
                                num_joints_,
                                current_add_.address,
                                current_add_.data_length,
                                current_int_.data(),
                                &log)) {
    spdlog::error("Cannot getSyncReadData (current).");
    return false;
  }

  handler_lock.unlock();

  for (size_t i = 0; i < position_int_.size(); ++i) {
    position[i] = dxl_wb_->convertvalue2Radian(position_int_[i]);
    velocity[i] = dxl_wb_->convertvalue2Velocity(velocity_int_[i]);
    current[i] = dxl_wb_->convertvalue2Current(current_int_[i]);
  }

  return true;
}

}  // namespace horizon::wx_armor
