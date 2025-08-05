#include "follow_model_base.h"

#include <fcntl.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace apollo {
namespace planning {

bool FollowModelBase::Init(const FollowModelConfig& config) {
  config_.MergeFrom(config);

  if (config_.model().type() != CarFollowModel::CUSTOM) {
    name_ = CarFollowModel_Type_Name(config_.model().type());
    return true;
  }

  custom_model_config_path_ = config_.model().config_path();
  name_ = config_.model().name();
  std::string model_path = config_.model().model_path();
  char message[512];
  memset(message, 0, sizeof(message));
  struct stat info;

  if (stat(model_path.c_str(), &info) != 0) {
    std::snprintf(message, sizeof(message), "model [%s] file is not exists.",
                  model_path.c_str());
    AERROR << message;
    return false;
  }

  if (config_.model().has_config_path() &&
      stat(custom_model_config_path_.c_str(), &info) != 0) {
    std::snprintf(message, sizeof(message), "config [%s] file is not exists.",
                  custom_model_config_path_.c_str());
    AERROR << message;
    return false;
  }

  return true;
}

bool FollowModelBase::Calculate(
    const NeighborVehicleInfo& neighbor_vehicle_info,
    CarFollowSpeedPoint& speed_point) {
  return true;
}

}  // namespace planning
}  // namespace apollo