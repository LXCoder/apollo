#include "follow_model_base.h"

#include <fcntl.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"
namespace apollo {
namespace planning {

bool FollowModelBase::Init(const FollowModelConfig& config) {
  config_.MergeFrom(config);

  if (config_.model().type() == CarFollowModel::CUSTOM) {
    struct stat info;
    custom_model_config_path_ = config_.model().config_path();
    std::string model_path = config_.model().model_path();
    if (stat(custom_model_config_path_.c_str(), &info) != 0 ||
        stat(model_path.c_str(), &info) != 0) {
      char message[512];
      memset(message, 0, sizeof(message));
      std::sprintf(message, "model [%s] or config [%s] file is not exists.",
                   model_path.c_str(), custom_model_config_path_.c_str());
      AERROR << message;
      return false;
    }
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