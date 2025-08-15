#include "follow_model_custom_example.h"

#include <fcntl.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
namespace apollo {
namespace planning {

bool FollowModelCustomExample::Init(const FollowModelConfig& config) {
  FollowModelBase::Init(config);
  // To be implemented.
  // Load custom config
  // FollowModelConfig custom_config;
  // FollowModelBase::LoadConfig<FollowModelConfig>(&custom_config);
  return true;
}

bool FollowModelCustomExample::Calculate(
    const NeighborVehicleInfo& neighbor_vehicle_info,
    CarFollowSpeedPoint& speed_point) {
  // To be implemented.
  auto model_param = config_.model_param();
  double ego_speed = neighbor_vehicle_info.ego.speed;
  double exepect_speed =
      model_param.has_expect_speed()
          ? model_param.expect_speed()
          : neighbor_vehicle_info.ego.lane_ptr->lane().speed_limit();
  double delta_v = ego_speed - neighbor_vehicle_info.front_vehicle.speed;
  double expect_distance =
      model_param.min_gap() +
      std::max(0.0, ego_speed * model_param.tau() +
                        (ego_speed * delta_v) /
                            (2 * std::sqrt(model_param.accel() *
                                           model_param.decel())));
  double acc =
      model_param.accel() *
      (1.0 - std::pow((ego_speed / exepect_speed), model_param.delta()) -
       std::pow(
           (expect_distance / neighbor_vehicle_info.front_vehicle.distance),
           2.0));

  speed_point.a = acc;

  return true;
}

REGISTER_CUSTOM_MODEL_PLUGIN(FollowModelCustomExample)

}  // namespace planning
}  // namespace apollo