#include "follow_model_idm.h"

#include <fcntl.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include "modules/map/hdmap/hdmap_common.h"
#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"
namespace apollo {
namespace planning {

bool FollowModelIDM::Init(const FollowModelConfig& config) {
  return FollowModelBase::Init(config);
}

bool FollowModelIDM::Calculate(const NeighborVehicleInfo& neighbor_vehicle_info,
                               CarFollowSpeedPoint& speed_point) {
  auto model_param = config_.model_param();
  double ego_speed = neighbor_vehicle_info.ego.speed;
  double exepect_speed =
      model_param.has_expect_speed()
          ? model_param.expect_speed()
          : neighbor_vehicle_info.ego.lane_ptr->lane().speed_limit();
  // 计算相对速度
  double delta_v = ego_speed - neighbor_vehicle_info.front_vehicle.speed;
  // 计算期望跟车距离
  double expect_distance =
      model_param.min_gap() +
      std::max(0.0, ego_speed * model_param.tau() +
                        (ego_speed * delta_v) /
                            (2 * std::sqrt(model_param.accel() *
                                           model_param.decel())));
  // 计算加速度
  double acc =
      model_param.accel() *
      (1.0 - std::pow((ego_speed / exepect_speed), model_param.delta()) -
       std::pow(
           (expect_distance / neighbor_vehicle_info.front_vehicle.distance),
           2.0));

  speed_point.a = acc;

  return true;
}

}  // namespace planning
}  // namespace apollo