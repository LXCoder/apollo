/**
 * @file follow_model_base.h
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

#include "modules/planning/tasks/follow_model/base/follow_model_base.h"

namespace apollo {
namespace planning {

class FollowModelCustomExample : public FollowModelBase {
 public:
  FollowModelCustomExample() = default;
  ~FollowModelCustomExample() = default;

  bool Init(const FollowModelConfig& config) override;
  bool Calculate(const NeighborVehicleInfo& neighbor_vehicle_info,
                 CarFollowSpeedPoint& speed_point) override;
};

}  // namespace planning
}  // namespace apollo
