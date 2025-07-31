/**
 * @file follow_model_base.h
 **/

#pragma once

#include <memory>
#include <string>

#include "follow_model_base.h"

#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

namespace apollo {
namespace planning {

class FollowModelIDM : public FollowModelBase {
 public:
  FollowModelIDM() = default;
  virtual ~FollowModelIDM() = default;

  bool Init(const FollowModelConfig& config) override;
  bool Calculate(const NeighborVehicleInfo& neighbor_vehicle_info,
                 CarFollowSpeedPoint& speed_point) override;

 protected:
  std::string custom_model_config_path_;
  std::shared_ptr<IDMParamsImpl> params_;
};

}  // namespace planning
}  // namespace apollo
