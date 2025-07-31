/**
 * @file follow_model_base.h
 **/

#pragma once

#include <memory>
#include <string>

#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/util/config_util.h"
namespace apollo {
namespace planning {

class IDMParamsImpl {
 public:
  virtual ~IDMParamsImpl() = default;

  virtual double ego_speed() const = 0;
  virtual double front_vehicle_speed() const = 0;
  virtual double front_vehicle_distance() const = 0;
};

struct CarFollowSpeedPoint {
  double a;
  double v;
  double s;
};

struct NeighborVehicleInfo {
  // ego
  double ego_speed;
  apollo::hdmap::LaneInfoConstPtr ego_lane_ptr;
  // closet in path front vehicle
  double front_vehicle_speed;
  double front_vehicle_distance;
  apollo::hdmap::LaneInfoConstPtr front_vehicle_lane_ptr;
  // closet in path rear vehicle
  double rear_vehicle_speed;
  double rear_vehicle_distance;
  apollo::hdmap::LaneInfoConstPtr rear_vehicle_lane_ptr;
};

class FollowModelBase {
 public:
  FollowModelBase() = default;
  virtual ~FollowModelBase() = default;

  virtual bool Init(const FollowModelConfig& config);
  virtual bool Calculate(const NeighborVehicleInfo& neighbor_vehicle_info,
                         CarFollowSpeedPoint& speed_point);

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  FollowModelConfig config_;
  std::string custom_model_config_path_;
  std::shared_ptr<IDMParamsImpl> params_;
};

template <typename T>
bool FollowModelBase::LoadConfig(T* config) {
  return ConfigUtil::LoadMergedConfig(custom_model_config_path_,
                                      custom_model_config_path_, config);
}

}  // namespace planning
}  // namespace apollo
