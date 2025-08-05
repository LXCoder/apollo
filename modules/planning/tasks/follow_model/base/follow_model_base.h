/**
 * @file follow_model_base.h
 **/

#pragma once

#include <memory>
#include <string>

#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

// #include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/util/config_util.h"

namespace apollo {

namespace hdmap {
class LaneInfo;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
}  // namespace hdmap

namespace planning {

#define REGISTER_CUSTOM_MODEL(ModelType)                           \
  extern "C" {                                                     \
  FollowModelBase* CreateFollowModel() { return new ModelType(); } \
                                                                   \
  void DestroyFollowModel(FollowModelBase* ptr) { delete ptr; }    \
  }

struct CarFollowSpeedPoint {
  double a;
  double v;
  double s;
};

struct Vehicle {
  double speed;
  double distance;
  hdmap::LaneInfoConstPtr lane_ptr;
};

struct NeighborVehicleInfo {
  Vehicle ego;            // ego
  Vehicle front_vehicle;  // closet in path front vehicle
  Vehicle rear_vehicle;   // closet in path rear vehicle
};

class FollowModelBase {
 public:
  FollowModelBase() = default;
  virtual ~FollowModelBase() = default;

  virtual bool Init(const FollowModelConfig& config);
  virtual bool Calculate(const NeighborVehicleInfo& neighbor_vehicle_info,
                         CarFollowSpeedPoint& speed_point);

  std::string Name() { return name_; }

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  FollowModelConfig config_;
  std::string custom_model_config_path_;
  std::string name_;
};

template <typename T>
bool FollowModelBase::LoadConfig(T* config) {
  return ConfigUtil::LoadMergedConfig(custom_model_config_path_,
                                      custom_model_config_path_, config);
}

}  // namespace planning
}  // namespace apollo
