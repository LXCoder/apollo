/******************************************************************************
 * Copyright 2025 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file follow_model_decider.h
 **/

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"
#include "modules/planning/tasks/follow_model/proto/follow_model_info.pb.h"

#include "cyber/node/node.h"
#include "cyber/node/writer.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/speed/speed_data.h"
#include "modules/planning/planning_interface_base/task_base/common/speed_optimizer.h"
#include "modules/planning/planning_interface_base/task_base/task.h"
#include "modules/planning/tasks/follow_model/base/follow_model_base.h"

using apollo::common::Status;

namespace apollo {
namespace planning {

class FollowModelDecider : public SpeedOptimizer {
 public:
  // load custom model
  using CreateModelFunc = FollowModelBase* (*)();
  using DestroyModelFunc = void (*)(FollowModelBase*);

  FollowModelDecider();
  ~FollowModelDecider();

  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

  Status Execute(Frame* frame, ReferenceLineInfo* reference_line_info) override;

  virtual Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;

 private:
  bool InitPointIsCollision();
  bool LoadCarFollowModel();
  bool LoadCustomCarFollowModel();
  void ReginsterCarFollowModel();
  std::vector<common::SpeedPoint> PredictNonUniformAcceleration(
      double v0, double s0, double a0,
      NeighborVehicleInfo& neighbor_vehicle_info);
  void PrintSpeedData(const SpeedData* const speed_data,
                      const std::vector<common::SpeedPoint>& speed_profile);

  void WriteFollowModelInfo();

  inline double CalculateScalarVelocity(double vx, double vy) {
    return std::sqrt(vx * vx + vy * vy);
  };

 private:
  void* handle_;
  uint32_t dimension_t_;
  double unit_t_;
  FollowModelConfig config_;
  FollowModelInfo debug_info_;
  std::shared_ptr<FollowModelBase> car_follow_model_;
  std::unique_ptr<apollo::cyber::Node> follow_model_node_;
  std::shared_ptr<cyber::Writer<FollowModelInfo>> follow_model_info_writer_;
  DestroyModelFunc destroy_model_;
  CreateModelFunc create_model_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::FollowModelDecider,
                                     apollo::planning::Task)
}  // namespace planning
}  // namespace apollo