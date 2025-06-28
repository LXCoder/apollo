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
 * @file idm_model_task.h
 **/

#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
// #include "modules/planning/tasks/idm_model/proto/idm_model_task.pb.h"
#include "bazel-out/k8-dbg/bin/modules/planning/tasks/idm_model/proto/idm_model_task.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_interface_base/task_base/common/speed_optimizer.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

class IDMModelTask : public SpeedOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

  common::Status Execute(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  virtual common::Status Process(const PathData& path_data,
                                 const common::TrajectoryPoint& init_point,
                                 SpeedData* const speed_data) override;

 private:
  double CalculateIDMModel(double ego_speed, double front_vehicle_speed,
                           double front_vehicle_distance);

  inline double CalculateScalarVelocity(double vx, double vy) {
    return std::sqrt(vx * vx + vy * vy);
  };

 private:
  IDMModelTaskConfig config_;
  Frame* frame_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::IDMModelTask,
                                     apollo::planning::Task)
}  // namespace planning
}  // namespace apollo