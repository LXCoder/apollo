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

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_interface_base/task_base/common/speed_optimizer.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

using apollo::common::Status;

namespace apollo {
namespace planning {

class FollowModelDecider : public SpeedOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

  Status Execute(Frame* frame, ReferenceLineInfo* reference_line_info) override;

  virtual Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;

 private:
  bool InitPointIsCollision();

  inline double CalculateScalarVelocity(double vx, double vy) {
    return std::sqrt(vx * vx + vy * vy);
  };

 private:
  FollowModelConfig config_;
  Frame* frame_;
  ReferenceLineInfo* reference_line_info_;
  double total_length_t_ = 0.0;
  double unit_t_ = 0.0;
  uint32_t dimension_t_ = 0;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::FollowModelDecider,
                                     apollo::planning::Task)
}  // namespace planning
}  // namespace apollo