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
 * @file idm_model_task.cc
 **/

#include "modules/planning/tasks/idm_model/idm_model_task.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include "google/protobuf/message.h"
#include "idm_utils.h"

#include "bazel-out/k8-dbg/bin/modules/common_msgs/basic_msgs/geometry.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/localization_msgs/localization.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/localization_msgs/pose.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "bazel-out/k8-dbg/bin/modules/planning/tasks/idm_model/proto/idm_model_task.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"

using namespace apollo;

namespace apollo {
namespace planning {

bool IDMModelTask::Init(const std::string& config_dir, const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;
  }
  // To be implemented.
  bool flag = SpeedOptimizer::LoadConfig<IDMModelTaskConfig>(&config_);
  printf("%s\n", config_.Utf8DebugString().c_str());
  // Load the config.
  // return SpeedOptimizer::LoadConfig<IDMModelTaskConfig>(&config_);
  return flag;
}

common::Status IDMModelTask::Execute(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  // To be implemented.
  frame_ = frame;
  auto ret =
      Process(reference_line_info->path_data(), frame->PlanningStartPoint(),
              reference_line_info->mutable_speed_data());
  return ret;
}

common::Status IDMModelTask::Process(const PathData& path_data,
                                     const common::TrajectoryPoint& init_point,
                                     SpeedData* const speed_data) {
  std::shared_ptr<prediction::PredictionObstacles> prediction =
      frame_->local_view().prediction_obstacles;
  std::shared_ptr<localization::LocalizationEstimate> location =
      frame_->local_view().localization_estimate;
  localization::Pose pose = location->pose();
  printf("seqnum: %u pos: [%f,%f,%f]\n", frame_->SequenceNum(),
         location->pose().position().x(), location->pose().position().y(),
         location->pose().position().z());

  prediction::PredictionObstacle cipv;
  double distance = 0.0;
  if (!SelectCIPVFromObstacles(pose, prediction, 10.0, &cipv, &distance)) {
    auto cipv_obs = cipv.perception_obstacle();
    double ego_speed = CalculateScalarVelocity(pose.linear_velocity().x(),
                                               pose.linear_velocity().y());
    double cipv_speed = CalculateScalarVelocity(cipv_obs.velocity().x(),
                                                cipv_obs.velocity().y());
    double acceleration = CalculateIDMModel(ego_speed, cipv_speed, distance);
    printf(
        "find cipv, id: %d distance: %.8f ,speed: %.8f ,ego_speed: %.8f\nidm "
        "acceleration: %.8f\n",
        cipv_obs.id(), distance, cipv_speed, ego_speed, acceleration);

    auto feature_v = PredictNonUniformAcceleration(config_, ego_speed, 0, 0, 1,
                                                   7, cipv_speed, distance);
    for (auto& item : feature_v) {
      std::cout << std::fixed << std::setprecision(2) << item.t << "\t"
                << item.v << "\t\t" << item.s << "\n";
    }

  } else {
    printf("no cipv\n");
  }
  return common::Status::OK();
}

double IDMModelTask::CalculateIDMModel(double ego_speed,
                                       double front_vehicle_speed,
                                       double front_vehicle_distance) {
  IDMModelTaskConfig config = config_;
  // 计算相对速度
  double delta_v = ego_speed - front_vehicle_speed;
  // 计算期望跟车距离
  double expect_distance =
      config.min_saft_distance() +
      std::max(0.0, ego_speed * config.saft_distance() +
                        (ego_speed * delta_v) /
                            (2 * std::sqrt(config.max_acceleration() *
                                           config.comfortable_deceleration())));
  // 计算加速度
  double acceleration =
      config.max_acceleration() *
      (1.0 -
       std::pow((ego_speed / config.expect_speed()),
                config.delta_acceleration()) -
       std::pow((expect_distance / front_vehicle_distance), 2.0));

  return acceleration;
}

}  // namespace planning
}  // namespace apollo