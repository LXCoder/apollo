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
 * @file follow_model_decider.cc
 **/

#include "modules/planning/tasks/follow_model/follow_model_decider.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "google/protobuf/message.h"
#include "idm_utils.h"

#include "bazel-out/k8-dbg/bin/modules/common_msgs/basic_msgs/geometry.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/localization_msgs/localization.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/localization_msgs/pose.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"

using namespace apollo;
using apollo::common::util::PointFactory;

namespace apollo {
namespace planning {

bool FollowModelDecider::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;
  }
  // To be implemented.
  bool flag = SpeedOptimizer::LoadConfig<FollowModelConfig>(&config_);
  unit_t_ = config_.unit_t();
  printf("%s\n", config_.Utf8DebugString().c_str());

  // Load the config.
  // return SpeedOptimizer::LoadConfig<IDMModelDeciderConfig>(&config_);
  return flag;
}

Status FollowModelDecider::Execute(Frame* frame,
                                ReferenceLineInfo* reference_line_info) {
  // To be implemented.
  printf("is near destination: %d\n", frame->is_near_destination());
  if (!config_.enable_idm() || frame->is_near_destination()) {
    return Status::OK();
  }

  frame_ = frame;
  reference_line_info_ = reference_line_info;
  total_length_t_ = reference_line_info->st_graph_data().total_time_by_conf();

  auto ret =
      Process(reference_line_info->path_data(), frame->PlanningStartPoint(),
              reference_line_info->mutable_speed_data());
  return ret;
}

Status FollowModelDecider::Process(const PathData& path_data,
                                const common::TrajectoryPoint& init_point,
                                SpeedData* const speed_data) {
  if (InitPointIsCollision()) {
    dimension_t_ = static_cast<uint32_t>(std::ceil(
                       total_length_t_ / static_cast<double>(unit_t_))) +
                   1;
    std::vector<common::SpeedPoint> speed_profile;
    double t = 0.0;
    for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {
      speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
    }
    *speed_data = SpeedData(speed_profile);
    return Status::OK();
  }

  std::shared_ptr<prediction::PredictionObstacles> prediction =
      frame_->local_view().prediction_obstacles;
  std::shared_ptr<localization::LocalizationEstimate> location =
      frame_->local_view().localization_estimate;
  localization::Pose pose = location->pose();
  printf("seqnum: %u pos: [%f,%f,%f]\n", frame_->SequenceNum(),
         location->pose().position().x(), location->pose().position().y(),
         location->pose().position().z());

  prediction::PredictionObstacle cipv;
  double distance = std::numeric_limits<double>().infinity();
  double ego_speed = CalculateScalarVelocity(pose.linear_velocity().x(),
                                             pose.linear_velocity().y());
  double cipv_speed = 0.0;
  if (!SelectCIPVFromObstacles(pose, prediction, config_.search_lane_distance(),
                               config_.search_lane_depth(), &cipv, &distance)) {
    auto cipv_obs = cipv.perception_obstacle();
    cipv_speed = CalculateScalarVelocity(cipv_obs.velocity().x(),
                                         cipv_obs.velocity().y());
    printf("find cipv, id: %d distance: %.8f ,speed: %.8f ,ego_speed: %.8f\n",
           cipv_obs.id(), distance, cipv_speed, ego_speed);

  } else {
    printf("no cipv\n");
  }

  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;

  auto speed_profile = PredictNonUniformAcceleration(
      config_, ego_speed, 0, 0, unit_t_, dimension_t_, cipv_speed, distance);
  AINFO<<"idm size: "<< speed_profile.size()<<", speed data size: "<< speed_data->size();
  printf("idm size: %d, speed data size: %d\n", speed_profile.size(),
         speed_data->size());
  for (int i = 0, n = std::min(speed_profile.size(), speed_data->size()); i < n;
       ++i) {
    std::cout << std::fixed << std::setprecision(2) << speed_profile[i].t()
              << "\t" << speed_profile[i].v() << "\t\t" << speed_profile[i].s()
              << "\t\t" << speed_data->at(i).v() << "\t\t"
              << speed_data->at(i).s() << "\n";
  }

  *speed_data = SpeedData(speed_profile);

  return Status::OK();
}

bool FollowModelDecider::InitPointIsCollision() {
  static constexpr double kBounadryEpsilon = 1e-2;
  for (const auto& boundary :
       reference_line_info_->st_graph_data().st_boundaries()) {
    // KeepClear obstacles not considered in Dp St decision
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // If init point in collision with obstacle, return speed fallback
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      return true;
    }
  }

  return false;
}

}  // namespace planning
}  // namespace apollo