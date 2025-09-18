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

#include <dlfcn.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/localization_msgs/pose.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/tasks/follow_model/base/common_utils.h"
#include "modules/planning/tasks/follow_model/base/follow_model_base.h"
#include "modules/planning/tasks/follow_model/base/model_registrar.h"
#include "modules/planning/tasks/follow_model/model/follow_model_idm.h"

using namespace apollo;
using apollo::common::util::PointFactory;

namespace apollo {
namespace planning {

FollowModelDecider::FollowModelDecider()
    : handle_(nullptr), car_follow_model_(nullptr) {}

FollowModelDecider::~FollowModelDecider() {
  car_follow_model_.reset();
  if (handle_) {
    dlclose(handle_);
    handle_ = nullptr;
  }
}

bool FollowModelDecider::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;
  }

  // Load the config.
  if (!SpeedOptimizer::LoadConfig<FollowModelConfig>(&config_)) {
    return false;
  }

  unit_t_ = config_.unit_t();
  if (config_.enable_follow_mode()) {
    FLAGS_default_cruise_speed = config_.model_param().expect_speed();
    ReginsterCarFollowModel();
    bool is_succeed = LoadCarFollowModel();
    config_.set_enable_follow_mode(is_succeed);
    if (!is_succeed) {
      return false;
    }
  }

  return true;
}

Status FollowModelDecider::Execute(Frame* frame,
                                   ReferenceLineInfo* reference_line_info) {
  // To be implemented.
  Task::Execute(frame, reference_line_info);
  printf("is near destination: %d\n", frame->is_near_destination());
  if (!config_.enable_follow_mode() || frame->is_near_destination()) {
    return Status::OK();
  }

  double total_time_t =
      reference_line_info->st_graph_data().total_time_by_conf();
  dimension_t_ = static_cast<uint32_t>(
                     std::ceil(total_time_t / static_cast<double>(unit_t_))) +
                 1;

  return Process(reference_line_info->path_data(), frame->PlanningStartPoint(),
                 reference_line_info->mutable_speed_data());
}

Status FollowModelDecider::Process(const PathData& path_data,
                                   const common::TrajectoryPoint& init_point,
                                   SpeedData* const speed_data) {
  if (InitPointIsCollision()) {
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

  NeighborVehicleInfo neighbor_vehicle_info;
  neighbor_vehicle_info.ego.speed = ego_speed;
  neighbor_vehicle_info.front_vehicle.speed = cipv_speed;
  neighbor_vehicle_info.front_vehicle.distance = distance;

  auto speed_profile =
      PredictNonUniformAcceleration(ego_speed, 0, 0, neighbor_vehicle_info);
  AINFO << "idm size: " << speed_profile.size()
        << ", speed data size: " << speed_data->size();
  printf("idm size: %d, speed data size: %d\n", speed_profile.size(),
         speed_data->size());

  PrintSpeedData(speed_data, speed_profile);

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

bool FollowModelDecider::LoadCustomCarFollowModel() {
  bool is_succeed = false;
  char message[512] = {0};
  memset(message, 0x00, sizeof(message));
  do {
    void* handle = dlopen(config_.model().model_path().c_str(), RTLD_LAZY);
    if (!handle) {
      std::snprintf(message, sizeof(message),
                    "Failed to load model [%s], dlopen error: %s.",
                    config_.model().model_path().c_str(), dlerror());
      break;
    }

    dlerror(); /* Clear any existing error */

    create_model_ =
        reinterpret_cast<CreateModelFunc>(dlsym(handle, "CreateFollowModel"));
    destroy_model_ =
        reinterpret_cast<DestroyModelFunc>(dlsym(handle, "DestroyFollowModel"));

    const char* err;
    if ((err = dlerror()) != nullptr) {
      std::snprintf(message, sizeof(message), "dlsym error: %s.", err);
      dlclose(handle);
      break;
    }

    // 创建对象
    FollowModelBase* model = create_model_();
    if (!model) {
      std::snprintf(message, sizeof(message),
                    "Failed to create car follow model.");
      break;
    }
    car_follow_model_.reset(model, destroy_model_);
    is_succeed = true;
  } while (0);

  if (!is_succeed) {
    AERROR << message;
  }

  return is_succeed;
}

bool FollowModelDecider::LoadCarFollowModel() {
  car_follow_model_.reset();
  bool is_succeed = true;

  if (config_.model().type() != CarFollowModel::CUSTOM) {
    std::string model_name = CarFollowModel_Type_Name(config_.model().type());
    car_follow_model_ =
        std::move(ModelRegistrar::Instance()->CreateProduct(model_name));
    if (!car_follow_model_) {
      AERROR << "Loading car following model [" << model_name
             << "] failed, close car following model module.";
      is_succeed = false;
    }
  } else {
    is_succeed = LoadCustomCarFollowModel();
  }

  if (is_succeed) {
    car_follow_model_->Init(config_);
  }
  AINFO << "Successfully created car following model: "
        << car_follow_model_->Name();

  return is_succeed;
}

std::vector<common::SpeedPoint>
FollowModelDecider::PredictNonUniformAcceleration(
    double v0, double s0, double a0,
    NeighborVehicleInfo& neighbor_vehicle_info) {
  std::vector<common::SpeedPoint> speed_profile;

  common::SpeedPoint init_point;
  init_point.set_t(0.0);
  init_point.set_s(0.0);
  init_point.set_v(v0);
  speed_profile.emplace_back(init_point);

  double v = v0;
  double s = s0;
  double a = a0;
  double dt = unit_t_;
  double origin_distance = neighbor_vehicle_info.front_vehicle.distance;

  CarFollowSpeedPoint speed_point;

  for (size_t i = 1; i < dimension_t_ - 1; ++i) {
    // update data
    double t = i * dt;
    neighbor_vehicle_info.ego.speed = v;
    neighbor_vehicle_info.front_vehicle.distance = origin_distance - s;
    // update speed & position
    car_follow_model_->Calculate(neighbor_vehicle_info, speed_point);
    // a = CalculateIDMModel(config_, v, cipv_speed, car_distance - s);
    a = speed_point.a;
    printf("idm acc:%.8f\n", a);
    // v += a * dt;
    // // s += std::min(0.0, v * dt + 0.5 * a * dt * dt);
    // s += v * dt + 0.5 * a * dt * dt;
    double t_s = std::floor((v * dt + 0.5 * a * dt * dt) * 10 + 0.5) /
                 10;  // Keep one decimal place
    s += std::max(t_s, 0.0);
    v += std::max(a * dt, 0.0);

    // update the distance between the front and ego vehicle
    origin_distance += neighbor_vehicle_info.front_vehicle.speed * dt;

    common::SpeedPoint sp;
    sp.set_t(t);
    sp.set_s(s);
    sp.set_v(v);
    speed_profile.emplace_back(sp);
  }

  // 末尾再推一个点
  common::SpeedPoint last_speed_point;
  last_speed_point.set_t((dimension_t_ - 1) * dt);
  last_speed_point.set_s(s + speed_profile.back().v() * dt);
  last_speed_point.set_v(0.0);
  speed_profile.emplace_back(last_speed_point);
  return speed_profile;
}

void FollowModelDecider::PrintSpeedData(
    const SpeedData* const speed_data,
    const std::vector<common::SpeedPoint>& speed_profile) {
  int n = std::min(speed_profile.size(), speed_data->size());
  std::vector<std::string> table_header{"t",     "v",     "s",
                                        "idm_t", "idm_v", "idm_s"};
  std::vector<std::vector<double>> table(n);

  for (int i = 0; i < n; ++i) {
    table[i] = {speed_data->at(i).t(), speed_data->at(i).v(),
                speed_data->at(i).s(), speed_profile[i].t(),
                speed_profile[i].v(),  speed_profile[i].s()};
  }

  int column_width = 12;
  std::string split_line = std::string(column_width * table_header.size(), '-');

  std::cout << "Model Name: " << car_follow_model_->Name() << std::endl;
  for (int j = 0, m = table_header.size(); j < m; ++j) {
    std::cout << std::left << std::setw(column_width) << table_header[j];
  }
  std::cout << std::endl;

  std::cout << split_line << std::endl;

  for (int i = 0; i < n; ++i) {
    for (int j = 0, m = table[i].size(); j < m; ++j) {
      std::cout << std::left << std::setw(column_width) << table[i][j];
    }
    std::cout << std::endl;
  }

  std::cout << split_line << std::endl << std::endl;
}

void FollowModelDecider::ReginsterCarFollowModel() {
  ModelRegistrar::Instance()->RegisterCarFollowModel<FollowModelIDM>(
      CarFollowModel_Type_Name(CarFollowModel::IDM));
}

}  // namespace planning
}  // namespace apollo