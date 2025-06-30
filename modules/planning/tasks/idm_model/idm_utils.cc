

#include "modules/planning/tasks/idm_model/idm_utils.h"

#include <climits>
#include <cmath>
#include <limits>
#include <memory>

// #include "bazel-out/k8-dbg/bin/modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
// #include "bazel-out/k8-dbg/bin/modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
// #include "bazel-out/k8-dbg/bin/modules/planning/tasks/idm_model/proto/idm_model_decider.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"

namespace apollo::planning {
int GetDisBetweenPointToSegment(const Point2D& point, const Point2D& line_p1,
                                const Point2D& line_p2, double* dis) {
  if (dis) {
    double p1_p2_x = line_p2.x() - line_p1.x();
    double p1_p2_y = line_p2.y() - line_p1.y();
    double p1_point_x = point.x() - line_p1.x();
    double p1_point_y = point.y() - line_p1.y();
    double p2_point_x = point.x() - line_p2.x();
    double p2_point_y = point.y() - line_p2.y();
    double p1_p2_len = std::sqrt(p1_p2_x * p1_p2_x + p1_p2_y * p1_p2_y);
    double p1_point_len =
        std::sqrt(p1_point_x * p1_point_x + p1_point_y * p1_point_y);
    double p2_point_len =
        std::sqrt(p2_point_x * p2_point_x + p2_point_y * p2_point_y);
    double r =
        (p1_p2_x * p1_point_x + p1_p2_y * p1_point_y) / (p1_p2_len * p1_p2_len);
    double p1_point_shadow_p1_p2 =
        (p1_p2_x * p1_point_x + p1_p2_y * p1_point_y) / p1_p2_len;
    double point_line_len =
        std::sqrt(p1_point_len * p1_point_len -
                  p1_point_shadow_p1_p2 * p1_point_shadow_p1_p2);

    if (r > 0 && r < 1) {
      *dis = point_line_len;
    } else if (r <= 0) {
      *dis = p1_point_len;
    } else {
      *dis = p2_point_len;
    }
    return 0;
  }
  return 1;
}

int GetTwoBoxNearstDis(const Box2d& box1, const Box2d& box2,
                       double* nearest_dis) {
  *nearest_dis = box1.DistanceTo(box2);

  return 0;
}

int SelectCIPVFromObstacles(
    const localization::Pose& ego_pose,
    const std::shared_ptr<prediction::PredictionObstacles>& prediction,
    double search_lane_distance, double search_lane_depth,
    prediction::PredictionObstacle* cipv_vehicle, double* distance) {
  if (prediction->prediction_obstacle_size() == 0) {
    AWARN << "prediction obstacles is empty";
    return -1;
  }

  double ego_length =
      common::VehicleConfigHelper::GetConfig().vehicle_param().length();
  double ego_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width();

  apollo::common::math::Box2d ego_bbox(
      {ego_pose.position().x(), ego_pose.position().y()}, ego_pose.heading(),
      ego_length, ego_width);

  auto hdmap_ptr = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::LaneInfoConstPtr ego_lane_ptr;
  double ego_nearest_s = 0.0, ego_nearest_l = 0.0;

  if (hdmap_ptr->GetNearestLaneWithDistance(ego_pose.position(),
                                            search_lane_distance, &ego_lane_ptr,
                                            &ego_nearest_s, &ego_nearest_l) ||
      !ego_lane_ptr->IsOnLane(ego_bbox)) {
    AWARN << "Failed to find the lane where the ego is located";
    return -1;
  }

  std::string ego_lane_id = ego_lane_ptr->id().id();

  double min_distance = std::numeric_limits<double>::max();
  perception::PerceptionObstacle candidate_vehile;
  int candidate_idx = -1;

  for (int i = 0, n = prediction->prediction_obstacle_size(); i < n; ++i) {
    perception::PerceptionObstacle obs =
        prediction->prediction_obstacle(i).perception_obstacle();

    if (obs.type() != perception::PerceptionObstacle::VEHICLE) {
      continue;
    }

    hdmap::LaneInfoConstPtr lane_ptr;
    double nearest_s = 0.0, nearest_l = 0.0;

    common::PointENU obs_pos;
    obs_pos.set_x(obs.position().x());
    obs_pos.set_y(obs.position().y());
    obs_pos.set_z(obs.position().z());

    apollo::common::math::Box2d obs_bbox(
        {obs_pos.x(), obs_pos.y()}, obs.theta(), obs.length(), obs.width());

    if (hdmap_ptr->GetNearestLaneWithDistance(
            obs_pos, search_lane_distance, &lane_ptr, &nearest_s, &nearest_l) ||
        !lane_ptr->IsOnLane(obs_bbox)) {
      continue;
    }

    if (ego_lane_id == lane_ptr->id().id()) {
      if (ego_nearest_s > nearest_s) {
        continue;
      }
    } else {
      if (!IsSuccessorLane(hdmap_ptr, ego_lane_ptr, lane_ptr,
                           search_lane_depth)) {
        continue;
      }
    }

    double dis = ego_bbox.DistanceTo(obs_bbox);

    if (min_distance - dis > 1e-6) {
      min_distance = dis;
      candidate_idx = i;
      candidate_vehile.CopyFrom(obs);
    }
  }

  if (candidate_idx != -1) {
    *distance = min_distance;
    cipv_vehicle->CopyFrom(prediction->prediction_obstacle(candidate_idx));
  }

  return candidate_idx != -1 ? 0 : 1;
}

bool IsSuccessorLane(const hdmap::HDMap* hdmap_ptr,
                     const hdmap::LaneInfoConstPtr& current_lane,
                     const hdmap::LaneInfoConstPtr& other_lane, int depth) {
  if (depth <= 0) {
    return false;
  }

  auto lane = current_lane->lane();
  std::string lane_id_str;
  lane_id_str.append(lane.id().id());
  lane_id_str.append(" successor_ids:");
  for (auto& lane_id : lane.successor_id()) {
    lane_id_str.append(lane_id.id());
    lane_id_str.append(" ");
  }
  // printf("%s\n", lane_id_str.c_str());
  for (int i = 0, n = lane.successor_id_size(); i < n; ++i) {
    auto lane_id = lane.successor_id(i);
    if (other_lane->id().id() == lane_id.id()) {
      // printf("current successor id is:%s\n", other_lane->id().id().c_str());
      return true;
    } else {
      auto next_lane = hdmap_ptr->GetLaneById(lane_id);
      if (next_lane &&
          IsSuccessorLane(hdmap_ptr, next_lane, other_lane, depth - 1)) {
        return true;
      }
    }
  }
  return false;
}

double CalculateIDMModel(const IDMModelDeciderConfig& config, double ego_speed,
                         double front_vehicle_speed,
                         double front_vehicle_distance) {
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

std::vector<common::SpeedPoint> PredictNonUniformAcceleration(
    const IDMModelDeciderConfig& config, double v0, double s0, double a0,
    double dt, int dt_steps, double cipv_speed, double car_distance) {
  // std::vector<State> trajectory;
  std::vector<common::SpeedPoint> speed_profile;

  common::SpeedPoint init_point;
  init_point.set_t(0.0);
  init_point.set_s(0.0);
  init_point.set_v(v0);
  speed_profile.push_back(init_point);
  // trajectory.push_back({0.0, 0.0, 0.0});

  double v = v0;
  double s = s0;
  double a = a0;

  for (size_t i = 1; i < dt_steps - 1; ++i) {
    // double a = accelerations[i];
    double t = i * dt;
    // 更新速度 & 位置
    a = CalculateIDMModel(config, v, cipv_speed, car_distance - s);
    printf("idm acc:%.8f\n", a);
    // v += a * dt;
    // // s += std::min(0.0, v * dt + 0.5 * a * dt * dt);
    // s += v * dt + 0.5 * a * dt * dt;
    s += v * dt + 0.5 * a * dt * dt;
    v += a * dt;

    car_distance += cipv_speed * dt;

    common::SpeedPoint speed_point;
    speed_point.set_t(t);
    speed_point.set_s(s);
    speed_point.set_v(v);
    speed_profile.push_back(speed_point);
    // trajectory.push_back({t, v, s});
  }

  // 末尾再推一个点
  common::SpeedPoint speed_point;
  speed_point.set_t(dt_steps * dt);
  speed_point.set_s(s + speed_profile.back().v() * dt);
  speed_point.set_v(0.0);
  speed_profile.push_back(speed_point);
  return speed_profile;
}

}  // namespace apollo::planning