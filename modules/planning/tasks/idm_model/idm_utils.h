/**
 * @file idm_utils.h
 **/

#pragma once

#include "bazel-out/k8-dbg/bin/modules/common_msgs/basic_msgs/geometry.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/localization_msgs/pose.pb.h"
#include "bazel-out/k8-dbg/bin/modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "bazel-out/k8-dbg/bin/modules/planning/tasks/idm_model/proto/idm_model_decider.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"

using apollo::common::Point2D;
using namespace apollo::common::math;
using namespace apollo;

namespace apollo::planning {

int GetDisBetweenPointToSegment(const Point2D& point, const Point2D& line_p1,
                                const Point2D& line_p2, double* dis);

int GetTwoBoxNearstDis(const Box2d& box1, const Box2d& box2,
                       double* nearest_dis);

int SelectCIPVFromObstacles(
    const localization::Pose& ego_pose,
    const std::shared_ptr<prediction::PredictionObstacles>& prediction,
    double search_lane_distance, double search_lane_depth,
    prediction::PredictionObstacle* cipv_vehicle, double* distance);

bool IsSuccessorLane(const hdmap::HDMap* hdmap_ptr,
                     const hdmap::LaneInfoConstPtr& current_lane,
                     const hdmap::LaneInfoConstPtr& other_lane, int depth);

double CalculateIDMModel(const IDMModelDeciderConfig& config, double ego_speed,
                         double front_vehicle_speed,
                         double front_vehicle_distance);

std::vector<common::SpeedPoint> PredictNonUniformAcceleration(
    const IDMModelDeciderConfig& config, double v0, double s0, double a0,
    double dt, int dt_steps, double cipv_speed, double car_distance);

}  // namespace apollo::planning