#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

#include <glog/logging.h>

#include "gflags/gflags.h"

#include "modules/common_msgs/map_msgs/map.pb.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"

DEFINE_string(output_dir, "/tmp/", "output map directory");
DEFINE_double(sample, 1.0, "sampling length");

using apollo::hdmap::Lane;
using apollo::hdmap::Map;

static void OutputMap(const Map& map_pb) {
  std::ofstream map_txt_file(FLAGS_output_dir + "/base_map.txt");
  map_txt_file << map_pb.DebugString();
  map_txt_file.close();

  std::ofstream map_bin_file(FLAGS_output_dir + "/base_map.bin");
  std::string map_str;
  map_pb.SerializeToString(&map_str);
  map_bin_file << map_str;
  map_bin_file.close();
}

void GenerateLeftRoadSample(const std::unordered_map<std::string, Lane*> lanes_,
                            Lane* lane_ptr) {
  // left sample
  for (int j = 0, m = lane_ptr->left_sample_size(); j < m; ++j) {
    auto sample = lane_ptr->left_sample(j);
    auto road_sample = lane_ptr->mutable_left_road_sample()->Add();
    road_sample->set_s(sample.s());
    road_sample->set_width(sample.width());
  }

  auto p = lane_ptr;
  while (p->left_neighbor_forward_lane_id_size() > 0) {
    std::string id = p->left_neighbor_forward_lane_id(0).id();
    auto it = lanes_.find(id);
    if (it == lanes_.end()) {
      AWARN << "not find " << id;
      break;
    }
    p = it->second;
    int l = 0, r = 0;
    for (;
         l < lane_ptr->left_road_sample_size() && r < p->left_sample_size();) {
      auto lh = lane_ptr->mutable_left_road_sample(l);
      auto rh = p->mutable_left_sample(r);

      if (std::abs(lh->s() - rh->s()) > FLAGS_sample) {
        if (lh->s() > rh->s()) {
          r++;
        } else if (lh->s() < rh->s()) {
          l++;
        }
      }
      lh->set_width(lh->width() + rh->width());
      l++;
      r++;
    }
  }
}

void GenerateRightRoadSample(
    const std::unordered_map<std::string, Lane*> lanes_, Lane* lane_ptr) {
  // right sample
  for (int j = 0, m = lane_ptr->right_sample_size(); j < m; ++j) {
    auto sample = lane_ptr->right_sample(j);
    auto road_sample = lane_ptr->mutable_right_road_sample()->Add();
    road_sample->set_s(sample.s());
    road_sample->set_width(sample.width());
  }

  auto p = lane_ptr;
  while (p->right_neighbor_forward_lane_id_size() > 0) {
    std::string id = p->right_neighbor_forward_lane_id(0).id();
    auto it = lanes_.find(id);
    if (it == lanes_.end()) {
      break;
    }
    p = it->second;
    int l = 0, r = 0;
    for (; l < lane_ptr->right_road_sample_size() &&
           r < p->right_sample_size();) {
      auto lh = lane_ptr->mutable_right_road_sample(l);
      auto rh = p->mutable_right_sample(r);

      if (std::abs(lh->s() - rh->s()) > FLAGS_sample) {
        if (lh->s() > rh->s()) {
          r++;
        } else if (lh->s() < rh->s()) {
          l++;
        }
      }
      lh->set_width(lh->width() + rh->width());
      l++;
      r++;
    }
  }
}

int main(int32_t argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  Map map_pb;
  const auto map_file = apollo::hdmap::BaseMapFile();

  ACHECK(apollo::cyber::common::GetProtoFromFile(map_file, &map_pb))
      << "Fail to open: " << map_file;

  std::unordered_map<std::string, Lane*> lanes_;
  for (int i = 0, n = map_pb.lane_size(); i < n; ++i) {
    auto lane_ptr = map_pb.mutable_lane(i);
    lanes_.insert(std::make_pair(lane_ptr->id().id(), lane_ptr));
  }

  for (int i = 0, n = map_pb.lane_size(); i < n; ++i) {
    auto lane_ptr = map_pb.mutable_lane(i);

    GenerateLeftRoadSample(lanes_, lane_ptr);
    GenerateRightRoadSample(lanes_, lane_ptr);
  }

  OutputMap(map_pb);
  AINFO << "sim_map generated at:" << FLAGS_output_dir;

  return 0;
}
