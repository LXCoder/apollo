#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common_msgs/external_command_msgs/geometry.pb.h"
#include "modules/common_msgs/map_msgs/map.pb.h"

#include "cyber/cyber.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map/hdmap/hdmap.h"

bool Bin2Txt(const std::string& bin_path, const std::string& out_path) {
  apollo::hdmap::Map map;
  printf("1. read map binary\n");
  if (!apollo::cyber::common::GetProtoFromBinaryFile(bin_path, &map)) {
    printf("failed to Bin2Txt\n");
    return false;
  }
  printf("2. conver to txt\n");
  int fd = open(out_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0777);
  google::protobuf::io::FileOutputStream* output =
      new google::protobuf::io::FileOutputStream(fd);
  google::protobuf::TextFormat::Print(map, output);
  delete output;
  close(fd);
  printf("3. finish conver\n");
  return true;
}

bool OpenDriver2Txt(const std::string& open_driver_path,
                    const std::string& out_path) {
  apollo::hdmap::Map map;
  printf("1. read open driver map\n");
  if (!apollo::hdmap::adapter::OpendriveAdapter::LoadData(open_driver_path,
                                                          &map)) {
    printf("failed to OpenDriver2Txt\n");
    return false;
  }

  printf("2. conver to txt\n");
  int fd = open(out_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0777);
  google::protobuf::io::FileOutputStream* output =
      new google::protobuf::io::FileOutputStream(fd);
  google::protobuf::TextFormat::Print(map, output);
  delete output;
  close(fd);
  printf("3. finish conver\n");
  return true;
}

bool GetNearestLaneWithHeading(const std::string& map_path, double x, double y,
                               double heading) {
  apollo::hdmap::HDMap* hdmap = new apollo::hdmap::HDMap();
  if (hdmap->LoadMapFromFile(map_path)) {
    AERROR << "failed to load map";
    return false;
  }
  apollo::hdmap::LaneInfoConstPtr nearest_lane;
  static constexpr double kSearchRadius = 3.0;
  static constexpr double kMaxHeadingDiff = 1.0;
  double nearest_s;
  double nearest_l;
  apollo::external_command::Pose pose;
  pose.set_x(x);
  pose.set_y(y);
  pose.set_heading(heading);
  apollo::common::PointENU point;
  point.set_x(pose.x());
  point.set_y(pose.y());
  if (hdmap->GetNearestLaneWithHeading(point, kSearchRadius, pose.heading(),
                                       kMaxHeadingDiff, &nearest_lane,
                                       &nearest_s, &nearest_l) < 0) {
    printf("Failed to get nearest lane with heading of pose:\n %s\n",
           pose.DebugString().c_str());
    return false;
  }

  printf(
      "lane id: %s\nroad id: %s\nsection id: %s\nnearest_s: %lf\nnearest_l: "
      "%lf",
      nearest_lane->id().id().c_str(), nearest_lane->road_id().id().c_str(),
      nearest_lane->section_id().id().c_str(), nearest_s, nearest_l);
  printf("lane info:\n%s\n", nearest_lane->lane().Utf8DebugString().c_str());

  printf("land headings:\n");
  auto headings = nearest_lane->headings();
  for (int i = 0, n = headings.size(); i < n; ++i) {
    if (i != 0 && i % 6 == 0) {
      printf("\n");
    }
    printf("%lf\t", headings[i]);
  }

  return true;
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    printf("please input map path and out path\n");
    return 0;
  }
  std::string map_path(argv[1]);
  std::string out_path(argv[2]);

  std::vector<std::string> tool_modes{
      "quit", "bin to txt", "open driver to txt", "GetNearestLaneWithHeading"};
  bool is_running = true;

  while (is_running) {
    for (int i = 0, n = tool_modes.size(); i < n; ++i) {
      if (i != 0 && i % 3 == 0) {
        printf("\n");
      }
      printf("%d. %s\t", i, tool_modes[i].c_str());
    }

    printf("\nplease select map tool mode number: ");
    int mode = 0;
    std::cin >> mode;

    if (mode == 0) {
      is_running = false;
      continue;
    } else if (mode < 0 || mode > static_cast<int>(tool_modes.size())) {
      continue;
    }

    double pos_x, pos_y, heading = 0.0;

    switch (mode) {
      case 1:
        Bin2Txt(map_path, out_path);
        break;
      case 2:
        OpenDriver2Txt(map_path, out_path);
        break;
      case 3:
        std::cin >> pos_x >> pos_y >> heading;
        GetNearestLaneWithHeading(map_path, pos_x, pos_y, heading);
        break;
      default:
        is_running = false;
    }
  };

  return 0;
}