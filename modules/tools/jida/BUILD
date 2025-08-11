load("//tools:apollo_package.bzl", "apollo_cc_binary","apollo_package")
load("//tools/proto:proto.bzl", "apollo_py_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "jida_map_tool",
    srcs = ["src/map_tool.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_proto",
        "//modules/common_msgs/external_command_msgs:external_geometry_proto",
        "//modules/map:apollo_map",
        "@com_google_protobuf//:protobuf",

    ],
)

apollo_py_binary(
    name = "record_data",
    srcs = ["script/record_data.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:command_status_py_pb2",


    ],
)

apollo_package()