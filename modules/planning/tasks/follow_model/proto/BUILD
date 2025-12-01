load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "follow_model_proto",
    srcs = [
        "follow_model_config.proto",
        "follow_model_info.proto"
    ],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_proto",
    ]
)

apollo_package()

cpplint()