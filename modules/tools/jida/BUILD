load("//tools:apollo_package.bzl", "apollo_cc_binary","apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "jida_map_tool",
    srcs = ["map_tool.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_proto",
        "//modules/common_msgs/external_command_msgs:external_geometry_proto",
        "//modules/map:apollo_map",
        "@com_google_protobuf//:protobuf",

    ],
)

apollo_package()