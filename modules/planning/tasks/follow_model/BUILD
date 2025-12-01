load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_cc_library(
    name = "follow_model_example",
    srcs = [
        "model/follow_model_custom_example.cc",
    ],
    hdrs = [
        "model/follow_model_custom_example.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/tasks/follow_model/base:follow_model_base",
    ],
)

apollo_plugin(
    name = "libfollow_model_decider.so",
    srcs = [
        "follow_model_decider.cc",
        "model/follow_model_idm.cc"
    ],
    hdrs = [
        "follow_model_decider.h",
        "model/follow_model_idm.h"
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugin_follow_model_description.xml",
    deps = [
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/follow_model/base:follow_model_base",
        "//modules/planning/tasks/follow_model/proto:follow_model_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_proto",
    ],
)

apollo_package()

cpplint()