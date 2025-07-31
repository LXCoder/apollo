load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libfollow_model_decider.so",
    srcs = ["follow_model_decider.cc", "idm_utils.cc","follow_model_base.cc","follow_model_idm.cc"],
    hdrs = ["follow_model_decider.h", "idm_utils.h","follow_model_base.h","follow_model_idm.h","follow_model_factory.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugin_follow_model_description.xml",
    deps = [
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/follow_model/proto:follow_model_config_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_proto",
    ],
)

apollo_package()

cpplint()