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
    name = "libidm_model_decider.so",
    srcs = ["idm_model_decider.cc", "idm_utils.cc"],
    hdrs = ["idm_model_decider.h", "idm_utils.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugin_idm_model_description.xml",
    deps = [
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/idm_model/proto:idm_model_decider_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_proto",
    ],
)

apollo_package()

cpplint()