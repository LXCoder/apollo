load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


apollo_cc_library(
    name = "follow_model_base",
    srcs = [
        "model_registrar.cc",
        "follow_model_base.cc",
        "common_utils.cc",
    ],
    hdrs = [
        "model_registrar.h",
        "follow_model_base.h",
        "common_utils.h",
        "factory.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/tasks/follow_model/proto:follow_model_config_proto",
    ],
)


apollo_package()

cpplint()