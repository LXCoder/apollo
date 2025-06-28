load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "idm_model_task_proto",
    srcs = ["idm_model_task.proto"],
)

apollo_package()

cpplint()