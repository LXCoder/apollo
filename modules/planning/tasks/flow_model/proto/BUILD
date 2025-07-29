load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "flow_model_decider_proto",
    srcs = ["flow_model_decider.proto"],
)

apollo_package()

cpplint()