load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "follow_model_config_proto",
    srcs = ["follow_model_config.proto"],
)

apollo_package()

cpplint()