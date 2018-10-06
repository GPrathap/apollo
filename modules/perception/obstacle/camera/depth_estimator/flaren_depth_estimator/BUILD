load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common",
        "@eigen",
        "@opencv2//:core"
    ],
)

cc_library(
    name = "flaren_depth_estimator",
    srcs = ["flaren_depth_estimator.cc"],
    hdrs = ["flaren_depth_estimator.h"],
    deps = [
        ":util",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/cuda_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/detector/common:feature_extractor",
        "//modules/perception/obstacle/camera/detector/common/proto:tracking_feature_proto",
        "//modules/perception/obstacle/camera/dummy",
        "//modules/perception/obstacle/camera/interface"
        "@eigen",
    ],
)

cpplint()
