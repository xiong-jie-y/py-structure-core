load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

cc_import(
    name="structure_core_lib",
    hdrs=glob(["ST/*.h"]),
    shared_library="structure_core/Linux/x86_64/libStructure.so",
    visibility = ["//visibility:public"],
)

pybind_extension(
    name="structure_core",
    srcs=["structure_core.cc"],
    deps=[
        ":structure_core_lib"
    ]
)