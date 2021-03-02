workspace(name = "structure_core_reader")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


http_archive(
  name = "pybind11_bazel",
  strip_prefix = "pybind11_bazel-16ed1b8f308d2b3dec9d7e6decaad49ce4d28b43",
  urls = ["https://github.com/pybind/pybind11_bazel/archive/16ed1b8f308d2b3dec9d7e6decaad49ce4d28b43.zip"],
)

# We still require the pybind library.
http_archive(
  name = "pybind11",
  build_file = "@pybind11_bazel//:pybind11.BUILD",
  strip_prefix = "pybind11-2.5.0",
  urls = ["https://github.com/pybind/pybind11/archive/v2.5.0.tar.gz"],
)
load("@pybind11_bazel//:python_configure.bzl", "python_configure")
python_configure(name = "local_config_python")
