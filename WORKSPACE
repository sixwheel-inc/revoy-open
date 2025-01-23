load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "ompl",
    build_file = "@//third-party/ompl:ompl.BUILD.bazel",
    sha256 = "f03daa95d2bbf1c21e91a38786242c245f4740f16aa9e9adbf7c7e0236e3c625",
    strip_prefix = "ompl-1.6.0",
    urls = ["https://github.com/ompl/ompl/archive/1.6.0.tar.gz"],
)

http_archive(
    name = "clipper2",
    # sha256 = "f03daa95d2bbf1c21e91a38786242c245f4740f16aa9e9adbf7c7e0236e3c625",
    build_file = "@//third-party/clipper2:clipper2.BUILD.bazel",
    strip_prefix = "Clipper2-Clipper2_1.4.0",
    urls = [
        "https://github.com/AngusJohnson/Clipper2/archive/refs/tags/Clipper2_1.4.0.zip",
    ],
)

