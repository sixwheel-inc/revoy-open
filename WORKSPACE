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
    build_file = "@//third-party/clipper2:clipper2.BUILD.bazel",
    integrity = "sha256-JMjP09PY82l8PEHKBD0YQ4VUvM3moNLyWQam9/qapf8=",
    strip_prefix = "Clipper2-Clipper2_1.4.0",
    urls = [
        "https://github.com/AngusJohnson/Clipper2/archive/refs/tags/Clipper2_1.4.0.zip",
    ],
)

# Unfortunately we have to use cc_foreign to build projectchrono
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

new_git_repository(
    name = "projectchrono",
    build_file = "@//third-party/projectchrono:projectchrono.BUILD.bazel",
    commit = "4aa1b10dff8a6b529ec95f39ab8461032e19ec86",
    remote = "https://github.com/ian-sixwheel/chrono",
)

# sometimes useful for local debugging
# new_local_repository(
#     name = "projectchrono",
#     build_file = "@//third-party/projectchrono:projectchrono.BUILD.bazel",
#     path = "~/chrono",
# )

# Unfortunately the BCR upstream foxglove-shemas seems to be broken, so we
# fallback to com_google_protobuf and com-foxglove-schema in WORKSPACE
http_archive(
    name = "com_google_protobuf",
    sha256 = "540200ef1bb101cf3f86f257f7947035313e4e485eea1f7eed9bc99dd0e2cb68",
    strip_prefix = "protobuf-3.25.0",
    # latest, as of 2023-11-02
    urls = [
        "https://github.com/protocolbuffers/protobuf/archive/v3.25.0.tar.gz",
    ],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

new_git_repository(
    name = "com-foxglove-schemas",
    build_file = "@//third-party/foxglove-schemas:com-foxglove-schemas.BUILD.bazel",
    commit = "4fc6a4f7cdbd0280b5efca77a5b8808d0822fe7b",
    remote = "https://github.com/foxglove/schemas",
    strip_prefix = "schemas/proto/",
)
