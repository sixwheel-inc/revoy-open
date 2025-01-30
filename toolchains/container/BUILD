load("@bazel_tools//tools/cpp:unix_cc_toolchain_config.bzl", "cc_toolchain_config")
load("@rules_python//python:defs.bzl", "py_runtime_pair")

cc_toolchain_config(
    name = "clang-cc-toolchain-config",
    abi_libc_version = "local",
    abi_version = "local",
    compile_flags = [
        "-U_FORTIFY_SOURCE",
        "-fstack-protector",
        "-fno-omit-frame-pointer",
        "-Wall",
        "-Wextra",
        "-pedantic",
        # TODO: enable
        # "-Werror",
    ],
    compiler = "compiler",
    coverage_compile_flags = ["--coverage"],
    coverage_link_flags = ["--coverage"],
    cpu = "k8",
    cxx_builtin_include_directories = [
        "/usr/lib/clang/18/include/",
        "/usr/local/include",
        "/usr/include/x86_64-linux-gnu",
        "/usr/include",
        "/usr/include/c++/9",
        "/usr/include/x86_64-linux-gnu/c++/9",
        "/usr/include/c++/9/backward",
    ],
    cxx_flags = ["-std=c++20"],
    dbg_compile_flags = ["-g"],
    host_system_name = "local",
    link_flags = [
        "-fuse-ld=gold",
        "-Wl,-no-as-needed",
        "-Wl,-z,relro,-z,now",
        "-B/usr/bin",
        # "-pass-exit-codes",
    ],
    link_libs = [
        "-lstdc++",
        "-lm",
        "-ldl",
        "-lpthread",
    ],
    opt_compile_flags = [
        "-g0",
        "-O2",
        "-D_FORTIFY_SOURCE=1",
        "-DNDEBUG",
        "-ffunction-sections",
        "-fdata-sections",
    ],
    opt_link_flags = ["-Wl,--gc-sections"],
    supports_start_end_lib = True,
    target_libc = "local",
    target_system_name = "local",
    tool_paths = {
        "ar": "/usr/bin/ar",
        "ld": "/usr/bin/ld",
        "llvm-cov": "/usr/bin/llvm-cov",
        "cpp": "/usr/bin/cpp",
        "gcc": "/usr/bin/clang",
        "dwp": "/usr/bin/dwp",
        "gcov": "/usr/bin/gcov",
        "nm": "/usr/bin/nm",
        "objcopy": "/usr/bin/objcopy",
        "objdump": "/usr/bin/objdump",
        "strip": "/usr/bin/strip",
    },
    toolchain_identifier = "local",
    unfiltered_compile_flags = [
        "-Wno-builtin-macro-redefined",
        "-D__DATE__=\"redacted\"",
        "-D__TIMESTAMP__=\"redacted\"",
        "-D__TIME__=\"redacted\"",
    ],
)

filegroup(name = "empty")

cc_toolchain(
    name = "clang-cc-toolchain",
    all_files = ":empty",
    ar_files = ":empty",
    as_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    linker_files = ":empty",
    module_map = None,
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 1,
    toolchain_config = ":clang-cc-toolchain-config",
)

toolchain(
    name = "clang",
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
        "@bazel_tools//tools/cpp:clang",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    toolchain = ":clang-cc-toolchain",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
    visibility = ["//visibility:public"],
)

py_runtime(
    name = "python3_runtime",
    interpreter_path = "/usr/bin/python",
    python_version = "PY3",
    visibility = ["//visibility:public"],
)

py_runtime_pair(
    name = "py_runtime",
    py2_runtime = None,
    py3_runtime = ":python3_runtime",
)

toolchain(
    name = "py_toolchain",
    toolchain = ":py_runtime",
    toolchain_type = "@bazel_tools//tools/python:toolchain_type",
)
