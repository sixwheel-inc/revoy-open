"""Cross-Platform format all source files in current directory tree, skipping generated files."""

import os
import subprocess
import shutil
import shlex
import sys

CPP_FILE_EXTENSIONS = {".c", ".cpp", ".hpp", ".h"}
EXCLUDED_DIRECTORIES = {
    "bazel-out",
    "bazel-bin",
    "bazel-revoy-open",
    "bazel-testlogs",
    "third-party",
    ".git",
    ".vscode",
    "build",
}
CLANG_FORMAT_COMMAND_STRING = "clang-format -i {}"

BUILDIFIEER_FORMAT_COMMAND_STRING = "buildifier -r --lint=fix {}"
BLACK_FORMAT_COMMAND_STRING = "black {}"


def format(top):
    """Call clang-format on all files with the correct extensions, ignore certain directories."""

    paths = get_paths_to_all_source_files(
        top, CPP_FILE_EXTENSIONS, EXCLUDED_DIRECTORIES
    )
    call_clang_format(paths)
    call_python_black(top)
    call_bazel_buildifier(top)


def get_paths_to_all_source_files(top, file_extensions, excluded_directories):
    """Return paths to all source files under top that match the file_extensions, outside of the excluded_directories."""
    paths = []
    for root, directories, files in os.walk(top, topdown=True):
        directories[:] = [d for d in directories if d not in excluded_directories]
        for file in files:
            file_extension = os.path.splitext(file)[-1]
            if file_extension in file_extensions:
                path = os.path.join(root, file)
                paths.append(os.path.normpath(path))
    return paths


def call_clang_format(paths):
    """Call clang-format on the list of paths."""
    paths = shlex.join(paths)
    command = CLANG_FORMAT_COMMAND_STRING.format(paths)
    subprocess.run(shlex.split(command))


def call_bazel_buildifier(top):
    """Call bazel buildifier on all bazel files."""
    command = BUILDIFIEER_FORMAT_COMMAND_STRING.format(top)
    subprocess.run(shlex.split(command))


def call_python_black(top):
    """Call black on entire directory (affects python files only)."""
    command = BLACK_FORMAT_COMMAND_STRING.format(top)
    subprocess.run(shlex.split(command))


if __name__ == "__main__":
    format(os.path.normpath(sys.argv[1]))
