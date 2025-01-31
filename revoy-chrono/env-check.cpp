
#include "revoy-chrono/env-check.h"

#include <chrono/core/ChGlobal.h>

#include <chrono_vehicle/ChVehicleModelData.h>

#include <filesystem>

namespace revoy {

bool IsIrrlichtDisabled() {
  // Check for REVOY_IRRLICHT_DISABLE, used to disable the viz and speed up
  // execution
  const char *revoyIrrlichDisableEnv = std::getenv("REVOY_IRRLICHT_DISABLE");
  return revoyIrrlichDisableEnv != nullptr;
}

bool IsRunningInTest() {
  // Check for TEST_TMPDIR, which is set only during `bazel test`
  const char *test_tmpdir = std::getenv("TEST_TMPDIR");
  return test_tmpdir != nullptr;
}

/// This is necessary to find the correct data_dir, the data is loaded at
/// runtime and the runtime code needs to know where the data is.
/// It is the user's responsibility to set this up, using --test_env or
/// --action_env as appropriate. Also, `bazel info bazel-bin` will return the
/// correct path absolute path. e.g.
/// ```bash
/// bazel test --test_env=BAZEL_BIN=$(bazel info bazel-bin) //chrono-stuff/...
/// ```
std::filesystem::path FindBazelBin() {

  const std::string BAZEL_BIN_ENV = "BAZEL_BIN";
  const char *env = std::getenv(BAZEL_BIN_ENV.c_str());

  if (!env) {
    return "";
  }

  const std::filesystem::path BAZEL_BIN_PATH(env);
  if (!std::filesystem::exists(BAZEL_BIN_PATH)) {
    return "";
  }

  return BAZEL_BIN_PATH;
}

void SetDataPaths() {

  const std::filesystem::path DATA_DIR(
      "external/projectchrono/projectchrono/share/chrono/data/");

  const std::filesystem::path bazelBin = FindBazelBin();
  /// if we cannot find the bazel-bin, then we cannot do anything,
  /// that's where the OBJ / etc files are, which we will load
  /// from at runtime.
  assert(!bazelBin.empty());

  /// base chrono data path
  const std::filesystem::path data = bazelBin / DATA_DIR;
  chrono::SetChronoDataPath(data);

  /// vehicle model data path
  const std::filesystem::path vehicle =
      data / std::filesystem::path("vehicle/");
  chrono::vehicle::SetDataPath(vehicle);

  /// outputs will be relative to the specific bazel-out subdir of the
  /// calling cc_binary().
  const std::filesystem::path out("");
  chrono::SetChronoOutputPath(out);
};

} // namespace revoy
