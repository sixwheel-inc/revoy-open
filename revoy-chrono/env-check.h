#pragma once

namespace revoy {

/// used to greatly speed up simulations by skipping online visualization
bool IsIrrlichtDisabled();

/// used to prevent crashing in tests when access to video is restricted
bool IsRunningInTest();

/// used to set the runtime locations of chrono data
void SetDataPaths();

} // namespace revoy
