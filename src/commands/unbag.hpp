#pragma once

namespace commands {

// Extracts all LiDAR scans as ASCII PCD files and all camera frames as JPEGs.
// Returns an exit code (0 = success).
int unbag(int argc, char** argv);

}  // namespace commands
