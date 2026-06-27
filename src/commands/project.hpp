#pragma once

namespace commands {

// Projects LiDAR point clouds onto camera images and saves the result as JPEGs.
// Returns an exit code (0 = success).
int project(int argc, char** argv);

}  // namespace commands
