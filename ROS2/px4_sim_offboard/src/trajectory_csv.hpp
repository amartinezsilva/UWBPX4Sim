#pragma once

// Trajectory CSV shared by the UAV and UGV offboard nodes.
//
// The first row is a header naming the columns. Required: x, y, z, yaw
// (spawn-relative ENU, the vehicle follows them in order). Optional:
// wait_x, wait_y, wait_r, which turn a row into a hold point: the vehicle
// stops there and waits until the *other* robot (its ground truth on
// `wait_for_topic`) is within wait_r metres (horizontally) of (wait_x, wait_y).
// wait_x/wait_y are in the same spawn-relative frame as this file's x/y.
// Rows that don't hold leave the three wait cells empty.

#include <array>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace px4_sim_offboard {

struct WaitCondition {
  double x = 0.0, y = 0.0, r = 0.0;  // spawn-relative on load; the node shifts it to world
};

struct TrajectoryCsv {
  std::vector<std::array<double, 4>> points;  // x, y, z, yaw
  std::map<size_t, WaitCondition> waits;      // point index -> condition to leave it
};

namespace detail {
inline std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}
inline std::vector<std::string> splitCsv(const std::string &line) {
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) cells.push_back(trim(cell));
  if (!line.empty() && line.back() == ',') cells.push_back("");
  return cells;
}
}  // namespace detail

// Returns false with `error` set when the file can't be read or a row is
// malformed (rows with a bad number are skipped with a warning in `warnings`).
inline bool loadTrajectoryCsv(const std::string &filename, TrajectoryCsv &out, std::string &error,
                              std::vector<std::string> &warnings) {
  out = TrajectoryCsv{};
  std::ifstream file(filename);
  if (!file.is_open()) {
    error = "failed to open " + filename;
    return false;
  }

  std::string line;
  if (!std::getline(file, line)) {
    error = filename + " is empty";
    return false;
  }
  const auto header = detail::splitCsv(line);
  auto column = [&](const std::string &name) -> int {
    for (size_t i = 0; i < header.size(); ++i)
      if (header[i] == name) return static_cast<int>(i);
    return -1;
  };
  const int cx = column("x"), cy = column("y"), cz = column("z"), cyaw = column("yaw");
  if (cx < 0 || cy < 0 || cz < 0 || cyaw < 0) {
    error = filename + ": the header must name the columns x, y, z and yaw";
    return false;
  }
  const int cwx = column("wait_x"), cwy = column("wait_y"), cwr = column("wait_r");
  const bool has_waits = cwx >= 0 && cwy >= 0 && cwr >= 0;
  if (!has_waits && (cwx >= 0 || cwy >= 0 || cwr >= 0)) {
    error = filename + ": wait columns need all of wait_x, wait_y and wait_r";
    return false;
  }

  size_t line_num = 1;
  while (std::getline(file, line)) {
    ++line_num;
    if (detail::trim(line).empty()) continue;
    const auto cells = detail::splitCsv(line);
    auto cell = [&](int c) -> std::string { return c < static_cast<int>(cells.size()) ? cells[c] : ""; };
    try {
      out.points.push_back({std::stod(cell(cx)), std::stod(cell(cy)), std::stod(cell(cz)), std::stod(cell(cyaw))});
    } catch (const std::exception &) {
      warnings.push_back("skipping invalid line " + std::to_string(line_num) + ": " + line);
      continue;
    }
    if (!has_waits) continue;
    const std::string wx = cell(cwx), wy = cell(cwy), wr = cell(cwr);
    if (wx.empty() && wy.empty() && wr.empty()) continue;
    try {
      out.waits[out.points.size() - 1] = WaitCondition{std::stod(wx), std::stod(wy), std::stod(wr)};
    } catch (const std::exception &) {
      error = filename + ": line " + std::to_string(line_num) + " has an incomplete wait condition";
      return false;
    }
  }
  return true;
}

}  // namespace px4_sim_offboard
