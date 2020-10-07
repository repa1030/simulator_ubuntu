// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Author: Patrick Rebling
// =================================================================

#include "lane_detector/lane_detector.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_detector");
  LaneDetector ld;
  ld.run();
  return 0;
}

