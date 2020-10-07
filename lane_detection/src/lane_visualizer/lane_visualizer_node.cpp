// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Author: Anna-Lena Marmein
// =================================================================

#include "lane_visualizer/lane_visualizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_visualizer");
  LaneVisualizer lv;
  lv.run();
  return 0;
}

