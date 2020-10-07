// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "lane_changer/lane_changer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_changer");
    LaneChanger lc;
    lc.run();
    return 0;
}

