// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "pure_pursuit_visualizer/pure_pursuit_visualizer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_visualizer");
    PurePursuitVisualizer ppv;
    ppv.run();
    return 0;
}

