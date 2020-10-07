// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "fake_localizer/fake_localizer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_localizer");
    FakeLocalizer fl;
    fl.run();
    return 0;
}

