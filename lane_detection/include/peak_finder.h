// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================
// Algorithm based on: https://github.com/claydergc/find-peaks
// =================================================================

#ifndef PEAK_FINDER_H
#define PEAK_FINDER_H

#include <vector>
#include <algorithm>
#include <cmath>

class PeakFinder {

    public:

        // Constructor
        PeakFinder();
        // Destructor
        ~PeakFinder();
        // Find peaks in vector x0_raw
        void findPeaks(std::vector<int> x0_raw, std::vector<int>& peakInds);

    private:

        // Constants
        // EPS is used for 0.0 derivation entries
        const float EPS;
        
        // Member functions
        // Analyse the sign of the vector elements and returns an output vector with range [1, -1]
        void signVector(std::vector<float> in, std::vector<int>& out);
        // Select elements in origin int vector with given indices vector
        void selectElements(std::vector<int> in, std::vector<int> indices, std::vector<int>& out);
        // Select elements in origin vector with given indices vector
        void selectElements(std::vector<float> in, std::vector<int> indices, std::vector<float>& out);
        // Find vector elements that are smaller than a given threshold
        void findIndicesLessThan(std::vector<float> in, float threshold, std::vector<int>& indices);
        // Calculate products of each vector elements from 0 to size(a) of two input vectors a and b
        void vectorProduct(std::vector<float> a, std::vector<float> b, std::vector<float>& out);
        // Calulcate derivation (delta) vector of an input in
        void diff(std::vector<float> in, std::vector<float>& out);

};

#endif
