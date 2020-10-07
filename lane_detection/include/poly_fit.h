// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================
// Algorithm based on: 
// https://www.bragitoff.com/2018/06/polynomial-fitting-c-program/
// =================================================================

#ifndef POLYFIT_H
#define POLYFIT_H

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

// Polynomial fitting class
class PolyFit {

    public:

        // Constructor for declaration
        PolyFit();

        // Constructor for use with OpenCV (recommend)
        PolyFit(std::vector<cv::Point2f> points, bool switch_axis=true, 
                        int polynom_degree=2);
        // Constructor for default use
        PolyFit(std::vector<double> points_x, std::vector<double> points_y, 
                        bool switch_axis=false, int polynom_degree=2);
        // Destructor
        ~PolyFit();

        // Returning y = a0*x^0 + a1*x^1 + a2^x2 + ...
        double getValueAt(double x);

        // Returning a0, a1, a2, ...
        std::vector<double> getPolynomCoefficients();
    
        // Check if instance was initialized correctly
        bool isInitialized();

    private:

        // Member variables
        std::vector<double> m_poly_coefficients;
        bool m_is_init;

        // Methods
        // Calculate the coefficients
        void calculateCoefficents(std::vector<cv::Point2f> points, 
                                    bool switch_axis, int polynom_degree);
};

#endif
