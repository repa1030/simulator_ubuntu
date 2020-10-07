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

#include "poly_fit.h"

// Constructor for declaration
PolyFit::PolyFit() :
  m_is_init(false) {

}

// Constructor for use with OpenCV (recommend)
PolyFit::PolyFit(std::vector<cv::Point2f> points, 
                bool switch_axis, int polynom_degree) :
  m_is_init(true) {

    // Calculate the coefficients
    calculateCoefficents(points, switch_axis, polynom_degree);

}

// Constructor for default use
PolyFit::PolyFit(std::vector<double> points_x, 
                std::vector<double> points_y, 
                bool switch_axis, int polynom_degree) :
  m_is_init(true) {

    // Convert input vectors to cv points
    std::vector<cv::Point2f> points; 
    for (int i = 0; i < points_x.size(); i++) {
        points.push_back(cv::Point(points_x[i], points_y[i]));
    }
    // Calculate the coefficients
    calculateCoefficents(points, switch_axis, polynom_degree);

}

// Destructor
PolyFit::~PolyFit() {
}

// Returning a0*x^0 + a1*x^1 + a2^x2 + ...
double PolyFit::getValueAt(double x) {
    double fx = 0;
    for (int i = 0; i < m_poly_coefficients.size(); i++) {
        fx += m_poly_coefficients[i] * pow(x, i);   
    }
    return fx;
}

// Returning a0, a1, a2, ...
std::vector<double> PolyFit::getPolynomCoefficients() {
    return m_poly_coefficients;
}

// Check if instance was initialized correctly
bool PolyFit::isInitialized() {
    return m_is_init;
}

// Calculate the coefficients
void PolyFit::calculateCoefficents(std::vector<cv::Point2f> points, 
                                        bool switch_axis, int polynom_degree) {

    int i,j,k,n,N;
    N = points.size();
    n = polynom_degree;

    // Given data points
    double x[N],y[N];

    // Switch axis if necessary
    for(i = 0; i < N; i++) {
        x[i] = switch_axis ? points[i].y : points[i].x;
        y[i] = switch_axis ? points[i].x : points[i].y;
    }

    // Create an array of the size 2*n+1 to store the independend components of the normal matrix
    // N, Sig xi, Sig xi^2, ....
    double X[2*n+1];                       
    for(i = 0; i < 2*n+1; i++) {
        X[i] = 0;
        for(j = 0; j < N; j++) {
            X[i] = X[i] + pow(x[j],i);
        }
    }

    // Create an array to store the values of 
    // sigma(yi), sigma(xi*yi), sigma(xi^2*yi) ... sigma(xi^n*yi)          
    double Y[n+1];                    
    for(i = 0; i < n+1; i++) {    
        Y[i] = 0;
        for(j = 0; j < N; j++) {
            Y[i]=Y[i]+pow(x[j],i)*y[j];
        }
    }

    // B is the normal matrix (augmented) that will store the equations
    double B[n+1][n+2];   
         
    // Build the normal matrix by storing the left side of the equations
    for(i = 0; i <= n; i++) {
        for(j = 0; j <= n; j++) {
            B[i][j]  =X[i+j];
        }
    }

    // Store the right side of the equations in the normal matrix
    for(i = 0; i <= n; i++) {
        B[i][n+1] = Y[i];
    }

    // n is made n+1 because the Gaussian Elimination part below was for n equations,
    // but here n is the degree of polynomial and for n degree we get n+1 equations
    n = n + 1;

    // From now Gaussian Elimination starts to solve the set of linear equations (Pivotisation)          
    for(i = 0; i < n+1; i++) {
        // Partial pivoting
        for(k = i+1; k < n; k++) {
            // If diagonal element (absolute value) is smaller than any of the terms below it
            if(fabs(B[i][i]) < fabs(B[k][i])) {
                // Swap the rows
                for (j = 0; j < n+1; j++) {
                    double temp;
                    temp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = temp;
                }
            }
        }
        // Begin Gauss Elimination
        for(k = i+1; k < n; k++) {
            double term = B[k][i] / B[i][i];
            for(j = 0; j < n+1; j++) {
                // Make the elements below the pivot elements equal to zero or eliminate the variables
                B[k][j] = B[k][j] - term * B[i][j];
            }
        }
    }

    // The final coefficients will be stored in a
    double a[n+1];

    // Begin back substitution
    for(i = n-1; i >= 0; i--) {
        // Make the variable to be calculated equal to the rhs of the last equation                       
        a[i] = B[i][n];
        for(j = i+1; j < n; j++) {
            // Then subtract all the lhs values except the coefficient of the variable whose value is being calculated
            a[i] = a[i] - B[i][j] * a[j];
        }
        // Now finally divide the rhs by the coefficient of the variable to be calculated
        a[i] = a[i] / B[i][i];
    }

    // Save the polynom coefficients in member variable so that a[0] + a[1]x + a[2]x^2 + ...
    for(i = 0; i < n; i++) {
        m_poly_coefficients.push_back(a[i]);
    }
}
