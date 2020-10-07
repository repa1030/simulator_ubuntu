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

#include "peak_finder.h"

// Constructor
PeakFinder::PeakFinder()
 : EPS(2.2204e-16f) {
}

// Destructor
PeakFinder::~PeakFinder() {
}

// Calulcate derivation (delta) vector of an input in
void PeakFinder::diff(std::vector<float> in, std::vector<float>& out) {
	out = std::vector<float>(in.size()-1);
	for(int i=1; i<in.size(); ++i)
		out[i-1] = in[i] - in[i-1];
}

// Calculate products of each vector elements from 0 to size(a) of two input vectors a and b
void PeakFinder::vectorProduct(std::vector<float> a, std::vector<float> b, std::vector<float>& out) {
	out = std::vector<float>(a.size());
	for(int i=0; i<a.size(); ++i)
		out[i] = a[i] * b[i];
}

// Find vector elements that are smaller than a given threshold
void PeakFinder::findIndicesLessThan(std::vector<float> in, float threshold, std::vector<int>& indices) {
	for(int i=0; i<in.size(); ++i)
		if(in[i]<threshold)
			indices.push_back(i+1);
}

// Select elements in origin float vector with given indices vector
void PeakFinder::selectElements(std::vector<float> in, std::vector<int> indices, std::vector<float>& out) {
	for(int i=0; i<indices.size(); ++i)
		out.push_back(in[indices[i]]);
}

// Select elements in origin int vector with given indices vector
void PeakFinder::selectElements(std::vector<int> in, std::vector<int> indices, std::vector<int>& out) {
	for(int i=0; i<indices.size(); ++i)
		out.push_back(in[indices[i]]);
}

// Analyse the sign of the vector elements and returns an output vector with range [1, -1]
void PeakFinder::signVector(std::vector<float> in, std::vector<int>& out) {
	out = std::vector<int>(in.size());
	for(int i=0; i<in.size(); ++i) {
	    if(in[i]>0)
			out[i]=1;
		else if(in[i]<0)
			out[i]=-1;
		else
			out[i]=0;
	}
}

// Find peaks in vector x0_raw
void PeakFinder::findPeaks(std::vector<int> x0_raw, std::vector<int>& peakInds) {
    // Convert the input int vector to float vector x0
    std::vector<float> x0;
    for(int i=0; i < x0_raw.size(); i++) {
        x0.push_back(static_cast<float>(x0_raw[i]));
    }

    // Search the global minimum and maximum inside x0
	int minIdx = std::distance(x0.begin(), min_element(x0.begin(), x0.end()));
	int maxIdx = std::distance(x0.begin(), max_element(x0.begin(), x0.end()));

    // Calculate (max-min)/4 for selection later
	float sel = (x0[maxIdx]-x0[minIdx])/4.0;
    
    // Get derivation vector of x0
	std::vector<float> dx;
	diff(x0, dx);
    
    // Replace 0.0 with -EPS
	replace(dx.begin(), dx.end(), 0.0f, -EPS);

    // Generate two vectors, one from dx[start] till dx[end-1]
    // and one from dx[start+1] to dx[end]. So there are is one
    // vector dx0 and one vector dx1 that is shifted by one
	std::vector<float> dx0(dx.begin(), dx.end()-1);
	std::vector<float> dx1(dx.begin()+1, dx.end());
	std::vector<float> dx2;

    // Calculate the product of the elements of dx0 and dx1
    // dx2[i] = dx0[i] * dx1[i]
    // This is done for knowledge of sign change inside the
    // derivation vector
	vectorProduct(dx0, dx1, dx2);

    // Find where the derivative changes sign as this is a
    // necessary condition for a peak
	std::vector<int> ind;
	findIndicesLessThan(dx2, 0, ind);
	
    // Generate a vector that stores the indices where the sign
    // changes in the derivation vector
	std::vector<float> x;
	std::vector<int> indAux(ind.begin(), ind.end());
    
    // Select the elements from the origin float vector x0 where the
    // derivative changes sign
	selectElements(x0, indAux, x);

    // Insert first and last element of origin vector to x as it is
    // not possible to get a significant derivation there (check separately)
	x.insert(x.begin(), x0[0]);
	x.insert(x.end(), x0[x0.size()-1]);;

    // Insert the two elements respectively their indices also into the index vector
	ind.insert(ind.begin(), 0);
	ind.insert(ind.end(), x0.size());

    // Select the Global Minimum and the minimum two its left (same element)
	int minMagIdx = distance(x.begin(), min_element(x.begin(), x.end()));
	float minMag = x[minMagIdx];
	float leftMin = minMag;
	int len = x.size();

    // Only if the vector has more elements than two
	if(len>2) {

		float tempMag = minMag;
    	bool foundPeak = false;
    	int ii;

    	// Deal with first point a little differently
        // Generate subvector from x (first 3 elements), 
        // calculate the differences of the elements and check the signs
    	std::vector<float> xSub0(x.begin(), x.begin()+3);
    	std::vector<float> xDiff;
    	std::vector<int> signDx;
    	diff(xSub0, xDiff);
    	signVector(xDiff, signDx);

        // The first point is larger or equal to the second
        if(signDx[0] <= 0) {
            // Want alternating signs, so erase if not alternating
            if(signDx[0] == signDx[1]) {
                x.erase(x.begin()+1);
                ind.erase(ind.begin()+1);
                len = len-1;
            }
        }

        // First point is smaller than the second
        else {
            // Want alternating signs, so erase if not alternating
            if(signDx[0] == signDx[1]) {
            	x.erase(x.begin());
            	ind.erase(ind.begin());
                len = len-1;
            }
        }

        // Define first iterator
    	if(x[0] >= x[1])
        	ii = 0;
    	else
        	ii = 1;

        // Defines a maximum number of peaks
    	float maxPeaks = ceil((float)len/2.0);
    	std::vector<int> peakLoc(maxPeaks,0);
    	std::vector<float> peakMag(maxPeaks,0.0);
    	int cInd = 1;
    	int tempLoc;
    
    	while(ii < len) {
            // Iterate through peaks
        	ii = ii+1;

        	// Reset peak finding if we had a peak and the next peak is bigger
        	// than the last or the left min was small enough to reset.
        	if(foundPeak) {
            	tempMag = minMag;
            	foundPeak = false;
            }
        
        	// Found new peak that was lager than temp mag and selectivity larger
        	// than the minimum to its left.
        	if(x[ii-1] > tempMag && x[ii-1] > leftMin + sel) {
            	tempLoc = ii-1;
            	tempMag = x[ii-1];
        	}

        	// Make sure we don't iterate past the length of our vector
        	if(ii == len)
                // We assign the last point differently out of the loop
            	break;

            // Move onto the valley
        	ii = ii+1;
        	
        	// Come down at least sel from peak
        	if(!foundPeak && tempMag > sel + x[ii-1]) {
                // We have found a peak           	
	            foundPeak = true;
	            leftMin = x[ii-1];
                // Add peak to index
	            peakLoc[cInd-1] = tempLoc;
	            peakMag[cInd-1] = tempMag;
	            cInd = cInd+1;
	        }
            // New left minima
        	else if(x[ii-1] < leftMin)
            	leftMin = x[ii-1];
        }

        // Check end point
        if(x[x.size()-1] > tempMag && x[x.size()-1] > leftMin + sel) {
            peakLoc[cInd-1] = len-1;
            peakMag[cInd-1] = x[x.size()-1];
            cInd = cInd + 1;
        }

        // Check if we still need to add the last point
        else if(!foundPeak && tempMag > minMag) {
            peakLoc[cInd-1] = tempLoc;
            peakMag[cInd-1] = tempMag;
            cInd = cInd + 1;
        }

    	//Create output
    	if(cInd > 0) {        	
        	std::vector<int> peakLocTmp(peakLoc.begin(), peakLoc.begin()+cInd-1);
			selectElements(ind, peakLocTmp, peakInds);
        }
	}
}
