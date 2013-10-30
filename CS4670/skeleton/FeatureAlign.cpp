///////////////////////////////////////////////////////////////////////////
//
// NAME
//  FeatureAlign.cpp -- image registration using feature matching
//
// SEE ALSO
//  FeatureAlign.h      longer description
//
// Based on code by Richard Szeliski, 2001.
// (modified for CSE576 Spring 2005, and for CS4670, Fall 2012-2013)
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "FeatureAlign.h"
#include "SVD.h"

#include <math.h>
#include <iostream>

CTransform3x3 ComputeHomography(const FeatureSet &f1, const FeatureSet &f2,
                                const vector<FeatureMatch> &matches)
{
    int numMatches = (int) matches.size();

    // first, we will compute the A matrix in the homogeneous linear equations Ah = 0
    int numRows = 2 * numMatches; // number of rows of A
    const int numCols = 9;        // number of columns of A

    // this allocates space for the A matrix
    AMatrixType A = AMatrixType::Zero(numRows, numCols);

    for (int i = 0; i < numMatches; i++) {
        const FeatureMatch &m = matches[i];
        const Feature &a = f1[m.id1];
        const Feature &b = f2[m.id2];

        // BEGIN TODO
        // fill in the matrix A in this loop.
        // To access an element of A, use parentheses, e.g. A(0,0)
        
        A(2i, 0) = a.x;
        A(2i, 1) = a.y;
        A(2i, 2) = 1;
        A(2i, 6) = - b.x * a.x;
        A(2i, 7) = - b.x * a.y;
        A(2i, 8) = - b.x;
        
        A(2i+1, 3) = a.x;
        A(2i+1, 4) = a.y;
        A(2i+1, 5) = 1;
        A(2i+1, 6) = -b.y * a.x;
        A(2i+1, 7) = -b.y * a.y;
        A(2i+1, 8) = -b.y;

        // END TODO
    }

    // Compute the SVD of the A matrix and get out the matrix V^T and the vector of singular values
    AMatrixType Vt;
    VectorXd sv;
    SVD(A, Vt, sv);

    CTransform3x3 H;
    // BEGIN TODO
    // fill the homography H with the appropriate elements of the SVD
    // To extract, for instance, the V matrix, use svd.matrixV()
    
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++){
            k = 3*i + j;
            H[i][j] = Vt(Vt.rows - 1, k);
        }
    }

    // END TODO

    return H;
}


/******************* TO DO *********************
 * alignPair:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *               Each match in 'matches' contains two feature ids of 
 *               matching features, id1 (in f1) and id2 (in f2).
 *		m: motion model
 *		nRANSAC: number of RANSAC iterations
 *		RANSACthresh: RANSAC distance threshold
 *		M: transformation matrix (output)
 *
 *	OUTPUT:
 *		repeat for nRANSAC iterations:
 *			choose a minimal set of feature matches
 *			estimate the transformation implied by these matches
 *			count the number of inliers
 *		for the transformation with the maximum number of inliers,
 *		compute the least squares motion estimate using the inliers,
 *		and store it in M
 */
int alignPair(const FeatureSet &f1, const FeatureSet &f2,
          const vector<FeatureMatch> &matches, MotionModel m, 
          int nRANSAC, double RANSACthresh, CTransform3x3& M)
{
    // BEGIN TODO
    // Write this entire method.  You need to handle two types of 
    // motion models, pure translations (m == eTranslation) and 
    // full homographies (m == eHomography).  However, you should
    // only have one outer loop to perform the RANSAC code, as 
    // the use of RANSAC is almost identical for both cases.
    //
    // Your homography handling code should call ComputeHomography.
    // This function should also call countInliers and, at the end,
    // leastSquaresFit.
    CTransform3x3 temp;
    int max_inliers_c = 0;
    vector<int> best_inliers;
    CTransform3x3 best;
    
    for (int i = 0; i < nRANSAC; i++){
        int s = matches.size();
        switch (m) {
            case eTranslate: {
                //2 DOF, so one match.
                int i = rand() % s;
                const FeatureMatch &match = matches[i];
                const Feature &a = f1[match.id1];
                const Feature &b = f2[match.id2];
                double tx = b.x - a.x;
                double ty = b.y - a.y;
                temp = CTransform3x3::Translation((float) tx, (float) ty);
                break;
            }
                
            case eHomography: {
                //8 DOF, so 4 matches.
                int a = rand() % s;
                int b = rand() % s;
                int c = rand() % s;
                int d = rand() % s;
                vector<FeatureMatch> rand_matches = new vector<FeatureMatch>();
                rand_matches.push_back(matches[a]);
                rand_matches.push_back(matches[b]);
                rand_matches.push_back(matches[c]);
                rand_matches.push_back(matches[d]);
                temp = ComputeHomography(f1, f2, rand_matches);
                break;
            }
        }
        
        vector<int> inliers;
        int inliers_c = countInliers(f1,f2,matches,m, temp, RANSACthresh, inliers);
        if (inliers_c > max_inliers_c){
            max_inliers_c = inliers_c;
            best = temp;
            best_inliers = inliers;
        }
        
    }
    leastSquaresFit(f1, f2, matches, m, best_inliers, best);
    M = best;
    return 0;

    // END TODO

    return 0;
}

/******************* TO DO *********************
 * countInliers:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *               Each match in 'matches' contains two feature ids of 
 *               matching features, id1 (in f1) and id2 (in f2).
 *		m: motion model
 *		M: transformation matrix
 *		RANSACthresh: RANSAC distance threshold
 *		inliers: inlier feature IDs
 *	OUTPUT:
 *		transform the features in f1 by M
 *
 *		count the number of features in f1 for which the transformed
 *		feature is within Euclidean distance RANSACthresh of its match
 *		in f2
 *
 *		store these features IDs in inliers
 *
 */
int countInliers(const FeatureSet &f1, const FeatureSet &f2,
                 const vector<FeatureMatch> &matches, MotionModel m,
                 CTransform3x3 M, double RANSACthresh, vector<int> &inliers)
{
    inliers.clear();

    for (unsigned int i = 0; i < matches.size(); i++) {
        // BEGIN TODO
        // determine if the ith matched feature f1[id1], when transformed by M,
        // is within RANSACthresh of its match in f2
        //
        // if so, append i to inliers
        const FeatureMatch &match = matches[i];
        const Feature &a = f1[match.id1];
        const Feature &b = f2[match.id2];
        
        CVector3 p;
        p[0] = a.x;
        p[1] = a.y;
        p[2] = 1;
        p = M * p;
        
        double distance = sqrt((p[0] - b.x)*(p[0] - b.x) + (p[1] - b.y)*(p[1] - b.y));
        if (distance <= RANSACthresh) inliers.push_back(i);

        // END TODO
    }

    return (int) inliers.size();
}

/******************* TO DO *********************
 * leastSquaresFit:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *      inliers: inlier match indices (indexes into 'matches' array)
 *		M: transformation matrix (output)
 *	OUTPUT:
 *		compute the transformation from f1 to f2 using only the inliers
 *		and return it in M
 */
int leastSquaresFit(const FeatureSet &f1, const FeatureSet &f2,
            const vector<FeatureMatch> &matches, MotionModel m, 
            const vector<int> &inliers, CTransform3x3& M)
{
    // This function needs to handle two possible motion models, 
    // pure translations and full homographies.

    switch (m) {
        case eTranslate: {
            // for spherically warped images, the transformation is a 
            // translation and only has two degrees of freedom
            //
            // therefore, we simply compute the average translation vector
            // between the feature in f1 and its match in f2 for all inliers
            double u = 0;
            double v = 0;

            for (int i=0; i < (int) inliers.size(); i++) {
			    // BEGIN TODO
			    // use this loop to compute the average translation vector
			    // over all inliers
                FeatureMatch match = matches[inliers[i]];
                const Feature &a = f1[match.id1];
                const Feature &b = f2[match.id2];
                u += (b.x - a.x);
                v += (b.y - a.y);

                // END TODO
            }

            u /= inliers.size();
            v /= inliers.size();

            M = CTransform3x3::Translation((float) u, (float) v);

            break;
        } 

        case eHomography: {
			M = CTransform3x3();

            // BEGIN TODO
		    // Compute a homography M using all inliers.
		    // This should call ComputeHomography.
            for (int i=0; i < (int) inliers.size(); i++) {
                FeatureMatch match = matches[inliers[i]];
                inlier_matches.push_back(match);
            }
            M = ComputeHomography(f1, f2, inlier_matches);

            // END TODO
        
            break;
        }
    }    

    return 0;
}

