/***************************************************************
 * CS4670/5670, Fall 2012 Project 4
 * File to be modified #1:
 * svmmath.cpp
 *		a routine for intersecting >2 lines (for vanishing point
 *		computation);
 *		routines for computing the homography for the reference
 *		plane and arbitrary polygons
 **************************************************************/

#pragma warning(disable : 4996)

#include "Eigen/Core"
#include "MinEig.h"

#include "svmmath.h"
#include "jacob.h"
#include "vec.h"
#include <cstring>
#include <cstdio>
#include <assert.h>
#include <iostream>

using namespace Eigen;
using namespace std;

//
// TODO 1: BestFitIntersect()
//		Given lines, the list of 3 or more lines to be intersected,
//		find the best fit intersection point.
//		See http://www-2.cs.cmu.edu/~ph/869/www/notes/vanishing.txt.
//	
SVMPoint BestFitIntersect(const std::list<SVMLine> &lines, int imgWidth, int imgHeight)
{
    // check
    if (lines.size() < 2)
	{
            fprintf(stderr, "Not enough lines to compute the best fit.");
            abort();
	}

    SVMPoint bestfit;
    list<SVMLine>::const_iterator iter;

    // To accumulate stuff
    typedef Matrix<double, Dynamic, 3, RowMajor> Matrix3;

    int numLines = (int) lines.size();
    Matrix3 A = Matrix3::Zero(numLines, 3);	

    // Transformation for numerical stability

    // Note: iterate through the lines list as follows:
    //		for (iter = lines.begin(); iter != lines.end(); iter++) {
    //			...iter is the pointer to the current line...
    //		}
    // Note: Function to find eigenvector with smallest eigenvalue is MinEig(A, eval, evec)
    //
    /******** BEGIN TODO ********/

    for (iter = lines.begin(); iter != lines.end(); iter++) {
        SVMLine l = iter;
        SVMPoint p1 = l.pnt1;
        SVMPoint p2 = l.pnt2;
        
        double a = p1.v * p2.w - p1.w * p2.v;
        double b = p1.w * p2.u - p1.u * p2.w;
        double c = p1.u * p2.v - p1.v * p2.u;
        
        A[0][0] += a*a;
        A[0][1] += a*b;
        A[0][2] += a*c;
        A[1][0] += b*a;
        A[1][1] += b*b;
        A[1][2] += b*c;
        A[2][0] += c*a;
        A[2][1] += c*b;
        A[2][2] += c*c;
        
    }
    double eval, h[3];
    MinEig(A, eval, h);
    
    bestfit.u = h[0];
    bestfit.v = h[1];
    bestfit.w = h[2];


    /******** END TODO ********/
	
    return bestfit;
}


//
// TODO 2: ConvertToPlaneCoordinate()
//		Given a plane defined by points, converts their coordinates into
//		plane coordinates. See the following document for more detail.
//		http://www.cs.cornell.edu/courses/cs4670/2012fa/projects/p4/homography.pdf.
//      The final divisors you apply to the u and v coordinates should be saved uScale and vScale
//
void ConvertToPlaneCoordinate(const vector<SVMPoint>& points, vector<Vec3d>& basisPts, double &uScale, double &vScale)
{
    int numPoints = points.size();

    /******** BEGIN TODO ********/
    Vec3d p = basisPts[0];
    Vec3d q = basisPts[1];
    Vec3d r = basisPts[2];
    
    Vec3d e_x = (p-r).normalize();
    Vec3d s = prod((q-r), e_x)*e_x;
    Vec3d t = (q-r) - s;
    Vec3d e_y = t.normalize();
    
    vector<Vec3d> plainPts;
    double max_u = std::numeric_limits<double>::min();
    double min_u = std::numeric_limits<double>::max();
    double max_v = std::numeric_limits<double>::min();
    double min_v = std::numeric_limits<double>::max();
    for (i = 0; i < numPoints; i++){
        Vec3d a = basisPts[i];
        
        double u = prod((a-r), e_x);
        double v = prod((a-r), e_y);
        
        if (u > max_u) max_u = u;
        if (u < min_u) min_u = u;
        if (v > max_v) max_v = v;
        if (v < min_v) min_v = v;
        plainPts.push_back(Vec3d(u, v, 1));
    }
    
    uScale = max_u - min_u;
    vScale = max_v - min_v;
    
    for(i = 0; i<numPoints; i++){
        double u = plainPts[i][0];
        double v = plainPts[i][1];
        basisPts[i] = Vec3d((u - min_u)/uScale, (v - min_v)/vScale);
    }

    /******** END TODO ********/
}



//
// TODO 3: ComputeHomography()
//		Computes the homography H from the plane specified by "points" to the image plane,
//		and its inverse Hinv.
//		If the plane is the reference plane (isRefPlane == true), don't convert the
//		coordinate system to the plane. Only do this for polygon patches where
//		texture mapping is necessary.
//		Coordinate system conversion is to be implemented in a separate routine
//		ConvertToPlaneCoordinate.
//		For more detailed explaination, see
//		http://www.cs.cornell.edu/courses/cs4670/2012fa/projects/p4/homography.pdf.
//
void ComputeHomography(CTransform3x3 &H, CTransform3x3 &Hinv, const vector<SVMPoint> &points, vector<Vec3d> &basisPts, bool isRefPlane)
{
    int i;
    int numPoints = (int) points.size();
    assert( numPoints >= 4 );

    basisPts.clear();
    if (isRefPlane) // reference plane
    {
        for (i=0; i < numPoints; i++) {
            Vec3d tmp = Vec3d(points[i].X, points[i].Y, points[i].W); // was Z, not W
            basisPts.push_back(tmp);
        }
    } 
    else // arbitrary polygon
    {
        double uScale, vScale; // unused in this function
        ConvertToPlaneCoordinate(points, basisPts, uScale, vScale);
    }

    // A: 2n x 9 matrix where n is the number of points on the plane
    //    as discussed in lecture
    int numRows = 2 * numPoints;
    const int numCols = 9;

    typedef Matrix<double, Dynamic, 9, RowMajor> MatrixType;
    MatrixType A = MatrixType::Zero(numRows, numCols);

    /******** BEGIN TODO ********/
    /* Fill in the A matrix for the call to MinEig */
    for (int i = 0; i < numPoints; i++) {
        Vec3d a = basisPts[i];
        SVMPoint b = points[i];
        
        A(2*i, 0) = a[0];
        A(2*i, 1) = a[1];
        A(2*i, 2) = 1;
        A(2*i, 6) = -b.u * a[0];
        A(2*i, 7) = -b.u * a[1];
        A(2*i, 8) = -b.u;
        
        A(2*i+1, 3) = a[0];
        A(2*i+1, 4) = a[1];
        A(2*i+1, 5) = 1;
        A(2*i+1, 6) = -b.v * a[0];
        A(2*i+1, 7) = -b.v * a[1];
        A(2*i+1, 8) = -b.v;
        
        // END TODO
    }



    double eval, h[9];
    MinEig(A, eval, h);

    H[0][0] = h[0];
    H[0][1] = h[1];
    H[0][2] = h[2];

    H[1][0] = h[3];
    H[1][1] = h[4];
    H[1][2] = h[5];

    H[2][0] = h[6];
    H[2][1] = h[7];
    H[2][2] = h[8];

    /******** END TODO ********/

    // compute inverse of H
    if (H.Determinant() == 0)
        fl_alert("Computed homography matrix is uninvertible \n");
    else
        Hinv = H.Inverse();

    int ii;
    printf("\nH=[\n");
    for (ii=0; ii<3; ii++)
        printf("%e\t%e\t%e;\n", H[ii][0]/H[2][2], H[ii][1]/H[2][2], H[ii][2]/H[2][2]);
    printf("]\nHinv=[\n");

    for (ii=0; ii<3; ii++)
        printf("%e\t%e\t%e;\n", Hinv[ii][0]/Hinv[2][2], Hinv[ii][1]/Hinv[2][2], Hinv[ii][2]/Hinv[2][2]);

    printf("]\n\n");
}

