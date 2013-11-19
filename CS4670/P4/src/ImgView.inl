/***************************************************************
 * CS4670/5670, Fall 2012 Project 4
 * File to be modified #2:
 * ImgView.inl (included from ImgView.cpp)
 *		contains routines for computing the 3D position of points
 ***************************************************************/
#include <math.h>

double mag(SVMPoint p) {
    return sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
}

SVMPoint sub(SVMPoint p1, SVMPoint p2) {
    return SVMPoint(p1.u / p1.w - p2.u / p2.w, p1.v / p1.w - p2.v / p2.w);
}

SVMPoint cross(SVMPoint p1, SVMPoint p2) {
    SVMPoint ret;
    ret.u = p1.v * p2.w - p1.w * p2.v;
    ret.v = p1.w * p2.u - p1.u * p2.w;
    ret.w = p1.u * p2.v - p1.v * p2.u;
    return ret;
}

double dot(SVMPoint p1, SVMPoint p2) {
    return p1.u * p2.u + p1.v * p2.v + p1.w * p2.w;
}

void transform(CTransform3x3 H, SVMPoint *p) {
    p->u = H[0][0] * p->X + H[0][1] * p->Y + H[0][2] * p->Z;
    p->v = H[1][0] * p->X + H[1][1] * p->Y + H[1][2] * p->Z;
    p->w = H[2][0] * p->X + H[2][1] * p->Y + H[2][2] * p->Z;
}

void transformInv(CTransform3x3 HInv, SVMPoint *p) {
    p->X = HInv[0][0] * p->u + HInv[0][1] * p->v + HInv[0][2] * p->w;
    p->Y = HInv[1][0] * p->u + HInv[1][1] * p->v + HInv[1][2] * p->w;
    p->Z = HInv[2][0] * p->u + HInv[2][1] * p->v + HInv[2][2] * p->w;
}

//
// TODO 4: sameXY()
//		Computes the 3D position of newPoint using knownPoint
//		that has the same X and Y coordinate, i.e. is directly
//		below or above newPoint.
//		See lecture slide on measuring heights.
//
// HINT1: make sure to dehomogenize points when necessary
// HINT2: there is a degeneracy that you should look out for involving points already in line with the reference
// HINT3: make sure to get the sign of the result right, i.e. whether it is above or below ground
void ImgView::sameXY()
{
	if (pntSelStack.size() < 2)
	{
		fl_alert("Not enough points on the stack.");
		return;
	}

	SVMPoint &newPoint = *pntSelStack[pntSelStack.size() - 1];
	SVMPoint &knownPoint = *pntSelStack[pntSelStack.size() - 2];

	if( !knownPoint.known() )
	{
		fl_alert("Can't compute relative values for unknown point.");
		return;
	}

	if( refPointOffPlane == NULL )
	{
		fl_alert("Need to specify the reference height first.");
		return;
	}

	/******** BEGIN TODO ********/

	// See the lecture note on measuring heights
	// using a known point directly below the new point.

	// printf("sameXY() to be implemented!\n");

    SVMPoint b;
    b.X = refPointOffPlane->X;
    b.Y = refPointOffPlane->Y;
    b.Z = 0;
    transform(H, &b);

    SVMPoint r;
    r.u = refPointOffPlane->u;
    r.v = refPointOffPlane->v;
    r.w = refPointOffPlane->w;

    SVMPoint t;

    if (knownPoint.X == refPointOffPlane->X && knownPoint.Y == refPointOffPlane->Y) {
        t = newPoint;
    }
    else {
        SVMPoint b0;
        b0.X = knownPoint.X;
        b0.Y = knownPoint.Y;
        b0.Z = 0;
        transform(H, &b0);

        SVMPoint v = cross(cross(b, b0), cross(xVanish, yVanish));
        t = cross(cross(v, newPoint), cross(r, b));
    }

    double numerator = mag(sub(t, b)) * mag(sub(zVanish, r));
    double denominator = mag(sub(r, b)) * mag(sub(zVanish, t));
    double imageCrossRatio;

    if (denominator != 0) imageCrossRatio = referenceHeight * numerator / denominator;
    else imageCrossRatio = 0;

    newPoint.X = knownPoint.X;
    newPoint.Y = knownPoint.Y;
    if (dot(sub(t, b), sub(r, b)) < 0) newPoint.Z = -imageCrossRatio;
    else newPoint.Z = imageCrossRatio;

	/******** END TODO ********/

	newPoint.known(true);

	printf( "Calculated new coordinates for point: (%e, %e, %e)\n", newPoint.X, newPoint.Y, newPoint.Z );

	redraw();
}



//
// TODO 5: sameZPlane()
//		Compute the 3D position of newPoint using knownPoint
//		that lies on the same plane and whose 3D position is known.
//		See the man on the box lecture slide.
//		If newPoint is on the reference plane (Z==0), use homography (this->H, or simply H) directly.
//
// HINT: For this function, you will only need to use the three vanishing points and the reference homography 
//       (in addition to the known 3D location of knownPoint, and the 2D location of newPoint)
void ImgView::sameZPlane()
{
	if (pntSelStack.size() < 2)
	{
		fl_alert("Not enough points on the stack.");
		return;
	}

	SVMPoint &newPoint = *pntSelStack[pntSelStack.size() - 1];
	SVMPoint &knownPoint = *pntSelStack[pntSelStack.size() - 2];

	if( !knownPoint.known() )
	{
		fl_alert("Can't compute relative values for unknown point.");
		return;
	}

	/******** BEGIN TODO ********/
    if (knownPoint.Z == 0) {
        transformInv(Hinv, &newPoint);
    }
    else {
        SVMPoint v = cross(cross(knownPoint, newPoint), cross(xVanish, yVanish));

        SVMPoint b1;
        b1.X = knownPoint.X;
        b1.Y = knownPoint.Y;
        b1.Z = 0;
        transform(H, &b1);

        SVMPoint b0 = cross(cross(b1, v), cross(zVanish, newPoint));

        transform(Hinv, &b0);

        b0.Z = knownPoint.Z;
    }

	/******** END TODO ********/

	newPoint.known(true);

	printf( "Calculated new coordinates for point: (%e, %e, %e)\n", newPoint.X, newPoint.Y, newPoint.Z );

	redraw();
}


