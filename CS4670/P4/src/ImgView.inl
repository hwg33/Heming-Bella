/***************************************************************
 * CS4670/5670, Fall 2012 Project 4
 * File to be modified #2:
 * ImgView.inl (included from ImgView.cpp)
 *		contains routines for computing the 3D position of points
 ***************************************************************/
#include <math.h>

double mag(SVMPoint p) {
    return sqrt(p.u * p.u + p.v * p.v);
}

SVMPoint sub(SVMPoint p1, SVMPoint p2) {
    if (p1.w == 0) p1.w = 1;
    if (p2.w == 0) p2.w = 1;
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
    if (p->w != 0) {
        p->u = p->u / p->w;
        p->v = p->v / p->w;
    }
    p->w = 1;
}

void transformInv(CTransform3x3 HInv, SVMPoint *p) {
    p->X = HInv[0][0] * p->u + HInv[0][1] * p->v + HInv[0][2] * p->w;
    p->Y = HInv[1][0] * p->u + HInv[1][1] * p->v + HInv[1][2] * p->w;
    p->Z = HInv[2][0] * p->u + HInv[2][1] * p->v + HInv[2][2] * p->w;
    if (p->Z != 0) {
        p->X = p->X / p->Z;
        p->Y = p->Y / p->Z;
    }
    p->Z = 1;
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
    printf("H[0][0], H[0][1], H[0][2] = %f, %f, %f\n", H[0][0], H[0][1], H[0][2]);
    printf("H[1][0], H[0][1], H[1][2] = %f, %f, %f\n", H[1][0], H[1][1], H[1][2]);
    printf("H[2][0], H[0][1], H[2][2] = %f, %f, %f\n", H[2][0], H[2][1], H[2][2]);
    printf("Hinv[0][0], Hinv[0][1], Hinv[0][2] = %f, %f, %f\n", Hinv[0][0], Hinv[0][1], Hinv[0][2]);
    printf("Hinv[1][0], Hinv[0][1], Hinv[1][2] = %f, %f, %f\n", Hinv[1][0], Hinv[1][1], Hinv[1][2]);
    printf("Hinv[2][0], Hinv[0][1], Hinv[2][2] = %f, %f, %f\n", Hinv[2][0], Hinv[2][1], Hinv[2][2]);
    printf("newPoint.u = %f\n", newPoint.u);
    printf("newPoint.v = %f\n", newPoint.v);
    printf("newPoint.w = %f\n", newPoint.w);
    printf("newPoint.X = %f\n", newPoint.X);
    printf("newPoint.Y = %f\n", newPoint.Y);
    printf("newPoint.Z = %f\n", newPoint.Z);
    printf("knownPoint.u = %f\n", knownPoint.u);
    printf("knownPoint.v = %f\n", knownPoint.v);
    printf("knownPoint.w = %f\n", knownPoint.w);
    printf("knownPoint.X = %f\n", knownPoint.X);
    printf("knownPoint.Y = %f\n", knownPoint.Y);
    printf("knownPoint.Z = %f\n", knownPoint.Z);
    printf("refPointOffPlane->u = %f\n", refPointOffPlane->u);
    printf("refPointOffPlane->v = %f\n", refPointOffPlane->v);
    printf("refPointOffPlane->w = %f\n", refPointOffPlane->w);
    printf("refPointOffPlane->X = %f\n", refPointOffPlane->X);
    printf("refPointOffPlane->Y = %f\n", refPointOffPlane->Y);
    printf("refPointOffPlane->Z = %f\n", refPointOffPlane->Z);

    SVMPoint b;
    b.X = refPointOffPlane->X;
    b.Y = refPointOffPlane->Y;
    b.Z = 1;
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
        b0.Z = 1;
        transform(H, &b0);
        printf("b0.u = %f\n", b0.u);
        printf("b0.v = %f\n", b0.v);
        printf("b0.w = %f\n", b0.w);
        printf("b.u = %f\n", b.u);
        printf("b.v = %f\n", b.v);
        printf("b.w = %f\n", b.w);

        SVMPoint v = cross(cross(b, b0), cross(xVanish, yVanish));
        if (v.w != 0) {
            v.u = v.u / v.w;
            v.v = v.v / v.w;
        }
        v.w = 1;
        printf("v.u = %f\n", v.u);
        printf("v.v = %f\n", v.v);
        printf("v.w = %f\n", v.w);
        t = cross(cross(v, newPoint), cross(r, b));
        if (t.w != 0) {
            t.u = t.u / t.w;
            t.v = t.v / t.w;
        }
        t.w = 1;
    }
    SVMPoint temp = sub(t, b);
    printf("t.u = %f\n", t.u);
    printf("t.v = %f\n", t.v);
    printf("t.w = %f\n", t.w);
    printf("b.u = %f\n", b.u);
    printf("b.v = %f\n", b.v);
    printf("b.w = %f\n", b.w);
    printf("sub(t, b).u = %f\n", temp.u);
    printf("sub(t, b).v = %f\n", temp.v);

    //printf("mag(sub(t, b)) = %f\n", mag(sub(t, b)));
    //printf("sub(zVanish, r) = %f\n", sub(zVanish, r));
    //printf("mag(sub(zVanish, r)) = %f\n", mag(sub(zVanish, r)));

    double numerator = mag(sub(t, b)) * mag(sub(zVanish, r));
    double denominator = mag(sub(r, b)) * mag(sub(zVanish, t));
    double imageCrossRatio;

    printf("referenceHeight = %f\n", referenceHeight);
    printf("numerator = %f\n", numerator);
    printf("denominator = %f\n", denominator);

    if (denominator != 0) imageCrossRatio = referenceHeight * numerator / denominator;
    else imageCrossRatio = 0;

    newPoint.X = knownPoint.X;
    newPoint.Y = knownPoint.Y;
    if (dot(sub(t, b), sub(r, b)) < 0) newPoint.Z = -imageCrossRatio;
    else newPoint.Z = imageCrossRatio;

    printf("newPoint.X = %f\n", newPoint.X);
    printf("newPoint.Y = %f\n", newPoint.Y);
    printf("newPoint.Z = %f\n", newPoint.Z);

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
    printf("newPoint.u = %f\n", newPoint.u);
    printf("newPoint.v = %f\n", newPoint.v);
    printf("newPoint.w = %f\n", newPoint.w);
    printf("newPoint.X = %f\n", newPoint.X);
    printf("newPoint.Y = %f\n", newPoint.Y);
    printf("newPoint.Z = %f\n", newPoint.Z);
    if (knownPoint.Z == 0) {
        transformInv(Hinv, &newPoint);
        newPoint.Z = knownPoint.Z;
    }
    else {
        SVMPoint v = cross(cross(knownPoint, newPoint), cross(xVanish, yVanish));

        SVMPoint b1;
        b1.X = knownPoint.X;
        b1.Y = knownPoint.Y;
        b1.Z = 1;
        transform(H, &b1);

        SVMPoint b0 = cross(cross(b1, v), cross(zVanish, newPoint));
        if (b0.w != 0) {
            b0.u = b0.u / b0.w;
            b0.v = b0.v / b0.w;
        }
        b0.w = 1;
        transformInv(Hinv, &b0);

        newPoint.X = b0.X;
        newPoint.Y = b0.Y;
        newPoint.Z = knownPoint.Z;
    }
    printf("newPoint.u = %f\n", newPoint.u);
    printf("newPoint.v = %f\n", newPoint.v);
    printf("newPoint.w = %f\n", newPoint.w);
    printf("newPoint.X = %f\n", newPoint.X);
    printf("newPoint.Y = %f\n", newPoint.Y);
    printf("newPoint.Z = %f\n", newPoint.Z);

	/******** END TODO ********/

	newPoint.known(true);

	printf( "Calculated new coordinates for point: (%e, %e, %e)\n", newPoint.X, newPoint.Y, newPoint.Z );

	redraw();
}


