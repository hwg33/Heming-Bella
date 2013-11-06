///////////////////////////////////////////////////////////////////////////
//
// NAME
//  BlendImages.cpp -- blend together a set of overlapping images
//
// DESCRIPTION
//  This routine takes a collection of images aligned more or less horizontally
//  and stitches together a mosaic.
//
//  The images can be blended together any way you like, but I would recommend
//  using a soft halfway blend of the kind Steve presented in the first lecture.
//
//  Once you have blended the images together, you should crop the resulting
//  mosaic at the halfway points of the first and last image.  You should also
//  take out any accumulated vertical drift using an affine warp.
//  Lucas-Kanade Taylor series expansion of the registration error.
//
// SEE ALSO
//  BlendImages.h       longer description of parameters
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
// (modified for CSE455 Winter 2003)
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "BlendImages.h"
#include <float.h>
#include <math.h>

#define MAX(x,y) (((x) < (y)) ? (y) : (x))
#define MIN(x,y) (((x) < (y)) ? (x) : (y))

/* Return the closest integer to x, rounding up */
static int iround(double x) {
    if (x < 0.0) {
        return (int) (x - 0.5);
    } else {
        return (int) (x + 0.5);
    }
}

void ImageBoundingBox(CImage &image, CTransform3x3 &M, 
    int &min_x, int &min_y, int &max_x, int &max_y)
{
    // This is a useful helper function that you might choose to implement
    // takes an image, and a transform, and computes the bounding box of the
    // transformed image.
printf("TODO: %s:%d\n", __FILE__, __LINE__); 

}


/******************* TO DO *********************
* AccumulateBlend:
*	INPUT:
*		img: a new image to be added to acc
*		acc: portion of the accumulated image where img is to be added
*      M: the transformation mapping the input image 'img' into the output panorama 'acc'
*		blendWidth: width of the blending function (horizontal hat function;
*	    try other blending functions for extra credit)
*	OUTPUT:
*		add a weighted copy of img to the subimage specified in acc
*		the first 3 band of acc records the weighted sum of pixel colors
*		the fourth band of acc records the sum of weight
*/
static void AccumulateBlend(CByteImage& img, CFloatImage& acc, CTransform3x3 M, float blendWidth)
{
    // BEGIN TODO
    // Fill in this routine
    /*
    int w = img.Shape().width;
    int h = img.Shape().height;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            CVector3 p;
            p[0] = x;
            p[1] = y;
            p[2] = 1;
            p = M * p;
            int m_x = iround(p[0] / p[2]);
            int m_y = iround(p[1] / p[2]);

            float alpha;
            if (x < blendWidth) alpha = static_cast<float>(x) / blendWidth;
            else if (x > w - 1 - blendWidth) alpha = static_cast<float>(w - 1 - x) / blendWidth;
            else alpha = 1.0;
            img.Pixel(x, y, img.alphaChannel) = iround(255.0 * alpha);

            if (img.Pixel(x, y, 0) != 0 || img.Pixel(x, y, 1) != 0 || img.Pixel(x, y, 2) != 0) {
                acc.Pixel(m_x, m_y, 0) += img.Pixel(x, y, 0) * img.Pixel(x, y, img.alphaChannel);
                acc.Pixel(m_x, m_y, 1) += img.Pixel(x, y, 1) * img.Pixel(x, y, img.alphaChannel);
                acc.Pixel(m_x, m_y, 2) += img.Pixel(x, y, 2) * img.Pixel(x, y, img.alphaChannel);
                acc.Pixel(m_x, m_y, acc.alphaChannel) += img.Pixel(x, y, img.alphaChannel);
            }
        }
    }
    */
    int w = acc.Shape().width;
    int h = acc.Shape().height;
    int imgW = img.Shape().width;
    int imgH = img.Shape().height;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            CVector3 p;
            p[0] = x;
            p[1] = y;
            p[2] = 1;
            p = M.Inverse() * p;
            int m_x = iround(p[0] / p[2]);
            int m_y = iround(p[1] / p[2]);
            if (m_x >= 0 && m_x < imgW && m_y >= 0 && m_y < imgH) {
                float alpha;
                if (m_x < blendWidth) alpha = static_cast<float>(m_x) / blendWidth;
                else if (m_x > imgW - 1 - blendWidth) alpha = static_cast<float>(imgW - 1 - m_x) / blendWidth;
                else alpha = 1.0;
                img.Pixel(m_x, m_y, img.alphaChannel) = iround(255.0 * alpha);

                if (img.Pixel(m_x, m_y, 0) != 0 || img.Pixel(m_x, m_y, 1) != 0 || img.Pixel(m_x, m_y, 2) != 0) {
                    acc.Pixel(x, y, 0) += img.Pixel(m_x, m_y, 0) * img.Pixel(m_x, m_y, img.alphaChannel);
                    acc.Pixel(x, y, 1) += img.Pixel(m_x, m_y, 1) * img.Pixel(m_x, m_y, img.alphaChannel);
                    acc.Pixel(x, y, 2) += img.Pixel(m_x, m_y, 2) * img.Pixel(m_x, m_y, img.alphaChannel);
                    acc.Pixel(x, y, acc.alphaChannel) += img.Pixel(m_x, m_y, img.alphaChannel);
                }
            }
        }
    }
    // END TODO
}



/******************* TO DO 5 *********************
* NormalizeBlend:
*	INPUT:
*		acc: input image whose alpha channel (4th channel) contains
*		     normalizing weight values
*		img: where output image will be stored
*	OUTPUT:
*		normalize r,g,b values (first 3 channels) of acc and store it into img
*/
static void NormalizeBlend(CFloatImage& acc, CByteImage& img)
{
    // BEGIN TODO
    // fill in this routine..
    int w = acc.Shape().width;
    int h = acc.Shape().height;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            if (acc.Pixel(x, y, acc.alphaChannel) != 0) {
                img.Pixel(x, y, 0) = iround(acc.Pixel(x, y, 0) / acc.Pixel(x, y, acc.alphaChannel));
                img.Pixel(x, y, 1) = iround(acc.Pixel(x, y, 1) / acc.Pixel(x, y, acc.alphaChannel));
                img.Pixel(x, y, 2) = iround(acc.Pixel(x, y, 2) / acc.Pixel(x, y, acc.alphaChannel));
            }
            img.Pixel(x, y, img.alphaChannel) = 255;
        }
    }
    // END TODO
}



/******************* TO DO 5 *********************
* BlendImages:
*	INPUT:
*		ipv: list of input images and their relative positions in the mosaic
*		blendWidth: width of the blending function
*	OUTPUT:
*		create & return final mosaic by blending all images
*		and correcting for any vertical drift
*/
CByteImage BlendImages(CImagePositionV& ipv, float blendWidth)
{
    printf("BlendImages\n");
    // Assume all the images are of the same shape (for now)
    CByteImage& img0 = ipv[0].img;
    CShape sh        = img0.Shape();
    int width        = sh.width;
    int height       = sh.height;
    int nBands       = sh.nBands;
    // int dim[2]       = {width, height};

    int n = ipv.size();
    if (n == 0) return CByteImage(0,0,1);

    bool is360 = false;

    // Hack to detect if this is a 360 panorama
    if (ipv[0].imgName == ipv[n-1].imgName) is360 = true;

    for (int i = 0; i < n; i++) {
        CTransform3x3 &M = ipv[i].position;
        printf("%f %f %f\n", M[0][0], M[0][1], M[0][2]);
        printf("%f %f %f\n", M[1][0], M[1][1], M[1][2]);
        printf("%f %f %f\n", M[2][0], M[2][1], M[2][2]);
        printf("-----------------------------------\n");
    }

    printf("Starting to compute bounding box\n");
    // Compute the bounding box for the mosaic
    float min_x = FLT_MAX, min_y = FLT_MAX;
    float max_x = 0, max_y = 0;
    int i;
    for (i = 0; i < n; i++)
    {
        CTransform3x3 &T = ipv[i].position;

        // BEGIN TODO
        // add some code here to update min_x, ..., max_y
        CVector3 topLeft, bottomRight;
        topLeft[0] = 0;
        topLeft[1] = 0;
        topLeft[2] = 1;
        bottomRight[0] = width - 1;
        bottomRight[1] = height - 1;
        bottomRight[2] = 1;
        topLeft = T * topLeft;
        bottomRight = T * bottomRight;
        min_x = MIN(topLeft[0], min_x);
        min_y = MIN(topLeft[1], min_y);
        max_x = MAX(bottomRight[0], max_x);
        max_y = MAX(bottomRight[1], max_y);
        // END TODO
    }
    printf("Computed bounding box\n");
    // Create a floating point accumulation image
    //printf("-%d-, -%d-\n", (int)(ceil(max_x) - floor(min_x)), (int)(ceil(max_y) - floor(min_y)));
    CShape mShape((int)(ceil(max_x) - floor(min_x)),
        (int)(ceil(max_y) - floor(min_y)), nBands + 1);
    CFloatImage accumulator(mShape);
    accumulator.ClearPixels();

    double x_init, x_final;
    double y_init, y_final;

    // Add in all of the images
    for (i = 0; i < n; i++) {
        // Compute the sub-image involved
        CTransform3x3 &M = ipv[i].position;
        //printf("(%f, %f)\n", min_x, min_y);
        //printf("(%f, %f)\n", max_x, max_y);
        /*
        printf("%f %f %f\n", M[0][0], M[0][1], M[0][2]);
        printf("%f %f %f\n", M[1][0], M[1][1], M[1][2]);
        printf("%f %f %f\n", M[2][0], M[2][1], M[2][2]);
        printf("-----------------------\n");
        */
        CTransform3x3 M_t = CTransform3x3::Translation(-min_x, -min_y) * M;
        /*
        printf("/ %f %f %f\n", M_t[0][0], M_t[0][1], M_t[0][2]);
        printf("/ %f %f %f\n", M_t[1][0], M_t[1][1], M_t[1][2]);
        printf("/ %f %f %f\n", M_t[2][0], M_t[2][1], M_t[2][2]);
        printf("-----------------------\n");
        */
        CByteImage& img = ipv[i].img;
        printf("Starting accumulating\n");
        // Perform the accumulation
        AccumulateBlend(img, accumulator, M_t, blendWidth);
        printf("Done accumulating\n");
        if (i == 0) {
            CVector3 p;
            p[0] = 0.5 * width;
            p[1] = 0.0;
            p[2] = 1.0;

            p = M_t * p;
            x_init = p[0];
            y_init = p[1];
        } else if (i == n - 1) {
            CVector3 p;
            p[0] = 0.5 * width;
            p[1] = 0.0;
            p[2] = 1.0;

            p = M_t * p;
            x_final = p[0];
            y_final = p[1];
        }
    }

    // Normalize the results
    mShape = CShape((int)(ceil(max_x) - floor(min_x)),
        (int)(ceil(max_y) - floor(min_y)), nBands);

    CByteImage compImage(mShape);
    NormalizeBlend(accumulator, compImage);
    bool debug_comp = false;
    if (debug_comp)
        WriteFile(compImage, "tmp_comp.tga");

    // Allocate the final image shape
    int outputWidth = 0;
    if (is360) {
        outputWidth = mShape.width - width;
    } else {
        outputWidth = mShape.width;
    }

    CShape cShape(outputWidth, mShape.height, nBands);

    CByteImage croppedImage(cShape);

    // Compute the affine transformation
    CTransform3x3 A = CTransform3x3(); // identify transform to initialize

    // BEGIN TODO
    // fill in appropriate entries in A to trim the left edge and
    // to take out the vertical drift if this is a 360 panorama
    // (i.e. is360 is true)
    if (is360) {
        A[1][0] = (y_final - y_init) / (x_final - x_init);
        A[0][2] = width;
    }

    // END TODO

    // Warp and crop the composite
    WarpGlobal(compImage, croppedImage, A, eWarpInterpLinear);

    return croppedImage;
}

