#include "correlation.h"

inline unsigned char PIXEL(const unsigned char* p, int i, int j, int c, int width)
{
    return *(p + 3 * (j * width + i) + c);
}
/************************ TODO 2 **************************/
/*
 *	INPUT:
 *		origImg:		the original image,
 *		imgWidth:		the width of the image
 *		imgHeight:		the height of the image
 *						the image is arranged such that
 *						origImg[3*(row*imgWidth+column)+0],
 *						origImg[3*(row*imgWidth+column)+1],
 *						origImg[3*(row*imgWidth+column)+2]
 *						are R, G, B values for pixel at (column, row).
 *
 *      kernel:			the 2D filter kernel,
 *		knlWidth:		the width of the kernel
 *		knlHeight:		the height of the kernel
 *
 *		scale, offset:  after correlating the kernel with the origImg,
 *						each pixel should be divided by scale and then added by offset
 *
 *		selection:      a byte array of the same size as the image,
 *						indicating where in the original image should be filtered, e.g.,
 *						selection[k] == 1 ==> pixel k should be filtered
 *                      selection[k] == 0 ==> pixel k should NOT be filtered
 *                      a special case is selection is a NULL pointer, which means all the pixels should be filtered.
 *
 *  OUTPUT:
 *		rsltImg:		the filtered image of the same size as original image.
 *						it is a valid pointer ( allocated already ).
 */

void image_filter(double* rsltImg, const unsigned char* origImg, const unsigned char* selection,
                  int imgWidth, int imgHeight,
                  const double* kernel, int knlWidth, int knlHeight,
                  double scale, double offset)
{
    for (int i = 0; i < imgWidth; i++){
        for (int j = 0; j < imgHeight; j++){
            double rsltPixel[3];
            pixel_filter(rsltPixel, i, j, origImg, imgWidth, imgHeight, kernel, knlWidth, knlHeight, scale, offset);
            rsltImg[j * imgWidth + i] = rsltPixel[0];
            rsltImg[j * imgWidth + i + 1] = rsltPixel[1];
            rsltImg[j * imgWidth + i + 2] = rsltPixel[2];
        }
    }


}

/************************ END OF TODO 2 **************************/


/************************ TODO 3 **************************/
/*
 *	INPUT:
 *      x:				a column index,
 *      y:				a row index,
 *		origImg:		the original image,
 *		imgWidth:		the width of the image
 *		imgHeight:		the height of the image
 *						the image is arranged such that
 *						origImg[3*(row*imgWidth+column)+0],
 *						origImg[3*(row*imgWidth+column)+1],
 *						origImg[3*(row*imgWidth+column)+2]
 *						are R, G, B values for pixel at (column, row).
 *
 *      kernel:			the 2D filter kernel,
 *		knlWidth:		the width of the kernel
 *		knlHeight:		the height of the kernel
 *
 *		scale, offset:  after correlating the kernel with the origImg,
 *						the result pixel should be divided by scale and then added by offset
 *
 *  OUTPUT:
 *		rsltPixel[0], rsltPixel[1], rsltPixel[2]:
 *						the filtered pixel R, G, B values at row y, column x;
 */

void pixel_filter(double rsltPixel[3], int x, int y, const unsigned char* origImg, int imgWidth, int imgHeight,
                  const double* kernel, int knlWidth, int knlHeight,
                  double scale, double offset)
{

    for (int i = -knlWidth/2; i <= (knlWidth-1)/2; i++) {
        for (int j = -knlHeight/2; j <= (knlHeight-1)/2; j++) {
            int ki = i + knlWidth/2;
            int kj = j + knlHeight/2;
            //pad boundary pixels with 0's
            if (x + i < imgWidth && x + i >= 0 && y + j < imgHeight && y + j >= 0) {
                rsltPixel[0] += (kernel[ki+kj*knlWidth] * PIXEL(origImg, x + i, y + j, 0, imgWidth)) / scale + offset;
                rsltPixel[1] += (kernel[ki+kj*knlWidth] * PIXEL(origImg, x + i, y + j, 1, imgWidth)) / scale + offset;
                rsltPixel[2] += (kernel[ki+kj*knlWidth] * PIXEL(origImg, x + i, y + j, 2, imgWidth)) / scale + offset;
            }
        }
    }
}

/************************ END OF TODO 3 **************************/

