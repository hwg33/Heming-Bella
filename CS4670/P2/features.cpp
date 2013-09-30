#include <assert.h>
#include <math.h>
#include <FL/Fl.H>
#include <FL/Fl_Image.H>
#include "features.h"
#include "ImageLib/FileIO.h"

#define PI 3.14159265358979323846

// Compute features of an image.
bool computeFeatures(CFloatImage &image, FeatureSet &features, int featureType, int descriptorType) {
    // TODO: Instead of calling dummyComputeFeatures, implement
    // a Harris corner detector along with a MOPS descriptor.  
    // This step fills in "features" with information necessary 
    // for descriptor computation.

    switch (featureType) {
    case 1:
        dummyComputeFeatures(image, features);
        break;
    case 2:
        ComputeHarrisFeatures(image, features);
        break;
    default:
        return false;
    }

    // TODO: You will implement two descriptors for this project
    // (see webpage).  This step fills in "features" with
    // descriptors.  The third "custom" descriptor is extra credit.
    switch (descriptorType) {
    case 1:
        ComputeSimpleDescriptors(image, features);
        break;
    case 2:
        ComputeMOPSDescriptors(image, features);
        break;
    case 3:
        ComputeCustomDescriptors(image, features);
        break;
    default:
        return false;
    }

    // This is just to make sure the IDs are assigned in order, because
    // the ID gets used to index into the feature array.
    for (unsigned int i=0; i<features.size(); i++) {
        features[i].id = i;
    }

    return true;
}

// Perform a query on the database.  This simply runs matchFeatures on
// each image in the database, and returns the feature set of the best
// matching image.
bool performQuery(const FeatureSet &f, const ImageDatabase &db, int &bestIndex, vector<FeatureMatch> &bestMatches, double &bestDistance, int matchType) {
    vector<FeatureMatch> tempMatches;

    for (unsigned int i=0; i<db.size(); i++) {
        if (!matchFeatures(f, db[i].features, tempMatches, matchType)) {
            return false;
        }

        bestIndex = i;
        bestMatches = tempMatches;
    }

    return true;
}

// Match one feature set with another.
bool matchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, int matchType) {

    // TODO: We have provided you the SSD matching function; you must write your own
    // feature matching function using the ratio test.

    printf("\nMatching features.......\n");

    switch (matchType) {
    case 1:
        ssdMatchFeatures(f1, f2, matches);
        return true;
    case 2:
        ratioMatchFeatures(f1, f2, matches);
        return true;
    default:
        return false;
    }
}

// Compute silly example features.  This doesn't do anything
// meaningful, but may be useful to use as an example.
void dummyComputeFeatures(CFloatImage &image, FeatureSet &features) {
    CShape sh = image.Shape();
    Feature f;

    for (int y=0; y<sh.height; y++) {
        for (int x=0; x<sh.width; x++) {
            double r = image.Pixel(x,y,0);
            double g = image.Pixel(x,y,1);
            double b = image.Pixel(x,y,2);

            if ((int)(255*(r+g+b)+0.5) % 100 == 1) {
                // If the pixel satisfies this meaningless criterion,
                // make it a feature.

                f.type = 1;
                f.id += 1;
                f.x = x;
                f.y = y;

                f.data.resize(1);
                f.data[0] = r + g + b;

                features.push_back(f);
            }
        }
    }
}

void ComputeHarrisFeatures(CFloatImage &image, FeatureSet &features)
{
    //Create grayscale image used for Harris detection
    CFloatImage grayImage = ConvertToGray(image);

    //Create image to store Harris values
    CFloatImage harrisImage(image.Shape().width, image.Shape().height, 1);

    //Create image to store local maximum harris values as 1, other pixels 0
    CByteImage harrisMaxImage(image.Shape().width, image.Shape().height, 1);

    CFloatImage orientationImage(image.Shape().width, image.Shape().height, 1);

    // computeHarrisValues() computes the harris score at each pixel position, storing the
    // result in in harrisImage. 
    // You'll need to implement this function.
    computeHarrisValues(grayImage, harrisImage, orientationImage);

    // Threshold the harris image and compute local maxima.  You'll need to implement this function.
    computeLocalMaxima(harrisImage, harrisMaxImage);

    CByteImage tmp(harrisImage.Shape());
    convertToByteImage(harrisImage, tmp);
    WriteFile(tmp, "harris.tga");
    // WriteFile(harrisMaxImage, "harrisMax.tga");

    // Loop through feature points in harrisMaxImage and fill in information needed for 
    // descriptor computation for each point above a threshold. You need to fill in id, type, 
    // x, y, and angle.
    int id = 0;
    for (int y=0; y < harrisMaxImage.Shape().height; y++) {
        for (int x=0; x < harrisMaxImage.Shape().width; x++) {
            if (harrisMaxImage.Pixel(x, y, 0) == 0)
                continue;
            Feature f;

            f.id = id;
            f.type = 2;
            f.x = x;
            f.y = y;
            f.angleRadians = orientationImage.Pixel(x, y, 0);

            features.push_back(f);
            id++;
        }
    }
}



//TO DO---------------------------------------------------------------------
// Loop through the image to compute the harris corner values as described in class
// srcImage:  grayscale of original image
// harrisImage:  populate the harris values per pixel in this image
void computeHarrisValues(CFloatImage &srcImage, CFloatImage &harrisImage, CFloatImage &orientationImage)
{
    int w = srcImage.Shape().width;
    int h = srcImage.Shape().height;
    
    CFloatImage xDerivative(w, h, 1);
    CFloatImage yDerivative(w, h, 1);
    Convolve(srcImage, xDerivative, ConvolveKernel_SobelX);
    Convolve(srcImage, yDerivative, ConvolveKernel_SobelY);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double a = 0, b = 0, c = 0;
            for (int i = -2; i < 3; i++) {
                for (int j = -2; j < 3; j++) {
                    if (x + i < w && x + i >= 0 && y + j < h && y + j >= 0) {
                        double ix = xDerivative.Pixel(x + i, y + j, 0);
                        double iy = yDerivative.Pixel(x + i, y + j, 0);
                        a += gaussian5x5[i + 2 + (j + 2) * 5] * ix * ix;
                        b += gaussian5x5[i + 2 + (j + 2) * 5] * ix * iy;
                        c += gaussian5x5[i + 2 + (j + 2) * 5] * iy * iy;
                    }
                }
            }
            double det = a*c - b*b;
            double trc = a+c;
            if (trc == 0) harrisImage.Pixel(x, y, 0) = 0;
            else harrisImage.Pixel(x, y, 0) = det / trc;
/*
            double eig = trc / 2 + sqrt(trc * trc / 4 - det);
            double ang;

            if (b != 0) ang = atan2(b, eig - c);
            else ang = atan2(0, 1);
            orientationImage.Pixel(x, y, 0) = ang;
*/

            double gradX = xDerivative.Pixel(x, y, 0);
            double gradY = yDerivative.Pixel(x, y, 0);
            //double mag = sqrt(gradX * gradX + gradY * gradY);
            //gradX = gradX / mag;
            //gradY = gradY / mag;
            orientationImage.Pixel(x, y, 0) = -atan2(gradY, gradX);
            /*
            double discriminant = (a + c)*(a + c) - 4*(a*c - b*b);
            if (discriminant < 0) orientationImage.Pixel(x, y, 0) = 0;
            else {
                double eigenValue = 0.5*(a + c + sqrt(discriminant));
                double eigenVectorX = -b / (a - eigenValue);
                orientationImage.Pixel(x, y, 0) = atan2(1, eigenVectorX);
            }*/
        }
    }
}



//TO DO---------------------------------------------------------------------
//Loop through the image to compute the harris corner values as described in class
// srcImage:  image with Harris values
// destImage: Assign 1 to local maximum in 3x3 window, 0 otherwise
void computeLocalMaxima(CFloatImage &srcImage,CByteImage &destImage)
{
    int w = srcImage.Shape().width;
    int h = srcImage.Shape().height;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double localMax = std::numeric_limits<double>::min();
            double maxX = 0, maxY = 0;
            for (int i = -2; i < 3; i++) {
                for (int j = -2; j < 3; j++) {
                    if (x + i < w && x + i >= 0 && y + j < h && y + j >= 0) {
                        double pixel = srcImage.Pixel(x + i, y + j, 0);
                        if (pixel > localMax) {
                            localMax = pixel;
                            maxX = i;
                            maxY = j;
                        }
                    }
                }
            }
            if (maxX == 0 && maxY == 0 && localMax > 0.0172) destImage.Pixel(x, y, 0) = 1;
            else destImage.Pixel(x, y, 0) = 0;
        }
    }

}

// TODO: Implement parts of this function
// Compute Simple descriptors.
void ComputeSimpleDescriptors(CFloatImage &image, FeatureSet &features)
{
    //Create grayscale image used for Harris detection
    CFloatImage grayImage=ConvertToGray(image);

    for (vector<Feature>::iterator i = features.begin(); i != features.end(); i++) {
        Feature &f = *i;

        int x = f.x;
        int y = f.y;

        f.data.resize(5 * 5);
        int k = 0;
        for (int j = -2; j < 3; j++) {
            for (int i = -2; i < 3; i++) {
                if (x+i >= 0 && x+i < grayImage.Shape().width && y+j >= 0 && y+j < grayImage.Shape().height)
                    f.data[(j+2)*5 + i+2] = grayImage.Pixel(x+i, y+j, 0);
            }
        }

    }
}

// TODO: Implement parts of this function
// Compute MOPs descriptors.
void ComputeMOPSDescriptors(CFloatImage &image, FeatureSet &features)
{
    // This image represents the window around the feature you need to compute to store as the feature descriptor
    const int windowSize = 8;
    CFloatImage destImage(windowSize, windowSize, 1);
    CFloatImage grayImage = ConvertToGray(image);

    CByteImage tmp(grayImage.Shape());
    convertToByteImage(grayImage, tmp);
    WriteFile(tmp, "grayImage.tga");

    CFloatImage blurredImage(grayImage.Shape().width, grayImage.Shape().height, 1);

    CFloatImage kernel(5, 5, 1);



    kernel.origin[0] = 2;
    kernel.origin[1] = 2;

    for (int i = 0; i< 5; i++){
        for(int j = 0; j<5; j++){
            kernel.Pixel(i,j,0) = gaussian5x5[i + j*5];
        }
    }
    Convolve(grayImage, blurredImage, kernel);
    CByteImage tmp2(blurredImage.Shape());
    convertToByteImage(blurredImage, tmp2);
    WriteFile(tmp2, "blurredImage.tga");
    /*
    for(int i = 0; i < blurredImage.Shape().width; i++){
        for(int j = 0; j < blurredImage.Shape().height; j++){
            printf("%f", blurredImage.Pixel(i,j,0));
        }
    }
    */
   int num = 0;
    for (vector<Feature>::iterator i = features.begin(); i != features.end(); i++) {
        Feature &f = *i;

        //TODO: Compute the inverse transform as described by the feature location/orientation.
        //You'll need to compute the transform from each pixel in the 8x8 image 
        //to sample from the appropriate pixels in the 40x40 rotated window surrounding the feature
        CTransform3x3 xform;
        CTransform3x3 scale;
        scale[0][0] = 5;
        scale[1][1] = 5;
        scale[2][2] = 1;
        double angle = f.angleRadians;

        CTransform3x3 center;
        center = center.Translation(-4, -4);
        scale = scale * center;
        CTransform3x3 rotate;
        rotate = scale.Rotation(angle * 180 / PI);
        xform = rotate.Translation(f.x, f.y);

        //Call the Warp Global function to do the mapping
        WarpGlobal(blurredImage, destImage, xform, eWarpInterpLinear);

        f.data.resize(windowSize * windowSize);
        for(int i = 0; i < windowSize; i++){
            for(int j = 0; j < windowSize; j++){
                f.data[i + j*windowSize] = destImage.Pixel(i,j,0);
                //printf("%f", f.data[k]);
            }
        }
        double sum = 0.0;
        for(int i  = 0; i<windowSize*windowSize; i++){
            sum += f.data[i];
        }
        double mean = sum/(windowSize*windowSize);
        //printf("mean: %f\n", mean);
        sum = 0.0;
        for(int i = 0; i<windowSize*windowSize; i++){
            sum += (f.data[i] - mean)*(f.data[i] - mean);
        }
        double standard_deviation = sqrt(sum);
        //printf("std: %f\n", standard_deviation);
        
        for(int i = 0; i<windowSize*windowSize; i++){
            if(standard_deviation != 0){
            f.data[i] = (f.data[i] - mean )/standard_deviation;
            } else {
                f.data[i] = (f.data[i] - mean )/0.0001;
            }
            //printf("%f\n",f.data[i]);
        }
     num++;
    }
}

// Compute Custom descriptors (extra credit)
void ComputeCustomDescriptors(CFloatImage &image, FeatureSet &features)
{

}

// Perform simple feature matching.  This just uses the SSD
// distance between two feature vectors, and matches a feature in the
// first image with the closest feature in the second image.  It can
// match multiple features in the first image to the same feature in
// the second image.
void ssdMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches) {
    int m = f1.size();
    int n = f2.size();

    matches.resize(m);

    double d;
    double dBest;
    int idBest;

    for (int i=0; i<m; i++) {
        dBest = 1e100;
        idBest = 0;

        for (int j=0; j<n; j++) {
            d = distanceSSD(f1[i].data, f2[j].data);

            if (d < dBest) {
                dBest = d;
                idBest = f2[j].id;
            }
        }

        matches[i].id1 = f1[i].id;
        matches[i].id2 = idBest;
        matches[i].distance = dBest;
    }
}

double abs(double d){
    if (d < 0) return -d;
    else return d;
}

//TODO: Write this function to perform ratio feature matching.  
// This just uses the ratio of the SSD distance of the two best matches
// and matches a feature in the first image with the closest feature in the second image.
// It can match multiple features in the first image to the same feature in
// the second image.  (See class notes for more information)
// You don't need to threshold matches in this function -- just specify the match distance
// in each FeatureMatch object, as well as the ids of the two matched features (see
// ssdMatchFeatures for reference).
void ratioMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches) 
{
    int m = f1.size();
    int n = f2.size();
    
    matches.resize(m);
    
    double d;
    double first;
    int first_id;
    double second;
    int second_id;
    int first_f;
    int second_f;
    
    for (int i = 0; i<m; i++){
        first = 1e100;
        second = 1e100;
        first_id = 0;
        second_id = 0;
        first_f = 0;
        second_f = 0;
        
        for(int j = 0; j<n; j++){
            d = distanceSSD(f1[i].data, f2[j].data);
            if (d < first){
                second_f = first_f;
                second = first;
                second_id = first_id;
                first_f = j;
                first = d;
                first_id = f2[j].id;
            } else if( first <= d && d < second){
                second_f = j;
                second = d;
                second_id = f2[j].id;
            }
        }
        double norm_1;
        for(int k; k< f1[i].data.size(); k++){
            norm_1 += abs(f1[i].data[k] - f2[first_f].data[k]);
        }
        double norm_2;
        for(int k; k< f1[i].data.size(); k++){
            norm_2 += abs(f1[i].data[k] - f2[second_f].data[k]);
        }
        matches[i].id1 = f1[i].id;
        matches[i].id2 = first_id;
        matches[i].distance = norm_1/norm_2;
    }

}


// Convert Fl_Image to CFloatImage.
bool convertImage(const Fl_Image *image, CFloatImage &convertedImage) {
    if (image == NULL) {
        return false;
    }

    // Let's not handle indexed color images.
    if (image->count() != 1) {
        return false;
    }

    int w = image->w();
    int h = image->h();
    int d = image->d();

    // Get the image data.
    const char *const *data = image->data();

    int index = 0;

    for (int y=0; y<h; y++) {
        for (int x=0; x<w; x++) {
            if (d < 3) {
                // If there are fewer than 3 channels, just use the
                // first one for all colors.
                convertedImage.Pixel(x,y,0) = ((uchar) data[0][index]) / 255.0f;
                convertedImage.Pixel(x,y,1) = ((uchar) data[0][index]) / 255.0f;
                convertedImage.Pixel(x,y,2) = ((uchar) data[0][index]) / 255.0f;
            }
            else {
                // Otherwise, use the first 3.
                convertedImage.Pixel(x,y,0) = ((uchar) data[0][index]) / 255.0f;
                convertedImage.Pixel(x,y,1) = ((uchar) data[0][index+1]) / 255.0f;
                convertedImage.Pixel(x,y,2) = ((uchar) data[0][index+2]) / 255.0f;
            }

            index += d;
        }
    }

    return true;
}

// Convert CFloatImage to CByteImage.
void convertToByteImage(CFloatImage &floatImage, CByteImage &byteImage) {
    CShape sh = floatImage.Shape();

    assert(floatImage.Shape().nBands == byteImage.Shape().nBands);
    for (int y=0; y<sh.height; y++) {
        for (int x=0; x<sh.width; x++) {
            for (int c=0; c<sh.nBands; c++) {
                float value = floor(255*floatImage.Pixel(x,y,c) + 0.5f);

                if (value < byteImage.MinVal()) {
                    value = byteImage.MinVal();
                }
                else if (value > byteImage.MaxVal()) {
                    value = byteImage.MaxVal();
                }

                // We have to flip the image and reverse the color
                // channels to get it to come out right.  How silly!
                byteImage.Pixel(x,sh.height-y-1,sh.nBands-c-1) = (uchar) value;
            }
        }
    }
}

// Compute SSD distance between two vectors.
double distanceSSD(const vector<double> &v1, const vector<double> &v2) {
    int m = v1.size();
    int n = v2.size();

    if (m != n) {
        // Here's a big number.
        return 1e100;
    }

    double dist = 0;

    for (int i=0; i<m; i++) {
        dist += pow(v1[i]-v2[i], 2);
    }


    return sqrt(dist);
}

// Transform point by homography.
void applyHomography(double x, double y, double &xNew, double &yNew, double h[9]) {
    double d = h[6]*x + h[7]*y + h[8];

    xNew = (h[0]*x + h[1]*y + h[2]) / d;
    yNew = (h[3]*x + h[4]*y + h[5]) / d;
}

// Evaluate a match using a ground truth homography.  This computes the
// average SSD distance between the matched feature points and
// the actual transformed positions.
double evaluateMatch(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9]) {
    double d = 0;
    int n = 0;

    double xNew;
    double yNew;

    unsigned int num_matches = matches.size();
    for (unsigned int i=0; i<num_matches; i++) {
        int id1 = matches[i].id1;
        int id2 = matches[i].id2;
        applyHomography(f1[id1].x, f1[id1].y, xNew, yNew, h);
        d += sqrt(pow(xNew-f2[id2].x,2)+pow(yNew-f2[id2].y,2));
        n++;
    }	

    return d / n;
}

void addRocData(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9],
    vector<bool> &isMatch, double threshold, double &maxD) 
{
    double d = 0;

    double xNew;
    double yNew;

    unsigned int num_matches = matches.size();
    for (unsigned int i=0; i<num_matches; i++) {
        int id1 = matches[i].id1;
        int id2 = matches[i].id2;
        applyHomography(f1[id1].x, f1[id1].y, xNew, yNew, h);

        // Ignore unmatched points.  There might be a better way to
        // handle this.
        d = sqrt(pow(xNew-f2[id2].x,2)+pow(yNew-f2[id2].y,2));
        if (d<=threshold) {
            isMatch.push_back(1);
        } else {
            isMatch.push_back(0);
        }

        if (matches[i].distance > maxD)
            maxD = matches[i].distance;
    }	
}

vector<ROCPoint> computeRocCurve(vector<FeatureMatch> &matches,vector<bool> &isMatch,vector<double> &thresholds)
{
    vector<ROCPoint> dataPoints;

    for (int i=0; i < (int)thresholds.size();i++)
    {
        //printf("Checking threshold: %lf.\r\n",thresholds[i]);
        int tp=0;
        int actualCorrect=0;
        int fp=0;
        int actualError=0;
        int total=0;

        int num_matches = (int) matches.size();
        for (int j=0;j < num_matches;j++) {
            if (isMatch[j]) {
                actualCorrect++;
                if (matches[j].distance < thresholds[i]) {
                    tp++;
                }
            } else {
                actualError++;
                if (matches[j].distance < thresholds[i]) {
                    fp++;
                }
            }

            total++;
        }

        ROCPoint newPoint;
        //printf("newPoints: %lf,%lf",newPoint.trueRate,newPoint.falseRate);
        newPoint.trueRate=(double(tp)/actualCorrect);
        newPoint.falseRate=(double(fp)/actualError);
        //printf("newPoints: %lf,%lf",newPoint.trueRate,newPoint.falseRate);

        dataPoints.push_back(newPoint);
    }

    return dataPoints;
}



// Compute AUC given a ROC curve
double computeAUC(vector<ROCPoint> &results)
{
    double auc=0;
    double xdiff,ydiff;
    for (int i = 1; i < (int) results.size(); i++)
    {
        //fprintf(stream,"%lf\t%lf\t%lf\n",thresholdList[i],results[i].falseRate,results[i].trueRate);
        xdiff=(results[i].falseRate-results[i-1].falseRate);
        ydiff=(results[i].trueRate-results[i-1].trueRate);
        auc=auc+xdiff*results[i-1].trueRate+xdiff*ydiff/2;

    }
    return auc;
}

