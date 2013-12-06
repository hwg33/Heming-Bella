#include "Feature.h"
#define PI 3.14159265359

static
double
_round(double d)
{
    return floor(d + 0.5);
}

CByteImage
FeatureExtractor::renderPosNegComponents(const Feature &feat) const
{
    // Create two images, one for the positive weights and another
    // one for the negative weights
    Feature pos(feat.Shape()), neg(feat.Shape());
    pos.ClearPixels();
    neg.ClearPixels();

    for(int y = 0; y < pos.Shape().height; y++) {
        float *svmIt = (float *) feat.PixelAddress(0, y, 0);
        float *p = (float *) pos.PixelAddress(0, y, 0);
        float *n = (float *) neg.PixelAddress(0, y, 0);

        for(int x = 0; x < pos.Shape().width * pos.Shape().nBands; x++, p++, n++, svmIt++) {
            if(*svmIt < 0) *n = fabs(*svmIt);
            else if(*svmIt > 0) *p = *svmIt;
        }
    }

    CByteImage negViz, posViz;
    posViz = this->render(pos, true);
    negViz = this->render(neg, true);

    // Put positive and negative weights images side by side in a color image.
    // Negative weights show up as red and positive weights show up as green.
    CByteImage negposViz(CShape(posViz.Shape().width * 2, posViz.Shape().height, 3));
    negposViz.ClearPixels();

    for(int y = 0; y < negposViz.Shape().height; y++) {
        uchar *n = (uchar *) negViz.PixelAddress(0, y, 0);
        uchar *np = (uchar *) negposViz.PixelAddress(0, y, 2);
        for(int x = 0; x < negViz.Shape().width; x++, n++, np += 3) {
            *np = *n;
        }

        uchar *p = (uchar *) posViz.PixelAddress(0, y, 0);
        np = (uchar *) negposViz.PixelAddress(posViz.Shape().width, y, 1);
        for(int x = 0; x < negViz.Shape().width; x++, p++, np += 3) {
            *np = *p;
        }
    }

    return negposViz;
}

void
FeatureExtractor::operator()(const CByteImage &img, Feature &feat) const
{
    CFloatImage img_(img.Shape());
    TypeConvert(img, img_);
    this->operator()(img_, feat);
}

void
FeatureExtractor::operator()(const CroppedImageDatabase &db, FeatureCollection &feats) const
{
    int n = db.getSize();

    feats.resize(n);
    for(int i = 0; i < n; i++) {
        CByteImage img;
        ReadFile(img, db.getFilename(i).c_str());

        (*this)(img, feats[i]);
    }
}

void
FeatureExtractor::operator()(const SBFloatPyramid &imPyr, FeaturePyramid &featPyr) const
{
    featPyr.resize(imPyr.getNLevels());

    CByteImage x;
    for (int i = 0; i < imPyr.getNLevels(); i++) {
        this->operator()(imPyr[i], featPyr[i]);
    }
}

CByteImage
FeatureExtractor::render(const Feature &f, bool normalizeFeat) const
{
    if(normalizeFeat) {
        CShape shape = f.Shape();
        Feature fAux(shape);

        float fMin, fMax;
        f.getRangeOfValues(fMin, fMax);

        for(int y = 0; y < shape.height; y++) {
            float *fIt = (float *) f.PixelAddress(0, y, 0);
            float *fAuxIt = (float *) fAux.PixelAddress(0, y, 0);

            for(int x = 0; x < shape.width * shape.nBands; x++, fAuxIt++, fIt++) {
                *fAuxIt = (*fIt) / fMax;
            }
        }

        return this->render(fAux);
    } else {
        return this->render(f);
    }
}

std::vector<CByteImage>
FeatureExtractor::render(const FeaturePyramid &f, bool normalizeFeat) const
{
    std::vector<CByteImage> res(f.getNLevels());
    for(int i = 0; i < res.size(); i++) {
        res[i] = render(f[i], normalizeFeat);
    }
    return res;
}

FeatureExtractor *
FeatureExtractor::create(const std::string &featureType, const ParametersMap &params)
{
    ParametersMap tmp = params;
    tmp[FEATURE_TYPE_KEY] = featureType;
    return FeatureExtractor::create(tmp);
}

FeatureExtractor *
FeatureExtractor::create(ParametersMap params)
{
    std::string featureType = params.getStr(FEATURE_TYPE_KEY);
    params.erase(FEATURE_TYPE_KEY);

    if(strcasecmp(featureType.c_str(), "ti") == 0) return new TinyImageFeatureExtractor(params);
    else if(strcasecmp(featureType.c_str(), "tig"    ) == 0) return new TinyImageGradFeatureExtractor(params);
    else if(strcasecmp(featureType.c_str(), "hog"    ) == 0) return new HOGFeatureExtractor(params);

    // Implement other features or call a feature extractor with a different set
    // of parameters by adding more calls here.
    else if(strcasecmp(featureType.c_str(), "custom1") == 0) throw CError("not implemented");
    else if(strcasecmp(featureType.c_str(), "custom2") == 0) throw CError("not implemented");
    else if(strcasecmp(featureType.c_str(), "custom3") == 0) throw CError("not implemented");
    else {
        throw CError("Unknown feature type: %s", featureType.c_str());
    }
}

ParametersMap
FeatureExtractor::getDefaultParameters(const std::string &featureType)
{
    ParametersMap params;
    if(strcasecmp(featureType.c_str(), "ti"     ) == 0) params = TinyImageFeatureExtractor::getDefaultParameters();
    else if(strcasecmp(featureType.c_str(), "tig"    ) == 0) params = TinyImageGradFeatureExtractor::getDefaultParameters();
    else if(strcasecmp(featureType.c_str(), "hog"    ) == 0) params = HOGFeatureExtractor::getDefaultParameters();

    // Implement other features or call a feature extractor with a different set
    // of parameters by adding more calls here.
    else if(strcasecmp(featureType.c_str(), "custom1") == 0) throw CError("not implemented");
    else if(strcasecmp(featureType.c_str(), "custom2") == 0) throw CError("not implemented");
    else if(strcasecmp(featureType.c_str(), "custom3") == 0) throw CError("not implemented");
    else {
        throw CError("Unknown feature type: %s", featureType.c_str());
    }

    params[FEATURE_TYPE_KEY] = featureType;

    return params;
}

void
FeatureExtractor::save(FILE *f, const FeatureExtractor *feat)
{
    ParametersMap params = feat->getParameters();
    params[FEATURE_TYPE_KEY] = feat->getFeatureType();
    params.save(f);
}

FeatureExtractor *
FeatureExtractor::load(FILE *f)
{
    ParametersMap params;
    params.load(f);
    return FeatureExtractor::create(params);
}

// ============================================================================
// TinyImage
// ============================================================================

static const char *SCALE_KEY = "scale";

ParametersMap
TinyImageFeatureExtractor::getDefaultParameters()
{
    ParametersMap params;
    params.set(SCALE_KEY, 0.2);
    return params;
}

ParametersMap
TinyImageFeatureExtractor::getParameters() const
{
    ParametersMap params;
    params.set(SCALE_KEY, _scale);
    return params;
}

TinyImageFeatureExtractor::TinyImageFeatureExtractor(const ParametersMap &params)
{
    _scale = params.getFloat(SCALE_KEY);
}

void
TinyImageFeatureExtractor::operator()(const CFloatImage &imgRGB, Feature &feat) const
{
    int targetW = _round(imgRGB.Shape().width * _scale);
    int targetH = _round(imgRGB.Shape().height * _scale);

    CFloatImage tinyImg(targetW, targetH, 1);

    CFloatImage imgG;
    convertRGB2GrayImage(imgRGB, imgG);

    CTransform3x3 s = CTransform3x3::Scale( 1. / _scale, 1. / _scale );

    WarpGlobal(imgG, tinyImg, s, eWarpInterpLinear);

    feat = tinyImg;
}

CByteImage
TinyImageFeatureExtractor::render(const Feature &f) const
{
    CByteImage viz;
    TypeConvert(f, viz);
    return viz;
}

// ============================================================================
// TinyImage Gradient
// ============================================================================

ParametersMap
TinyImageGradFeatureExtractor::getDefaultParameters()
{
    return TinyImageFeatureExtractor::getDefaultParameters();
}

ParametersMap
TinyImageGradFeatureExtractor::getParameters() const
{
    ParametersMap params;
    params.set(SCALE_KEY, _scale);
    return params;
}

TinyImageGradFeatureExtractor::TinyImageGradFeatureExtractor(const ParametersMap &params)
{
    _scale = params.getFloat(SCALE_KEY);

    static float derivKvals[3] = { -1, 0, 1};

    _kernelDx.ReAllocate(CShape(3, 1, 1), derivKvals, false, 1);
    _kernelDx.origin[0] = 1;

    _kernelDy.ReAllocate(CShape(1, 3, 1), derivKvals, false, 1);
    _kernelDy.origin[0] = 1;
}

void
TinyImageGradFeatureExtractor::operator()(const CFloatImage &imgRGB_, Feature &feat) const
{
    int targetW = _round(imgRGB_.Shape().width * _scale);
    int targetH = _round(imgRGB_.Shape().height * _scale);

    /******** BEGIN TODO ********/
    // Compute tiny image gradient feature, output should be a _targetW by _targetH
    // grayscale image, similar to tiny image. The difference here is that you will
    // compute the gradients in the x and y directions, followed by the gradient
    // magnitude.
    //
    // Steps are:
    // 1) Convert image to grayscale (see convertRGB2GrayImage in Utils.h)
    // 2) Resize image to be _targetW by _targetH
    // 3) Compute gradients in x and y directions
    // 4) Compute gradient magnitude
    //
    // Useful functions:
    // convertRGB2GrayImage, TypeConvert, WarpGlobal, Convolve


    //convert to grayscale
    CFloatImage imgG;
    convertRGB2GrayImage(imgRGB_, imgG);

    //resize image
    CFloatImage tinyImg(targetW, targetH, 1);
    CTransform3x3 s = CTransform3x3::Scale( 1. / _scale, 1. / _scale );
    WarpGlobal(imgG, tinyImg, s, eWarpInterpLinear);

    //compute gradients in x and y directions
    CFloatImage imgX(targetW, targetH, 1);
    CFloatImage imgY(targetW, targetH, 1);
    CFloatImage imgMag(targetW, targetH, 1);
    Convolve(tinyImg, imgX, _kernelDx);
    Convolve(tinyImg, imgY, _kernelDy);

    for (int y = 0; y < targetH; y++){
        for(int x = 0; x < targetW; x++){
            imgMag.Pixel(x, y, 0) = sqrt(imgX.Pixel(x, y, 0) * imgX.Pixel(x, y, 0) + imgY.Pixel(x, y, 0) * imgY.Pixel(x, y, 0));
        }
    }
    feat = imgMag;


    /******** END TODO ********/
}

CByteImage
TinyImageGradFeatureExtractor::render(const Feature &f) const
{
    CByteImage viz;
    TypeConvert(f, viz);
    return viz;
}

// ============================================================================
// HOG
// ============================================================================

const char *N_ANGULAR_BINS_KEY     = "n_angular_bins";
const char *UNSIGNED_GRADIENTS_KEY = "unsigned_gradients";
const char *CELL_SIZE_KEY          = "cell_size";

ParametersMap
HOGFeatureExtractor::getDefaultParameters()
{
    ParametersMap params;
    params.set(N_ANGULAR_BINS_KEY    , 18);
    params.set(UNSIGNED_GRADIENTS_KEY, 1);
    params.set(CELL_SIZE_KEY         , 6);
    return params;
}

ParametersMap
HOGFeatureExtractor::getParameters() const
{
    ParametersMap params;
    params.set(N_ANGULAR_BINS_KEY    , _nAngularBins);
    params.set(UNSIGNED_GRADIENTS_KEY, _unsignedGradients);
    params.set(CELL_SIZE_KEY         , _cellSize);
    return params;
}

HOGFeatureExtractor::HOGFeatureExtractor(const ParametersMap &params)
{
    _nAngularBins = params.getInt(N_ANGULAR_BINS_KEY);
    _unsignedGradients = params.getInt(UNSIGNED_GRADIENTS_KEY);
    _cellSize = params.getInt(CELL_SIZE_KEY);

    static float derivKvals[3] = { -1, 0, 1};

    _kernelDx.ReAllocate(CShape(3, 1, 1), derivKvals, false, 1);
    _kernelDx.origin[0] = 1;

    _kernelDy.ReAllocate(CShape(1, 3, 1), derivKvals, false, 1);
    _kernelDy.origin[0] = 1;

    // Visualization Stuff
    // A set of patches representing the bin orientations. When drawing a hog cell
    // we multiply each patch by the hog bin value and add all contributions up to
    // form the visual representation of one cell. Full HOG is achieved by stacking
    // the viz for individual cells horizontally and vertically.
    _oriMarkers.resize(_nAngularBins);
    const int ms = 11;
    CShape markerShape(ms, ms, 1);

    // FIXME: add patches for contrast sensitive dimensions (half filled circle)

    // First patch is a horizontal line
    _oriMarkers[0].ReAllocate(markerShape, true);
    _oriMarkers[0].ClearPixels();
    for(int i = 1; i < ms - 1; i++) _oriMarkers[0].Pixel(/*floor(*/ ms / 2 /*)*/, i, 0) = 1;

    // The other patches are obtained by rotating the first one
    CTransform3x3 T = CTransform3x3::Translation((ms - 1) / 2.0, (ms - 1) / 2.0);
    for(int angBin = 1; angBin < _nAngularBins; angBin++) {
        double theta;
        if(_unsignedGradients) theta = 180.0 * (double(angBin) / _nAngularBins);
        else theta = 360.0 * (double(angBin) / _nAngularBins);
        CTransform3x3 R  = T * CTransform3x3::Rotation(theta) * T.Inverse();

        _oriMarkers[angBin].ReAllocate(markerShape, true);
        _oriMarkers[angBin].ClearPixels();

        WarpGlobal(_oriMarkers[0], _oriMarkers[angBin], R, eWarpInterpLinear);
    }
}

double gaussian(int x, int y, int cell_size){
    return exp(-(x-cell_size/2)*(x-cell_size/2)/(0.5*cell_size*cell_size) - (y-cell_size/2)*(y-cell_size/2)/(0.5*cell_size*cell_size));
}

void
HOGFeatureExtractor::operator()(const CFloatImage &img, Feature &feat) const
{
    /******** BEGIN TODO ********/
    // Compute the Histogram of Oriented Gradients feature
    //
    // Steps are:
    // 1) Compute gradients in x and y directions. We provide the
    //    derivative kernel proposed in the paper in _kernelDx and
    //    _kernelDy.
    // 2) Compute gradient magnitude and orientation
    // 3) Add contribution each pixel to HOG cells whose
    //    support overlaps with pixel. The contribution should
    //    be weighted by a gaussian centered at the corresponding
    //    HOG cell. Each cell has a support of size
    //    _cellSize and each histogram has _nAngularBins. Note that
    //    pixels away from the borders of the image should contribute to
    //    at least four HOG cells.
    // 4) Normalize HOG for each cell. One simple strategy that is
    //    is also used in the SIFT descriptor is to first threshold
    //    the bin values so that no bin value is larger than some
    //    threshold (we leave it up to you do find this value) and
    //    then re-normalize the histogram so that it has norm 1. A more
    //    elaborate normalization scheme is proposed in Dalal & Triggs
    //    paper but we leave that as extra credit.
    //
    // Useful functions:
    // convertRGB2GrayImage, TypeConvert, WarpGlobal, Convolve

    int imgW = img.Shape().width;
    int imgH = img.Shape().height;

    int featW = (imgW + (_cellSize-1)) / _cellSize;
    int featH = (imgH + (_cellSize-1)) / _cellSize;
    CFloatImage imgHog(featW, featH, _nAngularBins);

    //compute gradients in x and y directions
    CFloatImage imgX(imgW, imgH, 3);
    CFloatImage imgY(imgW, imgH, 3);
    CFloatImage imgOri(imgW, imgH, 1);
    CFloatImage imgMag(imgW, imgH, 1);

    imgX.ClearPixels();
    imgY.ClearPixels();
    imgOri.ClearPixels();
    imgMag.ClearPixels();
    imgHog.ClearPixels();

    Convolve(img, imgX, _kernelDx);
    Convolve(img, imgY, _kernelDy);

    //compute gradient magnitudes
    for (int y = 0; y < imgH; y++){
        for(int x = 0; x < imgW; x++){
            int max = 0;
            double max_m = 0;
            for (int b = 0; b < 3; b++){
                double m = sqrt(imgX.Pixel(x, y, b) * imgX.Pixel(x, y, b) + imgY.Pixel(x, y, b) * imgY.Pixel(x, y, b));
                if ( m > max_m ){
                    max_m = m;
                    max = b;
                }
            }
            imgMag.Pixel(x, y, 0) = max_m;
            if (imgX.Pixel(x, y, max) == 0) imgOri.Pixel(x, y, 0) = 0;
            else {
                if (_unsignedGradients){
                    double ang = atan2(imgY.Pixel(x, y, max), imgX.Pixel(x, y, max)) * 180.0 / PI;
                    if (ang < 0){
                        imgOri.Pixel(x, y, 0) = ang + 180;
                    }else{
                        imgOri.Pixel(x, y, 0) = ang;
                    }
                }
                else {
                    double ang = atan2(imgY.Pixel(x, y, max), imgX.Pixel(x, y, max)) * 180.0 / PI;
                    if (ang < 0){
                        imgOri.Pixel(x, y, 0) = ang + 360;
                    }else{
                        imgOri.Pixel(x, y, 0) = ang;
                    }
                }
            }

            //printf("y, x:%f, %f", imgY.Pixel(x, y, max), imgX.Pixel(x, y, max));
            //printf("orientation:%f\n", imgOri.Pixel(x, y, 0));
        }
    }
    //compute HoG
    for (int y = 0; y < featH; y++){
        for(int x = 0; x < featW; x++){
            for (int i = 0; i < _cellSize; i++){
                for(int j = 0; j < _cellSize; j++){
                    if (x*_cellSize+i >= 0 && x*_cellSize+i < imgW && y*_cellSize + j >= 0 && y * _cellSize + j < imgH){
                        int b = 0;
                        if (_unsignedGradients){
                            //printf("unsori:%f\n", imgOri.Pixel(x * _cellSize + i, y * _cellSize + j, 0) + 90);
                            b = imgOri.Pixel(x * _cellSize + i, y * _cellSize + j, 0) / (180.0 / _nAngularBins);
                        }else{
                            //printf("sori:%f\n", imgOri.Pixel(x * _cellSize + i, y * _cellSize + j, 0) + 180);
                            b = imgOri.Pixel(x * _cellSize + i, y * _cellSize + j, 0) / (360.0 / _nAngularBins);
                        }
                        //printf("b = %d\n", b);
                        imgHog.Pixel(x, y, b) += imgMag.Pixel(x * _cellSize + i, y * _cellSize + j, 0) * gaussian(i, j, _cellSize);
                    }
                }
            }
        }
    }

    for (int y = 0; y < featH; y++){
        for(int x = 0; x < featW; x++){
            double length = 0;
            for(int b = 0; b < _nAngularBins; b++){
                length += imgHog.Pixel(x,y,b) * imgHog.Pixel(x,y,b);
            }
            length = sqrt(length) + 0.00001;
            for (int b = 0; b < _nAngularBins; b++){
                imgHog.Pixel(x, y, b) = imgHog.Pixel(x, y, b) / length;
            }
        }
    }

/*
    double threshold = 1;
    for (int y = 0; y < featH; y++){
        for(int x = 0; x < featW; x++){
            double length = 0;
            for(int b = 0; b < _nAngularBins; b++){
                printf("%f\n", imgHog.Pixel(x,y,b));
                if (imgHog.Pixel(x,y,b) > threshold){
                    length += threshold * threshold;
                }else{
                    length += imgHog.Pixel(x,y,b) * imgHog.Pixel(x,y,b);
                }
            }
            length = sqrt(length) + 0.00001;
            for (int b = 0; b < _nAngularBins; b++){
                imgHog.Pixel(x, y, b) = imgHog.Pixel(x, y, b) / length;
            }
        }
    }

*/

    feat = imgHog;
    /******** END TODO ********/
}

CByteImage
HOGFeatureExtractor::render(const Feature &f) const
{
    CShape cellShape = _oriMarkers[0].Shape();
    CFloatImage hogImgF(CShape(cellShape.width * f.Shape().width, cellShape.height * f.Shape().height, 1));
    hogImgF.ClearPixels();

    float minBinValue, maxBinValue;
    f.getRangeOfValues(minBinValue, maxBinValue);

    // For every cell in the HOG
    for(int hi = 0; hi < f.Shape().height; hi++) {
        for(int hj = 0; hj < f.Shape().width; hj++) {

            // Now _oriMarkers, multiplying contribution by bin level
            for(int hc = 0; hc < _nAngularBins; hc++) {
                float v = f.Pixel(hj, hi, hc) / maxBinValue;
                for(int ci = 0; ci < cellShape.height; ci++) {
                    float *cellIt = (float *) _oriMarkers[hc].PixelAddress(0, ci, 0);
                    float *hogIt = (float *) hogImgF.PixelAddress(hj * cellShape.height, hi * cellShape.height + ci, 0);

                    for(int cj = 0; cj < cellShape.width; cj++, hogIt++, cellIt++) {
                        (*hogIt) += v * (*cellIt);
                    }
                }
            }

        }
    }

    CByteImage hogImg;
    TypeConvert(hogImgF, hogImg);
    return hogImg;
}


