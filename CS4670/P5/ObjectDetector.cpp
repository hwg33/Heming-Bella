#include "ObjectDetector.h"

#define WIN_SIZE_NMS_KEY   "nms_win_size"
#define RESP_THESH_KEY     "sv_response_threshold"
#define OVERLAP_THRESH_KEY "detection_overlap_threshold"

ObjectDetector::ObjectDetector(const ParametersMap &params)
{
    _winSizeNMS = params.getInt(WIN_SIZE_NMS_KEY);
    _respThresh = params.getFloat(RESP_THESH_KEY);
    _overlapThresh = params.getFloat(OVERLAP_THRESH_KEY);
}

ParametersMap
ObjectDetector::getDefaultParameters()
{
    ParametersMap params;
    params.set(WIN_SIZE_NMS_KEY  , 11  );
    params.set(RESP_THESH_KEY    ,  0  );
    params.set(OVERLAP_THRESH_KEY,  0.2);
    return params;
}

ParametersMap
ObjectDetector::getParameters() const
{
    ParametersMap params;
    params.set(WIN_SIZE_NMS_KEY, _winSizeNMS);
    params.set(RESP_THESH_KEY, _respThresh);
    params.set(OVERLAP_THRESH_KEY, _overlapThresh);
    return params;
}

ObjectDetector::ObjectDetector(int winSizeNMS, double respThresh, double overlapThresh):
    _winSizeNMS(winSizeNMS),
    _respThresh(respThresh),
    _overlapThresh(overlapThresh)
{
}

void
ObjectDetector::operator()( const CFloatImage &svmResp, const Size &roiSize,
                            double featureScaleFactor, std::vector<Detection> &dets,
                            double imScale ) const
{
    /******** BEGIN TODO ********/
    // Non-Maxima Suppression on a single image
    //
    // For every window of size _winSizeNMS by _winSizeNMS determine if the
    // pixel at the center of the window is the local maxima and is also
    // greater than _respThresh. If so, create an instance of Detection and
    // store it in dets, remember to set the position of the central pixel
    // (x,y), as well as the dimensions of the detection (based on roiSize). Y
    // ou will have to correct location and dimensions using a scale factor
    // that is a function of featureScaleFactor and imScale.

    dets.resize(0);
    int width = svmResp.Shape().width;
    int height = svmResp.Shape().height;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            double localMax = std::numeric_limits<double>::min();
            int maxX = -1;
            int maxY = -1;
            for (int i = -_winSizeNMS / 2; i <= _winSizeNMS / 2; i++) {
                for (int j = -_winSizeNMS / 2; j <= _winSizeNMS / 2; j++) {
                    if (x + i < width && x + i >= 0 && y + j < height && y + j >= 0) {
                        double pixel = svmResp.Pixel(x + i, y + j, 0);
                        if (pixel > localMax) {
                            localMax = pixel;
                            maxX = i;
                            maxY = j;
                        }
                    }
                }
            }
            if (maxX == 0 && maxY == 0 && localMax > _respThresh) {
                Detection det;
                det.x = x / (imScale * featureScaleFactor);
                det.y = y / (imScale * featureScaleFactor);
                det.width = roiSize.width / (imScale * featureScaleFactor);
                det.height = roiSize.height / (imScale * featureScaleFactor);
                det.response = localMax;
                dets.push_back(det);
            }
        }
    }

    /******** END TODO ********/
}

bool
sortByResponse(const Detection &d1, const Detection &d2)
{
    return d1.response >= d2.response;
}

void
ObjectDetector::operator()( const SBFloatPyramid &svmRespPyr, const Size &roiSize,
                            double featureScaleFactor, std::vector<Detection> &dets ) const
{
    /******** BEGIN TODO ********/
    // Non-Maxima Suppression across pyramid levels
    //
    // Given the pyramid of SVM responses, for each level you will find
    // the non maximas within a window of size _winSizeNMS by _winSizeNMS.
    // This functionality is impelmented in the other operator() method above.
    // Once all detections for all levels are found we perform another round of
    // non maxima suppression, this time across all levels. In this step you will
    // use the relativeOverlap method from the class Detection to determine if
    // a detection "competes" with another. If there is enough overlap between them
    // (i.e., if the relative overlap is greater then _overlapThresh) then only the
    // detection with strongest response is kept.
    //
    // Steps are:
    // 1) Find the local maxima per level of the pyramid for all levels
    // 2) Perform non maxima suppression across levels
    //
    // Useful functions:
    // sortByResponse, relativeOverlap, opreator()

    dets.resize(0);

    // Find detections per level
    std::vector<Detection> allDets;
    for (int i = 0; i < svmRespPyr.getNLevels(); i++) {

        std::vector<Detection> levelDets;
        this->operator()(svmRespPyr[i], roiSize, featureScaleFactor, levelDets, svmRespPyr.levelScale(i));

        allDets.insert(allDets.end(), levelDets.begin(), levelDets.end());
    }

    std::sort(allDets.begin(), allDets.end(), sortByResponse);
    for (int i = 0; i < allDets.size(); i++) {
        bool shouldBeKept = true;
        for (int j = 0; j < dets.size(); j++) {
            if (allDets[i].relativeOverlap(dets[j]) > _overlapThresh) {
                shouldBeKept = false;
            }
        }
        if (shouldBeKept) dets.push_back(allDets[i]);
    }

    /******** END TODO ********/
}

