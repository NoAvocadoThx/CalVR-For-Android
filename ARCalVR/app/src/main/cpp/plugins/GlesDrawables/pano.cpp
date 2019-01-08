#include "pano.h"
#include <cvrUtil/ARCoreManager.h>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

panoStitcher::panoStitcher * mPtr = nullptr;
panoStitcher::panoStitcher * instance(){
    if(!mPtr) mPtr = new panoStitcher;
    return mPtr;
}

bool panoStitcher::check_stitch_condition(){
    return true;
}
inline bool findKeyPointsHomography(std::vector<KeyPoint>& kpts1, std::vector<KeyPoint>& kpts2,
                                    std::vector<DMatch>& matches, std::vector<char>& match_mask,
                                    Mat& H) {
    if (static_cast<int>(match_mask.size()) < 3) {
        return false;
    }
    std::vector<Point2f> pts1;
    std::vector<Point2f> pts2;
    for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
        pts1.push_back(kpts1[matches[i].queryIdx].pt);
        pts2.push_back(kpts2[matches[i].trainIdx].pt);
    }
    H = findHomography(pts1, pts2, cv::RANSAC, 4, match_mask);
    return true;
}
void panoStitcher::StitchImage(cv::Mat mat){}

void panoStitcher::StitchCurrentView(){
    if(!check_stitch_condition()) return;
    if(_pano.empty()) {
        _pano = cvr::ARCoreManager::instance()->getRGBImage();
        return;
    }

    Mat image = cvr::ARCoreManager::instance()->getGrayscaleImage();
    Mat image_scene;
    cvtColor(_pano, image_scene, CV_RGB2GRAY);

    //Detect the keypoints using SURF Detector
    Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(10, true);

    std::vector< KeyPoint > keypoints_object, keypoints_scene;
    Mat dest_object, dest_scene;

    detector->detectAndCompute(image, Mat(), keypoints_object, dest_object);
    detector->detectAndCompute(image_scene, Mat(), keypoints_scene, dest_scene);
//    detector->detect( image, keypoints_object );
//    detector->detect( _pano, keypoints_scene );

    //matches
    std::vector<DMatch> matches;
    BFMatcher desc_matcher(cv::NORM_L2, true);
    desc_matcher.match(dest_object, dest_scene, matches, Mat());
    std::sort(matches.begin(), matches.end());
    while (matches.front().distance * kDistanceCoef < matches.back().distance) {
        matches.pop_back();
    }
    while (matches.size() > kMaxMatchingSize) {
        matches.pop_back();
    }

    //homography
    std::vector<char> match_mask(matches.size(), 1);
    Mat H;
    if(!findKeyPointsHomography(keypoints_object, keypoints_scene,
                            matches, match_mask, H)) return;

    //warp
    Mat result;
    Mat rgb_object = cvr::ARCoreManager::instance()->getRGBImage();

    warpPerspective( _pano, result, H, cv::Size( _pano.cols+rgb_object.cols, _pano.rows) );
    cv::Mat half(result, cv::Rect(0, 0, rgb_object.cols, rgb_object.rows) );
    rgb_object.copyTo(half);

    //remove the black portion after stitching, and confine in a rectangular region
    // vector with all non-black point positions
    std::vector<cv::Point> nonBlackList;
    nonBlackList.reserve(result.rows*result.cols);

    // add all non-black points to the vector
    // there are more efficient ways to iterate through the image
    for(int j=0; j<result.rows; ++j)
        for(int i=0; i<result.cols; ++i)
        {
            // if not black: add to the list
            if(result.at<cv::Vec3b>(j,i) != cv::Vec3b(0,0,0))
            {
                nonBlackList.push_back(cv::Point(i,j));
            }
        }

    // create bounding rect around those points
    cv::Rect bb = cv::boundingRect(nonBlackList);
    result = result(bb);

    resize(result, _pano, Size(image.rows, result.rows * result.cols / image.rows));
}
unsigned char * panoStitcher::getPanoImageData(int & width, int & height){
    width = _pano.cols; height = _pano.rows;
    return _pano.data;
}