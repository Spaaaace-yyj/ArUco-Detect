#ifndef _ARUCO_MARK_HPP_
#define _ARUCO_MARK_HPP_

#include <opencv4/opencv2/opencv.hpp>
#include <kalman.hpp>
#include <vector>

using namespace std;
class ArucoMark{

public:
ArucoMark(int id, std::vector<cv::Point2f> markerCorners, cv::Mat marker_rvec, cv::Mat marker_tvec);

void update_ekf(double dt);

void draw_mark(cv::Mat &frame, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

~ArucoMark() = default;
public:
    int id;
    double markerLength = 0.048;
    std::vector<cv::Point2f> markerCorners;
    cv::Mat marker_rvec;
    cv::Mat marker_tvec;

    cv::Mat marker_tvec_prev;
    cv::Mat marker_tvec_future;

    cv::Point3f marker_position_prev;    
    cv::Point3f marker_position_future;

    vector<cv::Point3f> objectPoints;

public:
    EKF* ekf;
    bool isLost = false;
    float extrapolate_dt = 40.0f;
private:
    double alpha_q = 5e-5;
    double alpha_r = 1e-2;
    int stateSize = 6;
    int measSize = 3;  
};


#endif