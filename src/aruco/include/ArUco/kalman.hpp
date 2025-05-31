#ifndef KALMAN_HPP_
#define KALMAN_HPP_

#include <opencv2/opencv.hpp>

class EKF {
public:
    EKF(double alpha_q = 1e-5, double alpha_r = 1e-2, int stateSize = 6, int measSize = 3);

    void update(const cv::Mat& tvec, double dt);

    cv::Point3f getCurrentPosition() const;
    cv::Point3f predictFuturePosition(double extrapolate_dt, double scale) const;
    cv::Mat getCurrentPosition_Mat() const;
    cv::Mat getFuturePosition_Mat(double extrapolate_dt, double scale) const;
    void reset();

private:
    // 内部函数
    cv::Mat f(const cv::Mat& x, double dt) const;
    cv::Mat computeJacobianF(const cv::Mat& x, double dt) const;

private:
    int stateSize;
    int measSize;

    double alpha_q;
    double alpha_r;

    // 状态量和矩阵
    cv::Mat x;        // 当前状态
    cv::Mat P;        // 状态协方差
    cv::Mat Q;        // 过程噪声协方差
    cv::Mat R;        // 观测噪声协方差
    cv::Mat H;        // 观测矩阵
};

#endif  // KALMAN_HPP_
