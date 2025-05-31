#include "kalman.hpp"
#include <cmath>

EKF::EKF(double alpha_q, double alpha_r, int stateSize, int measSize)
    : stateSize(stateSize), measSize(measSize), alpha_q(alpha_q), alpha_r(alpha_r)
{
    x = cv::Mat::zeros(stateSize, 1, CV_64F);
    P = cv::Mat::eye(stateSize, stateSize, CV_64F);
    Q = alpha_q * cv::Mat::eye(stateSize, stateSize, CV_64F);
    R = alpha_r * cv::Mat::eye(measSize, measSize, CV_64F);
    H = cv::Mat::zeros(measSize, stateSize, CV_64F);

    for (int i = 0; i < measSize; ++i)
        H.at<double>(i, i) = 1.0;
}

cv::Mat EKF::f(const cv::Mat& x, double dt) const {
    cv::Mat x_pred = x.clone();
    for (int i = 0; i < 3; ++i) {
        double pos = x.at<double>(i);
        double vel = x.at<double>(i + 3);
        x_pred.at<double>(i)     = pos + vel * dt + 0.01 * sin(pos);
        x_pred.at<double>(i + 3) = vel + 0.005 * sin(pos);
    }
    return x_pred;
}

cv::Mat EKF::computeJacobianF(const cv::Mat& x, double dt) const {
    cv::Mat F = cv::Mat::eye(stateSize, stateSize, CV_64F);
    for (int i = 0; i < 3; ++i) {
        double pos = x.at<double>(i);
        F.at<double>(i, i + 3) = dt;
        F.at<double>(i, i)     += 0.01 * cos(pos);
        F.at<double>(i + 3, i) = 0.005 * cos(pos);
    }
    return F;
}

void EKF::update(const cv::Mat& tvec, double dt) {
    // 预测
    cv::Mat x_pred = f(x, dt);
    cv::Mat F = computeJacobianF(x, dt);
    cv::Mat P_pred = F * P * F.t() + Q;

    // 观测更新
    cv::Mat z = tvec.clone();
    if (z.rows != measSize || z.cols != 1)
        z = z.reshape(1, measSize);  // reshape to column if needed

    cv::Mat y = z - H * x_pred;                   // 残差
    cv::Mat S = H * P_pred * H.t() + R;           // 残差协方差
    cv::Mat K = P_pred * H.t() * S.inv();         // 卡尔曼增益
    x = x_pred + K * y;
    P = (cv::Mat::eye(stateSize, stateSize, CV_64F) - K * H) * P_pred;
}

cv::Point3f EKF::getCurrentPosition() const {
    return cv::Point3f(x.at<double>(0), x.at<double>(1), x.at<double>(2));
}

cv::Point3f EKF::predictFuturePosition(double extrapolate_dt, double scale) const {
    cv::Mat x_future = f(x, extrapolate_dt);
    x_future = x_future * scale;
    return cv::Point3f(x_future.at<double>(0), x_future.at<double>(1), x_future.at<double>(2));
}

void EKF::reset() {
    x = cv::Mat::zeros(stateSize, 1, CV_64F);
    P = cv::Mat::eye(stateSize, stateSize, CV_64F);
}

cv::Mat EKF::getCurrentPosition_Mat() const{
    return x;
}

cv::Mat EKF::getFuturePosition_Mat(double extrapolate_dt, double scale) const{
    cv::Mat x_future = f(x, extrapolate_dt);
    x_future = x_future * scale;
    return x_future;
}
