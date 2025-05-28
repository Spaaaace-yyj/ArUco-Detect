#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/objdetect/aruco_detector.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

#define P1_Color cv::Scalar(255, 0, 216)
#define P2_Color cv::Scalar(255, 118, 17)
#define P3_Color cv::Scalar(14, 255, 35)
#define P4_Color cv::Scalar(169, 11, 255)



void draw_aruco_target(std::vector<int> markerIds, std::vector<std::vector<cv::Point2f> > markerCorners, cv::Mat &dst) {
	for (int i = 0; i < markerCorners.size(); i++) {
		//cv::line(dst, markerCorners[i][0], markerCorners[i][1], Line_Color_X, 3, 8, 0);
		//cv::line(dst, markerCorners[i][0], markerCorners[i][3], Line_Color_Y, 3, 8, 0);
		cv::putText(dst, to_string(markerIds[i]), markerCorners[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1, 8);
		for (int j = 0; j < markerCorners[i].size(); j++) {
			switch (j)
			{
			case 0:
				cv::circle(dst, markerCorners[i][j], 6, P1_Color, 2);
				break;
			case 1:
				cv::circle(dst, markerCorners[i][j], 2, P2_Color, 3);
				break;
			case 2:
				cv::circle(dst, markerCorners[i][j], 2, P3_Color, 3);
				break;
			case 3:
				cv::circle(dst, markerCorners[i][j], 2, P4_Color, 3);
				break;
			default:
				break;
			}
		}
	}
};

bool solve_pnp(cv::Mat cameraMatrix, cv::Mat distCoeffs, vector<cv::Point3f> objectPoints,
					const vector<std::vector<cv::Point2f> > &markerCorners,
					vector <cv::Mat> &rvec, vector <cv::Mat> &tvec) {
	for (int i = 0; i < markerCorners.size(); i++) {
		cv::Mat rvec_temp, tvec_temp;
		bool can_solve = cv::solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec_temp, tvec_temp, false, cv::SOLVEPNP_IPPE_SQUARE);
		if (!can_solve) {
			cout << "Solve PnP faile at ID = " << i << endl;
			return false;
		}
		rvec.push_back(rvec_temp);
		tvec.push_back(tvec_temp);
	}
	return true;
}
class ArucoDetect : public rclcpp::Node
{
public:
	ArucoDetect() : Node("ArUco"){
		RCLCPP_INFO(this->get_logger(), "ArUco marker detector is running!");
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&ArucoDetect::imageCallback, this, std::placeholders::_1));
		broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	}

	double calculateDistanceToCamera(
    	const cv::Mat& rvec,     // 旋转向量
    	const cv::Mat& tvec,     // 平移向量
    	const cv::Point3f& point // 目标点在世界坐标系下的坐标
		) {
    	// 将旋转向量转换为旋转矩阵
    	cv::Mat rotationMatrix;
    	cv::Rodrigues(rvec, rotationMatrix);
	
    	// 构建目标点的齐次坐标 [x, y, z, 1]
    	cv::Mat pointWorld = (cv::Mat_<double>(4, 1) << 
    	    point.x, point.y, point.z, 1);
	
    	// 构建变换矩阵 [R | t]
    	cv::Mat transformMatrix = cv::Mat::zeros(3, 4, CV_64F);
    	rotationMatrix.copyTo(transformMatrix(cv::Rect(0, 0, 3, 3)));
    	tvec.copyTo(transformMatrix(cv::Rect(3, 0, 1, 3)));
    
    	// 计算目标点在相机坐标系下的坐标
    	cv::Mat pointCamera = transformMatrix * pointWorld;
    
    	// 计算距离
    	double distance = std::sqrt(
    	    pointCamera.at<double>(0, 0) * pointCamera.at<double>(0, 0) +
    	    pointCamera.at<double>(1, 0) * pointCamera.at<double>(1, 0) +
    	    pointCamera.at<double>(2, 0) * pointCamera.at<double>(2, 0)
    	);
    
    	return distance;
	}
	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
     {
         try
         {
             //ros_message to OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
 			image_mat_ = cv_ptr->image.clone();
			if(!image_mat_.empty()){
				cv::Mat frame = image_mat_.clone();
				// detect ArUco mark;
				std::vector<int> markerIds;
				std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
				cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
				cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
				cv::aruco::ArucoDetector detector(dictionary, detectorParams);
				detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);
				draw_aruco_target(markerIds, markerCorners, frame);
		
				//Pnp Solver

				//camera matrix（dji action4）
				cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
    						546.977143, 0, 642.357169,
    						0, 544.018998, 350.900651,
    						0, 0, 1);
				cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
    						0.003606, 0.004318, -0.000378, -0.000619, 0.000000);

				// ArUco real world length
				double markerLength = 0.07;  

				// define real world position
				vector<cv::Point3f> objectPoints;
				objectPoints.push_back(cv::Point3f(0, markerLength, 0)); 
				objectPoints.push_back(cv::Point3f(markerLength, markerLength, 0)); 
				objectPoints.push_back(cv::Point3f(markerLength, 0, 0));
				objectPoints.push_back(cv::Point3f(0, 0, 0)); 

				//result matrix
				vector <cv::Mat> rvec, tvec;

				solve_pnp(cameraMatrix, distCoeffs, objectPoints, markerCorners, rvec, tvec);

				for (int i = 0; i < rvec.size(); i++) {
					drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec[i], tvec[i], 0.1);
					double distance = calculateDistanceToCamera(rvec[i], tvec[i], cv::Point3f(0, 0, 0));
					cout << "distance [" << i << "]" << distance << endl;
					
					cv::Mat rvecs = rvec[i];  // 旋转向量（3x1矩阵）
					cv::Mat R;
					cv::Rodrigues(rvecs, R);  // 转换为3x3旋转矩阵

					// 将OpenCV矩阵转换为Eigen矩阵
					Eigen::Matrix3d eigen_R;
					eigen_R << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
					           R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
					           R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);

					// 从旋转矩阵创建四元数
					Eigen::Quaterniond q(eigen_R);

    				// 创建静态坐标变换（世界坐标系 -> 相机坐标系）
    				geometry_msgs::msg::TransformStamped transform_stamped;
    				transform_stamped.header.stamp = this->get_clock()->now();
    				transform_stamped.header.frame_id = "world"; // 世界坐标系名称（需与 RViz 一致）
    				transform_stamped.child_frame_id = "camera_link"; // 相机坐标系名称

    				// 设置平移
    				transform_stamped.transform.translation.x = tvec[i].at<double>(0, 0);
    				transform_stamped.transform.translation.y = tvec[i].at<double>(1, 0);
    				transform_stamped.transform.translation.z = tvec[i].at<double>(2, 0);

    				// 设置旋转（四元数）
    				transform_stamped.transform.rotation.x = q.x();
    				transform_stamped.transform.rotation.y = q.y();
    				transform_stamped.transform.rotation.z = q.z();
    				transform_stamped.transform.rotation.w = q.w();

    				// 发布静态坐标变换
    				broadcaster_->sendTransform(transform_stamped);
				}


				cv::imshow("Image", frame);
				cv::waitKey(1);
			}
         }
         catch (cv_bridge::Exception& e)
         {
             RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
         }
     }

private:
	cv::Mat image_mat_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_; 
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoDetect>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}