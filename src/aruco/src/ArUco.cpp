#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/objdetect/aruco_detector.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "kalman.hpp"
#include "aruco_mark.hpp"

using namespace std;

#define P1_Color cv::Scalar(255, 0, 216)
#define P2_Color cv::Scalar(255, 118, 17)
#define P3_Color cv::Scalar(14, 255, 35)
#define P4_Color cv::Scalar(169, 11, 255)



class ArucoDetect : public rclcpp::Node
{
public:
	ArucoDetect() : Node("ArUco"){
		RCLCPP_INFO(this->get_logger(), "ArUco marker detector is running!");
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&ArucoDetect::imageCallback, this, std::placeholders::_1));
		broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aruco_truePos", 10);
		marker_pub_future_ = this->create_publisher<visualization_msgs::msg::Marker>("/aruco_futurePos", 10);
	}

	void draw_aruco_target(std::vector<int> markerIds, std::vector<std::vector<cv::Point2f> > markerCorners, cv::Mat &dst, bool use_diff_color = false) {
		for (size_t i = 0; i < markerCorners.size(); i++) {
			//cv::line(dst, markerCorners[i][0], markerCorners[i][1], Line_Color_X, 3, 8, 0);
			//cv::line(dst, markerCorners[i][0], markerCorners[i][3], Line_Color_Y, 3, 8, 0);
			//cv::putText(dst, "ArUco" + to_string(markerIds[i]), markerCorners[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1, 8);
			for (size_t j = 0; j < markerCorners[i].size(); j++) {
				if(!use_diff_color){
					cv::line(dst, markerCorners[i][j], markerCorners[i][(j + 1) % 4], cv::Scalar(0, 0, 255), 1, 8, 0);
				}else{
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
		}
	}

	bool solve_pnp(cv::Mat cameraMatrix, cv::Mat distCoeffs, vector<cv::Point3f> objectPoints,
						const vector<std::vector<cv::Point2f> > &markerCorners,
						vector<int> markerIds, vector<ArucoMark> &aruco_marks) {
		for (size_t i = 0; i < markerCorners.size(); i++) {
			bool is_new = true;
			cv::Mat rvec_temp, tvec_temp;
			bool can_solve = cv::solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec_temp, tvec_temp, false, cv::SOLVEPNP_IPPE_SQUARE);
			
			if (!can_solve) {
				cout << "Solve PnP faile at ID = " << i << endl;
				return false;
			}

			if(aruco_marks.size() == 0){
				aruco_marks.push_back(ArucoMark(markerIds[i], markerCorners[i], rvec_temp, tvec_temp));
				continue;
			}else{
				for(size_t j = 0; j < aruco_marks.size(); j++){
					if(is_new == false){
						break;
					}
					if(markerIds[i] == aruco_marks[j].id){
						is_new = false;
						aruco_marks[j].isLost = false;
						aruco_marks[j].markerCorners = markerCorners[i];
						aruco_marks[j].marker_rvec = rvec_temp;
						aruco_marks[j].marker_tvec = tvec_temp;
						break;
					}
				}
				if(is_new){
					aruco_marks.push_back(ArucoMark(markerIds[i], markerCorners[i], rvec_temp, tvec_temp));
				}
			}
		}
		for(size_t i = 0; i < aruco_marks.size(); i++){
			if(markerIds.size() == 0){
				aruco_marks[i].isLost = true;
				continue;
			}
			for(size_t j = 0; j < markerIds.size(); j++){
				if(markerIds[j] == aruco_marks[i].id){
					break;
				}
				if(j == markerIds.size() - 1){
					aruco_marks[i].isLost = true;
				}
			}
		}
		return true;
	}
	visualization_msgs::msg::Marker makeMarker(
			const geometry_msgs::msg::Pose &pose, 
			const std::string &frame_id = "camera", 
			const std::string &child_frame_id = "Aruco",
			cv::Scalar color = cv::Scalar(0, 255, 0)) {
    	visualization_msgs::msg::Marker marker;
    	marker.header.frame_id = "camera"; // 与 rviz2 中 fixed_frame 对齐
    	marker.header.stamp = rclcpp::Clock().now();
    	marker.ns = "Aruco";
    	marker.id = 0;
    	marker.type = visualization_msgs::msg::Marker::CUBE;
    	marker.action = visualization_msgs::msg::Marker::ADD;

    	marker.pose = pose;

    	marker.scale.x = 0.1;
    	marker.scale.y = 0.1;
    	marker.scale.z = 0.01;

    	marker.color.r = color[0];
    	marker.color.g = color[1];
    	marker.color.b = color[2];
    	marker.color.a = 0.9;

    	marker.lifetime = rclcpp::Duration(0, 0); // 永久显示
    	return marker;
	}

	geometry_msgs::msg::Pose cvToPose(const cv::Mat &R_cv, const cv::Mat &t_cv) {
    	geometry_msgs::msg::Pose pose;

    	// 提取平移向量
    	pose.position.x = t_cv.at<double>(0);
    	pose.position.y = -t_cv.at<double>(2);
    	pose.position.z = t_cv.at<double>(1);

    	// OpenCV 的 R -> Eigen 矩阵
    	Eigen::Matrix3d R_eigen;
    	for (int i = 0; i < 3; ++i)
        	for (int j = 0; j < 3; ++j)
            	R_eigen(i, j) = R_cv.at<double>(i, j);

    	// 转四元数
    	Eigen::Quaterniond q(R_eigen);

		// 构造绕 Z 轴旋转 90 度的四元数
    	Eigen::AngleAxisd rot_z(M_PI / 2, Eigen::Vector3d::UnitX());
    	Eigen::Quaterniond q_rot(rot_z);

    	// 将旋转 90 度应用到原始旋转上（q_rot * q）
    	Eigen::Quaterniond q_result = q_rot * q;

    	// 归一化
    	q_result.normalize();

    	pose.orientation.x = q_result.x();
    	pose.orientation.y = q_result.y();
    	pose.orientation.z = q_result.z();
    	pose.orientation.w = q_result.w();

    	return pose;
	}
	double calculateDistanceToCamera(
    	const cv::Mat& rvec,     //rotation
    	const cv::Mat& tvec,     //transmision
    	const cv::Point3f& point //target world location
		) {
    	cv::Mat rotationMatrix;
    	cv::Rodrigues(rvec, rotationMatrix);
	
    	
    	cv::Mat pointWorld = (cv::Mat_<double>(4, 1) << 
    	    point.x, point.y, point.z, 1);
	
    	
    	cv::Mat transformMatrix = cv::Mat::zeros(3, 4, CV_64F);
    	rotationMatrix.copyTo(transformMatrix(cv::Rect(0, 0, 3, 3)));
    	tvec.copyTo(transformMatrix(cv::Rect(3, 0, 1, 3)));
    
    	cv::Mat pointCamera = transformMatrix * pointWorld;
    
    	
    	double distance = std::sqrt(
    	    pointCamera.at<double>(0, 0) * pointCamera.at<double>(0, 0) +
    	    pointCamera.at<double>(1, 0) * pointCamera.at<double>(1, 0) +
    	    pointCamera.at<double>(2, 0) * pointCamera.at<double>(2, 0)
    	);
    
    	return distance;
	}

	void publish_transform(
    	const cv::Mat& rvec,     //rotation
    	const cv::Mat& tvec,     //transmision
    	const std::string& frame_id, //world frame
    	const std::string& child_frame_id ) {
			// publish translation and rotation information
			cv::Mat rvecs = rvec;  // 旋转向量（3x1矩阵）
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
    		transform_stamped.header.frame_id = frame_id; // 世界坐标系名称（需与 RViz 一致）
    		transform_stamped.child_frame_id = child_frame_id; // 相机坐标系名称

    		// 设置平移
    		transform_stamped.transform.translation.x = tvec.at<double>(0, 0);
    		transform_stamped.transform.translation.y = -tvec.at<double>(2, 0);
    		transform_stamped.transform.translation.z = tvec.at<double>(1, 0);

    		// 设置旋转（四元数）
    		transform_stamped.transform.rotation.x = q.x();
    		transform_stamped.transform.rotation.y = q.y();
    		transform_stamped.transform.rotation.z = q.z();
    		transform_stamped.transform.rotation.w = q.w();

    				
    		broadcaster_->sendTransform(transform_stamped);
	}

	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
     {
         try
         {
			step++;
			if(step % 10000 == 0){
				step = 0;
			}
			int64_t start = cv::getTickCount();
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

				// define real world position
				vector<cv::Point3f> objectPoints;
				objectPoints.push_back(cv::Point3f(0, markerLength, 0)); 
				objectPoints.push_back(cv::Point3f(markerLength, markerLength, 0)); 
				objectPoints.push_back(cv::Point3f(markerLength, 0, 0));
				objectPoints.push_back(cv::Point3f(0, 0, 0)); 

				//result matrix
				vector <cv::Mat> rvec, tvec;

				solve_pnp(cameraMatrix, distCoeffs, objectPoints, markerCorners, markerIds, aruco_marks);
				
				for (size_t i = 0; i < aruco_marks.size(); i++) {
					//drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec[i], tvec[i], 0.1);

					aruco_marks[i].update_ekf(dt);

					aruco_marks[i].draw_mark(frame, cameraMatrix, distCoeffs);
					
					//publish marker to rviz2
					cv::Mat tvec_ekf = aruco_marks[i].ekf->getCurrentPosition_Mat();

					geometry_msgs::msg::Pose pose = cvToPose(aruco_marks[i].marker_rvec, tvec_ekf);
					visualization_msgs::msg::Marker marker = makeMarker(pose);

					cv::Mat tvec_ekf_future = aruco_marks[i].ekf->getFuturePosition_Mat(extrapolate_dt, 0.65);

					geometry_msgs::msg::Pose pose_future = cvToPose(aruco_marks[i].marker_rvec, tvec_ekf_future);
					visualization_msgs::msg::Marker marker_future = makeMarker(pose_future, "camera", "Aruco_future", cv::Scalar(255, 0, 0));

					marker_pub_->publish(marker);
					marker_pub_future_->publish(marker_future);


					publish_transform(aruco_marks[i].marker_rvec, aruco_marks[i].marker_tvec, "camera", "Aruco");
				}
				cv::putText(frame, "Latency: " + to_string(dt) + "ms", cv::Point2f(5, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1.5, 8);
				cv::imshow("Image", frame);
				cv::waitKey(1);
				
			}
			int64_t end = cv::getTickCount();	
			dt = (end - start) * 1000 / cv::getTickFrequency();
			//cout << "dt = " << dt << endl;
         }
         catch (cv_bridge::Exception& e)
         {
             RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
         }
     }

private:
	
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_future_;
	cv::Mat image_mat_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_; 

	double dt = 1.0f;
	float extrapolate_dt = 40.0f;

	double markerLength = 0.048;  

	int step = 0;

	double alpha_q = 1e-5;
    double alpha_r = 1e-2;
	// 状态维度：位置 + 速度
	const int stateSize = 6;  // [x, y, z, vx, vy, vz]
	const int measSize = 3;   // [x, y, z] 观测只测位置

	vector<ArucoMark> aruco_marks;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoDetect>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}