#include <aruco_mark.hpp>

ArucoMark::ArucoMark(int id, std::vector<cv::Point2f> markerCorners, cv::Mat marker_rvec, cv::Mat marker_tvec){
    this->id = id;
    this->markerCorners = markerCorners;
    this->marker_rvec = marker_rvec;
    this->marker_tvec = marker_tvec;
    this->ekf = new EKF(alpha_q, alpha_r, stateSize, measSize);

    objectPoints.push_back(cv::Point3f(0, markerLength, 0)); 
	objectPoints.push_back(cv::Point3f(markerLength, markerLength, 0)); 
	objectPoints.push_back(cv::Point3f(markerLength, 0, 0));
	objectPoints.push_back(cv::Point3f(0, 0, 0)); 
}

void ArucoMark::update_ekf(double dt){
    ekf->update(marker_tvec, dt);
    marker_tvec_prev = ekf->getCurrentPosition_Mat();
    marker_tvec_future = ekf->getFuturePosition_Mat(extrapolate_dt, 0.7);
    marker_position_future = ekf->predictFuturePosition(extrapolate_dt, 0.7);
    marker_position_prev = ekf->getCurrentPosition();
}

void ArucoMark::draw_mark(cv::Mat &frame, cv::Mat &cameraMatrix, cv::Mat &distCoeffs){
    cv::Mat camera_location = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	cv::Mat camera_rotation = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	vector<cv::Point3f> objPts = { marker_position_prev, marker_position_future};  // 3D点
	vector<cv::Point2f> imgPts;
	cv::projectPoints(objPts, camera_rotation, camera_location, cameraMatrix, distCoeffs, imgPts);
	if(!isLost){
        for(size_t i = 0; i < imgPts.size(); i++){
		    if(i == 0){
		    	cv::circle(frame, imgPts[i], 5, cv::Scalar(0, 0, 255), -1);
		    }else if(i == 1){
		    	cv::circle(frame, imgPts[i], 5, cv::Scalar(0, 255, 0), -1);
		    	cv::line(frame, imgPts[i-1], imgPts[i], cv::Scalar(0, 255, 0), 2);
		    }else if(i ==2){
		    	cv::circle(frame, imgPts[i], 5, cv::Scalar(255, 0, 0), -1);
		    	cv::line(frame, imgPts[i-1], imgPts[i], cv::Scalar(255, 0, 0), 2);
		    }
	    }
    }
	

    vector<cv::Point2f> imgPts_rect;
	cv::Mat tvec_rect = (cv::Mat_<double>(3, 1) << 
		marker_position_future.x, marker_position_future.y, marker_position_future.z);

	cv::projectPoints(objectPoints, marker_rvec, tvec_rect, cameraMatrix, distCoeffs, imgPts_rect);
	for(size_t i = 0; i < imgPts_rect.size(); i++){
        if(isLost){
			cv::putText(frame, "Target lost[ID" + to_string(id) + "]", markerCorners[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 1.5, 8);
            cv::line(frame, imgPts_rect[i], imgPts_rect[(i+1)%4], cv::Scalar(0, 0, 255), 1);
        }else{
			cv::putText(frame, "Tracking[ID" + to_string(id) + "]", markerCorners[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1.5, 8);
            cv::line(frame, imgPts_rect[i], imgPts_rect[(i+1)%4], cv::Scalar(0, 255, 0), 1);
        }
		
	}

}