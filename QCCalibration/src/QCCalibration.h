// QCCalibration.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
using namespace std;
using namespace cv;

class QCCalibration {
public :
	QCCalibration(int boardWidth, int boardHeight, int squareSize);
	~QCCalibration();
	void setPictureNums(int& imageNums);
	void changeBoardWidth(int& boardWidth);
	void changeBoardHeight(int& boardHeight);
	void changeSquareSize(int& squareSize);
	void changeBoardSize(int& boardWidth, int& boardHeight);
	void clear();
	bool fastCheckCorners(Mat image, vector<Point2f> corners);
	bool extractCorners(Mat leftImage, Mat rightImage);
	bool extractUndisCorners(Mat leftImage, Mat rightImage);
	
	void calObjectPoints();
	bool cameraCalibration();
	void reconstruction(Mat leftImage, Mat rightImage, int number);
	void convert2Object();
	bool checkAndDrawCorners(cv::Mat& image);
	void undistortion_iterate(Mat& leftImage, Mat& rightImage);
	void undistortion_gaussionNewton(Mat& leftImage, Mat& rightImage);
	void readData();
	void vertify_undistortion();
	void vertify_triangular(int number);
	void vertify_projection(int number);



private:

	int pictureNums;
	int cornerNums;
	int squareSize;
	int boardWidth, boardHeight;
	Size board_Size;
	Size camaraSize;
	string result_yml = "result.yml";

	Mat cameraMatrix_L = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Mat Cd_L		   = Mat(1, 5, CV_32FC1, Scalar::all(0));
	Mat cameraMatrix_R = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Mat Cd_R		   = Mat(1, 5, CV_32FC1, Scalar::all(0));


	vector<Mat> R_L, T_L, R_R, T_R;
	Mat R, T, E, F;
	Mat trans;
	

	vector<Point3f> objectPoints;
	vector<vector<Point3f>>  allObjectPoints;

	vector<Point3f> image2ObjectPoints_L;
	vector<Point3f> image2ObjectPoints_R;
	
	vector<Point2f> object2ImagePoints_L;
	vector<Point2f> object2ImagePoints_R;
	vector<vector<Point3f>>  allLeftImage2ObjectPoints;
	vector<vector<Point3f>>  allRightImage2ObjectPoints;
	vector<Point2f> cameraCorners_L;
	vector<Point2f> cameraCorners_R;

	vector<vector<Point2f>> allCameraCorners_L;
	vector<vector<Point2f>> allCameraCorners_R;
	vector<vector<Point2f>> allObject2ImagePoints_L;
	vector<vector<Point2f>> allObject2ImagePoints_R;

	vector<Point3d> camera_R_all;
	vector<Point3f> camera_L_all;
	vector<Point2f> pix_R_all;
	vector<Point2f> pix_L_all;
	vector<Point2f> pix_R2L_all;

	vector<Point2f> undisCameraCorners_L;
	vector<Point2f> undisCameraCorners_R;
	vector<vector<Point2f>> allUndisCameraCorners_L;
	vector<vector<Point2f>> allUndisCameraCorners_R;

};







// TODO: 在此处引用程序需要的其他标头。
