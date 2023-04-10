

// QCCalibration.cpp: 定义应用程序的入口点。
//

#include "QCCalibration.h"
#include <filesystem>
#include <algorithm>
using namespace std;
using namespace cv;

QCCalibration::QCCalibration(int boardWidth, int boardHeight, int squareSize)
{
	this->boardWidth = boardWidth;
	this->boardHeight = boardHeight;
	this->squareSize = squareSize;
	cornerNums = boardWidth * boardHeight;
	board_Size = Size(boardWidth, boardHeight);
	clear();
	
}

QCCalibration::~QCCalibration(){}

void QCCalibration::setPictureNums(int& imageNums)
{
	int default_pictureNums = 3;
	if (imageNums < default_pictureNums)
	{
		cout << "images are not enough" << endl;

	}
	else
	{
		pictureNums = imageNums;
	}
}

void QCCalibration::changeBoardWidth(int& width)
{
	boardWidth = width;
}

void QCCalibration::changeBoardHeight(int& height)
{
	boardHeight = height;

}

void QCCalibration::changeSquareSize(int& squareSize)
{
	this->squareSize = squareSize;
}

void QCCalibration::changeBoardSize(int& width, int& height)
{
	board_Size = Size(width, height);
}

void QCCalibration::clear()
{
	cameraCorners_L.clear();
	cameraCorners_R.clear();
	undisCameraCorners_L.clear();
	undisCameraCorners_R.clear();
	image2ObjectPoints_L.clear();
	image2ObjectPoints_R.clear();
	allCameraCorners_L.clear();
	allCameraCorners_R.clear();
	allUndisCameraCorners_L.clear();
	allUndisCameraCorners_R.clear();
}

bool QCCalibration::fastCheckCorners(Mat image, vector<Point2f> corners)
{
	return cv::findChessboardCorners(image, board_Size, corners);
}

bool QCCalibration::extractCorners(Mat leftImage, Mat rightImage)
{
	int i;
	
	if (fastCheckCorners(leftImage, cameraCorners_L) == false || fastCheckCorners(rightImage, cameraCorners_R)== false) {
		cout << "not success" << endl;
		i = 0;
	}
	else {

		findChessboardCorners(leftImage, board_Size, cameraCorners_L);
		cornerSubPix(leftImage, cameraCorners_L, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
		allCameraCorners_L.push_back(cameraCorners_L);

		findChessboardCorners(rightImage, board_Size, cameraCorners_R);
		cornerSubPix(rightImage, cameraCorners_R, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
		allCameraCorners_R.push_back(cameraCorners_R);
		camaraSize = Size(leftImage.cols, leftImage.rows);
		i = 1;
	}
	return i;
}

bool QCCalibration::extractUndisCorners(Mat leftImage, Mat rightImage)
{
	int i;

	if (fastCheckCorners(leftImage, cameraCorners_L) == false || fastCheckCorners(rightImage, cameraCorners_R) == false) {
		cout << "not success" << endl;
		i = 0;
	}
	else {
		cout << "success extract undiscorners" << endl;
		findChessboardCorners(leftImage, board_Size, undisCameraCorners_L);
		cornerSubPix(leftImage, undisCameraCorners_L, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
		allUndisCameraCorners_L.push_back(undisCameraCorners_L);

		findChessboardCorners(rightImage, board_Size, undisCameraCorners_R);
		cornerSubPix(rightImage, undisCameraCorners_R, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
		allUndisCameraCorners_R.push_back(undisCameraCorners_R);
		camaraSize = Size(leftImage.cols, leftImage.rows);
		i = 1;
	}
	
	return i;
}

void QCCalibration::calObjectPoints() 
{

	for (int i = 0; i < boardHeight; i++)
	{
		for (int j = 0; j < boardWidth; j++)
		{
			
			objectPoints.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
		}
	}
	
	
	for (int k = 0; k < pictureNums; k++)
	{
		allObjectPoints.push_back(objectPoints);
	}
	cout << "size " << allObjectPoints.size() << endl;
	
}


bool QCCalibration::cameraCalibration()
{
	if (allCameraCorners_L.size() < pictureNums)
	{
		cout << "images are not enough" << endl;
		return false;
	}
	else
	{
		cout << "starting calibrating cameras" << endl;
	}

	calObjectPoints();

	double leftCameraError = cv::calibrateCamera(allObjectPoints, allCameraCorners_L, camaraSize, cameraMatrix_L, Cd_L, R_L, T_L,0,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 150, DBL_EPSILON));
	
	cout << "R_L.size" << R_L.size() << endl;
	double rightCameraError = cv::calibrateCamera(allObjectPoints, allCameraCorners_R, camaraSize, cameraMatrix_R, Cd_R, R_R, T_R, 0,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 150, DBL_EPSILON));

	double stereoCameraError = cv::stereoCalibrate(allObjectPoints, allCameraCorners_L, allCameraCorners_R, cameraMatrix_L, Cd_L, cameraMatrix_R, Cd_R, camaraSize, R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS, TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 150, DBL_EPSILON));
	

	//store
	FileStorage fs(result_yml, FileStorage::WRITE);
	if (!fs.isOpened())
	{
		cout << "fail to open file" << endl;
	}
	else
	{
		cout << "open file" << endl;
		fs << "cameraMatrix_L" << cameraMatrix_L << "Cd_L" << Cd_L<<"R_L"<<R_L<<"T_L"<<T_L
			<< "cameraMatrix_R" << cameraMatrix_R << "Cd_R" << Cd_R<<"R_R"<<R_R<<"T_R"<<T_R
			<< "R" << R << "T" << T << "E" << E << "F" << F
			<< "leftCameraError" << leftCameraError << "rightCameraError" << rightCameraError << "stereoCameraError" << stereoCameraError;

		

		fs.release();

	}
	
	cout << "leftCameraError" << leftCameraError << endl;
	cout << "rightCameraError" << rightCameraError << endl;
	cout << "stereoCameraError" << stereoCameraError << endl;
	cout << "Calibration Finished" << endl;
	return true;
}
/*
int ReconstructionManager::calcu_XYZ_left(float x, float y, float x_p, float y_p, float& X, float& Y, float& Z)
{
	float M_XX[3][3], M_YY[3], M_XX_inv[3][3], M_end[3][3], result_end[3] = { 0, 0, 0 };

	M_XX[0][0] = x * M_1L[8] - M_1L[0];
	M_XX[0][1] = x * M_1L[9] - M_1L[1];
	M_XX[0][2] = x * M_1L[10] - M_1L[2];


	M_XX[1][0] = y * M_1L[8] - M_1L[4];
	M_XX[1][1] = y * M_1L[9] - M_1L[5];
	M_XX[1][2] = y * M_1L[10] - M_1L[6];


	M_XX[2][0] = x_p * M_2L[8] - M_2L[0];
	M_XX[2][1] = x_p * M_2L[9] - M_2L[1];
	M_XX[2][2] = x_p * M_2L[10] - M_2L[2];

	M_YY[0] = M_1L[3] - x * M_1L[11];
	M_YY[1] = M_1L[7] - y * M_1L[11];
	M_YY[2] = M_2L[3] - x_p * M_2L[11];

	// M_XX_inv
	float P = M_XX[0][0] * (M_XX[1][1] * M_XX[2][2] - M_XX[1][2] * M_XX[2][1]) - M_XX[1][0] * (M_XX[0][1] * M_XX[2][2] - M_XX[0][2] * M_XX[2][1]) + M_XX[2][0] * (M_XX[0][1] * M_XX[1][2] - M_XX[0][2] * M_XX[1][1]);


	if (abs(P) > 10e-5)
	{
		M_XX_inv[0][0] = (M_XX[1][1] * M_XX[2][2] - M_XX[1][2] * M_XX[2][1]) / P;
		M_XX_inv[1][0] = (M_XX[1][2] * M_XX[2][0] - M_XX[1][0] * M_XX[2][2]) / P;
		M_XX_inv[2][0] = (M_XX[1][0] * M_XX[2][1] - M_XX[1][1] * M_XX[2][0]) / P;
		M_XX_inv[0][1] = (M_XX[0][2] * M_XX[2][1] - M_XX[0][1] * M_XX[2][2]) / P;
		M_XX_inv[1][1] = (M_XX[0][0] * M_XX[2][2] - M_XX[0][2] * M_XX[2][0]) / P;
		M_XX_inv[2][1] = (M_XX[0][1] * M_XX[2][0] - M_XX[0][0] * M_XX[2][1]) / P;
		M_XX_inv[0][2] = (M_XX[0][1] * M_XX[1][2] - M_XX[0][2] * M_XX[1][1]) / P;
		M_XX_inv[1][2] = (M_XX[1][0] * M_XX[0][2] - M_XX[0][0] * M_XX[1][2]) / P;
		M_XX_inv[2][2] = (M_XX[0][0] * M_XX[1][1] - M_XX[1][0] * M_XX[0][1]) / P;
		for (int m = 0; m < 3; m++)
		{
			for (int n = 0; n < 3; n++)
			{
				result_end[m] += M_XX_inv[m][n] * M_YY[n];
			}

		}
		X = result_end[0];
		Y = result_end[1];
		Z = result_end[2];
	}
	else
	{
		X = 0;
		Y = 0;
		Z = 0;
	}

	return t;
}
*/


void QCCalibration::reconstruction(Mat leftImage, Mat rightImage, int number)
{

	
	//convert transform matrix to 4x4
	Mat assist;
	hconcat(R, T, assist);
	Mat addition = Mat(1, 4, CV_32FC1, Scalar(0));
	addition.at<float>(0, 3) = 1;
	addition.convertTo(addition, CV_32FC1);
	assist.convertTo(assist, CV_32FC1);
	vconcat(assist, addition, trans);


	//expand camera matrix to 3x4
	Mat addition_inner = Mat(3, 1, CV_32FC1, Scalar(0));
	Mat cameraMatrix_L1;
	Mat cameraMatrix_R1;
	addition_inner.convertTo(addition_inner, CV_32FC1);
	cameraMatrix_L.convertTo(cameraMatrix_L, CV_32FC1);
	cameraMatrix_R.convertTo(cameraMatrix_R, CV_32FC1);
	hconcat(cameraMatrix_L, addition_inner, cameraMatrix_L1);
	hconcat(cameraMatrix_R, addition_inner, cameraMatrix_R1);

	Mat P;
	Mat M;

	P = cameraMatrix_R1 * trans;
	M = cameraMatrix_L1;

	float m11 = M.at<float>(0, 0);
	float m12 = M.at<float>(0, 1);
	float m13 = M.at<float>(0, 2);
	float m14 = M.at<float>(0, 3);
	float m21 = M.at<float>(1, 0);
	float m22 = M.at<float>(1, 1);
	float m23 = M.at<float>(1, 2);
	float m24 = M.at<float>(1, 3);
	float m31 = M.at<float>(2, 0);
	float m32 = M.at<float>(2, 1);
	float m33 = M.at<float>(2, 2);
	float m34 = M.at<float>(2, 3);
	/*
	double p11 = P.at<double>(0, 0);
	double p12 = P.at<double>(0, 1);
	double p13 = P.at<double>(0, 2);
	double p14 = P.at<double>(0, 3);
	double p21 = P.at<double>(1, 0);
	double p22 = P.at<double>(1, 1);
	double p23 = P.at<double>(1, 2);
	double p24 = P.at<double>(1, 3);
	double p31 = P.at<double>(2, 0);
	double p32 = P.at<double>(2, 1);
	double p33 = P.at<double>(2, 2);
	double p34 = P.at<double>(2, 3);
	*/
	float p11 = P.at<float>(0, 0);
	float p12 = P.at<float>(0, 1);
	float p13 = P.at<float>(0, 2);
	float p14 = P.at<float>(0, 3);
	float p21 = P.at<float>(1, 0);
	float p22 = P.at<float>(1, 1);
	float p23 = P.at<float>(1, 2);
	float p24 = P.at<float>(1, 3);
	float p31 = P.at<float>(2, 0);
	float p32 = P.at<float>(2, 1);
	float p33 = P.at<float>(2, 2);
	float p34 = P.at<float>(2, 3);


	Mat A = Mat(4, 3, CV_32FC1);
	Mat B = Mat(4, 1, CV_32FC1);
	Mat C;
	Mat D;
	Mat world_L;
	Mat world_R;
	Mat camera_R;
	Mat camera_L;

	
	image2ObjectPoints_L.clear();
	image2ObjectPoints_R.clear();
	for (int i = 0; i < undisCameraCorners_L.size(); i++)
	{
		float ul = undisCameraCorners_L[i].x;
		float vl = undisCameraCorners_L[i].y;
		float ur = undisCameraCorners_R[i].x;
		float vr = undisCameraCorners_R[i].y;

		A.at<float>(0, 0) = ul * m31 - m11;
		A.at<float>(0, 1) = ul * m32 - m12;
		A.at<float>(0, 2) = ul * m33 - m13;
		A.at<float>(1, 0) = vl * m31 - m21;
		A.at<float>(1, 1) = vl * m32 - m22;
		A.at<float>(1, 2) = vl * m33 - m23;
		A.at<float>(2, 0) = ur * p31 - p11;
		A.at<float>(2, 1) = ur * p32 - p12;
		A.at<float>(2, 2) = ur * p33 - p13;
		A.at<float>(3, 0) = vr * p31 - p21;
		A.at<float>(3, 1) = vr * p32 - p22;
		A.at<float>(3, 2) = vr * p33 - p23;

		B.at<float>(0, 0) = m14 - ul*m34;
		B.at<float>(1, 0) = m24 - vl*m34;
		B.at<float>(2, 0) = p14 - ur*p34;
		B.at<float>(3, 0) = p24 - vr*p34;
		
		D = A.t() * A;
		invert(D, C);

		camera_L = C * A.t() * B;
		//convert 3x1 to 4x1

		Mat undisCorners_additional1 = (Mat_<float>(4, 1) << camera_L.at<float>(0,0), camera_L.at<float>(1, 0), camera_L.at<float>(2, 0), 1);
		Mat pix = cameraMatrix_L * camera_L;
		Mat undisCorners_additional2 = trans * undisCorners_additional1;
		//convert 4x1 to 3x1
		camera_R = (Mat_<float>(3, 1) << undisCorners_additional2.at<float>(0, 0), undisCorners_additional2.at<float>(1, 0), undisCorners_additional2.at<float>(2, 0));
		pix_L_all.push_back(Point2f(pix.at<float>(0, 0) / pix.at<float>(2, 0), pix.at<float>(1, 0) / pix.at<float>(2, 0)));
		image2ObjectPoints_R.push_back(Point3f(camera_R.at<float>(0, 0), camera_R.at<float>(1, 0), camera_R.at<float>(2.0)));
		image2ObjectPoints_L.push_back(Point3f(camera_L.at<float>(0, 0), camera_L.at<float>(1, 0), camera_L.at<float>(2.0)));


	}

	 
	allRightImage2ObjectPoints.push_back(image2ObjectPoints_R);
	allLeftImage2ObjectPoints.push_back(image2ObjectPoints_L);


	
}
	
	



bool QCCalibration::checkAndDrawCorners(cv::Mat& image)
{
	std::vector<cv::Point2f> corners;
	bool found = cv::findChessboardCorners(image, board_Size, corners, cv::CALIB_CB_FAST_CHECK);
	if (found) { drawChessboardCorners(image, board_Size, corners, found); }

	return found;
}




void QCCalibration::undistortion_iterate(Mat& leftImage, Mat& rightImage)
{
	double k1_L = Cd_L.at<double>(0, 0);
	double k2_L = Cd_L.at<double>(0, 1);
	double p1_L = Cd_L.at<double>(0, 2);
	double p2_L = Cd_L.at<double>(0, 3);
	double k3_L = Cd_L.at<double>(0, 4);

	double k1_R = Cd_R.at<double>(0, 0);
	double k2_R = Cd_R.at<double>(0, 1);
	double p1_R = Cd_R.at<double>(0, 2);
	double p2_R = Cd_R.at<double>(0, 3);
	double k3_R = Cd_R.at<double>(0, 4);

	double fx_L = cameraMatrix_L.at<double>(0, 0);
	double fy_L = cameraMatrix_L.at<double>(1, 1);
	double cx_L = cameraMatrix_L.at<double>(0, 2);
	double cy_L = cameraMatrix_L.at<double>(1, 2);

	double fx_R = cameraMatrix_R.at<double>(0, 0);
	double fy_R = cameraMatrix_R.at<double>(1, 1);
	double cx_R = cameraMatrix_R.at<double>(0, 2);
	double cy_R = cameraMatrix_R.at<double>(1, 2);



	extractCorners(leftImage, rightImage);
	checkAndDrawCorners(leftImage);
	checkAndDrawCorners(rightImage);

	

	// distort the left image
	for (int i = 0; i < cameraCorners_L.size(); i++)
	{
		double ul = cameraCorners_L[i].x;
		double vl = cameraCorners_L[i].y;
		double xl = (ul - cx_L) / fx_L;
		double yl = (vl - cy_L) / fy_L;
		
		
		double xl_0 = xl;
		double yl_0 = yl;
		
		
		double ur = cameraCorners_R[i].x;
		double vr = cameraCorners_R[i].y;
		double xr = (ur - cx_R) / fx_R;
		double yr = (vr - cy_R) / fy_R;
		
		double xr_0 = xr ;
		double yr_0 = yr ;

		double rl_square = xl * xl + yl * yl;
		double first_item_l = 1 + k1_L * rl_square + k2_L * rl_square * rl_square + k3_L * rl_square * rl_square * rl_square;
		double x_second_item_l = 2 * p1_L * xl * yl + p2_L * (rl_square + 2 * xl * xl);
		double y_second_item_l = 2 * p2_L * xl * yl + p1_L * (rl_square + 2 * yl * yl);

		double rr_square = xr * xr + yr * yr;
		double first_item_r = 1 + k1_R * rr_square + k2_R * rr_square * rr_square + k3_R * rr_square * rr_square * rr_square;
		double x_second_item_r = 2 * p1_R * xr * yr + p2_R * (rr_square + 2 * xr * xr);
		double y_second_item_r = 2 * p2_R * xr * yr + p1_R * (rr_square + 2 * yr * yr);
		double error_l = 100.12165;
		double error_r = 100.84984;
		for (int j = 0;j<300; j++)
		{
			if ( error_l < DBL_EPSILON && error_r < DBL_EPSILON)
			{
				cout << "j=" << j << endl;
				break;
			}
			
			xl = (xl_0 - x_second_item_l) / first_item_l;
			yl = (yl_0 - y_second_item_l) / first_item_l;
			cout << "xl=" << xl << endl;
			cout << "yl=" << yl << endl;
			cout << "first=" << first_item_l << endl;
			rl_square = xl * xl + yl * yl;
			first_item_l = 1 + k1_L * rl_square + k2_L * rl_square * rl_square + k3_L * rl_square * rl_square * rl_square;
			x_second_item_l = 2 * p1_L * xl * yl + p2_L * (rl_square + 2 * xl * xl);
			y_second_item_l = 2 * p2_L * xl * yl + p1_L * (rl_square + 2 * yl * yl);
			double xl_dis = xl * first_item_l + x_second_item_l;
			double yl_dis = yl * first_item_l + y_second_item_l;

			
			xr = (xr_0 - x_second_item_r) / first_item_r;
			yr = (yr_0 - y_second_item_r) / first_item_r;

			rr_square = xr * xr + yr * yr;
			first_item_r = 1 + k1_R * rr_square + k2_R * rr_square * rr_square + k3_R * rr_square * rr_square * rr_square;
			x_second_item_r = 2 * p1_R * xr * yr + p2_R * (rr_square + 2 * xr * xr);
			y_second_item_r = 2 * p2_R * xr * yr + p1_R * (rr_square + 2 * yr * yr);
			double xr_dis = xr * first_item_r + x_second_item_r;
			double yr_dis = yr * first_item_r + y_second_item_r;

			double ul_dis = xl_dis * fx_L + cx_L;
			double vl_dis = yl_dis * fy_L + cy_L;

			double ur_dis = xr_dis * fx_R + cx_R;
			double vr_dis = yr_dis * fy_R + cy_R;
			// calculate the distorted coordinate of current point and calculate the error
			error_l = sqrt(pow((ul - ul_dis), 2) + pow((vl - vl_dis), 2));
			error_r = sqrt(pow((ur - ur_dis), 2) + pow((vr - vr_dis), 2));


		}
		double ul_proj = xl * fx_L + cx_L;
		double vl_proj = yl * fy_L + cy_L;
		double ur_proj = xr * fx_R + cx_R;
		double vr_proj = yr * fy_R + cy_R;
		undisCameraCorners_L.push_back(Point2d(ul_proj, vl_proj));
		undisCameraCorners_R.push_back(Point2d(ur_proj, vr_proj));
	
	}
	
	allUndisCameraCorners_L.push_back(undisCameraCorners_L);
	allUndisCameraCorners_R.push_back(undisCameraCorners_R);

	cout << "end" << endl;
	

}



void QCCalibration::undistortion_gaussionNewton(Mat& leftImage, Mat& rightImage)
{

	undisCameraCorners_L.clear();
	undisCameraCorners_R.clear();
	double k1_L = Cd_L.at<double>(0, 0);
	double k2_L = Cd_L.at<double>(0, 1);
	double p1_L = Cd_L.at<double>(0, 2);
	double p2_L = Cd_L.at<double>(0, 3);
	double k3_L = Cd_L.at<double>(0, 4);

	double k1_R = Cd_R.at<double>(0, 0);
	double k2_R = Cd_R.at<double>(0, 1);
	double p1_R = Cd_R.at<double>(0, 2);
	double p2_R = Cd_R.at<double>(0, 3);
	double k3_R = Cd_R.at<double>(0, 4);

	double fx_L = cameraMatrix_L.at<double>(0, 0);
	double fy_L = cameraMatrix_L.at<double>(1, 1);
	double cx_L = cameraMatrix_L.at<double>(0, 2);
	double cy_L = cameraMatrix_L.at<double>(1, 2);

	double fx_R = cameraMatrix_R.at<double>(0, 0);
	double fy_R = cameraMatrix_R.at<double>(1, 1);
	double cx_R = cameraMatrix_R.at<double>(0, 2);
	double cy_R = cameraMatrix_R.at<double>(1, 2);



	extractCorners(leftImage, rightImage);
	checkAndDrawCorners(leftImage);
	checkAndDrawCorners(rightImage);

	Mat Jacobi = Mat(2, 2, CV_64FC1);
	Mat F = Mat(2, 1, CV_64FC1);
	Mat delta_X = Mat(2, 1, CV_64FC1,Scalar(1));
	Mat J_invert;

	//distort the left image
	
	for (int i = 0; i < cameraCorners_L.size(); i++)
	{
		//calculate x0 and y0
		double ul = cameraCorners_L[i].x;
		double vl = cameraCorners_L[i].y;
		double xl = (ul - cx_L) / fx_L;
		double yl = (vl - cy_L) / fy_L;


		double xl_0 = xl;
		double yl_0 = yl;


		double ur = cameraCorners_R[i].x;
		double vr = cameraCorners_R[i].y;
		double xr = (ur - cx_R) / fx_R;
		double yr = (vr - cy_R) / fy_R;

		double xr_0 = xr;
		double yr_0 = yr;

		double rl_square = xl * xl + yl * yl;
		double first_item_l = 1 + k1_L * rl_square + k2_L * rl_square * rl_square + k3_L * rl_square * rl_square * rl_square;
		double x_second_item_l = 2 * p1_L * xl * yl + p2_L * (rl_square + 2 * xl * xl);
		double y_second_item_l = 2 * p2_L * xl * yl + p1_L * (rl_square + 2 * yl * yl);

		double rr_square = xr * xr + yr * yr;
		double first_item_r = 1 + k1_R * rr_square + k2_R * rr_square * rr_square + k3_R * rr_square * rr_square * rr_square;
		double x_second_item_r = 2 * p1_R * xr * yr + p2_R * (rr_square + 2 * xr * xr);
		double y_second_item_r = 2 * p2_R * xr * yr + p1_R * (rr_square + 2 * yr * yr);
		double error_l = 100.12165;
		double error_r = 100.84984;

		//set initial value of x and y
		

		

		for (int j = 0;j<300; j++)
		{
			

			if (error_l < DBL_EPSILON && error_r<DBL_EPSILON)
			{
				break;
			}
			rl_square = xl * xl + yl * yl;
			first_item_l = 1 + k1_L * rl_square + k2_L * rl_square * rl_square + k3_L * rl_square * rl_square * rl_square;
			x_second_item_l = 2 * p1_L * xl * yl + p2_L * (rl_square + 2 * xl * xl);
			y_second_item_l = 2 * p2_L * xl * yl + p1_L * (rl_square + 2 * yl * yl);

			F.at<double>(0, 0) = xl * first_item_l + x_second_item_l - xl_0;
			F.at<double>(1, 0) = yl * first_item_l + y_second_item_l - yl_0;

			Jacobi.at<double>(0, 0) = first_item_l + xl * (2 * k1_L * xl + 4 * k2_L * xl * rl_square + 6 * k3_L * xl * rl_square * rl_square) + 2 * p1_L * yl + 6 * xl * p2_L;
			Jacobi.at<double>(0, 1) = xl * (2 * yl * k1_L + 4 * yl * k2_L * rl_square + 6 * yl * k3_L * rl_square * rl_square) + 2 * p1_L * xl + 2 * p2_L * yl;
			Jacobi.at<double>(1, 0) = yl * (2 * k1_L * xl + 4 * k2_L * xl * rl_square + 6 * k3_L * xl * rl_square * rl_square) + 2 * p2_L * yl + 2 * p1_L * xl;
			Jacobi.at<double>(1, 1) = first_item_l + yl * (2 * k1_L * yl + 4 * k2_L * yl * rl_square + 6 * k3_L * yl * rl_square * rl_square) + 2 * p2_L * xl + 6 * yl * p1_L;
			
			invert(Jacobi.t() * Jacobi, J_invert);
			delta_X = J_invert * (-Jacobi.t() * F);
			double error_l = sqrt(pow(delta_X.at<double>(0, 0), 2) + pow(delta_X.at<double>(1, 0), 2));

			xl = xl + delta_X.at<double>(0, 0);
			yl = yl + delta_X.at<double>(1, 0);

			rr_square = xr * xr + yr * yr;
			first_item_r = 1 + k1_R * rr_square + k2_R * rr_square * rr_square + k3_R * rr_square * rr_square * rr_square;
			x_second_item_r = 2 * p1_R * xr * yr + p2_R * (rr_square + 2 * xr * xr);
			y_second_item_r = 2 * p2_R * xr * yr + p1_R * (rr_square + 2 * yr * yr);

			F.at<double>(0, 0) = xr * first_item_r + x_second_item_r - xr_0;
			F.at<double>(1, 0) = yr * first_item_r + y_second_item_r - yr_0;

			Jacobi.at<double>(0, 0) = first_item_r + xr * (2 * k1_R * xr + 4 * k2_R * xr * rr_square + 6 * k3_R * xr * rr_square * rr_square) + 2 * p1_R * yr + 6 * xr * p2_R;
			Jacobi.at<double>(0, 1) = xr * (2 * yr * k1_R + 4 * yr * k2_R * rr_square + 6 * yr * k3_R * rr_square * rr_square) + 2 * p1_R * xr + 2 * p2_R * yr;
			Jacobi.at<double>(1, 0) = yr * (2 * k1_R * xr + 4 * k2_R * xr * rr_square + 6 * k3_R * xr * rr_square * rr_square) + 2 * p2_R * yr + 2 * p1_R * xr;
			Jacobi.at<double>(1, 1) = first_item_r + yr * (2 * k1_R * yr + 4 * k2_R * yr * rr_square + 6 * k3_R * yr * rr_square * rr_square) + 2 * p2_R * xr + 6 * yr * p1_R;



			invert(Jacobi.t() * Jacobi, J_invert);
			delta_X = J_invert * (-Jacobi.t() * F);
			double error_r = sqrt(pow(delta_X.at<double>(0, 0), 2) + pow(delta_X.at<double>(1, 0), 2));

			xr = xr + delta_X.at<double>(0, 0);
			yr = yr + delta_X.at<double>(1, 0);



		}
		double ul_proj = xl * fx_L + cx_L;
		double vl_proj = yl * fy_L + cy_L;
		undisCameraCorners_L.push_back(Point2d(ul_proj, vl_proj));
		double ur_proj = xr * fx_R + cx_R;
		double vr_proj = yr * fy_R + cy_R;
		undisCameraCorners_R.push_back(Point2d(ur_proj, vr_proj));

	}
	allUndisCameraCorners_L.push_back(undisCameraCorners_L);
	allUndisCameraCorners_R.push_back(undisCameraCorners_R);
}





void QCCalibration::readData()
{
	cv::FileStorage fs(result_yml, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		cout << "read the data" << endl;
		fs["cameraMatrix_L"] >> cameraMatrix_L;
		fs["cameraMatrix_R"] >> cameraMatrix_R;
		fs["Cd_L"] >> Cd_L;
		fs["Cd_R"] >> Cd_R;
		fs["R_L"] >> R_L;
		fs["R_R"] >> R_R;
		fs["T_L"] >> T_L;
		fs["T_R"] >> T_R;
		fs["R"] >> R;
		fs["T"] >> T;
		fs["E"] >> E;
		fs["F"] >> F;

	}
	else {
		cout << "result doesn't exist" << endl;
	}
}


void QCCalibration::vertify_undistortion()
{
	undisCameraCorners_L.clear();
	undisCameraCorners_R.clear();
	cv::undistortPoints(cameraCorners_L, undisCameraCorners_L, cameraMatrix_L, Cd_L,cv::noArray(), cameraMatrix_L);
	cv::undistortPoints(cameraCorners_R, undisCameraCorners_R, cameraMatrix_R, Cd_R,cv::noArray(), cameraMatrix_R);
	cout << "end" << endl;
}

void QCCalibration::vertify_triangular(int number)
{
	

	Mat camera_L= Mat(3, 1, CV_64FC1);
	Mat camera_R= Mat(3, 1, CV_64FC1);
	Mat world_coordinate;
	Mat left = Mat(2,1,CV_64FC1);
	Mat right = Mat(2, 1, CV_64FC1);

	double k1_L = Cd_L.at<double>(0, 0);
	double k2_L = Cd_L.at<double>(0, 1);
	double p1_L = Cd_L.at<double>(0, 2);
	double p2_L = Cd_L.at<double>(0, 3);
	double k3_L = Cd_L.at<double>(0, 4);

	double k1_R = Cd_R.at<double>(0, 0);
	double k2_R = Cd_R.at<double>(0, 1);
	double p1_R = Cd_R.at<double>(0, 2);
	double p2_R = Cd_R.at<double>(0, 3);
	double k3_R = Cd_R.at<double>(0, 4);

	double fx_L = cameraMatrix_L.at<double>(0, 0);
	double fy_L = cameraMatrix_L.at<double>(1, 1);
	double cx_L = cameraMatrix_L.at<double>(0, 2);
	double cy_L = cameraMatrix_L.at<double>(1, 2);

	double fx_R = cameraMatrix_R.at<double>(0, 0);
	double fy_R = cameraMatrix_R.at<double>(1, 1);
	double cx_R = cameraMatrix_R.at<double>(0, 2);
	double cy_R = cameraMatrix_R.at<double>(1, 2);




	Mat assist;
	hconcat(R, T, trans);

	Mat T1 = (Mat_<double>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);

	for (int i = 0; i < cameraCorners_L.size(); i++)
	{


		left.at<double>(0, 0) = undisCameraCorners_L[i].x;
		left.at<double>(1, 0) = undisCameraCorners_L[i].y;
		right.at<double>(0, 0) = undisCameraCorners_R[i].x;
		right.at<double>(1, 0) = undisCameraCorners_R[i].y;
		
		left.at<double>(0, 0) = (left.at<double>(0, 0) - cx_L) / fx_L;
		left.at<double>(1, 0) = (left.at<double>(1, 0) - cy_L) / fy_L;
		right.at<double>(0, 0) = (right.at<double>(0, 0) - cx_R) / fx_R;
		right.at<double>(1, 0) = (right.at<double>(1, 0) - cy_R) / fy_R;

		Mat proMat1 = T1;
		Mat proMat2 = trans;
		
		cv::triangulatePoints(proMat1, proMat2, left, right, world_coordinate);

		world_coordinate.at<double>(0, 0) = world_coordinate.at<double>(0, 0) / world_coordinate.at<double>(3, 0);
		world_coordinate.at<double>(1, 0) = world_coordinate.at<double>(1, 0) / world_coordinate.at<double>(3, 0);
		world_coordinate.at<double>(2, 0) = world_coordinate.at<double>(2, 0) / world_coordinate.at<double>(3, 0);
		world_coordinate.at<double>(3, 0) = world_coordinate.at<double>(3, 0) / world_coordinate.at<double>(3, 0);

		camera_L.at<double>(0, 0) = world_coordinate.at<double>(0, 0);
		camera_L.at<double>(1, 0) = world_coordinate.at<double>(1, 0);
		camera_L.at<double>(2, 0) = world_coordinate.at<double>(2, 0);


		Mat pix = cameraMatrix_L *camera_L ;
		pix_L_all.push_back(Point2d(pix.at<double>(0, 0) / pix.at<double>(2, 0), pix.at<double>(1, 0) / pix.at<double>(2, 0)));
		camera_L_all.push_back(Point3d(camera_L.at<double>(0, 0), camera_L.at<double>(1, 0), camera_L.at<double>(2.0)));
	}

	cout << "end" << endl;
}


