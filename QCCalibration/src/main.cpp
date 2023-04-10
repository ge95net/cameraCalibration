#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "QCCalibration.h"
using namespace std;
using namespace cv;

int main()
{
	int pictureNums = 6;
	QCCalibration QCCalibration (11,8,35);
	QCCalibration.setPictureNums(pictureNums);
	cv::FileStorage fs("result.yml", FileStorage::READ);
	bool calibration ;
	if (fs.isOpened()) {
		calibration = false;
	}
	else {
		calibration = true;
	}

	for (int i = 0; i < pictureNums; i++)
	{

		Mat Image_left = imread("D:\\Microsoft Visual Studio\\projects\\QCCalibration\\CalibData\\" + to_string(i + 1) + "\\L.bmp", 0);
		Mat Image_right = imread("D:\\Microsoft Visual Studio\\projects\\QCCalibration\\CalibData\\" + to_string(i + 1) + "\\R.bmp", 0);
		if (true == QCCalibration.extractCorners(Image_left, Image_right))
		{
			cout << "Image Sequence " << i + 1 << " Success" << endl;
		}
		else
		{
			cout << "Image Sequence " << i + 1 << " Failed" << endl;
		}
		if (calibration == false) {
			QCCalibration.readData();
			QCCalibration.undistortion_iterate(Image_left, Image_right);
			//QCCalibration.vertify_undistortion();
			//QCCalibration.undistortion_gaussionNewton(Image_left, Image_right);
			QCCalibration.reconstruction(Image_left, Image_right, i);
			//QCCalibration.vertify_triangular(i);

		}

	}
	if (calibration == true) {
		QCCalibration.cameraCalibration();
	}
	
	
	cout << "Camera Calibration Finished" << endl;
	system("pause");
	return 0;

}