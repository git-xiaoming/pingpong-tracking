//

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>

#define pi 3.14159

using namespace cv;
using namespace std;

void uv2xyz(Point2f uvLeft, Point2f uvRight, Point3f& point3D)
{
	Mat mLeftIntrinsic, mRightIntrinsic, R, T, 
		mLeftRotation=Mat::eye(3,3,CV_32F), 
		mLeftTranslation=Mat::zeros(3,1,CV_32F);
	//读取相机标定结果
	FileStorage fs;
	//内参
	fs.open("intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> mLeftIntrinsic;
		mLeftIntrinsic.convertTo(mLeftIntrinsic, CV_32F);
		fs["M2"] >> mRightIntrinsic;
		mRightIntrinsic.convertTo(mRightIntrinsic, CV_32F);
		fs.release();
	}
	else
		cout << "Failure to open the file:\"intrinsics.yml\"" << endl;
	//外参
	fs.open("extrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"] >> T;
		R.convertTo(R, CV_32F);
		T.convertTo(T, CV_32F);
		fs.release();
	}
	else
		cout << "Failure to open the file:\"extrinsics.yml\"" << endl;
	//左相机外参
	fs.open("leftRT.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["leftR"] >> mLeftRotation;
		fs["leftT"] >> mLeftTranslation;
		//invert(mLeftRotation, mLeftRotation);
		mLeftRotation.convertTo(mLeftRotation, CV_32F);
		mLeftTranslation.convertTo(mLeftTranslation, CV_32F);
		fs.release();
	}
	else
		cout << "Failure to open the file:\"leftRT.yml\"" << endl;
	//右相机外参
	Mat mRightRotation = R * mLeftRotation,
		mRightTranslation = T + R * mLeftTranslation;
	mRightRotation.convertTo(mRightRotation, CV_32F);
	mRightTranslation.convertTo(mRightTranslation, CV_32F);

	//left
	Mat mLeftRT = Mat(3, 4, CV_32F);
	hconcat(mLeftRotation, mLeftTranslation, mLeftRT);
	Mat mLeftM = mLeftIntrinsic * mLeftRT;
	//cout<<"左相机M矩阵 = "<<endl<<mLeftM<<endl;

	//right
	Mat mRightRT = Mat(3, 4, CV_32F);
	hconcat(mRightRotation, mRightTranslation, mRightRT);
	Mat mRightM = mRightIntrinsic * mRightRT;
	//cout<<"右相机M矩阵 = "<<endl<<mRightM<<endl;

	//最小二乘法A矩阵
	Mat A = Mat(4, 3, CV_32F);
	A.at<float>(0, 0) = uvLeft.x * mLeftM.at<float>(2, 0) - mLeftM.at<float>(0, 0);
	A.at<float>(0, 1) = uvLeft.x * mLeftM.at<float>(2, 1) - mLeftM.at<float>(0, 1);
	A.at<float>(0, 2) = uvLeft.x * mLeftM.at<float>(2, 2) - mLeftM.at<float>(0, 2);

	A.at<float>(1, 0) = uvLeft.y * mLeftM.at<float>(2, 0) - mLeftM.at<float>(1, 0);
	A.at<float>(1, 1) = uvLeft.y * mLeftM.at<float>(2, 1) - mLeftM.at<float>(1, 1);
	A.at<float>(1, 2) = uvLeft.y * mLeftM.at<float>(2, 2) - mLeftM.at<float>(1, 2);

	A.at<float>(2, 0) = uvRight.x * mRightM.at<float>(2, 0) - mRightM.at<float>(0, 0);
	A.at<float>(2, 1) = uvRight.x * mRightM.at<float>(2, 1) - mRightM.at<float>(0, 1);
	A.at<float>(2, 2) = uvRight.x * mRightM.at<float>(2, 2) - mRightM.at<float>(0, 2);

	A.at<float>(3, 0) = uvRight.y * mRightM.at<float>(2, 0) - mRightM.at<float>(1, 0);
	A.at<float>(3, 1) = uvRight.y * mRightM.at<float>(2, 1) - mRightM.at<float>(1, 1);
	A.at<float>(3, 2) = uvRight.y * mRightM.at<float>(2, 2) - mRightM.at<float>(1, 2);

	//最小二乘法B矩阵
	Mat B = Mat(4, 1, CV_32F);
	B.at<float>(0, 0) = mLeftM.at<float>(0, 3) - uvLeft.x * mLeftM.at<float>(2, 3);
	B.at<float>(1, 0) = mLeftM.at<float>(1, 3) - uvLeft.y * mLeftM.at<float>(2, 3);
	B.at<float>(2, 0) = mRightM.at<float>(0, 3) - uvRight.x * mRightM.at<float>(2, 3);
	B.at<float>(3, 0) = mRightM.at<float>(1, 3) - uvRight.y * mRightM.at<float>(2, 3);

	Mat XYZ = Mat(3, 1, CV_32F);
	//采用SVD最小二乘法求解XYZ
	solve(A, B, XYZ, DECOMP_SVD);

	//cout<<"空间坐标为 = "<<endl<<XYZ<<endl;

	//世界坐标系中坐标
	point3D.x = XYZ.at<float>(0, 0);
	point3D.y = XYZ.at<float>(1, 0);
	point3D.z = XYZ.at<float>(2, 0);
}

Mat track(Mat origin, Point2f& uv, bool& flag)
{
	int width = origin.size().width, height = origin.size().height, cmin = 700, cmax = 1000, rmin = 700, rmax = 1010;
	Mat hue, sat, element5(5, 5, CV_8U, Scalar(1)), img = origin(Range(rmin, rmax), Range(cmin, cmax)).clone();
	cvtColor(img, img, COLOR_BGR2HSV);
	vector<Mat> channels;
	split(img, channels);
	inRange(channels[0], 15, 30, hue);
	inRange(channels[1], 43, 255, sat);
	img = hue & sat;
	morphologyEx(img, img, MORPH_CLOSE, element5);
	vector<vector<Point>> contours;
	findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	int i, count = 0;
	float radius;
	Point2f center;
	for (i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 10)
		{
			minEnclosingCircle(contours[i], center, radius);
			double circleArea = pi * radius * radius;
			if (contourArea(contours[i]) > 0.5 * circleArea && arcLength(contours[i], true) > 50 && arcLength(contours[i], true) < 200)
			{
				//cout << "arclength:" << arcLength(contours[i], true) << endl;
				RotatedRect rrect = fitEllipse(contours[i]);
				uv.x = rrect.center.x + cmin, uv.y = rrect.center.y + rmin;
				ellipse(origin(Range(rmin, rmax), Range(cmin, cmax)), rrect, Scalar(0,255,0), 6, 8);
				count++;
			}
		}
	}
	flag = count == 1 ? true : false;
	return origin;
	//return img;
}


int main(int argc, char** argv)
{
	Point3f point3D[2];
	bool flag1 = false, flag2 = false;
	Mat origin1, origin2, img1, img2;
	Point2f uvLeft = Point2f(0, 0), uvRight = Point2f(0, 0);
	namedWindow("image1", WINDOW_NORMAL);
	namedWindow("image2", WINDOW_NORMAL);
	waitKey(0);
	for(int i=0;i<=1;i++)
	{
		origin1 = imread("left" + to_string(i+21) + ".jpg");
		origin2 = imread("right" + to_string(i+21) + ".jpg");
		img1=track(origin1.clone(), uvLeft, flag1);
		img2=track(origin2.clone(), uvRight, flag2);
		if (flag1 && flag2)
		{
			uv2xyz(uvLeft, uvRight, point3D[i]);
			cout << "图像坐标left" << uvLeft << endl;
			cout << "图像坐标right" << uvRight << endl;
			cout << "空间坐标 = " << point3D[i] << endl;
		}
		else
		{
			cout << "not found" << endl;
		}
		imshow("image1", img1);
		imshow("image2", img2);
		waitKey(0);
	}
	destroyAllWindows();
	float distance = sqrt(pow(point3D[0].x - point3D[1].x, 2) + pow(point3D[0].y - point3D[1].y, 2) + pow(point3D[0].z - point3D[1].z, 2));
	cout << "distance between two points:" << distance << endl;
	return 0;
}