#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fstream>

#define pi 3.14159

using namespace cv;
using namespace std;

void uv2xyz(Point2f uvLeft, Point2f uvRight, Point3f& point3D)
{
	Mat mLeftIntrinsic, mRightIntrinsic, R, T,
		mLeftRotation = Mat::eye(3, 3, CV_32F),
		mLeftTranslation = Mat::zeros(3, 1, CV_32F);
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
	/*fs.open("leftRT.yml", FileStorage::READ);
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
		cout << "Failure to open the file:\"leftRT.yml\"" << endl;*/
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

Mat track(Mat frame, Point2f & uv, bool & flag,bool & isfirst)
{
	int width = frame.size().width, height = frame.size().height, r = 400;
	Mat hue, sat, element5(5, 5, CV_8U, Scalar(1)), quadri(frame.size(), CV_8UC1, 255), img;
	float rmin = MAX(uv.y - r, 0), rmax = MIN(uv.y + r, height), cmin = MAX(uv.x -10, 0), cmax = MIN(uv.x +r, width);
	if (flag == true)
	{
		img = frame(Range(rmin, rmax), Range(cmin, cmax));
		isfirst = false;
	}
	else if(isfirst==true)
		img = frame(Range(0, height), Range(0,r));
	else
		img = frame;
	cvtColor(img, img, COLOR_BGR2HSV);
	vector<Mat> channels;
	split(img, channels);
	inRange(channels[0], 5, 20, hue);
	inRange(channels[1], 50, 255, sat);
	img = hue & sat;
	morphologyEx(img, img, MORPH_CLOSE, element5);
	vector<vector<Point>> contours;
	findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	int i, count = 0;
	float radius;
	Point2f center;
	//vector<vector<Point>>::iterator itc = contours.begin();
	for (i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 10)
		{
			minEnclosingCircle(contours[i], center, radius);
			double circleArea = pi * radius * radius;
			if (contourArea(contours[i]) > 0.4 * circleArea && arcLength(contours[i], true) > 150 && arcLength(contours[i], true) < 300)
			{
				RotatedRect rrect = fitEllipse(contours[i]);
				if (flag == true)
				{
					uv.x = rrect.center.x + cmin, uv.y = rrect.center.y + rmin;
					ellipse(quadri(Range(rmin, rmax), Range(cmin, cmax)), rrect, 0, 6, 8);
				}
				else if (isfirst == true)
				{
					uv.x = rrect.center.x + cmin, uv.y = rrect.center.y;
					ellipse(quadri(Range(0, height), Range(cmin, cmax)), rrect, 0, 6, 8);
				}
				else
				{
					uv.x = rrect.center.x, uv.y = rrect.center.y;
					ellipse(quadri, rrect, 0, 6, 8);
				}
				count++;
			}
		}
	}
	if (count == 1)
	{
		flag = true;
	}
	else
	{
		flag = false;
		uv = Point2f(0, 0);
	}
	return quadri;
}

int main(int argc, char** argv)
{
	VideoCapture cap1, cap2;
	int i, j = 0, width = 1920, height = 1080, k, frameNum=0, 
		hmin1 = 0 * height, hmax1 = 0.9 * height, hmin2 = 0 * height, hmax2 = 0.9 * height;
	cap1.open("video_l.avi"); //0+CAP_DSHOW);
	cap2.open("video_r.avi"); //1+CAP_DSHOW);
	/*cap1.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap1.set(CAP_PROP_FRAME_WIDTH, width);
	cap1.set(CAP_PROP_FRAME_HEIGHT, height);
	cap2.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap2.set(CAP_PROP_FRAME_WIDTH, width);
	cap2.set(CAP_PROP_FRAME_HEIGHT, height);*/
	if (!cap1.isOpened() || !cap2.isOpened())
		return 1;
	Point3f point3D[1000];
	bool stop(false), flag1 = false, flag2 = false, isfirst1 = false, isfirst2 = false, locate = false;
	char answer;
	Mat frame1, frame2, img1, img2;
	Point2f uvLeft = Point2f(0, 0), uvRight = Point2f(0, 0);
	namedWindow("origin1", WINDOW_NORMAL);
	namedWindow("origin2", WINDOW_NORMAL);
	namedWindow("track1", WINDOW_NORMAL);
	namedWindow("track2", WINDOW_NORMAL);
	waitKey(0);
	while (!stop)
	{
		if (!cap1.read(frame1) || !cap2.read(frame2))
			break;
		resize(frame1, frame1, Size(width, height));
		resize(frame2, frame2, Size(width, height));
		frame1 = frame1(Range(hmin1, hmax1), Range::all());
		frame2 = frame2(Range(hmin2, hmax2), Range::all());
		img1 = track(frame1.clone(), uvLeft, flag1, isfirst1);
		img2 = track(frame2.clone(), uvRight, flag2, isfirst2);
		imshow("origin1", frame1);
		imshow("origin2", frame2);
		imshow("track1", img1);
		imshow("track2", img2);
		k = waitKey(/*!(flag1 && flag2)*/1);
		if (k == 'l')
			locate = true;
		if (k == 'q')
			stop = true;
		if (flag1 && flag2 /*&& locate*/)
		{
			//uv2xyz(Point2f(1920 / 640 * uvLeft.x, 1080 / 480 * (uvLeft.y + hmin1)), Point2f(1920 / 640 * uvRight.x, 1080 / 480 * (uvRight.y + hmin2)), point3D[k]);
			uv2xyz(Point2f(uvLeft.x, uvLeft.y + hmin1), Point2f(uvRight.x, uvRight.y + hmin2), point3D[frameNum]);
			cout << "空间坐标 = " << point3D[frameNum] << endl;
			frameNum++;
			locate = false;
		}
		else
			cout << "None has been found" << endl;
	}
	destroyAllWindows();
	cout << "frameNum:" << frameNum << endl;
	cout << "Do you want to save the data of the 3d points?(y/n)" << endl;
	cin >> answer;
	if (answer == 'y')
	{
		fstream file("point3D.csv", ios::out);
		if (!file.fail())
		{
			cout << "start writing" << endl;
			for (i = 0; i < frameNum; i++)
			{
				file << point3D[i].x << " " << point3D[i].y << " " << point3D[i].z << "\r\n";
			}
			cout << "Successfully written" << endl;
		}
		else
			cout << "Failure to open the file" << endl;
	}
	return 0;
}
