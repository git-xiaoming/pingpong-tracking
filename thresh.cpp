//

#include<opencv2/opencv.hpp>
#include<iostream>
#include<opencv2/highgui.hpp>

#define pi 3.14159

using namespace cv;
using namespace std;



Mat track(Mat frame, Point2f& uv,bool & flag)
{
	Mat hue, sat, element5(5, 5, CV_8U, Scalar(1)), quadri(frame.size(), CV_8UC3, Scalar(0,0,0)),img=frame.clone();
	cvtColor(img, img, COLOR_BGR2HSV);
	vector<Mat> channels;
	split(img, channels);
	inRange(channels[0], 5, 20, hue);
	inRange(channels[1], 50, 255, sat);
	img = hue & sat;
	morphologyEx(img, img, MORPH_CLOSE, element5);
	vector<vector<Point>> contours;
	findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	int i, count = 0, k = 0;
	float radius, scale;
	Point2f center;
	for (i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 10)
		{
			minEnclosingCircle(contours[i], center, radius);
			double circleArea = pi * radius * radius;
			if (contourArea(contours[i]) > 0.4 * circleArea && arcLength(contours[i], true) > 150 && arcLength(contours[i], true) < 300)
			{
				scale = arcLength(contours[i], true);
				RotatedRect rrect = fitEllipse(contours[i]);
				uv.x = rrect.center.x, uv.y = rrect.center.y;
				count++;
			}
			else
				drawContours(img, contours, i, Scalar::all(0), -1);
		}
		else
			drawContours(img, contours, i, Scalar::all(0), -1);
	}
	if (count == 1)
	{
		flag = true;
		cout << "The arclength:" << scale << endl;	
	}
	else
	{
		flag = false;
		uv.x = 0, uv.y = 0;
	}
	return img;
}


int main(int argc, char** argv)
{
	VideoCapture cap(0 + CAP_DSHOW);
	int width = 1920, height = 1080, hmin = 0 * height, hmax = 1 * height;
	/*cap.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap.set(CAP_PROP_FRAME_WIDTH, width);
	cap.set(CAP_PROP_FRAME_HEIGHT, height);*/
	if (!cap.isOpened())
		return 1;
	bool stop(false), flag = false;
	Mat frame, img;
	Point2f uv;
	namedWindow("origin", WINDOW_NORMAL);
	namedWindow("track", WINDOW_NORMAL);
	waitKey(0);
	while (!stop)
	{
		if (!cap.read(frame))
			break;
		resize(frame, frame, Size(width, height));
		frame = frame(Range(hmin, hmax), Range::all());
		img = track(frame.clone(), uv, flag);
		imshow("origin", frame);
		imshow("track", img);
		if (flag)
			cout << Point2f(uv.x, uv.y + hmin) << endl;
		else
			cout << Point2f(0, 0) << endl;
		//cout << frame.size() << endl;
		if (waitKey(1) == (int)'q')
			stop = true;
	}
	destroyAllWindows();
	return 0;
}