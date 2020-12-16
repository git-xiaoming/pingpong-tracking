//乒乓球识别与追踪

#include<opencv2/opencv.hpp>
#include<iostream>
#include<opencv2/highgui.hpp>
#define pi 3.14159

using namespace cv;
using namespace std;


Mat track(Mat frame, Point2f & uv, bool & flag, bool & isfirst)
{
	int width = frame.size().width, height = frame.size().height, r = 100;
	Mat hue, sat, element5(5, 5, CV_8U, Scalar(1)), quadri(frame.size(), CV_8UC1, 255), img;
	float rmin = MAX(uv.y - r, 0), rmax = MIN(uv.y + r, height), cmin = MAX(uv.x - r, 0), cmax = MIN(uv.x + 10, width);
	if (flag == true)
	{
		img = frame(Range(rmin, rmax), Range(cmin, cmax));
		isfirst = false;
	}
	else if (isfirst == true)
		img = frame(Range::all(), Range(width - 200, width));
	else
		img = frame;
	cvtColor(img, img, COLOR_BGR2HSV);
	vector<Mat> channels;
	split(img, channels);
	inRange(channels[0], 5, 16, hue);
	inRange(channels[1], 43, 255, sat);
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
			if (contourArea(contours[i]) > 0.2 * circleArea && arcLength(contours[i], true) > 40 && arcLength(contours[i], true) < 80)
			{
				//cout << "arclength:" << arcLength(contours[i], true) << endl;
				RotatedRect rrect = fitEllipse(contours[i]);
				if (flag == true)
				{
					uv.x = rrect.center.x + cmin, uv.y = rrect.center.y + rmin;
					ellipse(quadri(Range(rmin, rmax), Range(cmin, cmax)), rrect, 0, 3, 8);
				}
				else if (isfirst == true)
				{
					uv.x = rrect.center.x + cmin, uv.y = rrect.center.y;
					ellipse(quadri(Range(0, height), Range(cmin, cmax)), rrect, 0, 3, 8);
				}
				else
				{
					uv.x = rrect.center.x, uv.y = rrect.center.y;
					ellipse(quadri, rrect, 0, 3, 8);
				}
				count++;
			}
		}
	}
	if (count == 1)
		flag = true;
	else
	{
		flag = false;
		uv = Point2f(0, 0);
	}
	return quadri;
}

int main(int argc, char** argv) 
{
	VideoCapture cap("video_r.avi");// 0 + CAP_DSHOW);
	int width = 640, height = 480, hmin = 0.4 * height, hmax = 0.9 * height, num=0;
	/*cap.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap.set(CAP_PROP_FRAME_WIDTH, width);
	cap.set(CAP_PROP_FRAME_HEIGHT, height);*/
	if (!cap.isOpened())
		return 1;
	bool stop(false),flag=false,isfirst=true;
	Mat frame,img;
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
		img = track(frame.clone(), uv, flag, isfirst);
		//uv.y += hmin;
		imshow("origin", frame);
		imshow("track", img);
		if (flag)
		{
			cout << Point2f(uv.x, uv.y + hmin) << endl;
			num++;
		}
		else
			cout << Point2f(0, 0) << endl;
		if (waitKey(1)==(int)'q')
			stop = true;
	}
	destroyAllWindows();
	cout << "frame number is:" << num << endl;
	return 0;
}



