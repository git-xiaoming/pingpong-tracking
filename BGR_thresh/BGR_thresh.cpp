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

using namespace cv;
using namespace std;

Mat track(Mat frame, Point2f& uv, bool& flag)
{
	Mat blue, green, red, element5(5, 5, CV_8U, Scalar(1)), quadri(frame.size(), CV_8UC3, Scalar(0, 0, 0)), img = frame.clone();
	//cvtColor(img, img, COLOR_BGR2HSV);
	vector<Mat> channels;
	split(img, channels);
	inRange(channels[0], 0, 67, blue);
	inRange(channels[1], 28, 125, green);
	inRange(channels[2], 105, 220, red);
	img = blue & green & red;
	morphologyEx(img, img, MORPH_CLOSE, element5);
	/*vector<vector<Point>> contours;
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
	}*/
	return img;
}

int main()
{
	Mat img, origin;
	Point2f uv;
	bool flag = false;
    double min[3], max[3];
    origin = imread("left3.jpg");
    img = origin.clone()(Range(360, 390), Range(560, 590));
	//img = track(origin.clone(), uv, flag);
    namedWindow("origin", WINDOW_NORMAL);
    namedWindow("image", WINDOW_NORMAL);
    imshow("origin", origin);
    imshow("image", img);
    char k = waitKey(0);
    if (k == 't')
    {
        vector<Mat> channels;
        split(img, channels);
        for (int i = 0; i < 3; i++)
        {
            minMaxLoc(channels[i], &min[i], &max[i], NULL, NULL);
            cout << "bgr range:" << min[i] << "~" << max[i] << endl;
        }
    }
}


