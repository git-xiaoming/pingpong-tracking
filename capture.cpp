//µ÷ÓÃË«Ä¿ÅÄÕÕ

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	VideoCapture cap1,cap2;
	cap1.open(0 + CAP_DSHOW);
	cap2.open(1 + CAP_DSHOW);
	int width = 1920, height = 1080;
	cap1.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap1.set(CAP_PROP_FRAME_WIDTH, width);
	cap1.set(CAP_PROP_FRAME_HEIGHT, height);
	cap2.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap2.set(CAP_PROP_FRAME_WIDTH, width);
	cap2.set(CAP_PROP_FRAME_HEIGHT, height);
	if (!cap1.isOpened() || !cap2.isOpened())
		return 1;
	bool stop(false);
	Mat frame1,frame2;
	namedWindow("video1", WINDOW_NORMAL);
	namedWindow("video2", WINDOW_NORMAL);
	waitKey(0);
	int i = 1;
	while (!stop)
	{
		if (!cap1.read(frame1) || !cap2.read(frame2))
			break;
		imshow("video1", frame1);
		imshow("video2", frame2);
		char k = waitKey(1);
		if (k == 's')
		{
			namedWindow("left" + to_string(i), WINDOW_NORMAL);
			namedWindow("right" + to_string(i), WINDOW_NORMAL);
			imshow("left" + to_string(i), frame1);
			imshow("right" + to_string(i), frame2);
			cout << "Do you want to save it?" << endl;
			char c = waitKey(0);
			destroyWindow("left" + to_string(i));
			destroyWindow("right" + to_string(i));
			if (c == 'y')
			{
				imwrite("d:\\Documents\\c project\\project1\\stereo_calib\\left" + to_string(i) + ".jpg", frame1);
				imwrite("d:\\Documents\\c project\\project1\\stereo_calib\\right" + to_string(i) + ".jpg", frame2);				
				cout << "Saved successfully!" << endl;
				i++;
			}
		}
		if (k == 'q')
			stop = true;
	}
	destroyAllWindows();
	return 0;
}

