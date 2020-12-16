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

using namespace cv;
using namespace std;


int main()
{
    Mat cameraMatrix, distCoeffs, rvec, tvec, Rotation;
    FileStorage fs;
    fs.open("intrinsics.yml", FileStorage::READ);
    if (fs.isOpened())
    {
        fs["M1"] >> cameraMatrix;
        fs["D1"] >> distCoeffs;
        fs.release();
    }
    else
        cout << "Failure to open the file:\"intrinsics.yml\"" << endl;
    //
    vector<Point2f>  imagePoints;
    vector<Point3f>  objectPoints;
    Size boardSize = Size(3, 3);
    float squareSize = 26;
    Mat img = imread("chessboard.jpg", 0);
    bool found = false;
    found = findChessboardCorners(img, boardSize, imagePoints,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
    //
    Mat cimg;
    cvtColor(img, cimg, COLOR_GRAY2BGR);
    drawChessboardCorners(cimg, boardSize, imagePoints, found);
    namedWindow("imagePoints", WINDOW_NORMAL);
    imshow("imagePoints", cimg);
    char c = (char)waitKey(0);
    if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
        exit(-1);
    cornerSubPix(img, imagePoints, Size(11, 11), Size(-1, -1),
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 0.01));
    destroyAllWindows();
    //
    for (int j = 0; j < boardSize.height; j++)
        for (int k = 0; k < boardSize.width; k++)
            objectPoints.push_back(Point3f(k * squareSize, j * squareSize, 0));
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    Rodrigues(rvec, Rotation);
    //
    cout << "leftR=" << Rotation << endl;
    cout << "leftT=" << tvec << endl;
    cout << "Do you want to save this(y/n)" << endl;
    char answer;
    cin >> answer;
    if (answer == 'y')
    {
        fs.open("leftRT.yml", FileStorage::WRITE);
        if (fs.isOpened())
        {
            fs << "leftR" << Rotation << "leftT" << tvec;
            fs.release();
        }
        else
            cout << "Error: can not save this\n";
    }
}


