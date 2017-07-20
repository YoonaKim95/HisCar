#include "opencv2/opencv.hpp"
#include <cmath>
#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <iostream>
#include <algorithm>
#include <stdio.h>

using namespace std;
using namespace cv;

void findandDrawContour(Mat& roi, char* windowName);
Mat preprocess(Mat& frame);
Point findLineAndVP(Mat& white, Mat& frame, Point& rec_point, float& prev_Rslope, float& prev_Lslope, Point intersectionPoint);


Mat preprocess(Mat& frame) {
	Mat gray, smot, sub, hsv, white, yellow, both, canny, left, right;
	Mat sub1, sub2, sub3, sub4, smot1, smot2, smot3, smot4, gray1, gray2, gray3, gray4, white1, white2, white3, white4, gray_original_sub;
	Mat grayl3, grayl4, grayr3, grayr4; //gray areas.
	Mat canny4, canny3, canny2;
	Mat contourCanny;

	vector<Vec4i> lines;
	Mat matForContour;
	Mat contour;

	char contourWindow[20] = "Contour";

	int interest_y = 168; // height of RoadRegion 
	int interest_x = 0;
	int width = frame.cols - interest_x; //width of ROI
	int height = frame.rows - interest_y; //height of ROI
	int subROIHeight = height / 16;  // Calculate the height of sub_ROIs

	matForContour = frame.clone();

	//VP standard
	circle(frame, Point(frame.cols / 2, frame.rows / 4), 5, Scalar(0, 0, 0), 3, LINE_AA);

	// Setting ROIs
	Point rec3_4_point(interest_x, interest_y + subROIHeight * 3);
	Rect rec3_4(rec3_4_point, Size(width, subROIHeight * 13));

	// use canny to make binary image
	contour = matForContour(rec3_4);
	Canny(contour, contourCanny, 200, 400);

	dilate(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);
	erode(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);
	dilate(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);
	erode(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);

	imshow("CannyforContour", contourCanny);
	findandDrawContour(contourCanny, contourWindow);
	return contourCanny;
}

void findandDrawContour(Mat &roi, char* windowName) {
	vector<vector<Point> > contours;

	int k = 0;
	int mode = RETR_EXTERNAL;
	int method = CHAIN_APPROX_NONE;

	vector<Vec4i> hierarchy;
	findContours(roi, contours, hierarchy, mode, method);
	cvtColor(roi, roi, COLOR_GRAY2BGR);

	if (contours.size() > 1) {
		Scalar color(255, 255, 10);
		vector<Rect> rect(contours.size());
		vector<Mat> matArr(contours.size());
		drawContours(roi, contours, -1, Scalar(255, 255, 255), -1);

		for (int i = 0; i < contours.size(); i++) {
			Rect temp = boundingRect(Mat(contours[i]));
			rect[k] = temp;
			k++;
		}
		char name[10];
		for (int i = 0; i < k; i++)      {
			sprintf(name, "Mat%d", i);
			matArr[i] = Mat(roi, rect[i]);
			Mat Cur_Mat = matArr[i];
			Mat calcMat;
			cvtColor(matArr[i], calcMat, CV_BGR2GRAY);
			int row = matArr[i].rows;
			vector<int> whiteCount(row);
			for (int y = 0; y < row; y++) {
				int count_white = 0;
				Mat rowMat = calcMat.row(y);
				count_white = countNonZero(rowMat);
				whiteCount[y] = count_white;
			}

			int maxCount = -1, minCount = 1000;
			float sum = 0.0, mean = 0.0, standardDeviation = 0.0;
			for (int z = 0; z < row; z++) {
				sum += whiteCount[z];
				if (whiteCount[z] > maxCount)
					maxCount = whiteCount[z];
				if (whiteCount[z] < minCount)
					minCount = whiteCount[z];
			}

			mean = sum / row;

			for (int zz = 0; zz < row; zz++)
				standardDeviation += pow(whiteCount[zz] - mean, 2);

			int stdevOfWhite = sqrt(standardDeviation / row);

			/*************** condition to delete non_lane contours ***************/
			if (stdevOfWhite >= 15 || mean >= 40 || (mean - stdevOfWhite) > 25)
				matArr[i].setTo(0);
		}
	}
}

Point findLineAndVP(Mat& white, Mat& frame, float& prev_Rslope, float& prev_Lslope, Point intersectionPoint) {
	Point rec_point(0, 168 + ((frame.rows - 168) / 16) * 3);

	//declaration of x,y variables used in lines.
	float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	float x3 = 0, x4 = 0, y3 = 0, y4 = 0;
	//initiate for the first time when there are no input values.
	float countright = 0, countleft = 0;

	float a1 = 0, a2 = 0, a3 = 0, a4 = 0;
	float b1 = 0, b2 = 0, b3 = 0, b4 = 0;
	int x = 0, y = 0;

	//convert white Matrix to canny Matrix.
	Mat canny;
	Canny(white, canny, 150, 300, 3);
	vector<Vec4i> lines;
	vector<Point> pointList;
	//20, 10, 140
	HoughLinesP(canny, lines, 1, CV_PI / 180, 20, 10, 140);
	//Merge part
	float selected_slopeR = 0;
	float selected_slopeL = 0;

	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];
		float slope = (((float)l[3] - (float)l[1]) / ((float)l[2] - (float)l[0]));

		//lines of right side
		if (slope >= 0.3 && slope <= 3) {
			countright++;
			x1 += l[0];
			y1 += l[1] + rec_point.y;
			x2 += l[2];
			y2 += l[3] + rec_point.y;
			selected_slopeR = slope;
		}
		//lines of left side
		if (slope <= -0.3  && slope >= -3) {
			countleft++;
			x3 += l[0];
			y3 += l[1] + rec_point.y;
			x4 += l[2];
			y4 += l[3] + rec_point.y;
			selected_slopeL = slope;
		}
	}

	float Rslope, Lslope, rb, lb;
	//if it is the first time, put initial values.


	/*RIGHT LANE*/
	if (countright == 0 && prev_Rslope != 0) {
		Rslope = prev_Rslope;
		rb = intersectionPoint.y - Rslope * intersectionPoint.x;
	}
	else  {
		Rslope = (y2 - y1) / (x2 - x1);
		prev_Rslope = Rslope;
		rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
	}


	/*LEFT LANE*/
	if (countleft == 0 && prev_Lslope != 0) {
		Lslope = prev_Lslope;
		lb = intersectionPoint.y - Lslope * intersectionPoint.x;
	}
	else {
		Lslope = (y4 - y3) / (x4 - x3);
		prev_Lslope = Lslope;
		lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
	}

	float lastx1 = (0 - rb) / Rslope;
	float lastx2 = (frame.rows - rb) / Rslope;
	float lastx3 = ((0 - lb) / Lslope);
	float lastx4 = ((frame.rows - lb) / Lslope);

	//point of line will be drawn. extend to the end of the frame.
	// a1 = lastx1 + x, a2 = lastx2 + x, a3 = lastx3 + x, a4 = lastx4 + x;
	a1 = lastx1, a2 = lastx2, a3 = lastx3, a4 = lastx4;
	b1 = 0, b2 = frame.rows, b3 = 0, b4 = frame.rows;

	float dataA[] = { (b2 - b1) / (a2 - a1), -1, (b4 - b3) / (a4 - a3), -1 };
	Mat A(2, 2, CV_32F, dataA);
	Mat invA;
	invert(A, invA);

	float dataB[] = { a1*(b2 - b1) / (a2 - a1) - b1, a3*(b4 - b3) / (a4 - a3) - b3 };
	Mat B(2, 1, CV_32F, dataB);
	//vanishing point.
	Mat X = invA*B;

	line(frame, Point(a1, 0), Point(a2, frame.rows), Scalar(255, 200, 20), 3);
	line(frame, Point(a3, 0), Point(a4, frame.rows), Scalar(255, 200, 20), 3);
	circle(frame, Point(X.at<float>(0, 0), X.at<float>(1, 0)), 5, Scalar(255, 200, 20), 3, LINE_AA);

	float innerAngleL = 0;
	float innerAngleR = 0;
	float innerA = 0;
	innerAngleR = abs(tan(((X.at<float>(0, 0)) - a1) / ((X.at<float>(1, 0)))));
	innerAngleL = abs(tan(((X.at<float>(0, 0)) - a3) / ((X.at<float>(1, 0)))));
	innerA = innerAngleR + innerAngleL;
	innerAngleR = (innerAngleR*180.0 / CV_PI);
	innerAngleL = (innerAngleL*180.0 / CV_PI);

	return Point(X.at<float>(0, 0), X.at<float>(1, 0));
}

int main() {
	char title[100] = "mono.mp4";
	VideoCapture capture(title);

	Mat frame, afterPreprocess;
	int key, frameNum = 1, frame_rate = 30;
	vector<Point> io_prev_lanes;
	float prev_Rslope = 0, prev_Lslope = 0;
	Point prev_intersectionPoint(0, 0);

	// videoRead
	while (1) {
		if (!capture.read(frame))
			break;

		afterPreprocess = preprocess(frame);
		imshow("afterPreprocess", afterPreprocess);
		prev_intersectionPoint = findLineAndVP(afterPreprocess, frame, prev_Rslope, prev_Lslope, prev_intersectionPoint);
		imshow("frame", frame);

		key = waitKey(frame_rate);
		if (key == 32) {
			if (frame_rate == 30)
				frame_rate = 0;
			else
				frame_rate = 30;
		}
		else if (key == 27) {
			break;
		}
		frameNum++;
	}
	return 0;
}