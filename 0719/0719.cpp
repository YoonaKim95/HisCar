#include "opencv2/opencv.hpp"
#include <cmath>
#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <queue>

using namespace std;
using namespace cv;

void findandDrawContour(Mat& roi, char* windowName, int type);
Mat preprocess(Mat& frame);
Point findLineAndVP(Mat& white, Mat& frame, Point& rec_point, float& prev_Rslope, float& prev_Lslope, Point intersectionPoint, queue <float>& rightSlope, int& rightLaneKept);
bool isStopLine(Mat& frame, Point vp, float lSlope, float rSlope);
void getMinMax(Mat& roi, int& min, int& max);



Mat preprocess(Mat& frame) {
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


  Mat grayFrame, afterInRange, afterCanny;
  int min, max;
  // use canny to make binary image
  cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
  GaussianBlur(grayFrame, grayFrame, Size(3, 3), 3);

  Mat roi(grayFrame, rec3_4);

  getMinMax(roi, min, max);

  inRange(roi, min, max, afterInRange);
  Canny(afterInRange, afterCanny, 50, 100);

  dilate(afterCanny, afterCanny, Mat(), Point(-1, -1), 3);
  erode(afterCanny, afterCanny, Mat(), Point(-1, -1), 3);
  dilate(afterCanny, afterCanny, Mat(), Point(-1, -1), 3);
  erode(afterCanny, afterCanny, Mat(), Point(-1, -1), 3);

  imshow("afterCanny", afterCanny);
  //findandDrawContour(afterCanny, contourWindow, 0);
  //imshow("afterInrangeContour", afterCanny);


  /////////////////////////////////////////////////////////////////////////////////////////
  contour = matForContour(rec3_4);
  Canny(contour, contourCanny, 100, 200);
  
  dilate(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);
  erode(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);
  dilate(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);
  erode(contourCanny, contourCanny, Mat(), Point(-1, -1), 3);
  
  imshow("CannyforContour", contourCanny);
  
//  findandDrawContour(contourCanny, contourWindow, 1);  
//  imshow("CannyafterContour", contourCanny);

  /////////////////////////////////////////////////////////////////////////////////////
  Mat testOne = contourCanny - afterCanny;
  imshow("testOne", testOne);

  Mat testTwo = afterCanny - contourCanny;
  imshow("testTwo", testTwo);

  Mat together = afterCanny + contourCanny; 
  Mat testThree = together - testOne - testTwo;
  imshow("testThree", testThree);

  dilate(testThree, testThree, Mat(), Point(-1, -1), 3);
  erode(testThree, testThree, Mat(), Point(-1, -1), 3);
  dilate(testThree, testThree, Mat(), Point(-1, -1), 3);
  erode(testThree, testThree, Mat(), Point(-1, -1), 3);

  findandDrawContour(testThree, contourWindow, 0);

  imshow("testThreeafterContour", testThree);
  return testThree;
}

void findandDrawContour(Mat &roi, char* windowName, int type) {
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
		  if (type == 0) {
			  if (stdevOfWhite >= 10 || mean >= 35 || (mean - stdevOfWhite) >= 20)
				  matArr[i].setTo(0);
		  }
		  else if (type == 1) {
			  if (stdevOfWhite >= 20 || mean >= 35 || (mean - stdevOfWhite) >= 15)
				  matArr[i].setTo(0);
		  }
	  }
  }
}

Point findLineAndVP(Mat& white, Mat& frame, float& prev_Rslope, float& prev_Lslope, Point intersectionPoint, queue <float>& rightSlope , int& rightLaneKept) {
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

  for (size_t i = 0; i < lines.size(); i++) {
      Vec4i l = lines[i];
      line(frame, Point(l[0], l[1] + rec_point.y), Point(l[2], l[3] + rec_point.y), Scalar(0, 0, 255), 1);
  }

  //Merge part
  float selected_slopeR = 0;
  float selected_slopeL = 0;

  for (size_t i = 0; i < lines.size(); i++) {
      Vec4i l = lines[i];
      float slope = (((float)l[3] - (float)l[1]) / ((float)l[2] - (float)l[0]));

      //lines of right side
	  if (slope >= 0.3 && slope <= 3 && l[0] > (frame.cols / 2)) {
		  if (prev_Rslope != 0) {
			  float prev_rightb = intersectionPoint.y - prev_Rslope * intersectionPoint.x;
			  float test_rightb = (l[1] + rec_point.y) - slope * l[0];
			  float rightBdiff = abs(prev_rightb - test_rightb);

			  cout << "prev right: " << prev_rightb << "test_rightb: " << test_rightb << "diff: " << rightBdiff << endl;

			  if (rightBdiff < 8) {
				  countright++;
				  x1 += l[0];
				  y1 += l[1] + rec_point.y;
				  x2 += l[2];
				  y2 += l[3] + rec_point.y;
				  selected_slopeR = slope;
				  line(frame, Point(l[0], l[1] + rec_point.y), Point(l[2], l[3] + rec_point.y), Scalar(0, 255, 0), 1);
			  }
		  }
		  else {
			 
					  countright++;
					  x1 += l[0];
					  y1 += l[1] + rec_point.y;
					  x2 += l[2];
					  y2 += l[3] + rec_point.y;
					  selected_slopeR = slope;
					  line(frame, Point(l[0], l[1] + rec_point.y), Point(l[2], l[3] + rec_point.y), Scalar(0, 255, 0), 1);

		  }
      }
      //lines of left side
	  if (slope <= -0.3  && slope >= -3 && l[0] < (frame.cols / 2)) {
		  if (prev_Rslope != 0) {
			  float prev_leftb = intersectionPoint.y - prev_Lslope * intersectionPoint.x;
			  float test_leftb = (l[1] + rec_point.y) - slope * l[0];
			  float leftBdiff = abs(prev_leftb - test_leftb);

			  cout << "prev LEFT: " << prev_leftb << "test_leftb: " << test_leftb << "diff: " << leftBdiff << endl;

			  if (leftBdiff < 8) {
				  countleft++;
				  x3 += l[0];
				  y3 += l[1] + rec_point.y;
				  x4 += l[2];
				  y4 += l[3] + rec_point.y;
				  selected_slopeL = slope;
				  line(frame, Point(l[0], l[1] + rec_point.y), Point(l[2], l[3] + rec_point.y), Scalar(0, 255, 0), 1);
			  }
		  }
		  else {
			  countleft++;
			  x3 += l[0];
			  y3 += l[1] + rec_point.y;
			  x4 += l[2];
			  y4 += l[3] + rec_point.y;
			  selected_slopeL = slope;
			  line(frame, Point(l[0], l[1] + rec_point.y), Point(l[2], l[3] + rec_point.y), Scalar(0, 255, 0), 1);
		  }
      }
  }

  float Rslope, Lslope, rb, lb;
  //if it is the first time, put initial values.


  /*RIGHT LANE*/
  float slopeToTestR = 0;
  float RIGHTSlopeDiffR = 0, RIGHTBDiff = 0;
  float prevBR = intersectionPoint.y - prev_Rslope * intersectionPoint.x, BToTestR = 0;
 

  if (countright == 0 && prev_Rslope != 0) {
	  Rslope = prev_Rslope;
	  rb = intersectionPoint.y - Rslope * intersectionPoint.x;
	  
	  cout << "R kept" << endl;
  }
  else  {
	  Rslope = (y2 - y1) / (x2 - x1);
	  prev_Rslope = Rslope;
	  rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
  }

  //// always get a new slope for the first frame
  //if (prev_Rslope == 0) {
  //    Rslope = (y2 - y1) / (x2 - x1);
  //    prev_Rslope = Rslope;
  //    rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
  //}
  //else if (countleft == 0) {
  //    Rslope = prev_Rslope;
  //    rb = intersectionPoint.y - Rslope * intersectionPoint.x;
  //}
  //else  {
  //    slopeToTestR = (y2 - y1) / (x2 - x1);
  //    RIGHTSlopeDiffR = abs(slopeToTestR - prev_Lslope);
  //    BToTestR = (y1 / countright + y) - slopeToTestR * (x1 / countright + x);
  //    RIGHTBDiff = abs(BToTestR - prevBR);
  //    cout << "Sloediff: " << RIGHTSlopeDiffR << " Currnet B: " << BToTestR << " prevB: " << prevBR << " rightDiff: " << RIGHTBDiff << endl;
  //    //      if (leftLaneKept < 5 && ((leftSlopeDiff < 0.04 && leftBDiff < 20) || (leftSlopeDiff > 0.3 &&  leftBDiff > 30) || leftBDiff > 30)){

  //    if (rightLaneKept < 10 && ((RIGHTSlopeDiffR > 0.3 || RIGHTBDiff > 20) || (RIGHTSlopeDiffR < 0.04 && RIGHTBDiff < 20))) {
	 // cout << "LeftLane Kept for " << rightLaneKept << " times" << endl;
	 // Rslope = prev_Rslope;
	 // rb = intersectionPoint.y - Rslope * intersectionPoint.x;
	 // cout << "Slope : " << Rslope << " Currnet B: " << rb << endl;
	 // rightLaneKept++;
  //    }
  //    else {
	 // Lslope = (y4 - y3) / (x4 - x3);
	 // prev_Lslope = Lslope;
	 // lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
	 // rightLaneKept = 0;
  //    }
  //}

  cout << "Right slope: " << Rslope << endl;

  /*LEFT LANE*/
  float slopeToTest = 0;
  float leftSlopeDiff= 0, leftBDiff = 0;
  float prevB = intersectionPoint.y - prev_Lslope * intersectionPoint.x, BToTest = 0;

  //// always get a new slope for the first frame
  //if (prev_Lslope == 0) {
  //    Lslope = (y4 - y3) / (x4 - x3);
  //    prev_Lslope = Lslope;
  //    lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
  //}
  //else if (countleft == 0) {
  //    Lslope = prev_Lslope;
  //    lb = intersectionPoint.y - Lslope * intersectionPoint.x;
  //}
  //else  {
  //    slopeToTest = (y4 - y3) / (x4 - x3);
  //    leftSlopeDiff = abs(slopeToTest - prev_Lslope);
  //    BToTest = (y3 / countleft + y) - slopeToTest * (x3 / countleft + x);
  //    leftBDiff = abs(BToTest - prevB);
  //    cout << "Sloediff: " << leftSlopeDiff << " Currnet B: " << BToTest << " prevB: " << prevB << " leftBDiff: " << leftBDiff << endl;
  //    //      if (leftLaneKept < 5 && ((leftSlopeDiff < 0.04 && leftBDiff < 20) || (leftSlopeDiff > 0.3 &&  leftBDiff > 30) || leftBDiff > 30)){

  //    if (leftLaneKept < 10 && ((leftSlopeDiff > 0.3 || leftBDiff > 20) || (leftSlopeDiff < 0.04 && leftBDiff < 20))&& leftBDiff < 200) {
	 // cout << "LeftLane Kept for " << leftLaneKept << " times" << endl;
	 // Lslope = prev_Lslope;
	 // lb = intersectionPoint.y - Lslope * intersectionPoint.x;
	 // cout << "Slope : " << Lslope << " Currnet B: " << lb << endl;
	 // leftLaneKept++;
  //    } else {
	 // Lslope = (y4 - y3) / (x4 - x3);
	 // prev_Lslope = Lslope;
	 // lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
	 // leftLaneKept = 0;
  //    }
  //}

  
   if (countleft == 0 && prev_Lslope != 0) {
      Lslope = prev_Lslope;
      lb = intersectionPoint.y - Lslope * intersectionPoint.x;
	  cout << "L kept" << endl;

   }
   else {
      Lslope = (y4 - y3) / (x4 - x3);
      leftSlopeDiff = abs(Lslope - prev_Lslope);
      prev_Lslope = Lslope;
      lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
   }

  cout << "Left slope: " << Lslope << endl <<endl;
  float lastx1 = (0 - rb) / Rslope;
  float lastx2 = (frame.rows - rb) / Rslope;
  float lastx3 = ((0 - lb) / Lslope);
  float lastx4 = ((frame.rows - lb) / Lslope);

  //point of line will be drawn. extend to the end of the frame.
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

  line(frame, Point(X.at<float>(0, 0), X.at<float>(1, 0)), Point(a2, frame.rows), Scalar(255, 0, 0), 2);
  line(frame, Point(X.at<float>(0, 0), X.at<float>(1, 0)), Point(a4, frame.rows), Scalar(255, 0, 0), 2);

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

bool isStopLine(Mat& frame, Point vp, float lSlope, float rSlope)
{
  // minus - L Slope
  // plus + R Slope

  int interest_y = 168;
  float lb, rb;
  float rIntersect;

  lb = vp.y - lSlope * vp.x;
  rb = vp.y - rSlope * vp.x;
  rIntersect = rSlope * frame.cols + rb;

  // lb = y intersect

  vector<Vec4i> lines;
  Mat grayFrame;
  Mat sobelVertical;
  int min, max;
  Mat binaryImg, subtractM, drawLine;
  Point rec_point(0, interest_y);
  Point end_point(frame.cols, frame.rows);
  Rect rec(rec_point, end_point);
  float angle;

  Mat mask(frame.rows, frame.cols, CV_8UC1);
  mask.setTo(255);

  Point pts[5] = { vp, Point(0, lb), Point(0, frame.rows), Point(frame.cols, frame.rows), Point(frame.cols, rIntersect) };
  fillConvexPoly(mask, pts, 5, Scalar(0));

  cvtColor(frame, grayFrame, COLOR_BGR2GRAY);

  subtract(grayFrame, mask, subtractM);

  GaussianBlur(subtractM, subtractM, Size(3, 3), 3);

  Mat roi(subtractM, rec);

  getMinMax(roi, min, max);
  inRange(roi, min, max, binaryImg);

  imshow("BinaryImg", binaryImg);

//  Canny(binaryImg, sobelVertical, 60, 120);
//  dilate(sobelVertical, sobelVertical, Mat(), Point(-1, -1), 3);
//  erode(sobelVertical, sobelVertical, Mat(), Point(-1, -1), 3);
  Sobel(binaryImg, sobelVertical, -1, 0, 2, 3);

  imshow("Sobel", sobelVertical);

  cvtColor(subtractM, drawLine, CV_GRAY2BGR);

  HoughLinesP(sobelVertical, lines, 1, CV_PI / 180, 10, 160, 3);
  for (size_t i = 0; i < lines.size(); i++)
    {
      int line_num = 0;
      Vec4i l = lines[i];
      angle = atan2(l[1] - l[3], l[0] - l[2]) * 180 / CV_PI;
      cout << "Angle = " << angle << endl;;
      if((angle > -5 && angle < 5) || (angle > 175 && angle < 185))
	{
	  line(drawLine, Point(l[0], l[1] + interest_y), Point(l[2], l[3] + interest_y), Scalar(70, 200, 255), 2);
	  line_num++;
	}
      if(line_num >= 2) return true;
    }

  imshow("drawLine", drawLine);

  return false;
}

void getMinMax(Mat& roi, int& min, int& max)
{
  // showing the brightest point
  float meanVal = 0, stdDevVal = 0;
  Point maxPoint;
  double maxPixelVal;
  Scalar mean;
  Scalar stdDev;

  // find min and max value and point
  minMaxLoc(roi, 0, &maxPixelVal, 0, &maxPoint);

  // Calculate mean and stdDev of ROI
  meanStdDev(roi, mean, stdDev);
  meanVal = mean.val[0];
  stdDevVal = stdDev.val[0];

  // calculate min value of inRange
  min = meanVal + 2 * stdDevVal;

  // max value = mean + stdDev
  max = meanVal + 3 * stdDevVal; // maxPixelVal;
}

int main() {
  char title[100] = "mono.mp4";
  VideoCapture capture(title);

  Mat frame, afterPreprocess;
  Mat originalFrame = frame.clone();

  int key, frameNum = 1, frame_rate = 30;
  vector<Point> io_prev_lanes;
  float prev_Rslope = 0, prev_Lslope = 0;
  Point prev_intersectionPoint(0, 0);
  int leftLaneKept = 0, rightLaneKept = 0;

  queue <float> rightSlope;
  queue <float> rightB;


  // videoRead
  while (1) {
      if (!capture.read(frame))
	break;

      Mat originalFrame = frame.clone();
      afterPreprocess = preprocess(frame);
      imshow("afterPreprocess", afterPreprocess);

	  prev_intersectionPoint = findLineAndVP(afterPreprocess, frame, prev_Rslope, prev_Lslope, prev_intersectionPoint, rightSlope, rightLaneKept);

      imshow("frame", frame);
      isStopLine(originalFrame, prev_intersectionPoint, prev_Lslope, prev_Rslope);


      key = waitKey(frame_rate);
      if (key == 32) {
	  if (frame_rate == 30)
	    frame_rate = 0;
	  else
	    frame_rate = 30;
	  }
	  else if (key == ']') {
		  capture.set(CV_CAP_PROP_POS_FRAMES, frameNum + 90);
		  frameNum += 90;
		  prev_Rslope = 0, prev_Lslope = 0;
	  }
	  else if (key == '[') {
		  capture.set(CV_CAP_PROP_POS_FRAMES, frameNum - 90);
		  frameNum -= 90;
		  prev_Rslope = 0, prev_Lslope = 0;
	  }
	  else if (key == 'd') {
		  capture.set(CV_CAP_PROP_POS_FRAMES, frameNum + 30);
		  frameNum += 30;
		  prev_Rslope = 0, prev_Lslope = 0;
	  }
	  else if (key == 'a') {
		  capture.set(CV_CAP_PROP_POS_FRAMES, frameNum - 30);
		  frameNum -= 30;
		  prev_Rslope = 0, prev_Lslope = 0;
	  }
      else if (key == 27) {
	  break;
      }
      frameNum++;
  }
  return 0;
}
