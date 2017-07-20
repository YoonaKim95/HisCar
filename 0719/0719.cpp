#include "opencv2/opencv.hpp"
#include <cmath>
#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <string>
#include <sstream>
#include "MSAC.h"

using namespace std;
using namespace cv;

void findandDrawContour(Mat& roi, char* windowName);
Mat preprocess(Mat& frame);
Point findLineAndVP(Mat& white, Mat& frame, Point& rec_point, float& prev_Rslope, float& prev_Lslope, Point intersectionPoint);
void laneMarkingsDetector(Mat& srcGray, Mat& dstGray, int tau);
float SplitROI(Mat& ROI, Mat& img, Mat& output, int Num_ROI,Point rec,int interest_y);

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
    int interest_y_MSAC = frame.rows / 3; //ROI value of y
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
    
    /**************** MSAC ********************/
    Mat output;
    output = frame.clone();

    //convert type to gray
    Mat MSAC_gray;
    cvtColor(frame, MSAC_gray, CV_BGR2GRAY);
    
    //get roi
    Point rec_MSAC(interest_x, interest_y_MSAC);
    Rect roiRect(0, MSAC_gray.rows / 3 , MSAC_gray.cols, MSAC_gray.rows / 3 *2);
    Mat roi = MSAC_gray(roiRect).clone();
    imshow("ROI", roi);
    Mat Roi;
    
    //set a ROI to the original input image at proper location.
    //BORDER_CONSTANT is border type. It always give -1 as a return.
    copyMakeBorder(roi, Roi, MSAC_gray.rows / 3, 0, 0, 0, BORDER_CONSTANT, Scalar(255, 0, 0));
    
    //apply marking detect algrithm
    Mat MarkingResult(Roi.rows, Roi.cols, Roi.type());
    //tau is divisor function in number theory.
    int tau = 18;
    //pre-processing to get binary image. (Almost similar to binary image)
    laneMarkingsDetector(Roi, MarkingResult, tau);
    imshow("Lane Marking Result", MarkingResult);
    
    
//    Point rec2and3_point(interest_x, interest_y);
//    Point rec2and3_Rpoint(interest_x+halfWidth, interest_y);
//    Rect rec2and3(rec2_point, Size(width, subROIHeight * 6)); // ROI 2
    
    //Thresholding
    Mat thresh;
    threshold(MarkingResult, thresh, 50, 255, THRESH_BINARY);
    imshow("Threshold", thresh);
    
    Mat ROI_MSAC ;
    ROI_MSAC = thresh(roiRect);
    
    float location = 0;
    location = SplitROI(ROI_MSAC, frame, output, 1, rec_MSAC, interest_y_MSAC);

    

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

void laneMarkingsDetector(Mat& srcGray, Mat& dstGray, int tau)
{
    //set all the pixels to 0 in dstGray.
    dstGray.setTo(0);
    
    int aux = 0;
    for (int j = 0; j < srcGray.rows; ++j)
    {
        //point of j th pixel of Row.
        unsigned char *ptRowSrc = srcGray.ptr<uchar>(j);
        unsigned char *ptRowDst = dstGray.ptr<uchar>(j);
        
        //tau is needed to cut left and right side of image.
        for (int i = tau; i < srcGray.cols - tau; ++i)
        {
            if (ptRowSrc[i] != 0)
            {
                //
                aux = 2 * ptRowSrc[i];
                aux += -ptRowSrc[i - tau];
                aux += -ptRowSrc[i + tau];
                aux += -abs((int)(ptRowSrc[i - tau] - ptRowSrc[i + tau]));
                
                //
                aux = (aux < 0) ? 0 : (aux);
                aux = (aux > 255) ? 255 : (aux);
                
                ptRowDst[i] = (unsigned char)aux;
                
            }
        }
    }
}

/***************Finding MSAC***********/
float SplitROI(Mat& ROI, Mat& img, Mat& output, int Num_ROI,Point rec,int interest_y)
{
    float location = 0 ;
    //canny
    Mat cannyDetect;
    Canny(ROI, cannyDetect, 150, 300, 3);
    //Use Hough transform
    Mat canny;
    Canny(ROI, canny, 30, 100);
    vector<vector<cv::Point>> lineSegments;
    vector<cv::Point> aux;
    vector<Vec4i> lines;
    int houghThreshold = 50;
    if (img.cols*img.rows < 400 * 400)
    {
        houghThreshold = 50;
    }
    if(Num_ROI==4)
        houghThreshold=5;
    if(Num_ROI==3)
        houghThreshold=5;
    HoughLinesP(canny, lines, 1, CV_PI / 180, houghThreshold, 20, 10);
    
    while (lines.size() > 200) //to decrease threshold of houghlines.
    {
        lines.clear();
        houghThreshold += 10;
        cv::HoughLinesP(canny, lines, 1, CV_PI / 180, houghThreshold, 10, 10);
    }
    
    
    for (size_t i = 0; i < lines.size(); i++)
    {
        Point pt1, pt2;
        pt1.x = lines[i][0];
        pt1.y = lines[i][1];
        pt2.x = lines[i][2];
        pt2.y = lines[i][3];
        //line(output, pt1+Point(rec.x,rec.y), pt2+Point(rec.x,rec.y), CV_RGB(0, 0, 0), 2);
        
        
        // Store into vector of pairs of Points for msac
        aux.clear();
        aux.push_back(pt1);
        aux.push_back(pt2);
        lineSegments.push_back(aux);
        
    }
    
    std::vector<cv::Mat> vps;
    vector<int> numInliers;
    std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;
    
    /*************MSAC Part ***************/
    
    //Initialization
    MSAC msac;
    msac.init(MODE_NIETO, Size(img.cols, img.rows), 1);
    // Execution (passing as argument the lines obtained with the Hough transform)
    msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, 1);
    
    
    
    
    //****** Marking vps found in MSAC **********/
    
    
    int pre_x;
    int pre_y;
    
    if (!vps.empty()){
        int x = (int)vps[0].at<float>(0, 0);
        int y = (int)vps[0].at<float>(1, 0);
        
        pre_x = x;
        pre_y = y;
        
//        //                if(Num_ROI==4)
//        //                  //  circle(output, Point(x+rec.x, y+rec.y), 5, Scalar(255, 0, 0), -1);
//        if(Num_ROI==3)
//            //                    if(y+rec.y>interest_y)
//            circle(output, Point(x+rec.x, y+rec.y), 5, Scalar(0, 255,0), -1);
//        else if(Num_ROI==2)
//            //                    if(y+rec.y<interest_y)
            circle(output, Point(x+rec.x, y+rec.y), 5, Scalar(0, 0, 255), -1);
        if(Num_ROI==1)
            circle(output, Point(pre_x+rec.x, pre_y+rec.y), 5, Scalar(255, 0, 255), -1);
        
    }
    

    else
    {
        if(Num_ROI==3)
            //                    if(pre_y+rec.y>interest_y)
            circle(output, Point(pre_x+rec.x, pre_y+rec.y), 5, Scalar(0, 255,0), -1);
        else if(Num_ROI==2)
            //                    if(pre_y+rec.y<interest_y)
            circle(output, Point(pre_x+rec.x, pre_y+rec.y), 5, Scalar(0, 0, 255), -1);
        else if(Num_ROI==1)
            circle(output, Point(pre_x+rec.x, pre_y+rec.y), 5, Scalar(255, 0, 255), -1);
        
        cout << "pre point" << endl;
    }
    
    imshow("img",img);
    imshow("Result", output);
    
    if(Num_ROI==4)
        imshow("Result4", ROI);
    else if(Num_ROI==3)
        imshow("Result3", ROI);
    else if(Num_ROI==2)
        imshow("Result2", ROI);
    
    return location;
    
}



int main() {
	char title[100] = "/Users/NAMSOO/Documents/Xcode/OpenCV/mono.mp4";
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
