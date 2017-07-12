#include <iostream>
#include <opencv2/opencv.hpp>
#include "MSAC.h"

#define _cplusplus

using namespace std;
using namespace cv;

void laneMarkingsDetector(Mat& srcGray, Mat& dstGray, int tau);

int main()
{
	Mat img;
	VideoCapture cap;
	char ch;
	int fps = 10;
	int fr_cnt = 0;

	cap.open("/Users/NAMSOO/Documents/Xcode/OpenCV/VanishingPoint/VanishingPoint/mono.mp4");
    
    

	while (1)
	{
		//cap >> img;
		if (!cap.read(img))
			break;
		imshow("Input Image", img);
		Mat output;
		output = img.clone();

		//convert type to gray
		Mat gray;
		cvtColor(img, gray, CV_BGR2GRAY);

		//get roi
		Rect roiRect(0, gray.rows / 3 , gray.cols, gray.rows / 3 *2);
		Mat roi = gray(roiRect).clone();
		imshow("ROI", roi);
		Mat Roi;
		copyMakeBorder(roi, Roi, gray.rows / 3, 0, 0, 0, BORDER_CONSTANT, Scalar(0, 0, 0));
		imshow("Roi_border", Roi);

		//apply marking detect algrithm
		Mat MarkingResult(Roi.rows, Roi.cols, Roi.type());
		int tau = 18;
		laneMarkingsDetector(Roi, MarkingResult, tau);
		imshow("Lane Marking Result", MarkingResult);

		//Thresholding
		Mat thresh;
		threshold(MarkingResult, thresh, 50, 255, THRESH_BINARY);
		imshow("Threshold", thresh);
        
        
        

		//Use Hough transform
		Mat canny;
		Canny(thresh, canny, 30, 100);
		vector<vector<cv::Point>> lineSegments;
		vector<cv::Point> aux;
		vector<Vec4i> lines;
		int houghThreshold = 70;
		if (gray.cols*gray.rows < 400 * 400)
			houghThreshold = 100;

		HoughLinesP(canny, lines, 1, CV_PI / 180, houghThreshold, 20, 10);

		while (lines.size() > 200)
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
			line(output, pt1, pt2, CV_RGB(0, 0, 0), 2);
			/*circle(outputImg, pt1, 2, CV_RGB(255,255,255), CV_FILLED);
			circle(outputImg, pt1, 3, CV_RGB(0,0,0),1);
			circle(outputImg, pt2, 2, CV_RGB(255,255,255), CV_FILLED);
			circle(outputImg, pt2, 3, CV_RGB(0,0,0),1);*/

			// Store into vector of pairs of Points for msac
			aux.clear();
			aux.push_back(pt1);
			aux.push_back(pt2);
			lineSegments.push_back(aux);
		}

		std::vector<cv::Mat> vps;
		vector<int> numInliers;
		std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;

		// Initialization
		MSAC msac;
		msac.init(MODE_NIETO, Size(img.cols, img.rows), 1);
		// Execution (passing as argument the lines obtained with the Hough transform)
		msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, 1);

		//char msg[30];
		//for(int i = 0; i < vps.size(); i++)
		//{
		//	sprintf(msg, "VPS %d", i);
		//	imshow(msg, vps[i]);
		//}
		if (!vps.empty()){
			int x = (int)vps[0].at<float>(0, 0);
			int y = (int)vps[0].at<float>(1, 0);
			circle(output, Point(x, y), 5, Scalar(0, 0, 255), -1);
		}
		
		//imshow("VPS", vps[0]);
		imshow("Result", output);
        
        /**********
         
         */
        
        
        

		fr_cnt = (int)cap.get(CV_CAP_PROP_POS_FRAMES);
		ch = waitKey(fps);

		if (ch == 27) {
			break; // escape main loop
		}
		else if (ch == 32)
		{
			if (fps == 10)
				fps = 0;
			else
				fps = 10;
		}
		else if (ch == '[') // 100프레임 앞으로 이동
		{
			fr_cnt -= 50;
			cap.set(CV_CAP_PROP_POS_FRAMES, fr_cnt - 1);
			fr_cnt--;
		}
		else if (ch == ']') // 100프레임 뒤로 이동
		{
			fr_cnt += 50;
			cap.set(CV_CAP_PROP_POS_FRAMES, fr_cnt - 1);
			fr_cnt--;
		}

	}
}

void laneMarkingsDetector(Mat& srcGray, Mat& dstGray, int tau)
{
	dstGray.setTo(0);

	int aux = 0;
	for (int j = 0; j < srcGray.rows; ++j)
	{
		unsigned char *ptRowSrc = srcGray.ptr<uchar>(j);
		unsigned char *ptRowDst = dstGray.ptr<uchar>(j);

		for (int i = tau; i < srcGray.cols - tau; ++i)
		{
			if (ptRowSrc[i] != 0)
			{
				aux = 2 * ptRowSrc[i];
				aux += -ptRowSrc[i - tau];
				aux += -ptRowSrc[i + tau];
				aux += -abs((int)(ptRowSrc[i - tau] - ptRowSrc[i + tau]));

				aux = (aux < 0) ? 0 : (aux);
				aux = (aux > 255) ? 255 : (aux);

				ptRowDst[i] = (unsigned char)aux;

			}
		}
	}
}
