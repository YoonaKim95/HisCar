#include <iostream>
#include <opencv2/opencv.hpp>
#include "MSAC.h"

#define _cplusplus

using namespace std;
using namespace cv;

void laneMarkingsDetector(Mat& srcGray, Mat& dstGray, int tau);
vector<vector<Point> > findandDrawContour(Mat &roi, char* windowName);

void SplitROI(Mat ROI4, Mat gray, Mat img, Mat output, int Num_ROI,Point rec);

int main()
{
    Mat img;
    VideoCapture cap;
    char ch;
    int fps = 10;
    int fr_cnt = 0;
    
    cap.open("/Users/NAMSOO/Documents/Xcode/OpenCV/mono.mp4");
    
    while (1)
    {
        //cap >> img;
        if (!cap.read(img))
            break;
        //imshow("Input Image", img);
        Mat output;
        output = img.clone();
        
        //convert type to gray
        Mat gray;
        cvtColor(img, gray, CV_BGR2GRAY);
        
        //get roi
        Rect roiRect(0, gray.rows / 3 , gray.cols, gray.rows / 3 *2);
        Mat roi = gray(roiRect).clone();
        //imshow("ROI", roi);
        Mat Roi;
        copyMakeBorder(roi, Roi, gray.rows / 3, 0, 0, 0, BORDER_CONSTANT, Scalar(0, 0, 0));
        //imshow("Roi_border", Roi);
        
        //apply marking detect algrithm
        Mat MarkingResult(Roi.rows, Roi.cols, Roi.type());
        int tau = 18;
        laneMarkingsDetector(Roi, MarkingResult, tau);
        imshow("Lane Marking Result", MarkingResult);
        
        //Thresholding
        Mat thresh;
        threshold(MarkingResult, thresh, 50, 255, THRESH_BINARY);
        imshow("Threshold", thresh);
        
        
        //*************ROI of Rectangles
        
        Mat grayl3, grayl4, grayr3, grayr4, frame_lane;
        
        int interest_y = 168; //ROI value of y
        int interest_x = 0;
        int width = img.cols - interest_x; //width of ROI
        int height = img.rows - interest_y; //height of ROI
        int halfWidth = width / 2;
        int subROIHeight = height / 16;
        
        Point rec2_point(interest_x, interest_y);
        Point rec2_Rpoint(interest_x+halfWidth, interest_y + subROIHeight);
        Rect rec2(rec2_point, Size(width, subROIHeight * 3)); // ROI 2
        
        Point rec3_point(interest_x, interest_y + subROIHeight * 3);
        Point rec3_Rpoint(interest_x + halfWidth, interest_y + subROIHeight * 3);
        Rect rec3(rec3_point, Size(width, subROIHeight * 3)); // ROI3
        
        Point rec4_point(interest_x, interest_y + subROIHeight * 6);
        Point rec4_Rpoint(interest_x + halfWidth, interest_y + subROIHeight * 6);
        Rect rec4(rec4_point, Size(width, subROIHeight * 10));
        
        Rect left_rec3(rec3_point, Size(halfWidth, subROIHeight * 3));
        Rect left_rec4(rec4_point, Size(halfWidth, subROIHeight * 10));
        Rect right_rec3(rec3_Rpoint, Size(halfWidth, subROIHeight * 3));
        Rect right_rec4(rec4_Rpoint, Size(halfWidth, subROIHeight * 10));
        
        /******************Canny and HoughlinesP*****************/

        Mat ROI4, ROI3, ROI2, ROI1;
        
        ROI4 = thresh(rec4);
        ROI3 = thresh(rec3);
        ROI2 = thresh(rec2);
        
        SplitROI(ROI4,gray,img,output,4,rec4_point);
       SplitROI(ROI3,gray,img,output,3,rec3_point);
       SplitROI(ROI2,gray,img,output,2,rec2_point);

       // Mat SplitROI(Mat ROI, Mat gray, int Num_ROI);
//        
//        Mat cannyDetect = SplitROI(ROI4, 4);
        
      //ROI4 = SplitROI(ROI4, gray, 4);
    
        
        Mat cannyDetect;
        Canny(thresh, cannyDetect, 150, 300, 3);
        /****
        Mat cannyDetect;
        Canny(thresh, cannyDetect, 150, 300, 3);
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
        
        while (lines.size() > 200) //why more than 200?
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
            
            
            // Store into vector of pairs of Points for msac
            aux.clear();
            aux.push_back(pt1);
            aux.push_back(pt2);
            lineSegments.push_back(aux);
        }
        
        std::vector<cv::Mat> vps;
        vector<int> numInliers;
        std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;
        
         ***/

        
        
        /******************Drawing Contour to lanes********************/
        
        char contourL3[10] = "ContourL3";
        char contourR3[10] = "ContourR3";
        char contourL4[10] = "ContourL4";
        char contourR4[10] = "ContourR4";
        
        frame_lane = cannyDetect.clone(); //get MAT from cannyEdge
        
        imshow("frame_lane",frame_lane);

        grayl3 = frame_lane(left_rec3);
        grayl4 = frame_lane(left_rec4);
        grayr3 = frame_lane(right_rec3);
        grayr4 = frame_lane(right_rec4);
        
        vector<vector<Point> > pointSetl3;
        vector<vector<Point> > pointSetl4;
        vector<vector<Point> > pointSetr3;
        vector<vector<Point> > pointSetr4;
        imshow("L11", grayl3);
        pointSetl3 = findandDrawContour(grayl3, contourL3);
        pointSetl4 = findandDrawContour(grayl4, contourL4);
        pointSetr3 = findandDrawContour(grayr3, contourR3);
        pointSetr4 = findandDrawContour(grayr4, contourR4);
        
        imshow("L1", grayl3);

        
        /******************* MSAC Part ********************************/
        
//        // Initialization
//        MSAC msac;
//        msac.init(MODE_NIETO, Size(img.cols, img.rows), 1);
//        // Execution (passing as argument the lines obtained with the Hough transform)
//        msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, 1);
//        
//        
//        if (!vps.empty()){
//            int x = (int)vps[0].at<float>(0, 0);
//            int y = (int)vps[0].at<float>(1, 0);
//            circle(output, Point(x, y), 5, Scalar(0, 0, 255), -1);
//        }
//                //imshow("VPS", vps[0]);
//                imshow("Result", output);
//
        
        /************************************/
        
        
        fr_cnt = (int)cap.get(CV_CAP_PROP_POS_FRAMES);
        ch = waitKey(fps);
        
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

void SplitROI(Mat ROI, Mat gray, Mat img, Mat output, int Num_ROI,Point rec)
{

Mat cannyDetect;
Canny(ROI, cannyDetect, 150, 300, 3);
//Use Hough transform
Mat canny;
Canny(ROI, canny, 30, 100);
vector<vector<cv::Point>> lineSegments;
vector<cv::Point> aux;
vector<Vec4i> lines;
int houghThreshold = 70;
if (gray.cols*gray.rows < 400 * 400)
    {
        houghThreshold = 100;
    }
    if(Num_ROI==4)
        houghThreshold=10;
    if(Num_ROI==3)
        houghThreshold=10;
HoughLinesP(canny, lines, 1, CV_PI / 180, houghThreshold, 20, 10);

while (lines.size() > 200) //why more than 200?
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
    line(output, pt1+Point(rec.x,rec.y), pt2+Point(rec.x,rec.y), CV_RGB(0, 0, 0), 2);
    
    
    // Store into vector of pairs of Points for msac
    aux.clear();
    aux.push_back(pt1);
    aux.push_back(pt2);
    lineSegments.push_back(aux);
    
}
    
    std::vector<cv::Mat> vps;
    vector<int> numInliers;
    std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;

//     Initialization
            MSAC msac;
            msac.init(MODE_NIETO, Size(img.cols, img.rows), 1);
            // Execution (passing as argument the lines obtained with the Hough transform)
            msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, 1);
    
    
            if (!vps.empty()){
                int x = (int)vps[0].at<float>(0, 0);
                int y = (int)vps[0].at<float>(1, 0);
                if(Num_ROI==4)
                    circle(output, Point(x+rec.x, y+rec.y), 5, Scalar(255, 0, 0), -1);
                else if(Num_ROI==3)
                    circle(output, Point(x+rec.x, y+rec.y), 5, Scalar(0, 255,0), -1);
                else if(Num_ROI==2)
                    circle(output, Point(x+rec.x, y+rec.y), 5, Scalar(0, 0, 255), -1);
            }
                    //imshow("VPS", vps[0]);
                    imshow("Result", output);
    if(Num_ROI==4)
         imshow("Result4", ROI);
    else if(Num_ROI==3)
        imshow("Result3", ROI);
    else if(Num_ROI==2)
       imshow("Result2", ROI);

}


vector<vector<Point> > findandDrawContour(Mat &roi, char* windowName)
{
    
    vector<vector<Point> > contours;
    //meaning of mode?
    int mode = RETR_EXTERNAL;
    
    dilate(roi, roi, Mat(), Point(-1, -1), 3);
    erode(roi, roi, Mat(), Point(-1, -1), 3);
    
    int method = CHAIN_APPROX_NONE;
    
    vector<Vec4i> hierarchy;
    
    findContours(roi, contours, hierarchy, mode, method);
    cout << "contours.size()=" << contours.size() << endl;
    imshow(windowName, roi);
    
    cvtColor(roi, roi, COLOR_GRAY2BGR);
    
    if (contours.size() > 1 && contours.size() <= 20) {
        Scalar color(255, 255, 10);
        vector<Rect> rect(contours.size());
        vector<Mat> matArr(contours.size());
        
        int k = 0, j = 0;
        int sizeOfRect = 30;
        
        for (int i = 0; i < contours.size(); i++) {
            Rect temp = boundingRect(Mat(contours[i]));
            if (temp.area() > 30) {
                rect[k] = temp;
                k++;
            }
        }
        
        for (int i = 0; i < k; i++)
            rectangle(roi, rect[i], color, 2, 8, 0);
        
        //imshow(windowName, roi);
    }
    
    return contours;
}
