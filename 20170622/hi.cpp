#include "opencv2/opencv.hpp"
#include <cmath>
#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <iostream>
#include <algorithm>

using namespace std;
using namespace cv;

#define LaneColor1 Scalar(10,10,255)
#define LaneColor2 Scalar(10,250,255)
#define LaneColor3 Scalar(251,10,255)
#define LaneColor4 Scalar(255,200,20)

float slopeL_PRE3 = 0 ;
float slopeR_PRE3 = 0;
float plb = 0;
float prb = 0;

float slopeL_PRE4 = 0;
float slopeR_PRE4 = 0;
float plb4 = 0;
float prb4 = 0;

void lane_detection(Mat frame)
{
    Mat gray, smot, sub, hsv, white, yellow, both, canny, left, right,frame_gray;
    Mat sub1, sub2, sub3, sub4, smot1, smot2, smot3, smot4, gray1, gray2, gray3, gray4, white1, white2, white3, white4,gray_original_sub;
    Mat canny1, canny2, canny3, canny4, left1, left2, left3, left4, right1, right2, right3, right4,hlsImg,hist;
    Mat grayl3,grayl4,grayr3,grayr4; //gray areas.
    
    vector<Mat> channels(hlsImg.channels()); // vector Mat for histogram
    vector<Vec4i> lines;
    //string A[100] = {gray1,sub1,smot1};
    
    // Variable for white detection
    int inRangeMaxl3 = 0,inRangeMaxl4 = 0,inRangeMaxr3 = 0,inRangeMaxr4 = 0;
    int inRangeMinl3 = 0,inRangeMinl4 = 0,inRangeMinr3 = 0,inRangeMinr4 = 0;
    
    
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0; //declaration of x,y variables used in lines.
    float x3 = 0, x4 = 0, y3 = 0, y4 = 0;
    
    //initiate for the first time when there are no input values.
    float countright = 0, countleft = 0;
    float xp1 = 0, xp2 = 0, yp1 = 0, yp2 = 0;
    float xp3 = 0, xp4 = 0, yp3 = 0, yp4 = 0;
    
    float a1 = 0, a2 = 0, a3 = 0, a4 = 0;
    float b1 = 0, b2 = 0, b3 = 0, b4 = 0;
    
    int x = 0, y = 0;
    // int width = frame.cols, height = frame.rows / 2;
    
    
    
    // resolution: 640 * 480
    //// mono.wmv :
    int interest_y = 168; //ROI value of y
    int interest_x = 0;
    int width = frame.cols - interest_x; //width of ROI
    int height = frame.rows - interest_y; //height of ROI
    int halfWidth = width / 2;
    int subROIHeight = height / 16;  // Calculate the height of sub_ROIs
    
    frame_gray = frame.clone(); //clone one frame. It will be used when it finds the brightest value.
    cvtColor(frame_gray, frame_gray, CV_BGR2GRAY);
    
    //Point of center in the frame.
    circle(frame, Point(frame.cols/2,frame.rows/4), 5, Scalar(0,0,0), 3, LINE_AA);
    
    
    //ROIs of each parts.
    
    Rect rec(interest_x, interest_y, width, height);  // overall ROI
    
    Point rec1_point(interest_x, interest_y);
    Point rec1_Rpoint(interest_x+halfWidth, interest_y); /// Rpoint is the start point of the right ROI
    Rect rec1(rec1_point, Size(width, subROIHeight * 2));
    
    Point rec2_point(interest_x, interest_y + subROIHeight);
    Point rec2_Rpoint(interest_x+halfWidth, interest_y + subROIHeight);
    Rect rec2(rec2_point, Size(width, subROIHeight * 2)); // ROI 2
    
    Point rec3_point(interest_x, interest_y + subROIHeight * 3);
    Point rec3_Rpoint(interest_x+halfWidth, interest_y + subROIHeight * 3);
    Rect rec3(rec3_point, Size(width, subROIHeight * 3)); // ROI3
    
    Point rec4_point(interest_x, interest_y + subROIHeight * 6);
    Point rec4_Rpoint(interest_x+halfWidth, interest_y + subROIHeight * 6);
    Rect rec4(rec4_point, Size(width, subROIHeight * 10));
    
    Rect left_rec1(rec1_point, Size(halfWidth , subROIHeight * 2));
    Rect left_rec2(rec2_point, Size(halfWidth , subROIHeight * 2));
    Rect left_rec3(rec3_point, Size(halfWidth , subROIHeight * 3));
    Rect left_rec4(rec4_point, Size(halfWidth , subROIHeight * 10));
    
    Rect right_rec1(rec1_Rpoint, Size(halfWidth , subROIHeight * 2));
    Rect right_rec2(rec2_Rpoint, Size(halfWidth , subROIHeight * 2));
    Rect right_rec3(rec3_Rpoint, Size(halfWidth , subROIHeight * 3));
    Rect right_rec4(rec4_Rpoint, Size(halfWidth , subROIHeight * 10));
    
    sub = frame(rec);
    sub1 = frame(rec1);
    sub2 = frame(rec2);
    sub3 = frame(rec3);
    sub4 = frame(rec4);
    
    //show Left and Right ROI
    
    // bgr2gray
    cvtColor(sub, gray, CV_BGR2GRAY);
    
    //parts
    cvtColor(sub1, gray1, CV_BGR2GRAY);
    cvtColor(sub2, gray2, CV_BGR2GRAY);
    cvtColor(sub3, gray3, CV_BGR2GRAY);
    cvtColor(sub4, gray4, CV_BGR2GRAY);
    
    //apply gray parts to frame_gray.
    
    grayl3 = frame_gray(left_rec3);
    grayl4 = frame_gray(left_rec4);
    grayr3 = frame_gray(right_rec3);
    grayr4 = frame_gray(right_rec4);
    
    // showing the brightest point
    
    Point maxPointl3,maxPointl4,maxPointr3,maxPointr4;
    double maxLocl3=0,maxLocl4=0,maxLocr3=0,maxLocr4=0;
    
    
    minMaxLoc(grayl3, 0,&maxLocl3,0,&maxPointl3);
    minMaxLoc(grayl4, 0,&maxLocl4,0,&maxPointl4);
    minMaxLoc(grayr3, 0,&maxLocr3,0,&maxPointr3);
    minMaxLoc(grayr4, 0,&maxLocr4,0,&maxPointr4);
    
    //mark circle to each seperated ROI.
    
    circle(frame, rec3_point + maxPointl3, 2, Scalar(255,10,10), 5, 8, 0);
    circle(frame, rec4_point + maxPointl4, 2, Scalar(255,10,10), 5, 8, 0);
    circle(frame, rec3_Rpoint + maxPointr3, 2, Scalar(255,10,10), 5, 8, 0);
    circle(frame, rec4_Rpoint + maxPointr4, 2, Scalar(255,10,10), 5, 8, 0);
    
    inRangeMaxl3 = maxLocl3;
    inRangeMaxl4 = maxLocl4;
    inRangeMaxr3 = maxLocr3;
    inRangeMaxr4 = maxLocr4;
    
    // calculate min value of inRange
    inRangeMinl3 = maxLocl3 - (2 * (260 - maxLocl3));
    inRangeMinl4 = maxLocl4 - (2 * (260 - maxLocl4));
    inRangeMinr3 = maxLocr3 - (2 * (257 - maxLocr3));
    inRangeMinr4 = maxLocr4 - (2 * (257 - maxLocr4));
    
    //appy min and max.
    inRange(grayl3, inRangeMinl3, inRangeMaxl3, grayl3);
    inRange(grayl4, inRangeMinl4, inRangeMaxl4, grayl4);
    inRange(grayr3, inRangeMinr3, inRangeMaxr3, grayr3);
    inRange(grayr4, inRangeMinr4, inRangeMaxr4, grayr4);
    
    
    //white1 = frame_gray(rec1);
    white3 = frame_gray(rec3);
    white4 = frame_gray(rec4);
    
    //dialte and erode white part to enlarge the part.
    
   
   
    dilate(white4,white4,Mat(),Point(-1,-1),3);
    erode(white4, white4, Mat());
    dilate(white3,white3,Mat(),Point(-1,-1),3);
    erode(white3, white3, Mat());

    
    
    imshow("gray_frame",frame_gray);
    imshow("grayr4",grayr4);
    imshow("grayr3",grayr3);
    
    
    /*************edge  detection  3 **************/
    Canny(white3, canny3, 150, 300, 3);
    imshow("canny3", canny3);
    
    vector<Vec4i> lines_R3;
    vector<Point> pointList_R3;
    
    //vanishing point VP
    Point vp_R3;
    
    //20, 10, 140
    HoughLinesP(canny3, lines_R3, 1, CV_PI / 180, 20, 10, 140);
    //Merge part
    
    for (size_t i = 0; i < lines_R3.size(); i++) {
        Vec4i l = lines_R3[i];
        
        float slope = ((float)l[3] - (float)l[1]) / ((float)l[2] - (float)l[0]);
        
        //lines of left side
        if (slope >= 0.3 && slope <= 3) {
            countright++;
            x1 += l[0];
            y1 += l[1] + rec3_point.y;
            x2 += l[2];
            y2 += l[3] + rec3_point.y;
        }
        //lines of right side
        if (slope <= -0.3 && slope >= -3) {
            countleft++;
            x3 += l[0];
            y3 += l[1] + rec3_point.y;
            x4 += l[2];
            y4 += l[3] + rec3_point.y;
        }
    }
    //if it is the first time, put initial values.
    if (countright == 0) {
        x1 = xp1;
        x2 = xp2;
        y1 = yp1;
        y2 = yp2;
    }
    if (countleft == 0) {
        x3 = xp3; //xp¥¬ past x.
        x4 = xp4;
        y3 = yp3;
        y4 = yp4;
    }
    
    //slopes of right and left.
    float Rslope = (y2 - y1) / (x2 - x1);
    float Lslope = (y4 - y3) / (x4 - x3);
    
    float differenceR = abs(slopeR_PRE3 - abs(Rslope));
    float differenceL = abs(slopeL_PRE3 - abs(Lslope));
    
    float rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
    float lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
    
    float lastx1, lastx2, lastx3, lastx4;
    
    bool drawR3 = false;
    
    /////// right draw //////////
    if (differenceR >= 0.09 || slopeR_PRE3 == 0) {
        lastx1 = (0 - rb) / Rslope;
        lastx2 = (frame.rows - rb) / Rslope;
        a1 = lastx1 + x;
        a2 = lastx2 + x;
        b1 = 0;
        b2 = frame.rows;
        prb = rb;
        slopeR_PRE3 = Rslope;
        
        line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor3, 3);
        
        drawR3 = true;
    }
    
    if (!drawR3) {
        lastx1 = (0 - prb) / slopeR_PRE3;
        lastx2 = (frame.rows - prb) / slopeR_PRE3;
        a1 = lastx1 + x;
        a2 = lastx2 + x;
        line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor3, 3);
    }
    // left
    drawR3 = false;
    if (differenceL >= 0.09  || slopeL_PRE3 == 0) {
        lastx3 = ((0 - lb) / Lslope);
        lastx4 = ((frame.rows - lb) / Lslope);
        a3 = lastx3 + x, a4 = lastx4 + x;
        b3 = 0, b4 = frame.rows;
        plb = lb;
        slopeL_PRE3 = Lslope;
        line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor3, 3);
        cout << "a3: " << a3 << "a4: " << a4 << endl;
        
        drawR3 = true;
    }
    
    if (!drawR3) {
        lastx3 = (0 - plb) / slopeL_PRE3;
        lastx4 = (frame.rows - plb) / slopeL_PRE3;
        a3 = lastx3 + x;
        a4 = lastx4 + x;
        cout << "PREV a3: " << a3 << "a4: " << a4 << endl;
        line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor3, 3);
    }
    
    
    float dataA[] = { (b2 - b1) / (a2 - a1), -1, (b4 - b3) / (a4 - a3), -1 };
    Mat A3(2, 2, CV_32F, dataA);
    Mat invA3;
    invert(A3, invA3);
    
    float dataB[] = { a1*(b2 - b1) / (a2 - a1) - b1, a3*(b4 - b3) / (a4 - a3) - b3 };
    Mat B3(2, 1, CV_32F, dataB);
    
    //vanishing point.
    Mat X3 = invA3*B3;
    
    
    circle(frame, Point(X3.at<float>(0, 0), X3.at<float>(1, 0)), 5, LaneColor3, 3, LINE_AA);
    
    
    /*************edge  detection  4 **************/
    
    
    Canny(white4, canny4, 150, 300, 3);
    imshow("canny4", canny4);
    
    dilate(white4,white4,Mat(),Point(-1,-1),5);
    vector<Vec4i> lines_R4;
    vector<Point> pointList_R4;
    
    Point vp_R4;
    float c1_R4, c2_R4;
    
    
    //20, 10, 140
    HoughLinesP(canny4, lines_R4, 1, CV_PI / 180, 20, 10, 140);
    //Merge part
    
    for (size_t i = 0; i < lines_R4.size(); i++) {
        Vec4i l = lines_R4[i];
        
        float slope = ((float)l[3] - (float)l[1]) / ((float)l[2] - (float)l[0]);
        
        if (slope >= 0.3 && slope <= 3) {
            //           line(frame, Point(l[0], l[1] + rec4_point.y), Point(l[2], l[3] + rec4_point.y), LaneColor4, 2);
            countright++;
            x1 += l[0];
            y1 += l[1] + rec4_point.y;
            x2 += l[2];
            y2 += l[3] + rec4_point.y;
        }
        if (slope <= -0.3 && slope >= -3) {
            //       line(frame, Point(l[0], l[1] + rec4_point.y), Point(l[2], l[3]+ rec4_point.y), LaneColor4, 2);
            countleft++;
            x3 += l[0];
            y3 += l[1] + rec4_point.y;
            x4 += l[2];
            y4 += l[3] + rec4_point.y;
        }
    }
    
    /*
     Rslope = (y2 - y1) / (x2 - x1);
     Lslope = (y4 - y3) / (x4 - x3);
     
     rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
     lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
     
     lastx1 = (0 - rb) / Rslope;
     lastx2 = (frame.rows - rb) / Rslope;
     
     lastx3 = ((0 - lb) / Lslope);
     lastx4 = ((frame.rows - lb) / Lslope);
     
     a1 = lastx1 + x, a2 = lastx2 + x, a3 = lastx3 + x, a4 = lastx4 + x;
     b1 = 0, b2 = frame.rows, b3 = 0, b4 = frame.rows;*/
    
    
    //slopes of right and left.
    Rslope = (y2 - y1) / (x2 - x1);
    Lslope = (y4 - y3) / (x4 - x3);
    
    differenceR = abs(slopeR_PRE4 - abs(Rslope));
    differenceL = abs(slopeL_PRE4 - abs(Lslope));
    
    rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
    lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
    
    //float lastx1, lastx2, lastx3, lastx4;
    
    bool drawR4 = false;
    
    /////// right draw //////////
    if (differenceR >= 0.09 || slopeR_PRE4 == 0) {
        lastx1 = (0 - rb) / Rslope;
        lastx2 = (frame.rows - rb) / Rslope;
        a1 = lastx1 + x;
        a2 = lastx2 + x;
        b1 = 0;
        b2 = frame.rows;
        prb4 = rb;
        slopeR_PRE4 = Rslope;
        
        line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor4, 3);
        
        drawR4 = true;
    }
    
    if (!drawR4) {
        lastx1 = (0 - prb4) / slopeR_PRE4;
        lastx2 = (frame.rows - prb4) / slopeR_PRE4;
        a1 = lastx1 + x;
        a2 = lastx2 + x;
        line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor4, 3);
    }
    // left
    drawR4 = false;
    if (differenceL >= 0.09 || slopeL_PRE4 == 0) {
        lastx3 = ((0 - lb) / Lslope);
        lastx4 = ((frame.rows - lb) / Lslope);
        a3 = lastx3 + x, a4 = lastx4 + x;
        b3 = 0, b4 = frame.rows;
        plb4 = lb;
        slopeL_PRE4 = Lslope;
        line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor4, 3);
        //cout << "a3: " << a3 << " a4: " << a4 << endl;
        
        drawR4 = true;
    }
    
    if (!drawR4) {
        lastx3 = (0 - plb4) / slopeL_PRE4;
        lastx4 = (frame.rows - plb4) / slopeL_PRE4;
        a3 = lastx3 + x;
        a4 = lastx4 + x;
        //cout << "PREV a3: " << a3 << " a4: " << a4 << endl;
        line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor4, 3);
    }
    
    
    float dataA4[] = { (b2 - b1) / (a2 - a1), -1, (b4 - b3) / (a4 - a3), -1 };
    Mat A4(2, 2, CV_32F, dataA4);
    Mat invA4;
    invert(A4, invA4);
    
    float dataB4[] = { a1*(b2 - b1) / (a2 - a1) - b1, a3*(b4 - b3) / (a4 - a3) - b3 };
    Mat B4(2, 1, CV_32F, dataB4);
    
    Mat X4 = invA4*B4;
    
    //  line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor4, 3);
    //  line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor4, 3);
    
    
    circle(frame, Point(X4.at<float>(0, 0), X4.at<float>(1, 0)), 5, LaneColor4, 3, LINE_AA);
    
    
    if(((X4.at<float>(0, 0)>frame.cols/2)||(X3.at<float>(0, 0)>frame.cols/2)))
    {
        putText(frame, "Turn right" ,Point(100,400) , FONT_HERSHEY_PLAIN, 2, Scalar(LaneColor4), 2, LINE_8);
    }
    else if(((X4.at<float>(0, 0)<frame.cols/2)&&(X3.at<float>(0, 0)<frame.cols/2)))
    {
        putText(frame, "Turn left" ,Point(100,400) , FONT_HERSHEY_PLAIN, 2, Scalar(LaneColor4), 2, LINE_8);
    }
    
    
    
}

// Namsoo's storage /Users/NAMSOO/Documents/Xcode/OpenCV/VanishingPoint/VanishingPoint/

int main() {
    char title[100] = "/Users/NAMSOO/Documents/Xcode/OpenCV/VanishingPoint/VanishingPoint/mono.mp4";
    VideoCapture capture(title);
    Mat frame;
    Mat origin;
    
    int key, frameNum = 1;
    int frame_rate = 30;
    
    capture.read(frame);
    
    // videoRead
    while (capture.read(frame)) {
        
        origin = frame.clone();
        lane_detection(frame);
        
        imshow("frame", frame); //show the original frame
        //imshow("origin",origin);
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
