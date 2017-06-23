//#include "opencv2\highgui\highgui.hpp"
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


float slopeR_Pre = 0;
float slope_Pre = 0;


void lane_detection(Mat frame)
{
    Mat gray, smot, sub, hsv, white, yellow, both, canny, left, right;
    
    Mat sub1, sub2, sub3, sub4, smot1, smot2, smot3, smot4, gray1, gray2, gray3, gray4, white1, white2, white3, white4;
    Mat canny1, canny2, canny3, canny4, left1, left2, left3, left4, right1, right2, right3, right4;
    
    vector<Vec4i> lines;
    
    // throwing away the first frame
    
    
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    float x3 = 0, x4 = 0, y3 = 0, y4 = 0;
    float countright = 0, countleft = 0;
    float xp1 = 0, xp2 = 0, yp1 = 0, yp2 = 0;
    float xp3 = 0, xp4 = 0, yp3 = 0, yp4 = 0;
    
    bool stop(false);
    int key = 0;
    int delay = 20;
    int MAX_KERNEL_LENGTH = 7;
    float a1 = 0, a2 = 0, a3 = 0, a4 = 0;
    float b1 = 0, b2 = 0, b3 = 0, b4 = 0;
    
    int x = 0, y = 0;
    // int width = frame.cols, height = frame.rows / 2;
    
    
    
    // resolution: 640 * 480
    //// mono.wmv :
    int interest_y = 168;
    int interest_x = 0;
    int width = frame.cols - interest_x - 00;
    int height = frame.rows - interest_y - 0;
    float leftmin = -0.5;
    float rightmin = 0.6;
    
    int subROIHeight = height / 16;  // Calculate the height of sub_ROIs
    
    //Point of center.중점은 검은색으로 고정시켰음.
     circle(frame, Point(frame.cols/2,frame.rows/4), 5, Scalar(0,0,0), 3, LINE_AA);
    /*
     cout << "col: " << frame.cols << endl;
     cout << "hei: " << frame.rows << endl;
     cout << "roi: " << subROIHeight << endl;
     */
    //height = 312 / 3 = 10
    
    Rect rec(interest_x, interest_y, width, height);  // overall ROI
    
    Rect rec1(interest_x, interest_y, width, subROIHeight * 2); // ROI1
    Point rec1_point(interest_x, interest_y);
    
    Rect rec2(interest_x, interest_y + subROIHeight, width, subROIHeight * 2); // ROI 2
    Point rec2_point(interest_x, interest_y + subROIHeight);
    
    Rect rec3(interest_x, interest_y + subROIHeight * 3, width, subROIHeight * 5); // ROI3
    Point rec3_point(interest_x, interest_y + subROIHeight * 3);
    
    Rect rec4(interest_x, interest_y + subROIHeight * 6, width, subROIHeight * 10);
    Point rec4_point(interest_x, interest_y + subROIHeight * 6);
    
    Rect left_rec(0, 0, width / 2, height);
    
    Rect left_rec1(0, 0, width / 2, subROIHeight * 2);
    Rect left_rec2(0, subROIHeight, width / 2, subROIHeight * 2);
    Rect left_rec3(0, subROIHeight * 3, width / 2, subROIHeight * 5);
    Rect left_rec4(0, subROIHeight * 6, width / 2, subROIHeight * 10);
    
    
    Rect right_rec(width / 2, 0, width / 2, height);
    
    Rect right_rec1(width / 2, 0, width / 2, subROIHeight * 2);
    Rect right_rec2(width / 2, subROIHeight, width / 2, subROIHeight * 2);
    Rect right_rec3(width / 2, subROIHeight * 3, width / 2, subROIHeight * 5);
    Rect right_rec4(width / 2, subROIHeight * 6, width / 2, subROIHeight * 10);
    
    sub = frame(rec);
    GaussianBlur(sub, sub, Size(3, 3), 0, 0);
    sub1 = frame(rec1);
    sub2 = frame(rec2);
    sub3 = frame(rec3);
    sub4 = frame(rec4);
    
    // bgr2gray
    cvtColor(sub, gray, CV_BGR2GRAY);
    
    cvtColor(sub1, gray1, CV_BGR2GRAY);
    cvtColor(sub2, gray2, CV_BGR2GRAY);
    cvtColor(sub3, gray3, CV_BGR2GRAY);
    cvtColor(sub4, gray4, CV_BGR2GRAY);
    
    /*************white detection**************/
    
    inRange(gray, 180, 255, white);
    //ªÛºˆ∞° æ∆¥œ∂Û øµªÛ¿« π‡±‚∏¶ ∫∏∞Ì adaptive«œ∞‘ ∏∏µÈæÓæﬂ «—¥Ÿ.
    //øµªÛ¿« π‡±‚∏¶ ∏Ó«¡∑π¿” ±‚¡ÿ¿∏∑Œ min, «•¡ÿ∆Ì¬˜, ∆Ú±’¿ª ∫∏∞Ì ¡§«ÿæﬂ «—¥Ÿ.
    //¬˜º±¿∫ min∞™ ∫∏¥Ÿ¥¬ ¥ı π‡æ∆æﬂ «—¥Ÿ. æÛ∏∂≥™ ¥ı π‡æ∆æﬂ «œ¥¬∞°? sigma »∞øÎ. ≈Î∞Ë ∞™ »∞øÎ.
    inRange(gray1, 180, 255, white1);
    inRange(gray2, 180, 255, white2);
    inRange(gray3, 180, 255, white3);
    inRange(gray4, 180, 255, white4);
    
    
    /*************edge detection**************/
    Canny(white, canny, 150, 300, 3);
    left = canny(left_rec);
    right = canny(right_rec);
    
    /*************edge  detection  1 **************/
    Canny(white3, canny3, 150, 300, 3);
    imshow("canny3", canny3);
    left3 = canny(left_rec3);
    right3 = canny(right_rec3);
    
    vector<Vec4i> lines_R3;
    vector<Point> pointList_R3;
    
    Point vp_R3;
    float c1_R3, c2_R3;
    
    
    //20, 10, 140
    HoughLinesP(canny3, lines_R3, 1, CV_PI / 180, 20, 10, 140);
    //Merge part
    
    for (size_t i = 0; i < lines_R3.size(); i++) {
        Vec4i l = lines_R3[i];
        
        float slope = ((float)l[3] - (float)l[1]) / ((float)l[2] - (float)l[0]);
        
        if (slope >= 0.3 && slope <= 3) {
            //           line(frame, Point(l[0], l[1] + rec3_point.y), Point(l[2], l[3] + rec3_point.y), LaneColor3, 2);
            countright++;
            x1 += l[0];
            y1 += l[1] + rec3_point.y;
            x2 += l[2];
            y2 += l[3] + rec3_point.y;
        }
        if (slope <= -0.3 && slope >= -3) {
            //       line(frame, Point(l[0], l[1] + rec3_point.y), Point(l[2], l[3]+ rec3_point.y), LaneColor3, 2);
            countleft++;
            x3 += l[0];
            y3 += l[1] + rec3_point.y;
            x4 += l[2];
            y4 += l[3] + rec3_point.y;
        }
    }
    
    if (countright == 0) {
        x1 = xp1;
        x2 = xp2;
        y1 = yp1;
        y2 = yp2;
    }
    //º±¿Ã æ»¿‚»˜∏È ¿Ã¿¸ «¡∑π¿”¿ª æ≤¥¬∞Õ.
    if (countleft == 0) {
        x3 = xp3; //xp¥¬ past x.
        x4 = xp4;
        y3 = yp3;
        y4 = yp4;
    }
    
    float Rslope = (y2 - y1) / (x2 - x1);
    float Lslope = (y4 - y3) / (x4 - x3);
    
    float rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
    float lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
    
    float lastx1 = (0 - rb) / Rslope;
    float lastx2 = (frame.rows - rb) / Rslope;
    
    float lastx3 = ((0 - lb) / Lslope);
    float lastx4 = ((frame.rows - lb) / Lslope);
    
    //º“Ω«¡° ±∏«œ¥¬ ∆ƒ∆Æ
    
    a1 = lastx1 + x, a2 = lastx2 + x, a3 = lastx3 + x, a4 = lastx4 + x;
    b1 = 0, b2 = frame.rows, b3 = 0, b4 = frame.rows;
    
    float dataA[] = { (b2 - b1) / (a2 - a1), -1, (b4 - b3) / (a4 - a3), -1 };
    Mat A3(2, 2, CV_32F, dataA);
    Mat invA3;
    invert(A3, invA3);
    
    float dataB[] = { a1*(b2 - b1) / (a2 - a1) - b1, a3*(b4 - b3) / (a4 - a3) - b3 };
    Mat B3(2, 1, CV_32F, dataB);
    
    Mat X3 = invA3*B3;
    
    //   cout << X << endl;
    //      cout << X.at<float>(0, 0) << endl;
    
    line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor3, 3);
    line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor3, 3);
    //   line(frame, Point(x1 / countright + x, y1 / countright + y), Point(x2 / countright + x, y2 / countright + y), Scalar(0, 255, 0), 2);
    //   line(frame, Point(x3 / countleft + x, y3 / countleft + y), Point(x4 / countleft + x, y4 / countleft + y), Scalar(0, 255, 0), 2);
    circle(frame, Point(X3.at<float>(0, 0), X3.at<float>(1, 0)), 5, LaneColor3, 3, LINE_AA);
    
    /*************edge  detection  2 **************/
    
    
    /*************edge  detection  1 **************/
    Canny(white1, canny1, 150, 300, 3);
    imshow("canny1", canny1);
    left1 = canny(left_rec1);
    right1 = canny(right_rec1);
    
    vector<Vec4i> lines_R1;
    vector<Point> pointList_R1;
    
    Point vp_R1;
    float c1_R1, c2_R1;
    
    
    //20, 10, 140
    HoughLinesP(canny1, lines_R1, 1, CV_PI / 180, 20, 10, 140);
    //Merge part
    
    for (size_t i = 0; i < lines_R1.size(); i++) {
        Vec4i l = lines_R1[i];
        
        float slope = ((float)l[1] - (float)l[1]) / ((float)l[2] - (float)l[0]);
        
        if (slope >= 0.1 && slope <= 1) {
            //           line(frame, Point(l[0], l[1] + rec1_point.y), Point(l[2], l[1] + rec1_point.y), LaneColor1, 2);
            countright++;
            x1 += l[0];
            y1 += l[1] + rec1_point.y;
            x2 += l[2];
            y2 += l[3] + rec1_point.y;
        }
        if (slope <= -0.3 && slope >= -3) {
            //       line(frame, Point(l[0], l[1] + rec3_point.y), Point(l[2], l[3]+ rec3_point.y), LaneColor3, 2);
            countleft++;
            x3 += l[0];
            y3 += l[1] + rec3_point.y;
            x4 += l[2];
            y4 += l[3] + rec3_point.y;
        }
    }
    
    if (countright == 0) {
        x1 = xp1;
        x2 = xp2;
        y1 = yp1;
        y2 = yp2;
    }
    //º±¿Ã æ»¿‚»˜∏È ¿Ã¿¸ «¡∑π¿”¿ª æ≤¥¬∞Õ.
    if (countleft == 0) {
        x3 = xp3; //xp¥¬ past x.
        x4 = xp4;
        y3 = yp3;
        y4 = yp4;
    }
    
    Rslope = (y2 - y1) / (x2 - x1);
    Lslope = (y4 - y3) / (x4 - x3);
    
    rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
    lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
    
    lastx1 = (0 - rb) / Rslope;
    lastx2 = (frame.rows - rb) / Rslope;
    
    lastx3 = ((0 - lb) / Lslope);
    lastx4 = ((frame.rows - lb) / Lslope);
    
    //º“Ω«¡° ±∏«œ¥¬ ∆ƒ∆Æ
    
    a1 = lastx1 + x, a2 = lastx2 + x, a3 = lastx3 + x, a4 = lastx4 + x;
    b1 = 0, b2 = frame.rows, b3 = 0, b4 = frame.rows;
    
    float dataA2[] = { (b2 - b1) / (a2 - a1), -1, (b4 - b3) / (a4 - a3), -1 };
    Mat A2(2, 2, CV_32F, dataA2);
    Mat invA2;
    invert(A2, invA2);
    
    float dataB2[] = { a1*(b2 - b1) / (a2 - a1) - b1, a3*(b4 - b3) / (a4 - a3) - b3 };
    Mat B2(2, 1, CV_32F, dataB2);
    
    Mat X2 = invA3*B3;
    
    //   cout << X << endl;
    //      cout << X.at<float>(0, 0) << endl;
    
    line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor3, 3);
    line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor3, 3);
    //   line(frame, Point(x1 / countright + x, y1 / countright + y), Point(x2 / countright + x, y2 / countright + y), Scalar(0, 255, 0), 2);
    //   line(frame, Point(x3 / countleft + x, y3 / countleft + y), Point(x4 / countleft + x, y4 / countleft + y), Scalar(0, 255, 0), 2);
    circle(frame, Point(X2.at<float>(0, 0), X2.at<float>(1, 0)), 5, LaneColor3, 3, LINE_AA);
    
    /*************edge  detection  4 **************/
    
    
    Canny(white4, canny4, 150, 300, 3);
    imshow("canny4", canny4);
    left4 = canny(left_rec4);
    right4 = canny(right_rec4);
    
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
     if (countright == 0) {
     x1 = xp1;
     x2 = xp2;
     y1 = yp1;
     y2 = yp2;
     }
     //º±¿Ã æ»¿‚»˜∏È ¿Ã¿¸ «¡∑π¿”¿ª æ≤¥¬∞Õ.
     if (countleft == 0) {
     x3 = xp3; //xp¥¬ past x.
     x4 = xp4;
     y3 = yp3;
     y4 = yp4;
     }
     */
    
    Rslope = (y2 - y1) / (x2 - x1);
    Lslope = (y4 - y3) / (x4 - x3);
    
    rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
    lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
    
    lastx1 = (0 - rb) / Rslope;
    lastx2 = (frame.rows - rb) / Rslope;
    
    lastx3 = ((0 - lb) / Lslope);
    lastx4 = ((frame.rows - lb) / Lslope);
    
    //º“Ω«¡° ±∏«œ¥¬ ∆ƒ∆Æ
    
    a1 = lastx1 + x, a2 = lastx2 + x, a3 = lastx3 + x, a4 = lastx4 + x;
    b1 = 0, b2 = frame.rows, b3 = 0, b4 = frame.rows;
    
    float dataA4[] = { (b2 - b1) / (a2 - a1), -1, (b4 - b3) / (a4 - a3), -1 };
    Mat A4(2, 2, CV_32F, dataA4);
    Mat invA4;
    invert(A4, invA4);
    
    float dataB4[] = { a1*(b2 - b1) / (a2 - a1) - b1, a3*(b4 - b3) / (a4 - a3) - b3 };
    Mat B4(2, 1, CV_32F, dataB4);
    
    Mat X4 = invA4*B4;
    
    //   cout << X << endl;
    //      cout << X.at<float>(0, 0) << endl;
    
    line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor4, 3);
    line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor4, 3);
    
    
    circle(frame, Point(X4.at<float>(0, 0), X4.at<float>(1, 0)), 5, LaneColor4, 3, LINE_AA);
   }

int main() {
    char title[100] = "/Users/NAMSOO/Documents/Xcode/OpenCV/SelfDriving/SelfDriving/mono.mp4";
    VideoCapture capture(title);
    Mat frame;
    
    int key, frameNum = 1;
    int frame_rate = 30;
    
    capture.read(frame);
    
    // videoRead
    while (capture.read(frame)) {
        
        lane_detection(frame);
        
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
