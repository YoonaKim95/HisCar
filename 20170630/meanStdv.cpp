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

Mat ROI(Mat white,Mat frame, Point rec_point,int color,int PrewayToGo);
void getMinMax(Mat roi, int* min, int* max, int color);
static int WayToGo=0;

void lane_detection(Mat frame)
{
    Mat gray, smot, sub, hsv, white, yellow, both, canny, left, right,frame_gray;
    Mat sub1, sub2, sub3, sub4, smot1, smot2, smot3, smot4, gray1, gray2, gray3, gray4, white1, white2, white3, white4,gray_original_sub;
    Mat grayl3,grayl4,grayr3,grayr4; //gray areas.
    Mat canny4,canny3,canny2;
    
    vector<Vec4i> lines;
    //string A[100] = {gray1,sub1,smot1};
    
    // Variable for white detection
    int inRangeMax1=0, inRangeMax2=0, inRangeMaxl3 = 0,inRangeMaxl4 = 0,inRangeMaxr3 = 0,inRangeMaxr4 = 0;
    int inRangeMin1=0, inRangeMin2=0, inRangeMinl3 = 0,inRangeMinl4 = 0,inRangeMinr3 = 0,inRangeMinr4 = 0;
    
   // static int WayToGo=preWayToGo; //0 : straight. 1 : left. 2 : right.

    
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
    Rect rec1(rec1_point, Size(width, subROIHeight));
    
    Point rec2_point(interest_x, interest_y);
    Point rec2_Rpoint(interest_x+halfWidth, interest_y + subROIHeight);
    Rect rec2(rec2_point, Size(width, subROIHeight * 3)); // ROI 2
    
    Point rec3_point(interest_x, interest_y + subROIHeight * 3);
    Point rec3_Rpoint(interest_x+halfWidth, interest_y + subROIHeight * 3);
    Rect rec3(rec3_point, Size(width, subROIHeight * 3)); // ROI3
    
    Point rec4_point(interest_x, interest_y + subROIHeight * 6);
    Point rec4_Rpoint(interest_x+halfWidth, interest_y + subROIHeight * 6);
    Rect rec4(rec4_point, Size(width, subROIHeight * 10));
    
    Rect left_rec1(rec1_point, Size(halfWidth , subROIHeight));
    Rect left_rec2(rec2_point, Size(halfWidth , subROIHeight));
    Rect left_rec3(rec3_point, Size(halfWidth , subROIHeight * 3)); 
    Rect left_rec4(rec4_point, Size(halfWidth , subROIHeight * 10));
    
    Rect right_rec1(rec1_Rpoint, Size(halfWidth , subROIHeight));
    Rect right_rec2(rec2_Rpoint, Size(halfWidth , subROIHeight));
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
    
    
    gray1 = frame_gray(rec1);
    gray2 = frame_gray(rec2);
    grayl3 = frame_gray(left_rec3);
    grayl4 = frame_gray(left_rec4);
    grayr3 = frame_gray(right_rec3);
    grayr4 = frame_gray(right_rec4);
    
    getMinMax(gray1, &inRangeMin1, &inRangeMax1,1);
    getMinMax(gray2, &inRangeMin2, &inRangeMax2,2);
    getMinMax(grayl3, &inRangeMinl3, &inRangeMaxl3,3);
    getMinMax(grayl4, &inRangeMinl4, &inRangeMaxl4,4);
    getMinMax(grayr3, &inRangeMinr3, &inRangeMaxr3,3);
    getMinMax(grayr4, &inRangeMinr4, &inRangeMaxr4,4);
    
    //appy min and max.
    inRange(gray1, inRangeMin1, inRangeMax1, gray1);
    inRange(gray2, inRangeMin2, inRangeMax2, gray2);
    inRange(grayl3, inRangeMinl3, inRangeMaxl3, grayl3);
    inRange(grayl4, inRangeMinl4, inRangeMaxl4, grayl4);
    inRange(grayr3, inRangeMinr3, inRangeMaxr3, grayr3);
    inRange(grayr4, inRangeMinr4, inRangeMaxr4, grayr4);
    
    
    
    white1 = frame_gray(rec1);
    white2 = frame_gray(rec2);
    white3 = frame_gray(rec3);
    white4 = frame_gray(rec4);
    
    Canny(white4, canny4, 150, 300, 3);
     Canny(white3, canny3, 150, 300, 3);
     Canny(white2, canny2, 150, 300, 3);
    
    imshow("canny4",canny4);
    imshow("canny3",canny3);
    imshow("canny2",canny2);
    
    
    //dialte and erode white part to enlarge the part.
    dilate(white2,white2,Mat(),Point(-1,-1),3);
    erode(white2, white2, Mat());
    dilate(white4,white4,Mat(),Point(-1,-1),3);
    erode(white4, white4, Mat());
    dilate(white3,white3,Mat(),Point(-1,-1),3);
    erode(white3, white3, Mat());
    
    imshow("gray1",gray1);
    imshow("gray2",gray2);
    imshow("gray1",gray1);
    imshow("white3",white3);
    
    
   
    /*************edge  detection  4 **************/
    Mat X4 = ROI(gray4,frame,rec4_point,4,WayToGo); //default value of PrewayToGo is straight.
    
    
    //staright way.
    if( abs((X4.at<float>(0, 0)-frame.cols/2))<=30)
    {
        WayToGo=0;
    }
    
    //left way
    else if(((X4.at<float>(0, 0)-frame.cols/2)<(-30)))
    {
        WayToGo=1;
        //putText(frame, "Turn left" ,Point(100,400) , FONT_HERSHEY_PLAIN, 2, Scalar(LaneColor4), 2, LINE_8);
    }
    
    //right way
    else if(((X4.at<float>(0, 0)-frame.cols/2)>(30)))
    {
        WayToGo=2;
    }
    
    if(WayToGo==0)
        putText(frame, "Staright" ,Point(100,400) , FONT_HERSHEY_PLAIN, 2, Scalar(LaneColor4), 2, LINE_8);
    else if(WayToGo==1)
        putText(frame, "Turn Left" ,Point(100,400) , FONT_HERSHEY_PLAIN, 2, Scalar(LaneColor4), 2, LINE_8);
    else if(WayToGo==2)
        putText(frame, "Turn Right" ,Point(100,400) , FONT_HERSHEY_PLAIN, 2, Scalar(LaneColor4), 2, LINE_8);


    /*************edge  detection  3 **************/
    Mat X3 = ROI(gray3,frame,rec3_point,3,WayToGo);
    /*************edge  detection  2 **************/
    Mat X2 = ROI(gray2,frame,rec2_point,2,WayToGo);
    /*************edge  detection  1 **************/
    //Mat X1 = ROI(white1,frame,rec1_point);
   
    
        //inner angle.
    
    imshow("frame_gray",frame_gray);
    imshow("gray2",gray2);
    imshow("gray3",gray3);
    imshow("gray4",gray4);
    
}

Mat ROI(Mat white,Mat frame, Point rec_point,int color,int WayToGo)
{
    
    Mat canny;
    
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0; //declaration of x,y variables used in lines.
    float x3 = 0, x4 = 0, y3 = 0, y4 = 0;
    
    //initiate for the first time when there are no input values.
    float countright = 0, countleft = 0;
    float xp1 = 0, xp2 = 0, yp1 = 0, yp2 = 0;
    float xp3 = 0, xp4 = 0, yp3 = 0, yp4 = 0;
    
    float a1 = 0, a2 = 0, a3 = 0, a4 = 0;
    float b1 = 0, b2 = 0, b3 = 0, b4 = 0;
    
    int x = 0, y = 0;
    
    //convert white Matrix to canny Matrix.
    Canny(white, canny, 150, 300, 3);
   // imshow("canny", canny);

    vector<Vec4i> lines;
    vector<Point> pointList;
    
    
    //20, 10, 140
    HoughLinesP(canny, lines, 1, CV_PI / 180, 20, 10, 140);
    //Merge part
    
    float selected_slopeR =0;
    float selected_slopeL =0;
    
    for (size_t i = 0; i < lines.size(); i++) {
        
        Vec4i l = lines[i];
        
        //get slope.
        float slope = (((float)l[3] - (float)l[1]) / ((float)l[2] - (float)l[0]));
      
       // float slope =  cvFastArctan(((float)l[3] - (float)l[1]), ((float)l[2] - (float)l[0]));
       
        
//                if (slope >= 0.3 && slope <= 3) {
//                    countright++;
//                    x1 = l[0];
//                    y1 = l[1] + rec_point.y;
//                    x2 = l[2];
//                    y2 = l[3] + rec_point.y;
//        //            cout << "R" << slope << endl;
//                }
//                //lines of right side
//                if (slope <= -0.3 && slope >= -3) {
//                    countleft++;
//                    x3 = l[0];
//                    y3 = l[1] + rec_point.y;
//                    x4 = l[2];
//                    y4 = l[3] + rec_point.y;
//        //            cout << "L" << slope << endl;
//                }

        
        //straight way
        if(WayToGo==0)
        {
            //lines of right side
            if (slope >= 0.3 && slope <= 4) {
                countright++;
                x1 += l[0];
                y1 += l[1] + rec_point.y;
                x2 += l[2];
                y2 += l[3] + rec_point.y;
               cout << "R" << slope << endl;
                selected_slopeR=slope;
            }
            //lines of left side
           if (slope <= -0.3  && slope >= -3){
            
                countleft++;
                x3 += l[0];
                y3 += l[1] + rec_point.y;
                x4 += l[2];
                y4 += l[3] + rec_point.y;
               cout << "L" << slope << endl;
               selected_slopeL=slope;

           }
        }
        
        //left turn situation
         else if(WayToGo==1)
         {
              //lines of right side
             if (slope >= 0.3 && slope <= 3 ) {
                 countright++;
                 x1 += l[0];
                 y1 += l[1] + rec_point.y;
                 x2 += l[2];
                 y2 += l[3] + rec_point.y;
                 selected_slopeR=slope;

             }
             //lines of left side
             if (slope <= -0.5 && slope >= -10) {
                 countleft++;
                 x3 += l[0];
                 y3 += l[1] + rec_point.y;
                 x4 += l[2];
                 y4 += l[3] + rec_point.y;
                 selected_slopeL=slope;

             }

         }
        
        //right turn situation.
         else if(WayToGo==2)
         {
             if(color==2)
             {
                 //lines of right side
                 if (slope >-0.3 && slope <= 0) {
                     countright++;
                     x1 += l[0];
                     y1 += l[1] + rec_point.y;
                     x2 += l[2];
                     y2 += l[3] + rec_point.y;
                     selected_slopeR=slope;
                     
                 }
                 //lines of left side
                 if (slope <= -0.3  && slope >= -3) {
                     countleft++;
                     x3 += l[0];
                     y3 += l[1] + rec_point.y;
                     x4 += l[2];
                     y4 += l[3] + rec_point.y;
                     selected_slopeL=slope;
                     
                 }
                 

             }
             
             else
             {
              //lines of right side
             if (slope >0.5 && slope <= 10) {
                 countright++;
                 x1 += l[0];
                 y1 += l[1] + rec_point.y;
                 x2 += l[2];
                 y2 += l[3] + rec_point.y;
                 selected_slopeR=slope;

             }
             //lines of left side
             if (slope <= -0.3  && slope >= -3) {
                 countleft++;
                 x3 += l[0];
                 y3 += l[1] + rec_point.y;
                 x4 += l[2];
                 y4 += l[3] + rec_point.y;
                 selected_slopeL=slope;

             }
             }
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
    
    float rb = (y1 / countright + y) - Rslope * (x1 / countright + x);
    float lb = (y3 / countleft + y) - Lslope * (x3 / countleft + x);
    
    
    float lastx1 = (0 - rb) / Rslope;
    float lastx2 = (frame.rows - rb) / Rslope;
    
    float lastx3 = ((0 - lb) / Lslope);
    float lastx4 = ((frame.rows - lb) / Lslope);
    
    
    //point of line will be drawn. extend to the end of the frame.
//    a1 = lastx1 + x, a2 = lastx2 + x, a3 = lastx3 + x, a4 = lastx4 + x;
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
    
    if(color==2)
    {
        line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor2, 3);
        line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor2, 3);
        circle(frame, Point(X.at<float>(0, 0), X.at<float>(1, 0)), 5, LaneColor2, 3, LINE_AA);
        cout << "slope 2R" <<  selected_slopeR << endl;
        cout << "slope 2L" <<  selected_slopeL << endl;
    }


    if(color==3)
    {
    line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor3, 3);
    line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor3, 3);
    circle(frame, Point(X.at<float>(0, 0), X.at<float>(1, 0)), 5, LaneColor3, 3, LINE_AA);
        cout << "slope 3R" <<  selected_slopeR << endl;
        cout << "slope 3L" <<  selected_slopeL << endl;
    }
    if(color==4)
    {
    line(frame, Point(a1, 0), Point(a2, frame.rows), LaneColor4, 3);
    line(frame, Point(a3, 0), Point(a4, frame.rows), LaneColor4, 3);
    circle(frame, Point(X.at<float>(0, 0), X.at<float>(1, 0)), 5, LaneColor4, 3, LINE_AA);
        cout << "slope 4R" <<   selected_slopeR << endl;
        cout << "slope 4L" <<  selected_slopeL << endl;

    }
    
    if(color==4)
    {
    float innerAngleL=0;
    float innerAngleR=0;
    float innerA=0;
  //  innerAngleL = abs((X.at<float>(0, 0)-x1)/(X.at<float>(1, 0)-y1));
        
        // innerAngleR = abs((X.at<float>(0, 0)-X)/(X.at<float>(1, 0)-y3));
        innerAngleR = abs(tan(((X.at<float>(0, 0))-a1)/((X.at<float>(1, 0))))) ;
        innerAngleL = abs(tan(((X.at<float>(0, 0))-a3)/((X.at<float>(1, 0))))) ;
       innerA = innerAngleR +  innerAngleL;
       innerAngleR = (innerAngleR*180.0/M_PI) ;
        innerAngleL = (innerAngleL*180.0/M_PI);

    cout << "ROI4 R : " << innerAngleR<< "  ROI4 L : " << innerAngleL << endl;
    }
    
    return X ;
    
    

}

void getMinMax(Mat roi, int* min, int* max,int color)
{
    // showing the brightest point
    float meanVal = 0, stdDevVal = 0;
    Point maxPoint;
    double maxPixelVal;
    Scalar mean;
    Scalar stdDev;
    
    // find min and max value and point
    minMaxLoc(roi, 0,&maxPixelVal,0,&maxPoint);
    
    // Calculate mean and stdDev of ROI
    meanStdDev(roi, mean, stdDev);
    meanVal = mean.val[0];
    stdDevVal = stdDev.val[0];
    
    // calculate min value of inRange
    if(color==2||color==1)
    *min = meanVal + 1 * stdDevVal;
    else
    *min = meanVal + 2 * stdDevVal;
    
    
    // max value = mean + stdDev
    *max = meanVal + 5   * stdDevVal; // maxPixelVal;
}


// Namsoo's storage Users/NAMSOO/Documents/Xcode/OpenCV/VanishingPoint/VanishingPoint/

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
