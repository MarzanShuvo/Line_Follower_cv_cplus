#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include<cmath>
#include<tuple>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

int getMaxAreaContourId(std::vector < std::vector<cv::Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
} // End function

cv::Mat getUpperHalfMask(cv::Mat image){
    int width = image.size().width;
    int height = image.size().height/2;
    for(int i=0; i<width; i++){
        for(int j=0; j<height; j++){
            image.at<uchar>(j,i) = 255;
        }
    }
    return image;
}


cv::Mat getLowerHalfMask(cv::Mat image){
    int width = image.size().width;
    int height = image.size().height;
    for(int i=0; i<width; i++){
        for(int j=height/2; j<height; j++){
            image.at<uchar>(j,i) = 255;
        }
    }
    return image;
}

std::tuple <int, int> diff_drive(double v, double w){
    double l=0.2;
    double D = 0.06;
    double vr = double(2*v+w*l)/D;
    double vl = double(2*v-w*l)/D;
    double wheel_rotation_l = double (vl*2)/D;
    double wheel_rotation_r = double (vr*2)/D;
    int rpm_l = int(wheel_rotation_l*60/(2*3.1416));
    int rpm_r = int(wheel_rotation_r*60/(2*3.1416));
    return std::make_tuple(rpm_l, rpm_r);


}

std::tuple <double, double> move_robot(double error_w1=0.0, double error_w2=0.0, double error_y1=0.0){
    double linear = 0.0;
    double angular = 0.0;
    double angular_vel_base = 0.2;
    double linear_vel_base = 0.3;
    double FACTOR_LINEAR_e1 = 0.01;
    double FACTOR_ANGULAR_e1 = 0.01;
    double FACTOR_ANGULAR_e2 = 0.02;
    angular = angular_vel_base * error_w1* FACTOR_ANGULAR_e1 + 0.01 * error_w2* FACTOR_ANGULAR_e2;
    //linear = linear_vel_base + error_y1*FACTOR_LINEAR_e1;
    linear = .01;
    return std::make_tuple(linear, angular);
}


int main(int argc, char **argv) {

    cv::VideoCapture video_capture;
    if (!video_capture.open(0)) {
        return 0;
    }

    cv::Mat frame;
    while (true) {
        video_capture >> frame;
        cv::Mat grayMat, mask, bitwise, upper_bit, lower_bit, upper_p, lower_p;
        cv::Mat gaus;
        int width, height;
        double theta1, theta2, theta3;
        double error_w1, error_w2,error_y1;
        int cenx1, ceny1, cenx2, ceny2, ori_cx, ori_cy;
        ori_cx = frame.cols/2;
        ori_cy = frame.rows/2;
        cv::cvtColor(frame, grayMat, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(grayMat,gaus, cv::Size(9, 9), 0);
        //cv::adaptiveThreshold(grayMat, binary, 200, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 3, 2);
        cv::threshold(gaus, mask, 150, 255, cv::THRESH_BINARY);
        cv::bitwise_not(mask, bitwise);
        //cv::bitwise_not(bitwise, bitwise);
        //cv::bitwise_not(bitwise, bitwise);
        cv::Mat image = cv::Mat::zeros(cv::Size(bitwise.cols, bitwise.rows),CV_8UC1);
        auto upper_mask = getUpperHalfMask(image);
        cv::Mat image1 = cv::Mat::zeros(cv::Size(bitwise.cols, bitwise.rows),CV_8UC1);
        auto lower_mask = getLowerHalfMask(image1);
        cv::bitwise_and(image, upper_mask, upper_bit);
        cv::bitwise_and(image1, lower_mask, lower_bit);
        cv::bitwise_and(bitwise, upper_bit, upper_p);
        cv::bitwise_and(bitwise, lower_bit, lower_p);

        
        //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
        std::vector<std::vector<cv::Point> > contours0;
        cv::Mat contourOutput1 = upper_p.clone();
        cv::findContours( contourOutput1, contours0, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        if(contours0.size()>0){
            auto contour1 = contours0.at(getMaxAreaContourId(contours0));

            if(contour1.size() != 0){
                auto moments = cv::moments(contour1,true);
                auto cx = int(moments.m10/moments.m00);
                auto cy = int(moments.m01/moments.m00);
                cenx1 = cx;
                ceny1 = cy;
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour1}, -1, cv::Scalar(0, 255, 0), 2);
                cv::Point centerCircle(cx,cy);
                int radiusCircle = 10;
                cv::Scalar colorCircle1(0,255,255);
                int thicknessCircle = 4;
                cv::circle(frame, centerCircle, radiusCircle, colorCircle1, thicknessCircle);
            }
        }
        else{
            continue;
        }

        std::vector<std::vector<cv::Point> > contours2;
        cv::Mat contourOutput2 = lower_p.clone();
        cv::findContours( contourOutput2, contours2, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        if(contours2.size()>0){
            auto contour2 = contours2.at(getMaxAreaContourId(contours2));

            if(contour2.size() != 0){
                auto moments2 = cv::moments(contour2,true);
                auto cx2 = int(moments2.m10/moments2.m00);
                auto cy2 = int(moments2.m01/moments2.m00);
                cenx2 = cx2;
                ceny2 = cy2;
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour2}, -1, cv::Scalar(0, 255, 0), 2);
                cv::Point centerCircle(cx2,cy2);
                int radiusCircle = 10;
                cv::Scalar colorCircle1(0,255,255);
                int thicknessCircle = 4;
                cv::circle(frame, centerCircle, radiusCircle, colorCircle1, thicknessCircle);
            }
        }
        else{
            continue;
        }
        cv::Point p2(cenx1,ceny1), p1(cenx2, ceny2);
        cv::line(frame, p1,p2, cv::Scalar(255,0,0), 3, cv::LINE_8);
        if(p1.x != p2.x){
            double y1 = (double) (p2.y - p1.y); 
            double x1 = (double) (p2.x - p1.x);
            theta2 = -1*atan2(y1,x1)*180/3.1415;
        }

        cv::Point p3(ori_cx,frame.rows);
        cv::line(frame, p3,p1, cv::Scalar(255,0,0), 3, cv::LINE_8);
        if(p3.x != p1.x){
            double y2 = (double) (p1.y - p3.y);
            double x2 = (double) (p1.x - p3.x);
            double error_y1 = y2;
            theta1 = -1*atan2(y2,x2)*180/3.1415;
        }
        cv::line(frame, p2,p3, cv::Scalar(0,0,255), 3, cv::LINE_8);
        cv::Point centerCircle3(ori_cx, frame.rows);
        int radiusCircle = 10;
        cv::Scalar colorCircle3(0,255,255);
        int thicknessCircle = 4;
        cv::circle(frame, centerCircle3, radiusCircle, colorCircle3, thicknessCircle);
        
        
        imshow("Frame", frame);
        imshow("mask", mask);
        //imshow("Lower", lower_p);

        /***if(p3.x != p2.x){
            double x3 = (double) (p2.y-p3.y);
            double y3 = (double) (p2.x-p3.x);
            theta3 = -1*atan2(y3,x3)*180/3.1415;
        }
        //std::cout<< "p2: cenx1 "<<cenx1<<"ceny1 "<< ceny1<<"p1: cenx2"<<cenx2<<" ceny2"<< "ori_cx "<<ori_cx<<" frame.rows"<<frame.rows<<std::endl;***/
        error_w1 = theta1-90;
        error_w2 = theta2-90;
        double linear, angular;
        int rpm_l, rpm_r;
        std::tie(linear, angular) = move_robot(error_w1, error_w2, error_y1);
        std::tie(rpm_l, rpm_r) = diff_drive(linear, angular);
        // std::cout<<"error_w1: "<<error_w1<<" error_w2: "<<error_w2<<" error_y1: "<<error_y1<<std::endl;
        std::cout<<"rpm_l: "<<rpm_l<<" rpm_r:"<<rpm_r<<std::endl;
        std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
        std::cout<<"            "<<std::endl;
        std::string rpm_l_str = std::to_string(rpm_l);
        std::string rpm_r_str = std::to_string(rpm_r);
        std::string data_to_send = rpm_l_str+","+rpm_r_str+"\n";
        char* char_arr;
        char_arr = &data_to_send[0];

        int fd;
        fd = serialOpen("/dev/ttyACM0",9600);
        if ((fd < 0))
            {
                std::cout<<"Connection is not availabale"<<std::endl;
                fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
                return 1 ;
            }

        serialPuts(fd, char_arr);
        std::cout<<char_arr<<std::endl;
        std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
        std::cout<<"            "<<std::endl;


        
        int esc_key = 27;
        if (cv::waitKey(10) == esc_key) {
            break;
        }
    }

    cv::destroyAllWindows();
    video_capture.release();

    return 0;
}