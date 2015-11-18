
#ifndef Particle_Filter_Util_h
#define Particle_Filter_Util_h

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "Particle.h"

using namespace std;

cv::Point rotate(cv::Point pt, double angle) {
    
    cv::Point newPt;
    
    newPt.x = cos(angle * M_PI/180.0) * pt.x + (-1)*sin(angle * M_PI/180.0) * pt.y;
    newPt.y = sin(angle * M_PI/180.0) * pt.x + cos(angle * M_PI/180.0) * pt.y;
    
    return newPt;
}


cv::Point translate(cv::Point pt, cv::Point centerPt) {
    
    cv::Point newPt;
    newPt.x = pt.x + centerPt.x;
    newPt.y = pt.y + centerPt.y;
    
    return newPt;
}


cv::Point scale(cv::Point pt, float scale) {
    
    cv::Point newPt;
    
    newPt.x = pt.x * scale;
    newPt.y = pt.y * scale;
    
    return newPt;
}

cv::Point rotateFinal(cv::Point pt, cv::Point centerPt, double angle) {
    
    pt = translate(pt, -centerPt);
    pt = rotate(pt, angle);
    pt = translate(pt, centerPt);
    
    return pt;
}


double getProbability(double u, double s, double x){
    double coefficient = 1.0/sqrt(2.0 * M_PI * s*s);
    return coefficient * exp(-(x-u)*(x-u)/(2.0 * s*s));
}

double distToEdge(cv::Point pt1, cv::Point pt2) {
    double x = pt1.x - pt2.x;
    double y = pt1.y - pt2.y;
    
    return sqrt(pow(x, 2) + pow(y, 2));
}

#endif
