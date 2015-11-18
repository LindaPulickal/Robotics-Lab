#ifndef Particle_Filter_Particle_h
#define Particle_Filter_Particle_h

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "Util.h"
#include <math.h>

using namespace std;

double randomDouble()
{
    //cout << "RAND_MAX = " << double(RAND_MAX) << endl;
    return double(rand()) / (double(RAND_MAX) + 1.0);
}

double randomDoubleFromNormal(const double s)
{
    double sum = 0;
    for(int i=0; i<12; i++){
        sum += randomDouble()*2*s - s;
    }
    return sum/2;
}

class Particle {
    
private:
    cv::Point particlePt;
		double angle;
    double weight;
    
public:
    Particle(cv::Point start = cv::Point(0,0), double direction = 0.0, double w = 1.0) {
        particlePt = start;
				angle = direction;
        weight = w;
    }
    ~Particle(){}
    
    void moveParticle(double direction, double distance, double variance) {
        angle = direction + 3*randomDoubleFromNormal(variance); //I use scale 3 to make the angle error bigger
        double realDistance = distance + randomDoubleFromNormal(variance);
        
        particlePt.x = round(double(particlePt.x) + realDistance* cos(angle * M_PI/180.0));
        particlePt.y = round(double(particlePt.y) - realDistance* sin(angle * M_PI/180.0));
    }
 
    void rotateParticle(double direction, double variance) {
        angle = direction + 3*randomDoubleFromNormal(variance); //I use scale 3 to make the angle error bigger
    }   
   
    cv::Point getPosition(){return particlePt;}
    // Get Particle's probability
		double getAngle(){return angle;}
		void setAngle(double a){angle = a;}
    double getWeight(){return weight;}
    // Modifies the Particle's probability
    void setWeight(double w){weight = w;}
    
    
};


#endif
