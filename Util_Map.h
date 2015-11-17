#ifndef Util_Map_h
#define Util_Map_h

#include <iostream>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "Util.h"
#include "Particle.h"

using namespace std;
using namespace cv;

void parseFile(
               std::vector<Point> &edges,
               std::vector<Point> &virus,
               std::vector<Point> &labA,
               Point &labB
               ) {
    std::ifstream myfile;
    //myfile.open ("map.txt");
    //myfile.open ("map_origin.txt");
    myfile.open ("map_origin_backup.txt");

    
    if (myfile.is_open()){
        while (!myfile.eof()){
            Point p1;
            Point p2;
            int type;// 0 is wall,1 is virus, 2 is LabA, 3 is LabB
            if (myfile >> p1.x >> p1.y >> p2.x >> p2.y >> type ) {
                p1.x = p1.x*30;
                p1.y = p1.y*30;
                p2.x = p2.x*30;
                p2.y = p2.y*30;
                if(type == 0) {
                    edges.push_back(p1);
                    edges.push_back(p2);
                }
                else if(type == 1){
                    //Do Virus
                    virus.push_back(p1);
                    virus.push_back(p2);
                }
                else if(type ==2){
                    // LabA format is X Y W H
                    // p1 is center of docking station
                    // p2.i is width and p2.j is height
                    labA.push_back(p1);
                    labA.push_back(p2);
                    
                }else if(type == 3){
                    labB.x = p1.x;
                    labB.y = p1.y;
                    
                }
            }
        }
    }
    myfile.close();
}



void drawMap(Mat& img,
                  std::vector<Point> &edges,
                  std::vector<Point> &virus,
                  std::vector<Point> &labA,
                  Point &labB
                  ) {
    parseFile(edges,virus,labA,labB);
    
    int thickness = 6; //Original 6
    int lineType = 4; //Original 8

    //Draw Wall
    for(int i = 0;i < edges.size()-1; i+=2){

        Point p1 = edges[i];
        Point p2 = edges[i+1];
        line (img, p1, p2, Scalar(255, 0, 0), thickness, lineType);
    }

    //Draw Virus Location
    for(int i = 0;i < virus.size()-1; i+=2) {
        Point p1 = virus[i];
        Point p2 = virus[i+1];
        
        Point topLeft1, topRight1, bottomLeft1, bottomRight1;
        topLeft1.x = int (p1.x - p2.x/2);
        topLeft1.y = int (p1.y - p2.y/2);
        topRight1.x = int (p1.x + p2.x/2);
        topRight1.y = int (p1.y - p2.y/2);
        bottomLeft1.x = int (p1.x - p2.x/2);
        bottomLeft1.y = int (p1.y + p2.y/2);
        bottomRight1.x = int (p1.x + p2.x/2);
        bottomRight1.y = int (p1.y + p2.y/2);
        
        line(img,topLeft1, bottomRight1, CV_RGB(0, 150, 150));
        line(img,topRight1, bottomLeft1, CV_RGB(0, 150, 150));
    }
    
    //Draw LabA Location
    for(int i = 0;i < labA.size()-1; i+=2){
        Point p1 = labA[i];
        Point p2 = labA[i+1];
        
        Point topLeft, topRight, bottomLeft, bottomRight;
        topLeft.x = int (p1.x - p2.x/2);
        topLeft.y = int (p1.y - p2.y/2);
        topRight.x = int (p1.x + p2.x/2);
        topRight.y = int (p1.y - p2.y/2);
        bottomLeft.x = int (p1.x - p2.x/2);
        bottomLeft.y = int (p1.y + p2.y/2);
        bottomRight.x = int (p1.x + p2.x/2);
        bottomRight.y = int (p1.y + p2.y/2);
        rectangle(img, topLeft, bottomRight, cvScalar(204, 0, 204, 0),1,8,0);
        
        line(img,topLeft, bottomRight, CV_RGB(204,0,204));
        line(img,topRight, bottomLeft, CV_RGB(204,0,204));
    }
    //Draw LabB Location
    circle(img, labB, 10, CV_RGB(255,0,0), 5, CV_AA, 0);
    
}






#endif

