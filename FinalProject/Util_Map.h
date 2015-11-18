#ifndef Particle_Filter_Util_Map_h
#define Particle_Filter_Util_Map_h

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
               std::vector<Point> &rocket,
               std::vector<Point> &dock,
               Point &arcReactorLoc
               ) {
    std::ifstream myfile;
    //myfile.open ("map_origin20151003.txt");
    myfile.open ("map.txt");
    if (myfile.is_open()){
        while (!myfile.eof()){
            Point p1;
            Point p2;
            int type;// 0 is wall,1 is rocket, 2 is docking
            if (myfile >> p1.x >> p1.y >> p2.x >> p2.y >> type ) {
                p1.x = p1.x*25;//convert ft to cm
                p1.y = p1.y*25;//convert ft to cm
                p2.x = p2.x*25;//convert ft to cm
                p2.y = p2.y*25;//convert ft to cm
                if(type == 0) {
                    edges.push_back(p1);
                    edges.push_back(p2);
                }
                else if(type == 1){
                    //Do door
                    rocket.push_back(p1);
                    rocket.push_back(p2);
                }
                else if(type ==2){
                    //Docking station format is X Y W H
                    // p1 is center of docking station
                    // p2.i is width and p2.j is height
                    dock.push_back(p1);
                    dock.push_back(p2);
                    
                }else if(type == 3){
                    arcReactorLoc.x = p1.x;
                    arcReactorLoc.y = p1.y;
                    
                }
            }
        }
    }
    myfile.close();
}

void drawMap(Mat& img,
                  std::vector<Point> &edges,
                  std::vector<Point> &rocket,
                  std::vector<Point> &docks,
                  Point &arcReactorLoc
                  ) {
    parseFile(edges,rocket,docks,arcReactorLoc);
    
    int thickness = 6;
    int lineType = 8;

    //Draw Wall
    for(int i = 0;i < edges.size()-1; i+=2){

        Point p1 = edges[i];
        Point p2 = edges[i+1];
        line (img, p1, p2, Scalar(255, 0, 0), thickness, lineType);
    }
    
}

#endif
