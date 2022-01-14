#ifndef ESTIMPOSEPOINTSOMNI_POINTTRACKING_H
#define ESTIMPOSEPOINTSOMNI_POINTTRACKING_H

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpColor.h>
#include <visp/vpList.h>

#include <fstream>
#include<iostream>
#include <cmath>
#include <algorithm>
#include <map>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/tracking.hpp>

#include "CPoint.h"
#include "CModel.h"

using namespace std;
using namespace cv;

struct tracker2point{
    Ptr<Tracker> tracker;
    int wo_id;
    CPoint p;
};

class PointTracking
{
public:
    vector <tracker2point> tp_tab;
    bool firstframe = true;
    int firstframelink[5] = {5,3,2,1,0};
public:
    PointTracking();
    ~PointTracking();

    //! return the area of the intersection between 2 rectangles
    int intersection_area(Rect r1, Rect r2);

    //! fills tab_ind with link between lightFound and lightTracked. index = lightFound index | value = worldpoint index
    void link(vector<Rect> lightFound, vector<Rect> lightTracked, int tab_ind[]);

    //! return the worldpoint index of the closest worldpoint to 2D (meter) point.
    //! note that op 2D values are updated with previous cMo
    int getClosestWorldPoint(CPoint newSeenPoint, CPoint op[]);

    //! calculate distance between world point->tracked and world point->new point
    //! true if the tracked one is the closest false otherwise
    bool keepTracked(int wo_id, int tp_ind, CPoint p2, CPoint op[]);

    //! Checks if the worldpoint is already tracked (in tp_tab list)
    //! if so check which one is closest
    //! keep only the closest one
    //! mainly a debug fonctionnality
    bool isInTP(int wo_id, CPoint p, CPoint op[]);



    //! image added display & analysis (locating the lamps from mask img)
    //! manages tp_tab for current lamp detection
    void getPoints(Mat& img, Mat& dil,CModel *cam,  CPoint op[]);

};
#endif //ESTIMPOSEPOINTSOMNI_POINTTRACKING_H














