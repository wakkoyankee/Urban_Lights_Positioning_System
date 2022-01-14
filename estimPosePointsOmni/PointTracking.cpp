//
// Created by hadrien on 10/06/2021.
//

#include "PointTracking.h"

PointTracking::PointTracking()
{

}

PointTracking::~PointTracking()
{

}


int PointTracking::intersection_area(Rect r1, Rect r2){
    int distx =  (std::min(r1.br().x,r2.br().x) - std::max(r1.tl().x,r2.tl().x));
    int disty = (std::min(r1.br().y,r2.br().y) - std::max(r1.tl().y,r2.tl().y));;
    if(distx<0 || disty<0){
        return 0;
    }
    return  distx*disty;
}

void PointTracking::link(vector<Rect> lightFound, vector<Rect> lightTracked, int tab_ind[]){
    int tab_area[lightFound.size()];
    std::fill_n(tab_ind, lightFound.size(), -1);
    std::fill_n(tab_area, lightFound.size(), -1);

    for(int i = 0; i< lightFound.size();i++){
        for(int j = 0; j<lightTracked.size(); j++){
            int int_area = intersection_area(lightFound[i],lightTracked[j]);
            if(tab_area[i] == -1 && int_area >0){//if not linked yet and intersects
                tab_area[i] = int_area;
                tab_ind[i] = j;
            }else if (tab_area[i] != -1 && int_area>tab_area[i]){ //if linked but intersection is bigger
                tab_area[i]= int_area;
                tab_ind[i] = j;
            }
        }
    }
}

int PointTracking::getClosestWorldPoint(CPoint newSeenPoint, CPoint op[]){//newseenPoint has pixel values
    int size = 11;
    float min = 100000000.0;
    int ind = -1;
    for(int i = 0; i< size;i++){
        float dist = sqrt(pow((newSeenPoint.get_u() - op[i].get_u()),2) + pow((newSeenPoint.get_v() - op[i].get_v()),2));
        if(dist < min){
            ind = i;
            min = dist;
        }
    }
    return ind;

}
bool PointTracking::keepTracked(int wo_id, int tp_ind, CPoint p2, CPoint op[]){
    float dist1 = sqrt(pow(op[wo_id].get_u() - tp_tab[tp_ind].p.get_u(),2) + pow(op[wo_id].get_v() - tp_tab[tp_ind].p.get_v(),2));
    float dist2 = sqrt(pow(op[wo_id].get_u() - p2.get_u(),2) + pow(op[wo_id].get_v() - p2.get_v(),2));
    if(dist1 < dist2){//keep tracked point
        return true;
    }else{//replace with the new point
        return false;
    }
}

bool PointTracking::isInTP(int wo_id, CPoint p, CPoint op[]){
    for(int i = 0; i<tp_tab.size(); i++){
        if(tp_tab[i].wo_id == wo_id){
            bool keep = keepTracked(wo_id, i, p, op);
            if(!keep){
                tp_tab.erase(tp_tab.begin() + i);
                return true;
            }
            else{
                return false;
            }
        }
    }
    return false;
}


void PointTracking::getPoints(Mat& img, Mat& dil,CModel *cam,  CPoint op[]) {
    vector <vector<Point>> contours;
    vector <Vec4i> hierarchy;

    findContours(dil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for(int i = 0; i < contours.size(); i++){ //removes too small contours
        if(contourArea(contours[i]) < 120){
            drawContours(img,  vector<vector<Point> >(1,contours[i]), -1, Scalar(255, 120, 120), 3);
            contours.erase(contours.begin()+i);
        }
    }
    cout<<"ou"<<endl;
    vector <Rect> boundRect(contours.size());
    vector <vector<Point>> contours_poly(contours.size());

    for (int i = 0; i < contours.size(); i++) { //create boundindRect && first frame

        approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = boundingRect(contours_poly[i]);
        rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 255), 2);


        if(firstframe){
            Ptr<Tracker> tracker = TrackerCSRT::create();
            tracker->init(dil,boundRect[i]);

            tracker2point tp;//INIT tp_tab
            tp.tracker = tracker;
            tp.wo_id = firstframelink[i];
            Moments mo = moments(contours[i], true);
            CPoint newP;
            newP.set_u(mo.m10 / mo.m00);
            newP.set_v(mo.m01 / mo.m00);
            cam->pixelMeterConversion(newP);
            newP.setWorldCoordinates(op[tp.wo_id].getWorldCoordinates());//link 3D point
            tp.p = newP;
            tp_tab.push_back(tp);

            //tab_tracker.push_back(tracker);
            rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 255), 2);
            putText(img, to_string(tp.wo_id+1), cv::Point(boundRect[i].x,boundRect[i].y),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);

        }
    }


    vector<Rect> lightTracked;
    if(!firstframe){
        for(int i = 0; i < tp_tab.size(); i++){//lightTacked ind linked to tp_tab ind
            Rect tmp_rect;
            bool inFrame = tp_tab[i].tracker->update(dil,tmp_rect);

            if(inFrame){
                lightTracked.push_back(tmp_rect);
            }
            else {
                tp_tab.erase(tp_tab.begin()+i);
            }
        }
        //if tracker areas intersects destroy one of them
        //lightTacked ind linked to tp_tab ind
        vector<int> to_destroy;
        for(int i = 0; i < tp_tab.size()-1; i++){
            for(int j = i+1; j < tp_tab.size(); j++){
                if(intersection_area(lightTracked[i], lightTracked[j]) > 0){
                    to_destroy.push_back(i);
                }
            }
        }
        for(int i = 0; i < to_destroy.size(); i++){
            tp_tab.erase(tp_tab.begin()+to_destroy[i]);
            lightTracked.erase(lightTracked.begin() +to_destroy[i]);
        }

        int tab_ind[boundRect.size()];
        link(boundRect, lightTracked, tab_ind); //ind boundRect -> ind lightTracked

        vector<bool> isUsed(tp_tab.size(), false);
        for(int i = 0; i < boundRect.size(); i++){//OLD LAMPS
            if(tab_ind[i] != -1){//already linked to world point
                Moments m = moments(contours[i], true);
                isUsed[tab_ind[i]] = true;//utilisé
                tp_tab[tab_ind[i]].p.set_u(m.m10 / m.m00);//update 2d coord
                tp_tab[tab_ind[i]].p.set_v(m.m01 / m.m00);
                cam->pixelMeterConversion(tp_tab[tab_ind[i]].p);
                rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 255, 255), 2);
            }
        }

        //delete old lamps that arent used
        for(int i=0; i< tp_tab.size();i++){
            if(!isUsed[i]){
                tp_tab.erase(tp_tab.begin()+i);
            }
        }


        map<int,int> mp;

        for(int i = 0; i < boundRect.size(); i++){//NEW LAMPES
            if(tab_ind[i] == -1){//new lamp
                tracker2point tmp_tp;
                Ptr<Tracker> tracker = TrackerCSRT::create();
                tracker->init(dil,boundRect[i]);
                tmp_tp.tracker = tracker;

                CPoint newP;
                Moments mo = moments(contours[i], true);
                newP.set_u(mo.m10 / mo.m00);
                newP.set_v(mo.m01 / mo.m00);
                cam->pixelMeterConversion(newP);
                tmp_tp.wo_id = getClosestWorldPoint(newP,op);

                cout<<tmp_tp.wo_id<<endl;
                if(isInTP(tmp_tp.wo_id, newP, op)){//already tracked? keep only the closest one
                    continue;
                }

                auto it = mp.find(tmp_tp.wo_id);
                if( it != mp.end()){//already added ? then delete both
                    tp_tab.erase(tp_tab.begin()+it->second);
                    continue;
                }
                newP.setWorldCoordinates(op[tmp_tp.wo_id].getWorldCoordinates());

                tmp_tp.p = newP;

                tp_tab.push_back(tmp_tp);
                mp.insert({tmp_tp.wo_id ,tp_tab.size()-1});
                rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 255, 255), 2);
            }
        }


        for(int i = 0 ; i< tp_tab.size();i++){//display lamps number
            putText(img, to_string(tp_tab[i].wo_id+1), cv::Point(tp_tab[i].p.get_u(),tp_tab[i].p.get_v()),FONT_HERSHEY_TRIPLEX, 1.0, cv::Scalar(100,100,250), 1, cv::LINE_AA);
        }

    }
    firstframe = false;

}

