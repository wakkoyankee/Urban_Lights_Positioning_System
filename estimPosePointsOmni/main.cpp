#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpColor.h>
#include <visp/vpList.h>

#include <fstream>
#include<iostream>
#include <cmath>
#include <algorithm>

#include <opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/tracking.hpp>

#include "CPose.h"
#include "CPoseOmni.h"
#include "CPoint.h"
#include "CModel.h"
#include "PointTracking.h"

using namespace std;
using namespace cv;

CPoint op[11];

//! reads pose file
void read_pose(int frame, float res[6]){
    std::ifstream file;
    String line;
    file.open("LightTracking/acquisitions2/acquisitions2/Poses.dat");
    if(!file){
        std::cout<< "no file" << std::endl;
        //return 0;
    }
    int cpt = 0;
    while ( std::getline(file,line) )
    {
        if(cpt == frame){
            //std::cout << line << '\n';
            int s1 = line.find(" ", 0);
            int s2 = line.find(" ", s1+1);
            int s3 = line.find(" ", s2+1);
            int s4 = line.find(" ", s3+1);
            int s5 = line.find(" ", s4+1);
            int s6 = line.find(" ", s5+1);
            int s7 = line.find("\n", s6);
            //std::cout << s1 << " " << s2 << " " << s3 << " " << s4 << " " << s5 << '\n';
            String x[6];
            x[0] = String(line,s1,s2-s1);
            x[1] = String(line,s2,s3-s2);
            x[2] = String(line,s3,s4-s3);
            x[3] = String(line,s4,s5-s4);
            x[4] = String(line,s5,s6-s5);
            x[5] = String(line,s6,s7-s6);

            //float res[6];
            int tmp;
            for(int j = 0 ; j < 6 ; j++){
                tmp = x[j].find(",", 0);
                x[j][tmp] = '.';
                std::string str = String(x[j]);
                res[j] = std::atof(str.c_str());
                //std::cout << res[j] << "\n";
            }
            //return res;
            break;
        }
        cpt++;
    }
    file.close();
}

//! updates 2d coordinates of world points with previous/new cMo
void update_op(vpHomogeneousMatrix cMo, CModel *cam){
    int size = sizeof(op)/sizeof(op[0]);
    for(int i = 0; i< size;i++){
        op[i].changeFrame(cMo);
        cam->project3DImage(op[i]);
        cam->meterPixelConversion(op[i]);
        if(i==0){
            cout<<op[i].get_u() << " " << op[i].get_v()<<endl;
        }
    }
}

int main(int argc, const char ** argv) {

    int val = std::stoi(argv[1]); // in case we need to call a specific frame

    //region Parameters
    vpPoseVector map[] = {
            vpPoseVector(-107.317, 351.5161, 266.011, 1.570796, 0, 0),
            vpPoseVector(-121.43, 351.5161, 266.011, 1.570796, 0, 0),
            vpPoseVector(-112.386, 351.5161, 286.936, 1.570796, 0, 0),
            vpPoseVector(-125.405, 351.5161, 282.511, 1.570796, 0, 0),
            vpPoseVector(-113.8387, 351.5159, 309.2179, 1.570776, 0.009488558, -0.009488558),
            vpPoseVector( -131.5294, 351.5159, 293.2065, 1.570776, 0.009488558, -0.009488558),
            vpPoseVector(-118.6997, 351.5159, 330.2722, 1.570776, 0.009488558, -0.009488558),
            vpPoseVector(-139.1696, 351.5159, 325.7671, 1.570776, 0.009488558, -0.009488558),
            vpPoseVector(-120.2476, 351.5159, 355.3677, 1.570776, 0.009488558, -0.009488558),
            vpPoseVector(-137.121, 351.5159, 330.2717, 1.570776, 0.009488558, -0.009488558),
            vpPoseVector(-127.1071, 351.5159, 346.6719, 1.570776, 0.009488558, -0.009488558)
    };

    float au = 250, av = 250, u0 = 150, v0 = 150, xi = 1.3;
    int width = 300, height = 300;
    CModel *cam = new COmni(au,av,u0,v0,xi);
    std::string fn;

    int hmin = 10, smin = 120, vmin = 164;
    int hmax = 40, smax = 255, vmax = 255;

    CPoseOmni PoseOmni;
    unsigned int nbInlierToReachConsensus;
    double threshold = 0.05;
    PoseOmni.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
    PoseOmni.setRansacThreshold(threshold);
    PoseOmni.setRansacMaxTrials(500);


    for(int i = 0; i < 11; i++){
        op[i].getPositionFromPose(map[i]);
    }
    //endregion

    //region Program to get good image processing values
    /*cv::namedWindow("Trackbars",(640,200));
    cv::createTrackbar("Hue Min", "Trackbars", &hmin,179);
    cv::createTrackbar("Hue Max", "Trackbars", &hmax,179);
    cv::createTrackbar("Sat Min", "Trackbars", &smin,255);
    cv::createTrackbar("Sat Max", "Trackbars", &smax,255);
    cv::createTrackbar("Val Min", "Trackbars", &vmin,255);
    cv::createTrackbar("Val Max", "Trackbars", &vmax,255);

    while(true){
        fn = "LightTracking/acquisitions2/acquisitions2/frames/Frame_120.png";
        cv::Mat img = imread(fn);
        cv::Mat imgHSV, mask, dil;
        cv::cvtColor(img,imgHSV, cv::COLOR_BGR2HSV);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10));

        cv::Scalar lower(hmin,smin,vmin);
        cv::Scalar upper(hmax,smax,vmax);
        cv::inRange(imgHSV, lower,upper,mask);
        cv::dilate(mask,dil, kernel);
        cv::imshow("Image",img);
        cv::imshow("HSV",imgHSV);
        cv::imshow("mask",mask);
        cv::imshow("dil",dil);
        cv::waitKey(1);
    }*/
    //endregion


    PointTracking pt;
    vpPoseVector camPose0(-152.8854, -293.778, -317.7528, 0.5664307, 0.08449064, 2.647697);//first pose
    vpHomogeneousMatrix prev_cMo;
    prev_cMo = vpHomogeneousMatrix(camPose0); //We know the first pose
    vpList<CPoint> prev_p;
    update_op(prev_cMo,cam);//update 2d coordinates of world points

    ofstream outdata;
    outdata.open("data3.txt");

    for(int i = 0; i<219; i++){//normalement 278
        PoseOmni.listP.kill(); //vide la liste de pointq

        fn = "LightTracking/acquisitions2/acquisitions2/frames/Frame_" + to_string(i) + std::string(".png");
        cout<<fn<<endl;

        //region Image processing
        cv::Mat img = imread(fn);
        cv::Mat imgHSV, mask, dil,ero;
        cv::cvtColor(img,imgHSV, cv::COLOR_BGR2HSV);
        cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(12,12));
        cv::Scalar lower(hmin,smin,vmin);
        cv::Scalar upper(hmax,smax,vmax);
        cv::inRange(imgHSV, lower,upper,mask);
        cv::dilate(mask,dil, kernel2);
        //endregion

        // GET detected lamp positions
        pt.getPoints(img,dil,cam,op); //get new points from tp_tab

        for(int j = 0; j < pt.tp_tab.size();j++){//add points to pose estimator
            PoseOmni.listP+=pt.tp_tab[j].p;
        }
        cout<<endl;

        //region Pose estimation
        if(i != 0){
            nbInlierToReachConsensus = (unsigned int)(70.0 * pt.tp_tab.size() / 100.0);
            PoseOmni.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
            PoseOmni.poseVirtualVS(cam, prev_cMo);
        }
        std::cout<<"#################################"<<std::endl;
        std::cout<<"cMo estimé n"<<std::endl;
        for (unsigned int k = 0; k < prev_cMo.getRows(); k++) {
            for (unsigned int j = 0; j < prev_cMo.getCols(); j++) {
                std::cout << prev_cMo[k][j] << " ";
            }
            std::cout << std::endl;
        }
        cout<<endl;
        //endregion

        //region Console display real cMo
        float campose[6];
        read_pose(i,campose);
        vpPoseVector ocam(campose[0], campose[1], campose[2], campose[3], campose[4], campose[5]);
        vpHomogeneousMatrix cMo = vpHomogeneousMatrix(ocam);

        std::cout<<"Real cMo"<<std::endl;
        for (unsigned int k = 0; k < cMo.getRows(); k++) {
            for (unsigned int j = 0; j < cMo.getCols(); j++) {
                std::cout << cMo[k][j] << " ";
            }
            std::cout << std::endl;
        }
        //endregion

        CPoint np1;
        np1.getPositionFromPoseM(prev_cMo);
        CPoint np2;
        np2.getPositionFromPoseM(cMo);

        outdata << np1.get_oX()<<","<< np1.get_oY()<<","<<np1.get_oZ()<<","<< np2.get_oX()<<","<< np2.get_oY()<<","<<np2.get_oZ()<<";"<<endl;

        update_op(prev_cMo,cam);
        cv::imshow("Image",img);
        cv::imshow("mask", mask);
        cv::imshow("dil",dil);

        //cv::waitKey(0); //uncomment if you want to see frame per frame

    }

    outdata.close();
    return 0;
}


