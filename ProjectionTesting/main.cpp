//
// Created by hadrien on 15/03/2021.
//
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

#include <cmath>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include<iostream>
#include <unistd.h>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;

//VALEURS SLIDER
int g_sliderRx; //slider pos value
int g_sliderRx_max; //slider max value
int g_sliderRy;
int g_sliderRy_max;
int g_sliderRz;
int g_sliderRz_max;
int g_sliderXi;
int g_sliderXi_max;
int g_sliderC;
int g_sliderC_max;

//VALEURS CAMERA
float xi = 1.1f;
vpMatrix K(3, 3);
int im_size = 700;
vpHomogeneousMatrix cMw;
vpPoseVector wcam;

//IMAGE
vpImage<vpRGBa> image;

//POINTS
const int numPoints = 10;
vpColVector wp[numPoints];
vpColor c[numPoints];


vpColVector Repere_cam(vpColVector wx){
    return cMw * wx;
}

vpColVector D3_to_pix_coord(vpColVector wx){
    vpColVector cx = Repere_cam(wx);
    //check div /0 ?
    //if(cx[2]<0){//Pas dans le champ de la cam
    //    return  vpColVector({-1,-1,1});
    //}
    float p = sqrt(pow(cx[0],2) + pow(cx[1],2) + pow(cx[2],2));
    vpColVector X({(cx[0] / (cx[2] + xi * p)), (cx[1] / (cx[2] + xi * p)),1});
    return K * X;
}

void displayPoints(vpColVector* u) {
    vpDisplay::display(image);
    for (int i = 0; i < numPoints; i++) {
        if (u[i][0] > 699 || u[i][0] < 0 || u[i][1] > 699 || u[i][1] < 0) { continue; }
        vpDisplay::displayCircle(image, u[i][1], u[i][0], 5, c[i], true);
        vpDisplay::displayText(image, u[i][1], u[i][0], "P" + to_string(i), c[i]);
    }
    vpDisplay::flush(image);
}
void on_trackbarRx(int, void*)
{
    wcam[3] = g_sliderRx*M_PI/6;
    cMw = vpHomogeneousMatrix(wcam);
    vpColVector u[numPoints];
    for(int i=0;i<numPoints;i++){
        u[i] = D3_to_pix_coord(wp[i]);
    }
    displayPoints(u);
    //printf("%d\n", g_slider);
}
void on_trackbarRy(int, void*)
{
    wcam[4] = g_sliderRy*M_PI/6;
    cMw = vpHomogeneousMatrix(wcam);
    vpColVector u[numPoints];
    for(int i=0;i<numPoints;i++){
        u[i] = D3_to_pix_coord(wp[i]);
    }
    displayPoints(u);
    //printf("%d\n", g_slider);
}
void on_trackbarRz(int, void*)
{
    wcam[5] = g_sliderRz*M_PI/4;
    cMw = vpHomogeneousMatrix(wcam);
    vpColVector u[numPoints];
    for(int i=0;i<numPoints;i++){
        u[i] = D3_to_pix_coord(wp[i]);
    }
    displayPoints(u);
    //printf("%d\n", g_slider);
}
void on_trackbarXi(int, void*)
{
    xi = 1 + ((float)g_sliderXi/10);
    vpColVector u[numPoints];
    for(int i=0;i<numPoints;i++){
        u[i] = D3_to_pix_coord(wp[i]);
    }
    displayPoints(u);
}
void on_trackbarC(int, void*)
{
    wcam[0] = g_sliderC-10;
    cMw = vpHomogeneousMatrix(wcam);
    vpColVector u[numPoints];
    for(int i=0;i<numPoints;i++){
        u[i] = D3_to_pix_coord(wp[i]);
    }
    displayPoints(u);
}

int main()
{
    //### Init matrice K ###
    K[0][0] = 340; K[0][1] =  0; K[0][2] = 350;
    K[1][0] =  0; K[1][1] = 340; K[1][2] =  350;
    K[2][0] =  0; K[2][1] = 0; K[2][2] =  1;


    //### Init points monde ###
    wp[0] = vpColVector({10,-3, 6,1});
    wp[1] = vpColVector({5,-3, 6,1});
    wp[2] = vpColVector({0,-3, 6,1});
    wp[3] = vpColVector({-5,-3, 6,1});
    wp[4] = vpColVector({-10,-3, 6,1});

    wp[5] = vpColVector({10,3, 6,1});
    wp[6] = vpColVector({5,3, 6,1});
    wp[7] = vpColVector({0,3, 6,1});
    wp[8] = vpColVector({-5,3, 6,1});
    wp[9] = vpColVector({-10,3, 6,1});

    c[0] = vpColor(255,0,0);
    c[1] = vpColor(0,255,0);
    c[2] = vpColor(0,0,255);
    c[3] = vpColor(255,255,0);
    c[4] = vpColor(0,255,255);
    c[5] = vpColor(255,0,255);
    c[6] = vpColor(255,255,255);
    c[7] = vpColor(128,128,255);
    c[8] = vpColor(255,128,128);
    c[9] = vpColor(128,255,128);

    //### Init cam dans monde ###
    wcam[0] = 0;    // tx coord scene * -1
    wcam[1] = 0;    // ty
    wcam[2] = -3;    // tz
    wcam[3] = 0;   // tux en RADIAN aussi M_PI_2
    wcam[4] = 0; // tuy
    wcam[5] = 0; // tuz

    //### Init matrice de passage dans le repere cam ###
    cMw = vpHomogeneousMatrix(wcam);

    vpHomogeneousMatrix Rx(0, 0, 0,  vpMath::rad(90), 0,0);
    vpHomogeneousMatrix Ry(0, 0, 0,  0, M_PI,0);
    vpHomogeneousMatrix Rz(0, 0, 0, 0, 0, M_PI/4);
    //cMw = Rx * cMw;
    //cMw = cMw * Ry;
    //cMw = cMw * Rz;

    for (unsigned int i = 0; i < cMw.getRows(); i++) {
        for (unsigned int j = 0; j < cMw.getCols(); j++) {
            std::cout << cMw[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    //### Coordonnées pixelique ###
    vpColVector u[numPoints];
    for(int i=0;i<numPoints;i++){
        u[i] = D3_to_pix_coord(wp[i]);
    }

    // VISP image
    image = vpImage<vpRGBa>(700, 700);
    vpDisplayX d(image, vpDisplay::SCALE_AUTO);
    vpDisplay::setTitle(image, "Light projection");
    vpDisplay::display(image);
    displayPoints(u);

    g_sliderRx = 0;
    g_sliderRx_max = 4;

    g_sliderRy = 0;
    g_sliderRy_max = 4;

    g_sliderRz = 0;
    g_sliderRz_max = 4;

    g_sliderXi = 0;
    g_sliderXi_max = 5;

    g_sliderC = 10;
    g_sliderC_max = 20;

    //window name
    namedWindow("Parameters", WINDOW_AUTOSIZE);

    //make trackbar call back
    createTrackbar("Rx", "Parameters", &g_sliderRx, g_sliderRx_max, on_trackbarRx);
    createTrackbar("Ry", "Parameters", &g_sliderRy, g_sliderRy_max, on_trackbarRy);
    createTrackbar("Rz", "Parameters", &g_sliderRz, g_sliderRz_max, on_trackbarRz);
    createTrackbar("Xi", "Parameters", &g_sliderXi, g_sliderXi_max, on_trackbarXi);
    createTrackbar("C", "Parameters", &g_sliderC, g_sliderC_max, on_trackbarC);

    //show the image
    while (1)
    {
        // Wait until user press some key
        int r = waitKey(10);
        if (r > 0)
            break;
    }

    std::cout << "A click to quit..." << std::endl;
    vpDisplay::getClick(image);
    //vpImageIo::writePNG(image, "Projection 1");

}