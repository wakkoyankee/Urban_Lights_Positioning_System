/*!
  \file vpPoseOmniVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach with omnidirectional images
*/

#include "CPose.h"

#include <visp/vpExponentialMap.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

#include <visp/vpImageIo.h>

#include "COmni.h"
#include "CPoseOmni.h"

#include <visp/vpRobust.h>
#include <visp/vpImageMorphology.h>
#include <visp/vpTime.h>

#define SAVDETAILSOPTPHOTO 0

#define LM

#pragma GCC diagnostic ignored "-Wwrite-strings"

using namespace std;

double CPose::threshold = 1e-30f;

/*!
  \brief Compute the pose using virtual visual servoing approach on omnidirectional images

*/

CPose::CPose() /*: moteur(NULL)*/ {
    init();
}

CPose::~CPose() {
}

void CPose::setRansacMaxTrials(int rMT) {
    ransacMaxTrials=rMT;
}

void CPose::setRansacNbInliersToReachConsensus(int rNIC) {
    ransacNbInlierConsensus=rNIC;
}

void CPose::setRansacThreshold(double Rthreshold) {
    ransacThreshold=Rthreshold;
}

void CPose::setLambda(double a) {
    lambda = a;
}

void CPose::setMu(double a) {
    muRef = a;
}

void CPose::setVvsIterMax(int nb) {
    vvsIterMax = nb;
}

void CPose::init() {
    clearPoint();

    lambda = 0.1 ;

    muRef = 0.01;

    vvsIterMax = 200 ;
}

void CPose::clearPoint() {
    listP.kill() ;
    npt = 0 ;
}

void CPose::addPoint(const CPoint& newP) {
    listP += newP;
    npt++;
}

void CPose::setListPoints(vpList<CPoint> & lP) {
    clearPoint();
    listP = lP;
    npt = lP.nbElements();
}

void CPose::poseVirtualVS(CModel *cam, vpHomogeneousMatrix & cMo) {

    //RANSAC OMNI
    srand(0);
    std::vector<unsigned int> best_consensus;
    std::vector<unsigned int> cur_consensus;
    std::vector<unsigned int> cur_outliers;
    std::vector<unsigned int> cur_randoms;
    unsigned int size = listP.nbElement();
    int nbTrials = 0;
    unsigned int nbMinRandom = 4 ;
    unsigned int nbInliers = 0;

    vpHomogeneousMatrix  cMo_ameller, best_cMo;
    double r,  r_ameller;

    bool foundSolution = false;

    cout<<"RANSAC OMNI"<<endl;

    //assez iteration et nombre de points qui correspondent avec le model suffisant
    while (nbTrials < ransacMaxTrials && nbInliers < (unsigned)ransacNbInlierConsensus) {
        //cout<<"Trials : "<<nbTrials<<endl;
        cur_outliers.clear();
        cur_randoms.clear(); // list des ids selectionner aleatoirement

        std::vector<bool> usedPt(size, false);//vecteur taille liste de tout les points init a false
        CPoseOmni poseMin ;

        for(unsigned int i = 0; i < nbMinRandom; i++) {
            unsigned int r = (unsigned int)rand()%size;
            while(usedPt[r] ) r = (unsigned int)rand()%size;
            usedPt[r] = true; //le point choisi devient true dans la liste

            CPoint pt;
            listP.front() ; //place le pointer au debut de la liste
            int k=0;


            while (!listP.outside()) { // si le pointer point encore dans le liste (cf VPlist)
                if(k==r) {
                    pt = listP.value() ;
                    ;
                    break;
                }
                k++ ;
                listP.next();
            }
            //recupere k index dans liste p du point a ajouté au model et pt = a ce point

            listP.front() ;
            poseMin.listP.front() ;
            //Les degenerated points sont des points soit dans le 2D soit dans le 3D trop proches l'un de l'autre
            //Si le point choisi Parmi les 4 du model est un degenerated point ne pas le prendre et re rechercher un autre points random (i--)
            bool degenerate = false;
            while (!poseMin.listP.outside()) { //parcours la liste de point de poseMin
                CPoint ptdeg = poseMin.listP.value() ;
                //fabs valeur absolue en float
                if( ((fabs(pt.get_x() - ptdeg.get_x()) < 1e-6) && (fabs(pt.get_y() - ptdeg.get_y()) < 1e-6))  ||
                        ((fabs(pt.get_oX() - ptdeg.get_oX()) < 1e-6) && (fabs(pt.get_oY() - ptdeg.get_oY()) < 1e-6) && (fabs(pt.get_oZ() - ptdeg.get_oZ()) < 1e-6))) {
                    degenerate = true;
                    break;
                }
                poseMin.listP.next() ;
            }
            if(!degenerate) {
                poseMin.addPoint(pt) ;
                cur_randoms.push_back(r);
            } else
                i--;
        }//creer un model apres besoin de le test

        listP.front() ;
        poseMin.listP.front();

        //ICI poseMin.listP contient 4 points pris au hasard et non dégénéré
        poseMin.poseInit(cMo_ameller,cam); // rend just cMo_ameller en mat identité

        r_ameller = 0; //cest le taux d erreur de la pose cMo_ameller
        r_ameller = poseMin.computeResidual(cam,cMo_ameller);
        r = sqrt(r_ameller)/(double)nbMinRandom;


        //cout<<"Erreur résiduelle : "<<r<<endl;

        cMo = cMo_ameller;

        if (r < ransacThreshold) {//si r plus petit que le threshold
            vpList<CPoint> listP2(listP) ; //listP2 = ListP
            unsigned int nbInliersCur = 0; //calculs de inliers du ransac?
            unsigned int iter = 0;

            listP.front();
            listP2.front();
            while (!listP.outside()) { //parcours tout les points
                CPoint pt = listP.value();
                CPoint p(pt) ;
                p.changeFrame(cMo);
                cam->project3DImage(p) ;//p = pt mais dans nouveaux repere cMo

                double d = vpMath::sqr(p.get_x() - pt.get_x()) + vpMath::sqr(p.get_y() - pt.get_y()) ;
                double error = sqrt(d) ; //calcul erreur
                if(error < ransacThreshold) { //si l'erreur du nouveau (avec cMo) est en dessous du threshold
                    bool degenerate = false;

                    for(unsigned int it_inlier_index = 0; it_inlier_index< cur_consensus.size(); it_inlier_index++) {
                        listP2.front();
                        for(int i=0; i<cur_consensus[it_inlier_index]; i++)
                            listP2.next();
                        //cur consensus stock list d'index de points de listP
                        CPoint pt = listP2.value();//listP2 est placé sur le point d index sur le current point de cur_consensus
                        CPoint ptdeg = listP.value() ; // current point of while loop
                        if( ((fabs(pt.get_x() - ptdeg.get_x()) < 1e-6) && (fabs(pt.get_y() - ptdeg.get_y()) < 1e-6))  ||
                                ((fabs(pt.get_oX() - ptdeg.get_oX()) < 1e-6) && (fabs(pt.get_oY() - ptdeg.get_oY()) < 1e-6) && (fabs(pt.get_oZ() - ptdeg.get_oZ()) < 1e-6))) {
                            degenerate = true;
                            break;
                        }
                    }//regarde si le current point de listP est trop proche d'un point de cur_consensus si oui alors degenerate=true

                    if(!degenerate) {//si pas un point degenerer alors inlier
                        nbInliersCur++;
                        cur_consensus.push_back(iter);
                    } else {//sinon outlier
                        cur_outliers.push_back(iter);
                    }
                } else{
                    cur_outliers.push_back(iter);
                    }

                listP.next() ;
                iter++;

            }

            //cout<<">>> "<<nbInliersCur<<endl;

            if(nbInliersCur > nbInliers) {
                foundSolution = true;
                best_consensus = cur_consensus;
                nbInliers = nbInliersCur;
                best_cMo = cMo_ameller;
            }

            nbTrials++;
            cur_consensus.clear();

            if(nbTrials >= ransacMaxTrials) { //Prend le denier model qui a le mieux marché
                //cout<<"Nombre max de trials de RANSAC atteint"<<endl;
                foundSolution = true;
            }
        }
    }
    //cout<<"Nombre d'inliers : "<<nbInliers<<endl;
    //cout<<"Sortie RANSAC -> VVS avec les inliers"<<endl;

    if(foundSolution) {
        if(nbInliers >= (unsigned)ransacNbInlierConsensus) { //si le model est suffisant
            CPoseOmni pose ;
            for(unsigned i = 0 ; i < best_consensus.size(); i++) { //ajoute tout les inliers du ransac dans pose
                listP.front();
                for(int j=0; j< best_consensus[i]; j++) {
                    listP.next();
                }
                //pour chaque points de best_consensus
                CPoint pt = listP.value();

                pose.addPoint(pt) ;
                ransacInliers.push_back(pt);
            }

            // calcul de pose linéaire avec les inliers ?
            cMo = best_cMo;

            //cout<<"VVS FTW"<<endl;

            //VVS sur les inliers
            try {
                double  residu_1 = 1e8 ;
                double r =1e8-1;

                int iter = 0 ;

                int nb = pose.listP.nbElement() ;
                vpMatrix L(2*nb,6), Lp;
                vpColVector err(2*nb) ;
                vpColVector sd(2*nb),s(2*nb) ; //vecteur colonne avec toutes les coordonnées 2D de tout les points

                vpMatrix H, Hs;
                vpMatrix Id; // matrice identite

                vpColVector v, gradient_du_cout;

                pose.listP.front() ;
                vpList<CPoint> lP ;

                CPoint P;

                // create sd
                int k =0 ;
                while (!pose.listP.outside()) {
                    P = pose.listP.value() ;
                    sd[2*k] = P.get_x() ;
                    sd[2*k+1] = P.get_y() ;
                    lP += P ;
                    pose.listP.next() ;
                    k++ ;
                }

                mu = muRef;//0.01
                vpMatrix Ls;

                while((int)((residu_1 - r)*1e12) !=0) {
                    L = 0;

                    residu_1 = r ;

                    // Compute the interaction matrix and the error
                    int k =0 ;
                    lP.front() ;
                    while (!lP.outside()) {//pour tout les points
                        P = lP.value() ;

                        P.changeFrame(cMo) ;
                        cam->project3DImage(P) ;

                        lP.modify(P); //Sauvegarde

                        s[2*k] = P.get_x();  /* point projected from cMo */
                        s[2*k+1] = P.get_y();

                        computeJacobianForVVS(P, cam, Ls); //utile?? fait rien

                        for(int i = 2*k, ii = 0 ; ii < 2 ; i++, ii++) //
                            for(int j = 0 ; j < 6 ; j++)
                                L[i][j] = Ls[ii][j];

                        lP.next() ;

                        k++ ;
                    }
                    err = s - sd ;

                    // compute the residual
                    r = err.sumSquare() ;

                    if(r < residu_1)
                        mu /= 2.0;
                    /*else
                    	if(r > residu_1)
                    		mu *= 2.0;*/

                    Lp = L.t();

                    gradient_du_cout = Lp * err;

                    Hs = L.AtA();
                    int nr = Hs.getRows();
                    Id.eye(nr);
                    for(int ind = 0 ; ind < nr ; ind++) Id[ind][ind] = Hs[ind][ind];

                    H = (mu * Id + Hs).inverseByLU();//.pseudoInverse(); //

                    v = -lambda * H * gradient_du_cout;

                    // update the pose
                    cMo = vpExponentialMap::direct(v).inverse()*cMo ;
                    if (iter++>vvsIterMax) break ;
                }
                lP.kill() ;
            } catch(...) {
                vpERROR_TRACE(" ") ;
                throw ;
            }

        }
    }

}

double CPose::computeResidual(CModel *cam, vpHomogeneousMatrix &cMo) {
    double residual = 0 ;
    CPoint P ;
    int x=0; //utile ??
    listP.front();

    while (!listP.outside()) {//visite tout les points
        P = listP.value();
        double x = P.get_x() ; //previous 2D coordinates
        double y = P.get_y() ;

        P.changeFrame(cMo);//change p de repere
        cam->project3DImage(P) ;//projection Omni? change P.get_x() depuis le nouveau repere

        residual += vpMath::sqr(x-P.get_x()) + vpMath::sqr(y-P.get_y()) ;
        listP.next();
    }
    //retourne l'erreur
    return residual;
}

// BESOIN ?
static double derivativeFilterX(vpImage<unsigned char> & fr, const int r, const int c) {
    return (2047.0 *(fr[r][c+1] - fr[r][c-1])
            +913.0 *(fr[r][c+2] - fr[r][c-2])
            +112.0 *(fr[r][c+3] - fr[r][c-3]))*0.000118793062;// /8418.0;
}

// BESOIN ?
static double derivativeFilterY(vpImage<unsigned char> & fr, const int r, const int c) {
    return (2047.0 *(fr[r+1][c] - fr[r-1][c])
            +913.0 *(fr[r+2][c] - fr[r-2][c])
            +112.0 *(fr[r+3][c] - fr[r-3][c]))*0.000118793062;// /8418.0;
}

//BESOIN ?
static double derivativeFilterX(vpImage<float> & fr, const int r, const int c) {
    return (2047.0 *(fr[r][c+1] - fr[r][c-1])
            +913.0 *(fr[r][c+2] - fr[r][c-2])
            +112.0 *(fr[r][c+3] - fr[r][c-3]))*0.000118793062;// /8418.0;
}

//BESOIN ?
static double derivativeFilterY(vpImage<float> & fr, const int r, const int c) {
    return (2047.0 *(fr[r+1][c] - fr[r-1][c])
            +913.0 *(fr[r+2][c] - fr[r-2][c])
            +112.0 *(fr[r+3][c] - fr[r-3][c]))*0.000118793062;// /8418.0;
}

//#define AFFDETAILSOPTPHOTO

#define SEUILGRAD 0

//BESOIN ?
void imgAndimg(unsigned char *src1, unsigned char *src2, unsigned char *dst, int larg, int haut)

{
    long taille = larg*haut, i;

    for(i = 0 ; i < taille ; i++, src1++, src2++, dst++)
        *dst = (*src2!=0)?*src1:0;//*src1 & *src2;
}

// BESOIN ?
void fltImgAndimg(float *src1, unsigned char *src2, float *dst, int larg, int haut)

{
    int taille = larg*haut, i;

    for(i = 0 ; i < taille ; i++, src1++, src2++, dst++)
        *dst = (*src2!=0)?*src1:-1;
}


//BESOIN ?
int CPose::photometricPoseVirtualVS(vpImage<unsigned char> &Id, CModel *cam, vpHomogeneousMatrix & cMo, int numImg) {

Mat IVirtuelle,Frame, IVirtuelleRaw;
    int iter=0, ItMax=200, CptAugmMax=30;
    int larg = Id.getWidth();
    int haut = Id.getHeight();

    int tailleImage = larg*haut;

    int curPoint;
    int nbPointTotal = 0;

    double  residu_1 = 1e10 ;
    double r =residu_1-1;
    double rprec=0;
    double nbCols = 6;
    double threshold = 1e-30f;

    double res=0;
    double resPrec=0;

    double mu = 0.01 ; //1
    double lambda= 1;


    bool FirstAugmentation=false, quitter=false;
    int CptAugm=0;


    vpHomogeneousMatrix cMo_inv, cMo_bak;
    cMo_bak = cMo;

    vpImage<vpRGBa> Idfbords(haut,larg);
    vpImage<unsigned char> Idbords(haut, larg);
    vpImage<unsigned char> IS(haut, larg);
    vpImage<unsigned char> MaskUsedPt(haut,larg);
    vpImage<unsigned char> Mask(haut,larg);
    vpImage<unsigned char> MaskPrec(haut,larg);
    vpImage<float> Idf(haut, larg);
    vpImage<float> ISf(haut, larg);
    vpImage<float> ISfPrec(haut, larg);
    vpImage<float> ISD(haut,larg);
    float *ptr_ISfPrec;
    unsigned char *ptr_MaskPrec;

    int tol=0;

unsigned char *ptr_Id ,*ptr_IS, *ptr_Mask, *ptr_MaskUsedPt;
    float *ptr_ISf, *ptr_Idf, *ptr_ISD;
    double *ptr_Pd, *ptr_P;
    vpImage<unsigned char> IDiff(haut, larg);
    unsigned char *ptr_IDiff;

    vpRobust robust(0);

    CPoint Pt;
    vpList<CPoint> listP;
    vpList<vpRowVector> listGrad;
    vpColVector P;
    vpColVector Pd;
    vpColVector error ;
    vpColVector w;
    vpColVector weighted_error, weighted_error_bak;
    vpColVector gradient_du_cout,gradient_du_coutAug;
    vpColVector Tc(nbCols);

    vpMatrix L, L_bak, Lsr, Lux(2,2), Lp;
    vpMatrix H, Hs,HsAug;
    vpMatrix Ide,IdeAug;

    Lux[0][0]=cam->getau();
    Lux[1][1]=cam->getav();

    vpRowVector gradPt(2),LGrad;

    vpDisplayX disp,disp2, disp3,disp4;

    disp4.init(Idfbords,1010,20,"Idbords");


     vpImageFilter::canny(Id, Idbords, 5, 50, 3);
     vpImageIo::read(Mask,"../Mask.png");

     ofstream ficResidu("residu.txt"), ficcMo("cMos.txt");
     Mat mask;

      do {
	ficcMo<<cMo<<endl;

        cMo_inv = cMo.inverse();
        nbPointTotal = 0;
        r = 0. ;
        curPoint = 0 ;
        resPrec=0;
        res=0;

        MaskUsedPt = Mask;
	ptr_ISD = ISD.bitmap;
	ptr_MaskUsedPt = MaskUsedPt.bitmap;
        for(long int i = 0 ; i < tailleImage ; i++, ptr_MaskUsedPt++, ptr_ISD++) {
		if((*ptr_MaskUsedPt)==255 && (*ptr_ISD)<0.0)
			(*ptr_MaskUsedPt) = 0;

	}
	vpImageConvert::convert(MaskUsedPt, mask);
        Mat element = getStructuringElement( MORPH_RECT, Size( 7,7 ) );
        erode( mask, mask, Mat());
	vpImageConvert::convert(mask,MaskUsedPt);



        int nbVisPix = 0;
        //ptr_Mask = Mask.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        for(long int i = 0 ; i < tailleImage ; i++, ptr_Mask++) {
            if((*ptr_Mask)==255) {
                nbVisPix++;
            }
        }


        double moyIS = 0.;
        ptr_IS = IS.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        for(int i = 0 ; i < tailleImage ; i++, ptr_IS++, ptr_Mask++) {
            if((*ptr_Mask)==255) {
                moyIS += *ptr_IS;
            }
        }

        moyIS /= nbVisPix;

        ptr_IS = IS.bitmap;
        ptr_ISf = ISf.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        double moyNrm = 0.;
        for(int i = 0 ; i < tailleImage ; i++, ptr_IS++, ptr_ISf++, ptr_Mask++) {
            if((*ptr_Mask)==255) {
                *ptr_ISf = *ptr_IS - moyIS;
                moyNrm += fabs(*ptr_ISf);
            }
        }

 moyNrm /= nbVisPix;

        ptr_ISf = ISf.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        for(int i = 0 ; i < tailleImage ; i++, ptr_ISf++, ptr_Mask++) {
            if((*ptr_Mask)==255) {
                *ptr_ISf /= moyNrm;
            }
        }

        fltImgAndimg(ISf.bitmap, MaskUsedPt.bitmap, ISf.bitmap, larg, haut);


	  nbPointTotal = nbVisPix;
	nbVisPix=0;
        ptr_Idf = Idf.bitmap;
        ptr_Id = Id.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        double moyId = 0.;
        for(int i = 0 ; i < tailleImage ; i++, ptr_Id++, ptr_Mask++) {
            if((*ptr_Id)!=255) {
                moyId += *ptr_Id;
		nbVisPix++;
            }
        }

        moyId /= nbVisPix;

        ptr_Id = Id.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        moyNrm = 0.;
        for(int i = 0 ; i < tailleImage ; i++, ptr_Id++, ptr_Idf++, ptr_Mask++) {
            if((*ptr_Id)!=255) {
                *ptr_Idf = *ptr_Id - moyId;
                moyNrm += fabs(*ptr_Idf);
            }
        }

        moyNrm /= nbVisPix;

        ptr_Idf = Idf.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        for(int i = 0 ; i < tailleImage ; i++, ptr_Idf++, ptr_Mask++) {
            if((*ptr_Mask)==255) {
                *ptr_Idf /= moyNrm;
            }
        }

        Pd.resize(nbVisPix);
        P.resize(nbVisPix);
        ptr_Id = Id.bitmap;
        ptr_IS = IS.bitmap;
        ptr_Idf = Idf.bitmap;
        ptr_ISf = ISf.bitmap;
	ptr_ISD = ISD.bitmap;
        ptr_Pd = Pd.data;
        ptr_P = P.data;
        listP.kill();
        listP.front();
        listGrad.kill();
        listGrad.front();
        ptr_IDiff = IDiff.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;

        //Mat ImgGrad(haut,larg,CV_8UC1);
        //ImgGrad=Scalar(0);

for(int v = 0 ; v < haut; v++) {
            for(int u = 0 ; u < larg; u++, ptr_Idf++, ptr_ISf++, ptr_Id++, ptr_IS++, ptr_Mask++,ptr_ISD++) {
                if((*ptr_Mask)==255) {
                    *ptr_Pd = *ptr_Idf;
                    *ptr_P = *ptr_ISf;
                    ptr_Pd++;
                    ptr_P++;

                    if(!OutOfImage(u, v, 3, 3, larg-4, haut-4)) {
                        gradPt[0] = derivativeFilterX(ISf, v, u);
                        gradPt[1] = derivativeFilterY(ISf, v, u);

                        double gradnrm = sqrt(gradPt.sumSquare());

                    } else {
                        gradPt = 0;
                    }

                    listGrad.addRight(gradPt);

                    Pt.setPixUV(u, v);
                    cam->pixelMeterConversion(Pt);

                    vpColVector X(4);
                        ((COmni *)cam)->projectImageSphere(Pt, X[0], X[1], X[2]);
                        X[0] *= *ptr_ISD;
                        X[1] *= *ptr_ISD;
                        X[2] *= *ptr_ISD;
                    X = cMo_inv*X;
                    Pt.setWorldCoordinates(X[0], X[1], X[2]);
                    Pt.changeFrame(cMo);

                    listP.addRight(Pt);
                } else {
                    //*ptr_IDiff = *ptr_Id;
                    //ptr_IDiff++;
                }

      }
        }


        error.resize(nbPointTotal);
        curPoint = 0 ;

        listP.front();
        listGrad.front();



        for (int i=0 ; i < nbPointTotal; i++) {
            LGrad = listGrad.value();
            if( (LGrad[0] != 0) || (LGrad[1] != 0) ) {
                Pt = listP.value();

                error[curPoint] = P[i]-Pd[i];
            } else {
                error[curPoint] = 0;
            }
            curPoint++;
            listP.next();
            listGrad.next();
        }

        robust.setThreshold(0);
        weighted_error.resize(nbPointTotal) ;
        w.resize(nbPointTotal);
        w = 1;
        robust.MEstimator(vpRobust::TUKEY, error,w);

  int i;
        double num=0;
        double den=0;
        int nbPointSuppressedByRobust = 0 ;
        int nbPointOkByRobust =0 ;
        int nb_good_datas=0;
        double wi ;
        double eri ;

        for( i=0; i<nbPointTotal; i++) {
            wi = w[i] ;
            eri = error[i] ;

            weighted_error[i] =  wi*eri ;
        }



        curPoint = 0;
        listP.front();
        listGrad.front();
        L.resize(nbPointTotal, nbCols);
        for (int i=0 ; i < nbPointTotal; i++) {
            LGrad = listGrad.value();
            if( (LGrad[0] != 0) || (LGrad[1] != 0) ) {
                Pt = listP.value();
                computeJacobianForVVS(Pt, cam, Lsr);
                Lsr = -LGrad*Lux*Lsr;

                for(int k = 0 ; k < nbCols ; k++)
                    L[curPoint][k] = Lsr[0][k];

            } else {
                for(int k = 0 ; k < nbCols ; k++)
                    L[curPoint][k] = 0;

            }
            curPoint++;
            listP.next();
           listGrad.next();
        }

        for (int i=0 ; i < nbPointTotal ; i++) {
            for (int j=0 ; j < nbCols ; j++) {
                L[i][j] = w[i]*L[i][j] ;
            }
        }


        char wait;
        if(iter==0) {


            ISfPrec = ISf;
            MaskPrec = MaskUsedPt;
            cMo_bak=cMo;
            weighted_error_bak = weighted_error;
            L_bak = L ;


            Lp = L.t();
            gradient_du_cout = Lp * weighted_error;
            Hs = L.AtA();
            int nr = Hs.getRows();
            Ide.eye(nr);
            for(int ind = 0 ; ind < nr ; ind++) Ide[ind][ind] = Hs[ind][ind];
            H = (mu * Ide + Hs).inverseByLU();

            Tc = -lambda * H * gradient_du_cout;
            cMo = vpExponentialMap::direct(Tc).inverse()*cMo ;


            cMo_inv = cMo.inverse();
            nbPointTotal = 0;
            r = 0. ;
            curPoint = 0 ;

  ptr_Mask = MaskUsedPt.bitmap;
            int nbVisPix = 0;
            for(long int i = 0 ; i < tailleImage ; i++, ptr_Mask++) {
                if((*ptr_Mask)==255) {
                    nbVisPix++;
                }
            }

            double moyIS = 0.;
            ptr_IS = IS.bitmap;
            ptr_Mask = MaskUsedPt.bitmap;
            for(int i = 0 ; i < tailleImage ; i++, ptr_IS++, ptr_Mask++) {
                if((*ptr_Mask)==255) {
                    moyIS += *ptr_IS;
                }
            }

            moyIS /= nbVisPix;

            ptr_IS = IS.bitmap;
            ptr_ISf = ISf.bitmap;
            ptr_Mask = MaskUsedPt.bitmap;
            double moyNrm = 0.;
            for(int i = 0 ; i < tailleImage ; i++, ptr_IS++, ptr_ISf++, ptr_Mask++) {
                if((*ptr_Mask)==255) {
                    *ptr_ISf = *ptr_IS - moyIS;
                    moyNrm += fabs(*ptr_ISf);
                }
            }
            moyNrm /= nbVisPix;

            ptr_ISf = ISf.bitmap;
            ptr_Mask = MaskUsedPt.bitmap;
            for(int i = 0 ; i < tailleImage ; i++, ptr_ISf++, ptr_Mask++) {
                if((*ptr_Mask)==255) {
                    *ptr_ISf /= moyNrm;
                }
            }

            fltImgAndimg(ISf.bitmap, MaskUsedPt.bitmap, ISf.bitmap, larg, haut);

            ptr_Idf = Idf.bitmap;
            ptr_Id = Id.bitmap;
            ptr_Mask = MaskUsedPt.bitmap;
            double moyId = 0.;
            for(int i = 0 ; i < tailleImage ; i++, ptr_Id++, ptr_Mask++) {
                if((*ptr_Mask)==255) {
                    moyId += *ptr_Id;
                }
            }
            moyId /= nbVisPix;

            ptr_Id = Id.bitmap;
            ptr_Mask = MaskUsedPt.bitmap;
            moyNrm = 0.;
            for(int i = 0 ; i < tailleImage ; i++, ptr_Id++, ptr_Idf++, ptr_Mask++) {
                if((*ptr_Mask)==255) {
                    moyNrm += fabs(*ptr_Idf);
                }
            }
            moyNrm /= nbVisPix;

            ptr_Idf = Idf.bitmap;
            ptr_Mask = MaskUsedPt.bitmap;
            for(int i = 0 ; i < tailleImage ; i++, ptr_Idf++, ptr_Mask++) {
                if((*ptr_Mask)==255) {
                    *ptr_Idf /= moyNrm;
                }
            }

            nbPointTotal = nbVisPix;
Pd.resize(nbVisPix);
            P.resize(nbVisPix);
            ptr_Id = Id.bitmap;
            ptr_IS = IS.bitmap;
            ptr_Idf = Idf.bitmap;
            ptr_ISf = ISf.bitmap;
            ptr_Pd = Pd.data;
            ptr_P = P.data;
            listP.kill();
            listP.front();
            listGrad.kill();
            listGrad.front();
            ptr_IDiff = IDiff.bitmap;
            ptr_Mask = MaskUsedPt.bitmap;

for(int v = 0 ; v < haut; v++) {
                for(int u = 0 ; u < larg; u++, ptr_Idf++, ptr_ISf++, ptr_Id++, ptr_IS++, ptr_Mask++) {
                    if((*ptr_Mask)==255) {
                        *ptr_Pd = *ptr_Idf;
                        *ptr_P = *ptr_ISf;

                        ptr_Pd++;
                        ptr_P++;

                        if(!OutOfImage(u, v, 3, 3, larg-4, haut-4)) {
                            gradPt[0] = derivativeFilterX(ISf, v, u);
                            gradPt[1] = derivativeFilterY(ISf, v, u);
                            double gradnrm = sqrt(gradPt.sumSquare());
                        } else
                            gradPt = 0;

                        listGrad.addRight(gradPt);

                        Pt.setPixUV(u, v);
                        cam->pixelMeterConversion(Pt);

                        vpColVector X(4);
                        ((COmni *)cam)->projectImageSphere(Pt, X[0], X[1], X[2]);
                        X[0] *= *ptr_ISD;
                        X[1] *= *ptr_ISD;
                        X[2] *= *ptr_ISD;

                        X = cMo_inv*X;
                        Pt.setWorldCoordinates(X[0], X[1], X[2]);
                        Pt.changeFrame(cMo);

                        listP.addRight(Pt);

                    }
                }
            }

 error.resize(nbPointTotal);
            curPoint = 0 ;

            listP.front();
            listGrad.front();

            for (int i=0 ; i < nbPointTotal; i++) {
                LGrad = listGrad.value();
                if( (LGrad[0] != 0) || (LGrad[1] != 0) ) {
                    Pt = listP.value();

                    error[curPoint] = P[i]-Pd[i];

                } else {
                    error[curPoint] = 0;
                }
                curPoint++;
                listP.next();
                listGrad.next();
            }


            weighted_error.resize(nbPointTotal) ;
            w.resize(nbPointTotal);
            w = 1;
            robust.MEstimator(vpRobust::TUKEY, error,w);


            int i;
            double num=0;
            double den=0;
            int nbPointSuppressedByRobust = 0 ;
            int nbPointOkByRobust =0 ;
            int nb_good_datas=0;
            double wi ;
            double eri ;

 for( i=0; i<nbPointTotal; i++) {
                wi = w[i] ;
                eri = error[i] ;

                weighted_error[i] =  wi*eri ;
            }

            curPoint = 0;
            listP.front();
            listGrad.front();
            L.resize(nbPointTotal, nbCols);
            for (int i=0 ; i < nbPointTotal; i++) {
                LGrad = listGrad.value();
                if( (LGrad[0] != 0) || (LGrad[1] != 0) ) {
                    Pt = listP.value();
                    computeJacobianForVVS(Pt, cam, Lsr);
                    Lsr = -LGrad*Lux*Lsr;

                    for(int k = 0 ; k < nbCols ; k++)
                        L[curPoint][k] = Lsr[0][k];

                } else {
                    for(int k = 0 ; k < nbCols ; k++)
                        L[curPoint][k] = 0;

                }
                curPoint++;
                listP.next();
                listGrad.next();
            }

 for (int i=0 ; i < nbPointTotal ; i++) {
                for (int j=0 ; j < nbCols ; j++) {
                    L[i][j] = w[i]*L[i][j] ;
                }
            }

        }

        ptr_Idf = Idf.bitmap;
        ptr_ISf = ISf.bitmap;
        ptr_ISfPrec = ISfPrec.bitmap;
        ptr_IDiff = IDiff.bitmap;
        ptr_Mask = MaskUsedPt.bitmap;
        ptr_MaskPrec = MaskPrec.bitmap;
        res=0;
        resPrec=0;

        for(int v = 0 ; v < haut; v++) {
            for(int u = 0 ; u < larg; u++, ptr_Idf++, ptr_ISf++, ptr_ISfPrec++,ptr_Mask++, ptr_MaskPrec++) {
                if(((*ptr_Mask)==255) && ((*ptr_MaskPrec)==255)) {
                    res+=vpMath::sqr( (*ptr_ISf) - (*ptr_Idf));
                    resPrec+=vpMath::sqr( (*ptr_ISfPrec) - (*ptr_Idf));
                }

            }
        }

	ficResidu<<res<<endl;

 	if( ((res < resPrec)))  {
            std::cout <<"\E[32;1m"<<iter<<"> L'erreur residuelle diminue :" <<res<< "("<<resPrec-res<<")\E[m"<< std::endl;

                mu/=10 ;


            CptAugm=0;
            weighted_error_bak = weighted_error;
            L_bak = L ;
            cMo_bak = cMo;
            residu_1 = r;

            ISfPrec = ISf;
            MaskPrec = MaskUsedPt;


        } else if(vpMath::equal(res,resPrec,threshold) == false) {

            std::cout << "\E[31;1m"<<iter<<"> L'erreur residuelle  augmente :" <<res<< "("<<resPrec-res<<")\E[m"<< std::endl;

            cMo = cMo_bak;
            L = L_bak;
            weighted_error = weighted_error_bak;


           if(iter==0)
                iter--;

            mu *= 100 ;//4000
            CptAugm++;

            quitter = false;
            if(CptAugm>=CptAugmMax) {
                quitter = true;
            } else if(CptAugm>1)
                iter--;

        } else {
            quitter = true;
            std::cout <<"\E[33;1m"<<iter<<"> L'erreur residuelle est constante :" <<res<< "("<<resPrec-res<<")\E[m"<< std::endl;
  }
        if(!quitter) {

            Lp = L.t();
            gradient_du_cout = Lp * weighted_error;
            Hs = L.AtA();
            int nr = Hs.getRows();
            Ide.eye(nr);
            for(int ind = 0 ; ind < nr ; ind++) Ide[ind][ind] = Hs[ind][ind];
            H = (mu * Ide + Hs).inverseByLU();

            Tc = -lambda * H * gradient_du_cout;


           cMo = vpExponentialMap::direct(Tc).inverse()*cMo ;

        }


        for(int j=0;j<haut;j++){
                for(int i=0;i<larg;i++){
                        if(Idbords[j][i]==255){
                                Idfbords[j][i].R=255;
                                Idfbords[j][i].G=0;
                                Idfbords[j][i].B=0;
                        }else{
                                Idfbords[j][i].R=IS[j][i];
                                Idfbords[j][i].G=IS[j][i];
                                Idfbords[j][i].B=IS[j][i];
                        }
                }
        }

vpDisplay::display(Idfbords);
        vpDisplay::flush(Idfbords);
        vpImageIo::write(Idfbords,"img/Contours/Contours_"+boost::lexical_cast<string>(iter)+".png");
        rprec=r;


        iter++;
    } while(!quitter && iter!=ItMax );

    ficResidu.close();
    ficcMo.close();
    cMo = cMo_bak;

	return 0;

}


// BESOIN ? return que identité il me semble
void CPose::poseInit(vpHomogeneousMatrix & cMo, CModel *cam) {
    int i, j;
    CPoint P, Pbis;
    vpList<CPoint> lP;
    vpColVector sP[4];

    lP.kill();//vide LP
    lP += listP;//LP prend tout les points

    if(initViewLines(cam, sP, 4)) { //initview lines useless pour l'instant return que 1
        cMo.eye();
        return;
    }

    vpMatrix c(4,4), sqrDistance(4,4);

    listP.front();
    i = 0;
    while(i < 3) {
        P = listP.value();
        lP.front();
        lP.suppress();
        j = i+1;
        while(j < 4) {
            Pbis = lP.value();

            c[i][j] = c[j][i] = -2.0*(sP[i][0]*sP[j][0] + sP[i][1]*sP[j][1] + sP[i][2]*sP[j][2]);
            vpColVector oP = P.oP-Pbis.oP;
            oP.resize(3,false);
            sqrDistance[i][j] = sqrDistance[j][i] = oP.sumSquare();

            lP.next();
            j++;
        }
        listP.next();
        i++;
    }

    vpMatrix A(24,24);

    A[0][0] = 1.0;
    A[0][4] = c[0][1];
    A[0][7] = 1.0;
    A[0][20] = sqrDistance[0][1];
    A[1][1] = 1.0;
    A[1][4] = 1.0;
    A[1][7] = c[0][1];
    A[1][21] = sqrDistance[0][1];
    A[2][5] = 1.0;
    A[2][8] = 1.0;
    A[2][16] = c[0][1];
    A[2][22] = sqrDistance[0][1];
    A[3][6] = 1.0;
    A[3][9] = 1.0;
    A[3][17] = c[0][1];
    A[3][23] = sqrDistance[0][1];

    A[4][0] = 1.0;
    A[4][5] = c[0][2];
    A[4][11] = 1.0;
    A[4][20] = sqrDistance[0][2];
    A[5][4] = 1.0;
    A[5][10] = 1.0;
    A[5][16] = c[0][2];
    A[5][21] = sqrDistance[0][2];
    A[6][2] = 1.0;
    A[6][5] = 1.0;
    A[6][11] = c[0][2];
    A[6][22] = sqrDistance[0][2];
    A[7][6] = 1.0;
    A[7][12] = 1.0;
    A[7][18] = c[0][2];
    A[7][23] = sqrDistance[0][2];

    A[8][0] = 1.0;
    A[8][6] = c[0][3];
    A[8][15] = 1.0;
    A[8][20] = sqrDistance[0][3];
    A[9][4] = 1.0;
    A[9][13] = 1.0;
    A[9][17] = c[0][3];
    A[9][21] = sqrDistance[0][3];
    A[10][5] = 1.0;
    A[10][14] = 1.0;
    A[10][18] = c[0][3];
    A[10][22] = sqrDistance[0][3];
    A[11][3] = 1.0;
    A[11][6] = 1.0;
    A[11][15] = c[0][3];
    A[11][23] = sqrDistance[0][3];

    A[12][7] = 1.0;
    A[12][11] = 1.0;
    A[12][16] = c[1][2];
    A[12][20] = sqrDistance[1][2];
    A[13][1] = 1.0;
    A[13][8] = c[1][2];
    A[13][10] = 1.0;
    A[13][21] = sqrDistance[1][2];
    A[14][2] = 1.0;
    A[14][8] = 1.0;
    A[14][10] = c[1][2];
    A[14][22] = sqrDistance[1][2];
    A[15][9] = 1.0;
    A[15][12] = 1.0;
    A[15][19] = c[1][2];
    A[15][23] = sqrDistance[1][2];

    A[16][7] = 1.0;
    A[16][15] = 1.0;
    A[16][17] = c[1][3];
    A[16][20] = sqrDistance[1][3];
    A[17][1] = 1.0;
    A[17][9] = c[1][3];
    A[17][13] = 1.0;
    A[17][21] = sqrDistance[1][3];
    A[18][8] = 1.0;
    A[18][14] = 1.0;
    A[18][19] = c[1][3];
    A[18][22] = sqrDistance[1][3];
    A[19][3] = 1.0;
    A[19][9] = 1.0;
    A[19][13] = c[1][3];
    A[19][23] = sqrDistance[1][3];

    A[20][11] = 1.0;
    A[20][15] = 1.0;
    A[20][17] = c[2][3];
    A[20][20] = sqrDistance[2][3];
    A[21][9] = c[2][3];
    A[21][10] = 1.0;
    A[21][13] = 1.0;
    A[21][21] = sqrDistance[2][3];
    A[22][2] = 1.0;
    A[22][14] = 1.0;
    A[22][19] = c[2][3];
    A[22][22] = sqrDistance[2][3];
    A[23][3] = 1.0;
    A[23][12] = 1.0;
    A[23][13] = c[2][3];
    A[23][23] = sqrDistance[2][3];

    vpColVector Sa(A.getRows());
    vpMatrix Va(A.getRows(), A.getCols());

#ifdef VISP_HAVE_GSL
    A.svd(Sa, Va);
#else
    //on utilise la svd de OpenCV
    CvMat AMat=cvMat(A.getRows(), A.getCols(), CV_64F, A.data),
          SaMat=cvMat(Sa.getRows(), 1, CV_64F, Sa.data),
          VaMat=cvMat(Va.getRows(), Va.getCols(), CV_64F, Va.data);

    cvSVD(&AMat, &SaMat, NULL, &VaMat, CV_SVD_MODIFY_A);
#endif


    // Pour chaque point, on recherche sa distance au centre de projection
    double x[4] = {0.0,0.0,0.0,0.0};

    for(i = 0 ; i < 4 ; i++)
        x[i] = sqrt(fabs(Va[i][23] / Va[20+i][23]));


    vpColVector cP[4], oP[4];
    // Calcul des 4 points dans le repère caméra
    listP.front();
    i = 0;
    while (i < 4) {
        P = listP.value() ;

        P.cP = x[i] * sP[i];
        //lP.modify(P);
        cP[i].resize(4);
        cP[i] = P.cP;
        cP[i].resize(3,false);

        oP[i].resize(4);
        oP[i] = P.oP;
        oP[i].resize(3,false);

        listP.next();
        i++;
    }

    //Calcul de la matrice de transformation du repère objet/monde au repère caméra cMo
    //Calcul des 4 bases différentes possibles avec 4 points dans les deux repères pour calculer la rotation d'abord
    vpMatrix cBase(3,12), oBase(3,12);
    vpColVector X, Y, Z;
    int ix, iy, iz;
    //Repère caméra
    //Premiere base
    X = cP[1]-cP[0];
    Y = cP[2]-cP[0];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = 0 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        cBase[i][ix] = X[i];
        cBase[i][iy] = Y[i];
        cBase[i][iz] = Z[i];
    }

    //Deuxieme base
    X = cP[2]-cP[1];
    Y = cP[3]-cP[1];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = iz + 1 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        cBase[i][ix] = X[i];
        cBase[i][iy] = Y[i];
        cBase[i][iz] = Z[i];
    }

    //Troisieme base
    X = cP[1]-cP[0];
    Y = cP[3]-cP[0];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = iz + 1 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        cBase[i][ix] = X[i];
        cBase[i][iy] = Y[i];
        cBase[i][iz] = Z[i];
    }

    //Quatrieme base
    X = cP[2]-cP[0];
    Y = cP[3]-cP[0];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = iz + 1 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        cBase[i][ix] = X[i];
        cBase[i][iy] = Y[i];
        cBase[i][iz] = Z[i];
    }

    //Repère objet/monde
    //Premiere base
    X = oP[1]-oP[0];
    Y = oP[2]-oP[0];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = 0 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        oBase[i][ix] = X[i];
        oBase[i][iy] = Y[i];
        oBase[i][iz] = Z[i];
    }

    //Deuxieme base
    X = oP[2]-oP[1];
    Y = oP[3]-oP[1];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = iz + 1 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        oBase[i][ix] = X[i];
        oBase[i][iy] = Y[i];
        oBase[i][iz] = Z[i];
    }

    //Troisieme base
    X = oP[1]-oP[0];
    Y = oP[3]-oP[0];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = iz + 1 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        oBase[i][ix] = X[i];
        oBase[i][iy] = Y[i];
        oBase[i][iz] = Z[i];
    }

    //Quatrieme base
    X = oP[2]-oP[0];
    Y = oP[3]-oP[0];
    Z = vpColVector::cross(X, Y);
    Y = vpColVector::cross(Z, X);
    X.normalize();
    Y.normalize();
    Z.normalize();
    ix = iz + 1 ;
    iy = ix + 1 ;
    iz = iy + 1;
    for(i=0 ; i < 3 ; i++) {
        oBase[i][ix] = X[i];
        oBase[i][iy] = Y[i];
        oBase[i][iz] = Z[i];
    }

    vpMatrix R;
    R = cBase * ( oBase.t() * (oBase * oBase.t()).inverseByLU());

    vpColVector S(R.getRows());
    vpMatrix U(R.getRows(), R.getCols()), V(R.getRows(), R.getCols());

#ifdef VISP_HAVE_GSL
    U = R;
    U.svd(S, V);
#else
    //on utilise la svd de OpenCV
    CvMat RMat=cvMat(R.getRows(), R.getCols(), CV_64F, R.data),
          UMat=cvMat(U.getRows(), U.getCols(), CV_64F, U.data),
          SMat=cvMat(S.getRows(), 1, CV_64F, S.data),
          VMat=cvMat(V.getRows(), V.getCols(), CV_64F, V.data);

    cvSVD(&RMat, &SMat, &UMat, &VMat, CV_SVD_MODIFY_A);
#endif
    //elimination des facteurs d'échelle introduits par du bruit
    R = U * V.t();

    vpColVector rcP[4], mrcP(3), mcP(3), t;
    for(i = 0 ; i < 4 ; i++) {
        rcP[i] = R * oP[i];
        mrcP += rcP[i];
        mcP += cP[i];
    }
    mrcP *= 0.25;
    mcP *= 0.25;

    t = mcP-mrcP;

    vpTranslationVector cto(t[0], t[1], t[2]);
    vpRotationMatrix cRo;
    for(i=0; i<3; i++)
        for(j=0; j<3; j++)
            cRo[i][j]=R[i][j];
    cMo.buildFrom(cto, cRo);

}
