#ifndef CPOSE_HH
#define CPOSE_HH
#ifdef True
#undef True
#endif

#ifdef False
#undef False
#endif
#include "opencv2/highgui.hpp"
//#include "opencv/cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpImageFilter.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpList.h>
#include <visp/vpImage.h>
#include <visp/vpColor.h>
#include "CModel.h"
#include "CPoint.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/progress.hpp>


using namespace cv;

class CPose {
public:

    int npt ;       //!< number of point used in pose computation
    vpList<CPoint> listP ;     //!< array of point (use here class CPoint)

    vpImage<unsigned char> Mask;

    static double threshold;

    double residual ;     //!< compute the residual in meter
    double lambda ;//!< parameters use for the virtual visual servoing approach
    double mu, muRef;
    int vvsIterMax ; //! define the maximum number of iteration in VVS

    int ransacMaxTrials;
    int ransacNbInlierConsensus;
    double ransacThreshold;

    std::vector<CPoint> ransacInliers;

    //! constructor
    CPose();
    //! destructor
    virtual ~CPose();

    void setLambda(double a);
    void setMu(double a);
    void setVvsIterMax(int nb);

    void setRansacMaxTrials(int rMT);
    void setRansacNbInliersToReachConsensus(int rNIC);
    void setRansacThreshold(double threshold);


    void init() ;
    //! suppress all the point in the array of point
    void clearPoint() ;
    //! Add a new point in this array
    void addPoint(const CPoint& P) ;
    //! Add a new point in this array
    void setListPoints(vpList<CPoint> & lP) ;

    //! compute the pose using virtual visual servoing approach
    void poseVirtualVS(CModel *cam, vpHomogeneousMatrix & cMo);

    void setImageMask(vpImage<unsigned char> &_Mask) {
        Mask = _Mask;
    }
    int photometricPoseVirtualVS(vpImage<unsigned char> &Id, CModel *cam, vpHomogeneousMatrix & cMo, int numImg = 0);
    int nbPointOkByRobust ;
    int nbPointSuppressedByRobust ; //seuil a 0.05

    double computeResidual(CModel *cam,vpHomogeneousMatrix &cMo);
    //!Calcul l'erreur résiduelle

    virtual void poseInit(vpHomogeneousMatrix & cMo, CModel *cam=NULL);
    virtual void computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls)=0;

    virtual void PoseLagrange(vpHomogeneousMatrix &cMo)=0;


    virtual int initViewLines(CModel *cam, vpColVector *sP, int nbPts=-1) {
        return 1;
    };

    static bool OutOfImage(double u, double v, double uinf, double vinf, double usup, double vsup) {
        return (u < uinf) || (u >= usup) || (v < vinf) || (v >= vsup);
    }

} ;


#endif

