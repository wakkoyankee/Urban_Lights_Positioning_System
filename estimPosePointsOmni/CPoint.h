#ifndef AFX_CPoint
#define AFX_CPoint

#include <iostream>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpPoseVector.h>
#include <visp/vpTranslationVector.h>

class CPoint//:public vpPoint, public vpSubPixel
{
public:
	double u, v;
	vpColVector oP, cP, p;
public:
	CPoint();
	~CPoint();
	double get_u() const {return u;}
	double get_v() const {return v;}

	inline void set_u(const double _u) { u = _u;}
	inline void set_v(const double _v) { v = _v;}

				   //! set the point coordinates (camera frame)
				   inline void set_X(const double X) { cP[0] = X ; }
				   inline void set_Y(const double Y) { cP[1] = Y ; }
				   inline void set_Z(const double Z) { cP[2] = Z ; }
				   inline void set_W(const double W) { cP[3] = W ; }

				   //! set the point coordinates (object frame)
				   inline void set_oX(const double X) { oP[0] = X ; }
				   inline void set_oY(const double Y) { oP[1] = Y ; }
				   inline void set_oZ(const double Z) { oP[2] = Z ; }
				   inline void set_oW(const double W) { oP[3] = W ; }

				   //! get the point coordinates (camera frame)
				   double get_X()  const { return cP[0] ; }
				   double get_Y()  const { return cP[1] ; }
				   double get_Z() const  { return cP[2] ; }
				   double get_W()  const { return cP[3] ; }

				   //! get the point coordinates (object frame)
				   double get_oX() const { return oP[0] ; }
				   double get_oY() const { return oP[1] ; }
				   double get_oZ() const { return oP[2] ; }
				   double get_oW() const { return oP[3] ; }

				   //! set the point xyw-coordinates
				   inline void set_x(const double x) {  p[0] = x ; }
				   inline void set_y(const double y) {  p[1] = y ; }
				   inline void set_w(const double w) {  p[2] = w ; }

				   //! get the point xyw-coordinates
				   double get_x()  const { return p[0] ; }
				   double get_y()  const { return p[1] ; }
				   double get_w()  const { return p[2] ; }

	//! set the point world coordinates
	void setWorldCoordinates(const double ox,
							 const double oy,
							 const double oz) ;
	//! set the point world coordinates
	void setWorldCoordinates(const vpColVector &_oP) ;
	//! get the point world coordinates
	void getWorldCoordinates(double& ox,
							 double& oy,
							 double& oz) ;
	//! set the point world coordinates
	void getWorldCoordinates(vpColVector &_oP) ;
	vpColVector getWorldCoordinates(void) ;

	//!Compute the 3D coordinates _cP  (camera frame)
	void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;

	//! Update the 3D coordinates of the point (camera frame).
	//!Update the object attribute cP  (3D coordinates in the camera frame)
	inline void changeFrame(const vpHomogeneousMatrix &cMo) {

        double X = cMo[0][0]*oP[0]+ cMo[0][1]*oP[1]+ cMo[0][2]*oP[2]+ cMo[0][3]*oP[3] ;
		double Y = cMo[1][0]*oP[0]+ cMo[1][1]*oP[1]+ cMo[1][2]*oP[2]+ cMo[1][3]*oP[3] ;
		double Z = cMo[2][0]*oP[0]+ cMo[2][1]*oP[1]+ cMo[2][2]*oP[2]+ cMo[2][3]*oP[3] ;
		double W = cMo[3][0]*oP[0]+ cMo[3][1]*oP[1]+ cMo[3][2]*oP[2]+ cMo[3][3]*oP[3] ;

		double d = 1/W ;
		cP[0] =X*d ;
		cP[1] =Y*d ;
		cP[2] =Z*d ;
		cP[3] =1 ;
	}
    void getPositionFromPose(vpPoseVector pose);
    void getPositionFromPoseM( vpHomogeneousMatrix cMo);
	void getImageMetric(double &x, double &y, double &w)const;
	void setImageMetric(const double x,const double y, const double w);
	void getPixUV(double &u, double &v)const;
	void setPixUV(const double u, const double v);
	void setObjetImage(const double oX, const double oY, const double oZ, const double u, const double v);
	CPoint& operator=(const CPoint&);
};

#endif
