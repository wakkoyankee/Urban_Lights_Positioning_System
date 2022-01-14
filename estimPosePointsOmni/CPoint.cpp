#include "CPoint.h"

CPoint::CPoint()
{
	oP.resize(4); oP = 0 ; oP[3] = 1 ;
	cP.resize(4); cP = 0 ; cP[3] = 1 ;
	p.resize(3); p = 0 ; p[2] = 1 ;
}

CPoint::~CPoint()
{

}


//! set the point world coordinates
void
CPoint::setWorldCoordinates(const double ox,
							 const double oy,
							 const double oz)
{
	oP[0] = ox ;
	oP[1] = oy ;
	oP[2] = oz ;
	oP[3] = 1 ;
}


void
CPoint::setWorldCoordinates(const vpColVector &_oP)
{
	oP[0] = _oP[0] ;
	oP[1] = _oP[1] ;
	oP[2] = _oP[2] ;
	oP[3] = _oP[3] ;
	
	oP /= oP[3] ;
}

void
CPoint::getWorldCoordinates(double& ox,
							 double& oy,
							 double& oz)
{
	ox = oP[0] ;
	oy = oP[1] ;
	oz = oP[2] ;
}


void
CPoint::getWorldCoordinates(vpColVector &_oP)
{
	_oP[0] = oP[0] ;
	_oP[1] = oP[1] ;
	_oP[2] = oP[2] ;
	_oP[3] = oP[3] ;
}


vpColVector
CPoint::getWorldCoordinates(void)
{
	return this->oP;
}

//! Compute the new 3D coordinates of the point in the new camera frame.
void
CPoint::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP)
{
	
	_cP.resize(4) ;
	
	_cP[0] = cMo[0][0]*oP[0]+ cMo[0][1]*oP[1]+ cMo[0][2]*oP[2]+ cMo[0][3]*oP[3] ;
	_cP[1] = cMo[1][0]*oP[0]+ cMo[1][1]*oP[1]+ cMo[1][2]*oP[2]+ cMo[1][3]*oP[3] ;
	_cP[2] = cMo[2][0]*oP[0]+ cMo[2][1]*oP[1]+ cMo[2][2]*oP[2]+ cMo[2][3]*oP[3] ;
	_cP[3] = cMo[3][0]*oP[0]+ cMo[3][1]*oP[1]+ cMo[3][2]*oP[2]+ cMo[3][3]*oP[3] ;
	
	double d = 1.0/_cP[3] ;
	_cP[0] *=d ;
	_cP[1] *=d ;
	_cP[2] *=d ; 
	_cP[3] *=d ;
}

void CPoint::getImageMetric(double &x, double &y, double &w)const
{
	x = get_x();
	y = get_y();
	w = get_w();
}

void CPoint::setImageMetric(const double x,const double y, const double w)
{
	set_x(x);
	set_y(y);
	set_w(w);
}

void CPoint::getPixUV(double &ui, double &vi)const
{
	ui = get_u();
	vi = get_v();
}

void CPoint::setPixUV(const double u, const double v)
{
	set_u(u);
	set_v(v);
}

void CPoint::setObjetImage(const double oX, const double oY, const double oZ, const double u, const double v)
{
	set_oX(oX);
	set_oY(oY);
	set_oZ(oZ);
	setPixUV(u,v);
}

CPoint& CPoint::operator=(const CPoint& pt)
{
	set_u(pt.get_u());
	set_v(pt.get_v());
	set_oX(pt.get_oX());
	set_oY(pt.get_oY());
	set_oZ(pt.get_oZ());
	set_X(pt.get_X());
	set_Y(pt.get_Y());
	set_Z(pt.get_Z());
	set_x(pt.get_x());
	set_y(pt.get_y());

	return *this;
}

void CPoint::getPositionFromPose(vpPoseVector pose) {
    vpHomogeneousMatrix cMo(pose);
    vpRotationMatrix cRo;
    vpTranslationVector cto, Pos;
    cMo.extract(cRo);
    cMo.extract(cto);
    Pos = (-1 * cRo.t()) * cto;
    setWorldCoordinates(Pos[0],Pos[1], Pos[2]);
}

void CPoint::getPositionFromPoseM( vpHomogeneousMatrix cMo) {
    vpRotationMatrix cRo;
    vpTranslationVector cto, Pos;
    cMo.extract(cRo);
    cMo.extract(cto);
    Pos = (-1 * cRo.t()) * cto;
    setWorldCoordinates(Pos[0],Pos[1], Pos[2]);
}
