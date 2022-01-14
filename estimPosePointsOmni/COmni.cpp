#include "COmni.h"

COmni::COmni(double au,double av,double u0,double v0,double xii, double k1,double k2,double k3,double k4,double k5):CModel(au,av,u0,v0, k1, k2, k3, k4, k5)
{
	type = Omni;
	name = "Omni";
	xi = xii;

	nbActiveParameters++;
    nbActiveParametersBase++;
}

COmni::~COmni()
{
}

void COmni::init(double _au,double _av,double _u0,double _v0,double _xi, double _k1,double _k2,double _k3,double _k4,double _k5)
{
	CModel::init(_au,_av,_u0,_v0,_k1,_k2,_k3, _k4, _k5);
/*	au = _au;
	av = _av;
	u0 = _u0;
	v0 = _v0;
	inv_au = 1.0/au;
	inv_av = 1.0/av;*/

	type = Omni;
	name = "Omni";
	xi=_xi;

}

void COmni::project3DImage(CPoint & P)
{
	double X = P.get_X(), Y = P.get_Y(), Z = P.get_Z(), den;
	den = Z + xi * sqrt(X*X+Y*Y+Z*Z);

	P.set_x(X/den);
	P.set_y(Y/den);
}

void COmni::projectSphereImage(CPoint & P)
{

}

void COmni::project3DSphere(CPoint & P, double & Xs, double & Ys, double & Zs)
{
	double inv_norme;

	Xs = P.get_X();
	Ys = P.get_Y();
	Zs = P.get_Z();

	inv_norme = 1.0 / sqrt(Xs*Xs+Ys*Ys+Zs*Zs);

	Xs *= inv_norme;
	Ys *= inv_norme;
	Zs *= inv_norme;
}

int COmni::projectImageSphere(CPoint & P, double & Xs, double & Ys, double & Zs)
{
	double x = P.get_x(), y = P.get_y(), fact;
  double srac = 1.0+(1.0-xi*xi)*(x*x + y*y);
  if(srac >= 0)
  {
	  fact = (xi + sqrt(srac)) / (x*x + y*y + 1.0);

	  Xs = fact * x;
	  Ys = fact * y;
	  Zs = fact - xi;

    return 0;
  }
  return -1;
}

COmni& COmni::operator=(const COmni& cam)
{
	CModel::operator=(cam);
	this->xi = cam.xi;
	return *this;
}

