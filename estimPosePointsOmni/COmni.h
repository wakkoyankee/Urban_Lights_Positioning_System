#ifndef AFX_COmni
#define AFX_COmni

#include "CModel.h"

class COmni : public CModel
{
public://protected:
	double xi;

public:
	COmni(double au=0,double av=0,double u0=0,double v0=0,double xi=1, double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
	~COmni();

	void init(double au,double av,double u0,double v0,double xi, double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
	double getXi(){return xi;};
	void project3DImage(CPoint &);
	void project3DSphere(CPoint & P, double & Xs, double & Ys, double & Zs);
	virtual void projectSphereImage(CPoint &);
	virtual int projectImageSphere(CPoint &, double & Xs, double & Ys, double & Zs);
	
	COmni& operator=(const COmni& cam); 

};

#endif
