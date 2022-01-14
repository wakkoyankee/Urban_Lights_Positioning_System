#ifndef AFX_CModel
#define AFX_CModel

#include <iostream>
#include <string>
#include "CPoint.h"
#include <visp/vpMatrix.h>

typedef enum
{
    Omni,
    Equirect,
    Persp,
    Fisheye,
    Paraboloid
} ModelType;

class CModel
{
public:
	double au,av,u0,v0,inv_av,inv_au;
	ModelType type;
	std::string name;
	bool distorsions;

	double k[5];
	bool activek[5];
  //distorsions inverses
	double ik[5];

	int nbActiveParameters, nbActiveParametersBase;

public:
	CModel(double au=0,double av=0,double u0=0,double v0=0, double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
	~CModel();

	void init(double au=0,double av=0,double u0=0,double v0=0, double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
	void init(const CModel& c);

	void meterPixelConversion(CPoint &);
	void pixelMeterConversion(CPoint &);
	virtual void project3DImage(CPoint &)=0;

	inline ModelType getType(){return type;}
	inline std::string getName() const { return name; }
	inline double getau() const { return au; }
	inline double getav() const { return av; }
	inline double getu0() const { return u0; }
	inline double getv0() const { return v0; }

	inline double getk1() const { return k[0]; }
	inline double getk2() const { return k[1]; }
	inline double getk3() const { return k[2]; }
	inline double getk4() const { return k[3]; }
	inline double getk5() const { return k[4]; }

	inline double getik1() const { return ik[0]; }
	inline double getik2() const { return ik[1]; }
	inline double getik3() const { return ik[2]; }
	inline double getik4() const { return ik[3]; }
	inline double getik5() const { return ik[4]; }

	inline int getNbActiveParameters() const { return nbActiveParameters; }
	inline int getNbActiveDistorsionParameters() const { return nbActiveParameters-nbActiveParametersBase; }

	vpMatrix getK() const;

	inline void setDistorsions(bool d){ distorsions=d;};
	inline void setType(ModelType m){ type=m;};
	inline void setName(std::string n) { name=n; };
	void setPixelRatio(double au, double av);
	void setPrincipalPoint(double u0, double v0);
	void setDistorsionParameters(double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
    void setUndistorsionParameters(double ik1=0,double ik2=0,double ik3=0,double ik4=0,double ik5=0);
	void setActiveDistorsionParameters(bool k1=true,bool k2=true,bool k3=true,bool k4=true,bool k5=true);

	virtual CModel& operator=(const CModel& cam); //!< surcharge
	virtual std::ostream& operator << (std::ostream & os);
};
#endif
