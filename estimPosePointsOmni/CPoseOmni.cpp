#include "CPoseOmni.h"
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

CPoseOmni::CPoseOmni()
{
}

CPoseOmni::~CPoseOmni()
{
}

void
CPoseOmni::displayFrame(vpImage<unsigned char> &I,
					vpHomogeneousMatrix &cMo,
					CModel *_cam,
					double size,
					vpColor col)
{
	COmni *cam = (COmni *)_cam;

	vpColVector Xs0(3), Xs(3), N(3), a(5);
	// used by display
	CPoint o; o.setWorldCoordinates ( 0.0,0.0,0.0 ) ;
	CPoint x; x.setWorldCoordinates ( size,0.0,0.0 ) ;
	CPoint y; y.setWorldCoordinates ( 0.0,size,0.0 ) ;
	CPoint z; z.setWorldCoordinates ( 0.0,0.0,size ) ;

	o.changeFrame(cMo) ;
	cam->project3DSphere(o, Xs0[0], Xs0[1], Xs0[2]);
	cam->project3DImage(o);
	cam->meterPixelConversion(o);

	x.changeFrame(cMo) ;
	cam->project3DSphere(x, Xs[0], Xs[1], Xs[2]);
	computeNormalFromTwoPoints(Xs0, Xs, N);
	computeConicFromNormal(N, a, cam);

	cam->project3DImage(x);
	cam->meterPixelConversion(x);
	if ( col == vpColor::none )
		displayConic(I,
				 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
				 vpMath::round ( x.get_v() ), vpMath::round ( x.get_u() ),
				 a, //ellipseParameters dans l'espace métrique
				 cam,
				 vpColor::red);
	else
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( x.get_v() ), vpMath::round ( x.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 col);

	y.changeFrame(cMo) ;
	cam->project3DSphere(y, Xs[0], Xs[1], Xs[2]);
	computeNormalFromTwoPoints(Xs0, Xs, N);
	computeConicFromNormal(N, a, cam);

	cam->project3DImage(y) ;
	cam->meterPixelConversion(y);
	if ( col == vpColor::none )
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( y.get_v() ), vpMath::round ( y.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 vpColor::green);
	else
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( y.get_v() ), vpMath::round ( y.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 col);

	z.changeFrame(cMo) ;
	cam->project3DSphere(z, Xs[0], Xs[1], Xs[2]);
	computeNormalFromTwoPoints(Xs0, Xs, N);
	computeConicFromNormal(N, a, cam);
	cam->project3DImage(z) ;
	cam->meterPixelConversion(z);
	if ( col == vpColor::none )
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( z.get_v() ), vpMath::round ( z.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 vpColor::blue);
	else
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( z.get_v() ), vpMath::round ( z.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 col);

}

void
CPoseOmni::computeNormalFromTwoPoints(vpColVector & Xs0, vpColVector & Xs, vpColVector & N)
{
	N = vpColVector::cross(Xs0, Xs);
}

void
CPoseOmni::computeConicFromNormal(vpColVector & N, vpColVector & a, COmni *cam)
{
	double xi = cam->getXi(), xi2;
	xi2 = xi*xi;

	a.resize(5);

	a[0] = (N[0]*N[0]/(N[2]*N[2]))*(1-xi2) - xi2;
	a[1] = (N[1]*N[1]/(N[2]*N[2]))*(1-xi2) - xi2;
	a[2] = (N[0]*N[1]/(N[2]*N[2]))*(1-xi2);
	a[3] = N[0]/N[2];
	a[4] = N[1]/N[2];
}

void
CPoseOmni::displayConic(vpImage<unsigned char>&I,
						  int i1,
						  int j1,
						  int i2,
						  int j2,
						  vpColVector a, //ellipseParameters dans l'espace métrique
						  CModel *cam,
						  vpColor color,
						  int l)
{
	vpColVector Pt(3), PtHat(3), PtStop(3), PtStopHat(3), PtStep(3), PtStepHat(3), axe(2), PtLast(3), PtPrecLast(3);
	CPoint CPt;
	double alpha[3], difAgl;
	vpMatrix invP(3,3);
	int rows = I.getRows() ;
	int cols = I.getCols() ;

	CPt.setPixUV(j1, i1);
	cam->pixelMeterConversion(CPt);
	Pt[0] = CPt.get_x();
	Pt[1] = CPt.get_y();
	Pt[2] = 1.0;

	CPt.setPixUV(j2, i2);
	cam->pixelMeterConversion(CPt);
	PtStop[0] = CPt.get_x();
	PtStop[1] = CPt.get_y();
	PtStop[2] = 1.0;

	axe[0] = j2 - j1;
	axe[1] = i2 - i1;
	axe.normalize();

	CPt.setPixUV(j1+axe[0], i1+axe[1]);
	cam->pixelMeterConversion(CPt);
	PtStep[0] = CPt.get_x();
	PtStep[1] = CPt.get_y();
	PtStep[2] = 1.0;

	invP = computeConic2Circle(a);

	//On passe le premier point de la conique sur le cercle unitaire
	PtHat = invP * Pt;
	for(int j = 0 ; j < 2 ; j++)
		PtHat[j] /= PtHat[2];
	PtHat[2] = 1.0;
	//Calcul de sa coordonnée angulaire dans le cercle
	alpha[0] = atan2(PtHat[1], PtHat[0]);
	if(alpha[0] < 0.0)
		alpha[0] += 2.0*M_PI;

	//On passe le dernier point de la conique sur le cercle unitaire
	PtStopHat = invP * PtStop;
	for(int j = 0 ; j < 2 ; j++)
		PtStopHat[j] /= PtStopHat[2];
	PtStopHat[2] = 1.0;
	//Calcul de sa coordonnée angulaire dans le cercle
	alpha[1] = atan2(PtStopHat[1], PtStopHat[0]);
	if(alpha[1] < 0.0)
		alpha[1] += 2.0*M_PI;

	//On passe le point step de la conique sur le cercle unitaire
	PtStepHat = invP * PtStep;
	for(int j = 0 ; j < 2 ; j++)
		PtStepHat[j] /= PtStepHat[2];
	PtStepHat[2] = 1.0;
	//Calcul de sa coordonnée angulaire dans le cercle
	alpha[2] = atan2(PtStepHat[1], PtStepHat[0]);
	if(alpha[2] < 0.0)
		alpha[2] += 2.0*M_PI;

	//Calcul de la longueur d'un échantillon
	difAgl = vpMath::abs(alpha[2] - alpha[0]);
	if(difAgl > M_PI)
		difAgl = 2.0*M_PI - difAgl;

	// Choose starting point
	double alphaS = alpha[0], n_sample;

	//définissons le nombre d'échantillons
	n_sample = vpMath::abs(alpha[1] - alpha[0]);
	if(n_sample > M_PI)
	{
		n_sample = 2.0*M_PI-n_sample;
		difAgl *= (alpha[0]<alpha[1])?(-1):1;
	}
	else
		difAgl *= (alpha[0]<alpha[1])?1:(-1);

	n_sample /= vpMath::abs(difAgl);

	vpMatrix P = invP.inverseByLU();

	vpMatrix Rot(3,3);
	Rot[0][0] = cos(difAgl); 	Rot[0][1] = -sin(difAgl);
	Rot[1][0] = sin(difAgl);	Rot[1][1] = cos(difAgl);
	Rot[2][2] = 1.0;

	for(int i = 0 ; i <= n_sample ; i++)
	{
		Pt = P * PtHat;
		for(int j = 0 ; j < 2 ; j++)
			Pt[j] /= Pt[2];
		Pt[2] = 1.0;

		CPt.set_x(Pt[0]);
		CPt.set_y(Pt[1]);
		cam->meterPixelConversion(CPt);
		Pt[0] = CPt.get_u();
		Pt[1] = CPt.get_v();

		// If point is in the image, draw it
		if(!OutOfImage(vpMath::round(Pt[1]), vpMath::round(Pt[0]), 0, rows, cols))
			vpDisplay::displayPoint(I, vpMath::round(Pt[1]), vpMath::round(Pt[0]), color);

		if (i%6 == 0)
			PtPrecLast = PtLast;

		PtLast = Pt;

		PtHat = Rot * PtHat;

		alphaS += difAgl;

	}

	vpDisplay::displayArrow ( I,
							 vpMath::round ( PtPrecLast[1] ), vpMath::round ( PtPrecLast[0] ),
							 vpMath::round ( PtLast[1] ), vpMath::round ( PtLast[0] ),
							 color, 6, 4 ) ;
}

vpMatrix
CPoseOmni::computeConic2Circle(vpColVector a)
{
	vpMatrix C(3,3), Rp(3,3), R(3,3), abc(3,3);
	vpMatrix invP(3,3);
	C[0][0] = a[0]; 	C[0][1] = a[2]; 	C[0][2] = a[3];
	C[1][0] = a[2]; 	C[1][1] = a[1]; 	C[1][2] = a[4];
	C[2][0] = a[3]; 	C[2][1] = a[4]; 	C[2][2] = 1.0;
	vpColVector EigVals(3);
	vpMatrix EigVects(3,3);
	std::cout << "toto" << std::endl;
//	//Ok si GSL
	C.eigenValues(EigVals, EigVects);
	//classement des valeurs et vecteurs propres par ordre croissant
	for(int i = 0 ; i < 3 ; i++)
		for(int j = i+1 ; j < 3 ; j ++)
		{
			if(EigVals[j] < EigVals[i])
			{
				double tmp = EigVals[j];
				EigVals[j] = EigVals[i];
				EigVals[i] = tmp;

				for(int k = 0 ; k < 3 ; k++)
				{
					tmp = EigVects[k][j];
					EigVects[k][j] = EigVects[k][i];
					EigVects[k][i] = tmp;
				}
			}
		}

/*	std::cout << "EigVals : " << EigVals.t() << std::endl;
	std::cout << "EigVects : " << EigVects << std::endl;

	CvMat CMat = cvMat(3, 3, CV_64F, C.data),
			EigValsMat = cvMat(3, 1, CV_64F, EigVals.data),
			EigVectsMat = cvMat(3, 3, CV_64F, EigVects.data);
	cvEigenVV(&CMat, &EigVectsMat, &EigValsMat, DBL_EPSILON);
	EigVects = EigVects.t();
	//classement des valeurs et vecteurs propres par ordre croissant
	for(int i = 0 ; i < 3 ; i++)
		for(int j = i+1 ; j < 3 ; j ++)
		{
			if(EigVals[j] < EigVals[i])
			{
				double tmp = EigVals[j];
				EigVals[j] = EigVals[i];
				EigVals[i] = tmp;

				for(int k = 0 ; k < 3 ; k++)
				{
					tmp = EigVects[k][j];
					EigVects[k][j] = EigVects[k][i];
					EigVects[k][i] = tmp;
				}
			}
		}

	std::cout << "EigVals : " << EigVals.t() << std::endl;
	std::cout << "EigVects : " << EigVects << std::endl;
	*/

	Rp = EigVects;

	if( (vpMath::sign(EigVals[1]) == vpMath::sign(EigVals[2])) && (vpMath::sign(EigVals[0]) != vpMath::sign(EigVals[1])))
	{//a est la valeur particulière
		vpMatrix transfRp(3,3);
		transfRp[0][2] = 1.0; transfRp[1][1] = -1.0; transfRp[2][0] = 1.0;
		R = transfRp*Rp;
		for(int i = 0; i < 3 ; i++)
			abc[i][i] = 1.0/sqrt(vpMath::abs(EigVals[2-i]));
	}
	else if( (vpMath::sign(EigVals[0]) == vpMath::sign(EigVals[2])) && (vpMath::sign(EigVals[1]) != vpMath::sign(EigVals[0])))
	{//b est la valeur particulière
		vpMatrix transfRp(3,3);
		transfRp[0][0] = -1.0; transfRp[1][2] = 1.0; transfRp[2][1] = 1.0;
		R = transfRp*Rp;
		abc[0][0] = 1.0/sqrt(vpMath::abs(EigVals[0]));
		abc[1][1] = 1.0/sqrt(vpMath::abs(EigVals[2]));
		abc[2][2] = 1.0/sqrt(vpMath::abs(EigVals[1]));
	}
	else if( (vpMath::sign(EigVals[0]) == vpMath::sign(EigVals[1])) && (vpMath::sign(EigVals[2]) != vpMath::sign(EigVals[0])))
	{//c est la valeur particulière
		R = Rp;
		for(int i = 0; i < 3 ; i++)
			abc[i][i] = 1.0/sqrt(vpMath::abs(EigVals[i]));
	}
	else
		std::cout << "ce n est pas une conique" << std::endl;

	invP = R * abc;
	invP = invP.inverseByLU();
	return invP;
}

int
CPoseOmni::OutOfImage(int i, int j, int half, int rows, int cols)
{
	return (! ((i> half+2) &&
			   (i< rows -(half+2)) &&
			   (j>half+2) &&
			   (j<cols-(half+2)))
			) ;
}

void CPoseOmni::computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls)
{
	double x = P.get_x(), y = P.get_y();
	double Z = P.get_Z(), Xi = ((COmni *)cam)->getXi() ;

	double X = P.get_X(), Y = P.get_Y();
	double rho = sqrt(X*X+Y*Y+Z*Z);

	Ls.resize(2,6);

	Ls[0][0] = -( (rho*Z + Xi*Z*Z)/(rho*vpMath::sqr(Z + Xi*rho)) + Xi*y*y/rho) ;
	Ls[0][1] = Xi*x*y/rho;
	Ls[0][2] = x * ((rho+Xi*Z) / (rho*(Z + Xi*rho)));
	Ls[0][3] = x*y;
	Ls[0][4] = -( x*x + (Z*Z + Z*Xi*rho)/(vpMath::sqr(Z + Xi*rho)) );
	Ls[0][5] = y;

	Ls[1][0] = Xi*x*y/rho;
	Ls[1][1] = -( (rho*Z + Xi*Z*Z)/(rho*vpMath::sqr(Z + Xi*rho)) + Xi*x*x/rho );
	Ls[1][2] = y * ( (rho+Xi*Z)/(rho*(Z + Xi*rho)) );
	Ls[1][3] = ( (Z*Z + Z*Xi*rho)/(vpMath::sqr(Z + Xi*rho)) ) + y*y;
	Ls[1][4] = -x*y;
	Ls[1][5] = -x;
}

int CPoseOmni::initViewLines(CModel *cam, vpColVector *sP, int nbPts)
{
	int i = 0, nbP;
	CPoint P;

  if(nbPts == -1)
    nbP = listP.nbElement();
  else
    nbP = nbPts;

	listP.front();
	while (i < nbP)
	{
		P = listP.value() ;

		sP[i] = vpColVector(3);

		((COmni *)cam)->projectImageSphere(P, sP[i][0], sP[i][1], sP[i][2]);

		listP.next() ;
		i++;
	}

	return 0;
}



static
void
calculTranslation (vpMatrix &a, vpMatrix &b, unsigned int nl, unsigned int nc1,
                   unsigned int nc3, vpColVector &x1, vpColVector &x2)
{

  try
  {
    unsigned int i,j;

    vpMatrix ct(3,nl) ;
    for (i=0 ; i < 3 ; i++)
    {
      for (j=0 ; j < nl ; j++)
        ct[i][j] = b[j][i+nc3] ;
    }

    vpMatrix c ;
    c = ct.t() ;

    vpMatrix ctc ;
    ctc = ct*c ;

    vpMatrix ctc1 ; // (C^T C)^(-1)
    ctc1 = ctc.inverseByLU() ;

    vpMatrix cta ;
    vpMatrix ctb ;
    cta = ct*a ;  /* C^T A	*/
    ctb = ct*b ;  /* C^T B	*/


    vpColVector X2(nc3)  ;
    vpMatrix CTB(nc1,nc3) ;
    for (i=0 ; i < nc1 ; i++)
    {
      for (j=0 ; j < nc3 ; j++)
        CTB[i][j] = ctb[i][j] ;
    }

    for (j=0 ; j < nc3 ; j++)
      X2[j] = x2[j] ;

    vpColVector sv ;       // C^T A X1 + C^T B X2)
    sv = cta*x1 + CTB*X2 ;// C^T A X1 + C^T B X2)



    vpColVector X3 ; /* X3 = - (C^T C )^{-1} C^T (A X1 + B X2) */
    X3 = -ctc1*sv ;



    for (i=0 ; i < nc1 ; i++)
      x2[i+nc3] = X3[i] ;
  }
  catch(...)
  {

    // en fait il y a des dizaines de raisons qui font que cette fonction
    // rende une erreur (matrice pas inversible, pb de memoire etc...)
    vpERROR_TRACE(" ") ;
    throw ;
  }


}




static
void
lagrange (vpMatrix &a, vpMatrix &b, vpColVector &x1, vpColVector &x2)
{
  try{
    unsigned int i,imin;

    vpMatrix ata ; // A^T A
    ata = a.t()*a ;
    vpMatrix btb ; // B^T B
    btb = b.t()*b ;

    vpMatrix bta ;  // B^T A
    bta = b.t()*a ;

    vpMatrix btb1 ;  // (B^T B)^(-1)

    if (b.getRows() >= b.getCols()) btb1 = btb.inverseByLU() ;
    else btb1 = btb.pseudoInverse();



    vpMatrix r ;  // (B^T B)^(-1) B^T A
    r = btb1*bta ;

    vpMatrix e ;  //   - A^T B (B^T B)^(-1) B^T A
    e = - (a.t()*b) *r ;

    e += ata ; // calcul E = A^T A - A^T B (B^T B)^(-1) B^T A



    //   vpColVector sv ;
    //    vpMatrix v ;
    e.svd(x1,ata) ;// destructif sur e
    // calcul du vecteur propre de E correspondant a la valeur propre min.

    /* calcul de SVmax	*/
    imin = 0;
    // FC : Pourquoi calculer SVmax ??????
    //     double  svm = 0.0;
    //    for (i=0;i<x1.getRows();i++)
    //    {
    //      if (x1[i] > svm) { svm = x1[i]; imin = i; }
    //    }
    //    svm *= EPS;	/* pour le rang	*/

    for (i=0;i<x1.getRows();i++)
      if (x1[i] < x1[imin]) imin = i;


    for (i=0;i<x1.getRows();i++)
      x1[i] = ata[i][imin];

    x2 = - (r*x1) ; // X_2 = - (B^T B)^(-1) B^T A X_1


  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

}


void CPoseOmni::PoseLagrange(vpHomogeneousMatrix &cMo){
    double s;
    unsigned int i;

    unsigned int k=0;
    unsigned int nl=4*2;

    vpMatrix a(nl,3)  ;
    vpMatrix b(nl,9);
    b =0 ;

    CPoint P ;
    i=0 ;
    listP.front();
    while(!listP.outside())
    {
      P = listP.value();
      a[k][0]   = -P.get_oX();
      a[k][1]   = 0.0;
      a[k][2]   = P.get_oX()*P.get_x();

      a[k+1][0] = 0.0;
      a[k+1][1] = -P.get_oX();
      a[k+1][2] = P.get_oX()*P.get_y();

      b[k][0]   = -P.get_oY();
      b[k][1]   = 0.0;
      b[k][2]   = P.get_oY()*P.get_x();

      b[k][3]   = -P.get_oZ();
      b[k][4]   =  0.0;
      b[k][5]   =  P.get_oZ()*P.get_x();

      b[k][6]   =  -1.0;
      b[k][7]   =  0.0;
      b[k][8]   =  P.get_x();

      b[k+1][0] =  0.0;
      b[k+1][1] = -P.get_oY();
      b[k+1][2] =  P.get_oY()*P.get_y();

      b[k+1][3] =  0.0;
      b[k+1][4] = -P.get_oZ();
      b[k+1][5] =  P.get_oZ()*P.get_y();

      b[k+1][6] =  0.0;
      b[k+1][7] = -1.0;
      b[k+1][8] =  P.get_y();

      k += 2;
      listP.next();
    }
    vpColVector X1(3) ;
    vpColVector X2(9) ;

    lagrange(a,b,X1,X2);

    if (X2[8] < 0.0)
    {		/* car Zo > 0	*/
      X1 *= -1 ;
      X2 *= -1 ;
    }
    s = 0.0;
    for (i=0;i<3;i++) {s += (X1[i]*X2[i]);}
    for (i=0;i<3;i++)  {X2[i] -= (s*X1[i]);} /* X1^T X2 = 0	*/

    s = 0.0;
    for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}

    s = 1.0/sqrt(s);
    for (i=0;i<3;i++)  {X2[i] *= s;}		/* X2^T X2 = 1	*/

    X2[3] = (X1[1]*X2[2])-(X1[2]*X2[1]);
    X2[4] = (X1[2]*X2[0])-(X1[0]*X2[2]);
    X2[5] = (X1[0]*X2[1])-(X1[1]*X2[0]);

    calculTranslation (a, b, nl, 3, 6, X1, X2) ;


    for (i=0 ; i<3 ; i++)
    {
      cMo[i][0] = X1[i];
      cMo[i][1] = X2[i];
      cMo[i][2] = X2[i+3];
      cMo[i][3] = X2[i+6];
    }

}



void CPoseOmni::poseInit(vpHomogeneousMatrix & cMo, CModel *cam)
{
	int i, j;
	CPoint P, Pbis;
	vpList<CPoint> lP;
	vpColVector sP[4];

	lP.kill();
	lP += listP;

    cam->pixelMeterConversion(P);

	if(initViewLines(cam, sP, 4))
	{
		cMo.eye();
		return;
	}

	vpMatrix c(4,4), sqrDistance(4,4);

	listP.front();
	i = 0;
	while(i < 3)
	{
		P = listP.value();
		lP.front();
		lP.suppress();
		j = i+1;
		while(j < 4)
		{
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

	A[0][0] = 1.0; A[0][4] = c[0][1]; A[0][7] = 1.0;      A[0][20] = sqrDistance[0][1];
	A[1][1] = 1.0; A[1][4] = 1.0;     A[1][7] = c[0][1];  A[1][21] = sqrDistance[0][1];
	A[2][5] = 1.0; A[2][8] = 1.0;     A[2][16] = c[0][1]; A[2][22] = sqrDistance[0][1];
	A[3][6] = 1.0; A[3][9] = 1.0;     A[3][17] = c[0][1]; A[3][23] = sqrDistance[0][1];

	A[4][0] = 1.0; A[4][5] = c[0][2];  A[4][11] = 1.0;      A[4][20] = sqrDistance[0][2];
	A[5][4] = 1.0; A[5][10] = 1.0;     A[5][16] = c[0][2];  A[5][21] = sqrDistance[0][2];
	A[6][2] = 1.0; A[6][5] = 1.0;      A[6][11] = c[0][2];  A[6][22] = sqrDistance[0][2];
	A[7][6] = 1.0; A[7][12] = 1.0;     A[7][18] = c[0][2];  A[7][23] = sqrDistance[0][2];

	A[8][0] = 1.0;  A[8][6] = c[0][3];    A[8][15] = 1.0;       A[8][20] = sqrDistance[0][3];
	A[9][4] = 1.0;  A[9][13] = 1.0;       A[9][17] = c[0][3];   A[9][21] = sqrDistance[0][3];
	A[10][5] = 1.0; A[10][14] = 1.0;      A[10][18] = c[0][3];  A[10][22] = sqrDistance[0][3];
	A[11][3] = 1.0; A[11][6] = 1.0;       A[11][15] = c[0][3];  A[11][23] = sqrDistance[0][3];

	A[12][7] = 1.0;  A[12][11] = 1.0;      A[12][16] = c[1][2];   A[12][20] = sqrDistance[1][2];
	A[13][1] = 1.0;  A[13][8] = c[1][2];   A[13][10] = 1.0;       A[13][21] = sqrDistance[1][2];
	A[14][2] = 1.0;  A[14][8] = 1.0;       A[14][10] = c[1][2];   A[14][22] = sqrDistance[1][2];
	A[15][9] = 1.0;  A[15][12] = 1.0;      A[15][19] = c[1][2];   A[15][23] = sqrDistance[1][2];

	A[16][7] = 1.0;  A[16][15] = 1.0;     A[16][17] = c[1][3];   A[16][20] = sqrDistance[1][3];
	A[17][1] = 1.0;  A[17][9] = c[1][3];  A[17][13] = 1.0;       A[17][21] = sqrDistance[1][3];
	A[18][8] = 1.0;  A[18][14] = 1.0;     A[18][19] = c[1][3];   A[18][22] = sqrDistance[1][3];
	A[19][3] = 1.0;  A[19][9] = 1.0;      A[19][13] = c[1][3];   A[19][23] = sqrDistance[1][3];

	A[20][11] = 1.0;    A[20][15] = 1.0; A[20][17] = c[2][3];   A[20][20] = sqrDistance[2][3];
	A[21][9] = c[2][3]; A[21][10] = 1.0; A[21][13] = 1.0;       A[21][21] = sqrDistance[2][3];
	A[22][2] = 1.0;     A[22][14] = 1.0; A[22][19] = c[2][3];   A[22][22] = sqrDistance[2][3];
	A[23][3] = 1.0;     A[23][12] = 1.0; A[23][13] = c[2][3];   A[23][23] = sqrDistance[2][3];

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

	//std::cout << Sa << std::endl;
	//exit(9);

	// Pour chaque point, on recherche sa distance au centre de projection
	double x[4] = {0.0,0.0,0.0,0.0};

	for(i = 0 ; i < 4 ; i++)
		x[i] = sqrt(fabs(Va[i][23] / Va[20+i][23]));

	//std::cout << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << std::endl;

	vpColVector cP[4], oP[4];
	// Calcul des 4 points dans le repère caméra
	listP.front();
	i = 0;
	while (i < 4)
	{
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
	X = cP[1]-cP[0]; Y = cP[2]-cP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = 0 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }

	//Deuxieme base
	X = cP[2]-cP[1]; Y = cP[3]-cP[1];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }

	//Troisieme base
	X = cP[1]-cP[0]; Y = cP[3]-cP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }

	//Quatrieme base
	X = cP[2]-cP[0]; Y = cP[3]-cP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }

	//Repère objet/monde
	//Premiere base
	X = oP[1]-oP[0]; Y = oP[2]-oP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = 0 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }

	//Deuxieme base
	X = oP[2]-oP[1]; Y = oP[3]-oP[1];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }

	//Troisieme base
	X = oP[1]-oP[0]; Y = oP[3]-oP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }

	//Quatrieme base
	X = oP[2]-oP[0]; Y = oP[3]-oP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
	{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }

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
	for(i = 0 ; i < 4 ; i++)
	{
		rcP[i] = R * oP[i];
		mrcP += rcP[i];
		mcP += cP[i];
	}
	mrcP *= 0.25;
	mcP *= 0.25;

	t = mcP-mrcP;

	vpTranslationVector cto(t[0], t[1], t[2]);
	vpRotationMatrix cRo;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			cRo[i][j]=R[i][j];
	cMo.buildFrom(cto, cRo);

	/*vpPoseVector r(cMo);
	 r.print();*/
}






