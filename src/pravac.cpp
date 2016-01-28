//
// C++ Implementation: pravac
//
// Description: 
//
//
// Author:  <>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifdef use_namespace
//using namespace NEWMAT;              // access NEWMAT namespace
#endif
//#define PI 3.14159265
#include "pravac.h"

Pravac::Pravac(double RR1, double thetat)
{
	R=RR1;
	theta=thetat;
	NEWMAT::Matrix P(2,2);
	P = 0.0;
	varijanca = P;
	if (R < 0) 
	{
		R = abs(R);
		theta = theta + PI ;
	}
	while (theta>2*PI) theta= theta - 2*PI;
	if (theta<0){
		//printf("theta=%f, R=%f\n",theta,R);
		//theta+=2*PI;
	}
}

Pravac::Pravac(double RR1, double thetat, NEWMAT::Matrix P)
{
	R=RR1;
	theta=thetat;
	varijanca=P; 
	if (R < 0) 
	{
	R = abs(R);
	theta = theta + PI ;
	}
	while (theta>2*PI) theta= theta - 2*PI;
}

Pravac::Pravac() { };
Pravac::~Pravac() 
{
};
NEWMAT::Matrix Pravac::operator-(Pravac drugi) 
    {
// Operator oduzimanja nad dvama pravcima je definiran kao matrica sa razlikom njihovih parametara R i theta
	    NEWMAT::Matrix Q(2,1); 
	    Q << R - drugi.R << theta - drugi.theta;
	    return Q;	
    };

bool Pravac::operator==(Pravac drugi)
	{
	if (R == drugi.R && theta == drugi.theta && varijanca == drugi.varijanca) return 1;
	return 0;
	};
	
Tocka Pravac::najbliza(Tocka A)
    {
// metoda vraca tocku najblizu pravcu (odnosno tocku na pravcu)
	    double p, m1,n1, k, l,x1,y1;
	    Tocka A1;

	    p = R;
	    m1 = A.vratiX();
	    n1 = A.vratiY();

            //kut za spec. slucajeve moze biti 0, PI, PI/2 i 3*PI/2
	    if (theta == 0)
	    {
		    A1 = Tocka(p,n1);
	    }
	    else if (theta == PI/2)
	    {
		    A1 = Tocka(m1,p);
	    }
	    else if (theta == 3*PI/2){
	    	    A1 = Tocka(m1,-p);
	    }
	    else if (theta == PI){
		    A1 = Tocka(-p,n1);	    
	    }
	    else
	    {
		    k = - cos(theta) / sin (theta);
		    l = p/sin(theta);
		    x1 = (n1+(1/k)*m1-l)/(k+1/k);
		    y1 = k*x1+l;
		    A1 = Tocka(x1,y1);
	    }

	    return A1;

    }

double Pravac::operator-(Tocka Druga) 
    {
	    Tocka p = najbliza(Druga);
	    return Druga - p;
    };

void Pravac::postaviVar(NEWMAT::Matrix P)
    {
// naknadno postavlja varijancu pravcu
	    varijanca = P;
	    return;
    }

double Pravac::vratiR(void)
    {
	    return abs(R);
    }

double Pravac::vratiTheta(void)
    {
	    if (R<0) return theta+PI;
	    return theta;
    }


Pravac Pravac::kroz2tocke(Tocka A, Tocka B) 
    {
// metoda postavlja vraća pravac kroz dvije tocke u R,theta formatu
	    double x1,x2,y1,y2,k,l,R,theta;
	    x1 = A.vratiX();
	    y1 = A.vratiY();
	    x2 = B.vratiX();
	    y2 = B.vratiY();
            
	    if (fabs(y2-y1)<0.001) //(y2-y1 == 0) 
	    { 
	       R=y2;
	       theta=PI/2;//tu je pisalo 3.14 i onda to nije nikad jednako PI/2 sto se ispituje drugdje
	    }
	    
	    else {
	       if (fabs(x2-x1)<0.001) //(x2 - x1 == 0) 
	    {
		    R = x2;
		    theta = 0 ;  
	    }
	    else
	    {
		    k = (y2-y1) / (x2-x1);
		    l = -k*x1 + y1;
		    theta = atan(-1/k);
		    R = l*sin(theta);
	    }
	    }
	    return Pravac(R,theta);
    }


NEWMAT::Matrix Pravac::vratiVarijanca(void)
    {
	    return varijanca;
    }

void Pravac::print() 
    {
	    std::cout <<"("<< R << "," << theta << ")\n"<<  10000000*varijanca;
    }
	
