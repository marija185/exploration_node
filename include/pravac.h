//
// C++ Interface: pravac
//
// Description: 
//
//
// Author:  <>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "tocka.h"
#ifndef PRAVAC_H
#define PRAVAC_H

/**
	Definira klasu pravac u obliku 
	R = x*cos(theta)+y*sin(theta)
        i ima atribut varijanca 

Autor : Sandor Iles
*/
class Pravac 
{
	    double R;
	    double theta;
	    NEWMAT::Matrix varijanca;

	    public:
		
   		Pravac(double RR1, double thetat);
	        Pravac(double RR1, double thetat, NEWMAT::Matrix P);
		Pravac();
		NEWMAT::Matrix operator-(Pravac drugi);
		bool operator==(Pravac drugi);
		Tocka najbliza(Tocka A);
		double operator-(Tocka Druga);
		void postaviVar(NEWMAT::Matrix P);
		double vratiR(void);
		double vratiTheta(void);
		Pravac kroz2tocke(Tocka A, Tocka B);
		NEWMAT::Matrix vratiVarijanca(void);
		void print();
		~Pravac();
};

#endif
