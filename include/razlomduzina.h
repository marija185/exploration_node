//
// C++ Interface: razlomljeni_pravac
//
// Description: 
//
//
// Author: Marija Seder,,, <marija@marija-laptop>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef RAZLOMLJENI_PRAVAC_H
#define RAZLOMLJENI_PRAVAC_H
#include "duzina.h"
//#define MINSKOCNI 0.378
#define MINSKOCNI 1.2
/**
	@author Marija Seder,,, <marija@marija-laptop>
*/
class Razlomljena_Duzina
{
	public:

		vector <Tocka> A;
		vector <Tocka> B;
		Duzina nerazlomljena;
		Pravac p;		
	// Defaultni konstruktor
		Razlomljena_Duzina(Duzina AB);
		Razlomljena_Duzina();
		// Defaultni destruktor
		~Razlomljena_Duzina();
		void dodaj(Duzina AB);
		Duzina vratiDuzinu(int i);
		Pravac vratiPravac();
		
		void postaviVar(NEWMAT::Matrix P);
		void print(void);
};
		

Razlomljena_Duzina linefit3(Duzina ref, Tocka *ocitanja,int *duljina, double R_estim1=1,double alpha_estim1=1, double sigma_d=0.005, double sigma_fi=0.0001, int detectrupa=1);// Funkcija vraca duzinu 


#endif
