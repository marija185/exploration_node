//
// C++ Interface: duzina
//
// Description: 
//
//
// Author:  <>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef DUZINA_H
#define DUZINA_H
#include "pravac.h"
	
/**
	@author : Sandor Iles
	Sadrzi klasu Duzina te neke korisne funkcije vezano za Duzine
*/



class Duzina {
	Tocka A;
	Tocka B;
	Pravac p;
	
	public :
		int slijedeci;
		int prethodni;
		int prvi;
		int put;	
		double kut;
		Tocka goal; //kandidatna pozicija
		int stvarni;
		int odabran;
		bool nova;
		
		// Konstruktori
		Duzina(Tocka Aa, Tocka Bb);
                Duzina(Tocka Aa, Tocka Bb, bool sort);
		Duzina(Tocka Aa, Tocka Bb, NEWMAT::Matrix P);
		Duzina();
		// Defaultni destruktor
		~Duzina();
		// Metode
		double operator-(Tocka T); // Vraca udaljenost najblize tocke od duzine
		double operator-(Duzina druga); // Vraca max udaljenost krajnjih tocaka ili udaljenost pravaca ako se preklapaju
		bool operator==(Duzina druga);
		Pravac vratiPravac(void); // Vraca pravac na kojemu duzina lezi
		double duljina(void); // Vraca duljinu duzine
		Duzina VelikiRazmak(Duzina druga); // Vraca duzinu koja je najdulja medju svim kombinacijama spajanja tocaka
		Tocka vratiA(void); // Vraca tocku A
		Tocka vratiB(void); // Vraca tocku B
		Tocka poloviste(void);
		Duzina MaliRazmak(Duzina druga); // Vraca najmanji od AC AD i BC i BD
		Duzina VelikiKutMaliRazmak(Duzina druga); // Vraca najmanji od AC AD i BC i BD
		void print(void); // Ispis duzine
};

Duzina najblizaPravcu(Tocka A, Tocka B, Pravac p1); // Vraca duzinu najblizu tockama A i B koja lezi na pravcu p1
Duzina preklapajuse(Duzina AB, Duzina CD); // Vraca duzinu odredjenu sa projekcijama preklapajucih tocaka na suprotne duzine
bool uvjet_parc_preklapajuce(Duzina AB, Duzina CD,double uvjet); // Vraca da li je udaljenost preklapajuceg dijela < 
double L(double dk,double fi,double alpha,double R,double sigma_d=0.005, double sigma_fi=0.0001);

Duzina linefit2(Duzina ref, Tocka *ocitanja,int *duljina, double R_estim=1,double alpha_estim=1, double sigma_d=0.005, double sigma_fi=0.0001, int detectrupa=1);// Funkcija vraca duzinu dobivenu WLS algoritmom (Duzina sadrzi dobiveni pravac i varijancu!) 


#endif
