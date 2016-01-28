//
// C++ Implementation: tocka
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


// Konstruktor Tocke


Tocka::Tocka(double xx, double yy=0) :
		x(xx), y(yy) { }

Tocka::Tocka() { };
Tocka::~Tocka() { };        
double Tocka::operator-(Tocka druga) 
    {
	    // operator oduzimanja nad dvije tocke je  definirana kao funkcija tipa double koja vraca njihovu udaljenost
	    return sqrt( pow((x-druga.x),2) +pow((y-druga.y),2));
    }
    Tocka Tocka::operator+(Tocka druga) 
    {
	    // operator zbrajanja vraća rezultat Točku sa zbrojenim koordinatama prve i druge točke
	    return Tocka(x+druga.x,y+druga.y);
    }
    Tocka Tocka::operator*(double skalar)
    {
    return Tocka(x*skalar, y*skalar);
    }
    
    bool Tocka::operator!=(Tocka druga) 
    {
	    if ((x!=druga.x) || (y!=druga.y)) 
	    {
		    return 1;
	    }
	    return 0;
    }
    bool Tocka::operator==(Tocka druga) 
    {
    return !(*this!=druga);
    }

    double Tocka::vratiX(void)
    {
	    return x;
    }
    double Tocka::vratiY(void)
    {
	    return y;
    }

    double Tocka::kut(Tocka T) 
    {
	    return atan2(y - T.y, x - T.x); 
    };
    
    CvPoint Tocka::convert2cvPoint(int disk)
    {
// Napomena to je isto naopako
	    return cvPoint(int(y*disk),int(x*disk));
    }

    CvPoint2D32f Tocka::convert2cvPointFloat(void)
    {
// Napomena to je isto naopako
	    return cvPoint2D32f(x,y);
    }

    void Tocka::print() 
    {
	    std::cout <<"("<< x << "," << y<< ")"; 
    };


