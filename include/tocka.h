//
// C++ Interface: tocka
//
// Description: 
//
//
// Author:  <>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
//#define ELECTRICROS 0
#include <iostream>
//#if (ELECTRICROS==1)
//#include <opencv-2.3.1/opencv2/imgproc/imgproc.hpp>
//#include <opencv-2.3.1/opencv2/imgproc/imgproc_c.h>
//#else
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
//#endif
// #include <cv.h>
#include <cmath>
//#include "newmat10/newmatap.h"                // need matrix applications
//#include "newmat10/newmatio.h"                // need matrix output routines
#include <newmat/newmatap.h>                // need matrix applications
#include <newmat/newmatio.h>                // need matrix output routines
// #include <cv.h>
#include <opencv2/highgui/highgui.hpp>
// #include <highgui.h>
// #include <cmath>
#include <fstream>

#define PI 3.14159265
# define DOMET 8
# define minLINIJA 5

#include <vector>

#ifndef GEOMETRIJATOCKA_H
#define GEOMETRIJATOCKA_H

/**
	@author 
*/
	class Tocka
	{
		protected: 
			double x;
			double y;
		public:
			Tocka(double xx, double yy);
			Tocka();
    
			double operator-(Tocka druga);
// operator oduzimanja nad dvije tocke je  definirana kao funkcija tipa double koja vraca njihovu udaljenost
			Tocka operator+(Tocka druga); 
			Tocka operator*(double skalar);
// operator zbrajanja vraća rezultat Točku sa zbrojenim koordinatama prve i druge točke
			bool operator!=(Tocka druga);
			bool operator==(Tocka druga);
			double vratiX(void);
			double vratiY(void);
			CvPoint convert2cvPoint(int disk); // Konvertira u cvPoint (int(DISK*(x,y))
			CvPoint2D32f convert2cvPointFloat(void); // Konvertira u cvPoint (float)
			double kut(Tocka T); // Vraca kut tocke (kod tocke u polarnom priakzu) 
			void print() ;
			~Tocka();
	};


	
#endif
