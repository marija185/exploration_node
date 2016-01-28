#include "duzina.h"
#include "gpc2.h"
#include "moj.h"
#include "cudo.h"
#include "func.h"

#define prag 1.8
#define MIN_AREA 8

struct ROOM {
  Duzina line;
  int oznaka;
  bool inroom;
} ;



class room {

private:
	vector <Duzina> sobaMatlab;
	vector <int> LR;
	vector <int> NP;
	int najbliza;
	int brojanje;
	int oznaka;
	void vrati_najblizu();
	vector<ROOM> soba1;      //local copy of room
	float polygon_area();
	int broj_prekida;
	float udaljenost;


public:
	gpc_polygon gpcpoly;
	vector <Duzina> poly;
	vector < vector < int > > R;
	room(); //default constructor
	vector<ROOM> soba;          // vector which contains of detected lines in environment
	//vector < vector < int > > Room_detector();          // main function for room detection
	void Room_detector(gpc_polygon &gpc_globalni_poligon, vector<Duzina> & globskocni);
	//void Room_detector();
	vector <Duzina> sobni;




};
