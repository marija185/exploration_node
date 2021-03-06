
#include <iostream>
#define Benough 14
#include <vector>
#include <fstream>
#include "room.h"
//#include "moj.h"
#include "GridMap.h"
#include "DStar.h"

using namespace std;

struct xyzpoint {
  	    int x,y,z;
  	    } ;

  struct xypoint {
    int x,y;
    } ;

class explore3D
{

private:
	vector <char> vo;          //initialization of char vector with value 'U'
	  	//v.reserve(50);
	vector < vector <char> > dv;  // initialization of 2D array with value 'U'
	  	//dv.reserve(50);
	vector <vector<vector<char > > > tv;  // initialization of 3D array with value 'U'
	short int stepX,stepY,stepZ,X,Y,Z;
	int brojacPP, brojac;
	int xi,yi,zi;
	int si,sj,sk;
	int max;
	int zastavica;
	xypoint PParray[2500];       // potential position voxels
	xyzpoint BUarray[20000];      //boundary unseen voxels
	xyzpoint  rp;
	ifstream pose, myfile;
	float pozicijax,pozicijay,pozicijat;
	float pomz,pomx,pomy;
	float tMaxX,tMaxY,tMaxZ,tDeltaX,tDeltaY,tDeltaZ;
	double diff;
	time_t start,end;

public:

	float minbax,minbay;  //min of detected room
	bool still_room;
	void explore_room(int br_pokretanja, room Rfind, DStar *DS, GridMap *GM);
	explore3D(); //constructor
	xyzpoint np_old, np;

};
