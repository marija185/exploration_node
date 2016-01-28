/*
 * main.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: ivan
 */

#include "func.h"
#include "moj.h"   //DStar.h, Params.h, Planner.h
#include "WorkHorse.h"    //zbog WH
#include "cudo.h"
// #include "DStar.h"    //zbog DS
#include "GridMap.h"    //zbog GM
#include "DynamicWindow.h" //zbog DW
#include <iostream>
#include <math.h>
//#include "args.h"
#include "duzina.h"

#include "ros/ros.h"
//#include "room.h"
#include "explore3D.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_srvs/Empty.h"

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"

#define Benough 14
#define GMAPPING 1
#define NORIEGL 1
#define LASER_INVERTED false
#define LASER_POINTS 720
#define ROOM_POINTS_SIZE 721
#define GRID_MAP_OCCUPANCY_TRESHOLD 1

//extern variables
WorkHorse *WH;
moj *M;
extern DStar *DS;
extern GridMap *GM;
extern DynamicWindow *DW;
extern Planner *PL;



	//  	vector <char> vo(100,'U');          //initialization of char vector with value 'U'
	  	//v.reserve(50);
	//  	vector < vector <char> > dv(100,vo);  // initialization of 2D array with value 'U'
	 // 	//dv.reserve(50);
	 // 	vector <vector<vector<char > > > tv(20,dv);  // initialization of 3D array with value 'U'



//globalne varijable za callbackove
int br_pokretanja=-1; //koliko puta je mjerio
float v, omega;
float ranges[730];
float ranges2[800];
float ranges3[800];


int lasercount=0;
int lasercount2=0;
int lasercount3=0;
float angle_min, angle_increment;
float angle_min2, angle_increment2;
float angle_min3, angle_increment3;
float cxx=0, cxy=0, cyx=0, cyy=0, cthth=0;
int lokalizirao=0;
bool go=false;
bool detroom=false;
//bool still_room=false;
bool kukoc;
tf::TransformListener *listener;
int c, b;
room Rfind;              //object for room detection
explore3D explore_in_3D;

#define ISPIS 0

double izracunajKut2(Tocka A, Duzina d){
#if (ISPIS==1)
	printf("A(%f,%f), duzina.A(%f,%f), duzina.B(%f,%f), atan2(Ay-duzina.Ay,Ax-duzina.Ax)=%f, atan2(Ay-duzina.By,Ax-duzina.Bx)=%f \n",A.vratiX(),A.vratiY(),d.vratiA().vratiX(),d.vratiA().vratiY(),d.vratiB().vratiX(),d.vratiB().vratiY(),atan2(A.vratiY() - d.vratiA().vratiY(), A.vratiX() - d.vratiA().vratiX()),atan2(A.vratiY() - d.vratiB().vratiY(), A.vratiX() - d.vratiB().vratiX()));
#endif
//svodjenje na -PI, PI
double first=atan2(A.vratiY() - d.vratiA().vratiY(), A.vratiX() - d.vratiA().vratiX());
double second=atan2(A.vratiY() - d.vratiB().vratiY(), A.vratiX() - d.vratiB().vratiX());
double difference=fabs(first-second);
if (difference>PI)
	difference=fabs(difference-2*PI);
	printf("kut=%f st\n",difference*RuS);
	return difference;
}







int kriterij2 (Razlomljena_Duzina *karta, int *br_duzina, vector<Duzina> & globskocni)
{
	float max=0.;
	int najboljiSkocni=0;
	float kut;
	float xpresjek3,ypresjek3;
	int nemoguce,dodatno;
	//

	for (int i=0; i<globskocni.size(); i++){  //petlja po svim skocnim bridovima
		kut=0;
		for (int j=0; j<globskocni.size(); j++){

			//provjera da li je moguce vidjeti sa pozicije globskocni[i].goal brid globskocni[j]
			Pravac p3;
			nemoguce=0;
			// odredivanje pravaca koji spajaju .goal i krajnje tocke skocnog brida

			if (0){
			p3=p3.kroz2tocke(globskocni[i].goal,globskocni[j].poloviste());

			for (int k=0; k< *br_duzina;k++){    // za svaku duljinu provjeriti da li je vidljiva
				if (karta[k].A.size()==0)
					continue;
				ypresjek3=(p3.vratiR()/cos(p3.vratiTheta())*cos(karta[k].p.vratiTheta())-karta[k].p.vratiR())/(tan(p3.vratiTheta())*cos(karta[k].p.vratiTheta())-sin(karta[k].p.vratiTheta()));     // presjek stvarnog pravca u prostoru i pravaca vidljivosti
				xpresjek3=(p3.vratiR()-ypresjek3*sin(p3.vratiTheta()))/cos(p3.vratiTheta());
#if (ISPIS==1)
				printf("presjek %d - te duzine sa spojnicom izmedju %d-tog kandidata i %d-tim skocnim bridom (%f,%f)\n",k,i,j,xpresjek3,ypresjek3);
#endif
				if  ((Tocka(xpresjek3,ypresjek3)-Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste() < globskocni[j].poloviste() - Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste()))
					//if (((xpresjek3 <= globskocni[i].goal.vratiX() && xpresjek3 >= globskocni[j].poloviste().vratiX() ) || ((xpresjek3 <= globskocni[j].poloviste().vratiX() && xpresjek3 >= globskocni[i].goal.vratiX() ))) && (ypresjek3 <= globskocni[i].goal.vratiY() && ypresjek3 >= globskocni[j].poloviste().vratiY() || ypresjek3 <= globskocni[j].poloviste().vratiY() && ypresjek3 >= globskocni[i].goal.vratiY()))
				{
					nemoguce=1;
					//printf("stvarni brid ispred skocnog!\n");
					dodatno=1;//ako ne udje u donju petlju
					//provjeravam koja je duljina unutar razlomljene duzine ta koja sijece pravac vidljivosti
					for (int m = 0; m<karta[k].A.size();m++){
						dodatno=0;
#if (ISPIS==1)
						printf("duzina A=(%f,%f), B=(%f,%f)\n",karta[k].A[m].vratiX(),karta[k].A[m].vratiY(),karta[k].B[m].vratiX(),karta[k].B[m].vratiY());
#endif
						if  ((Tocka(xpresjek3,ypresjek3)-karta[k].vratiDuzinu(m).poloviste() < karta[k].vratiDuzinu(m).poloviste() - karta[k].A[m]))
						{
							dodatno=1;
							//printf("unutar brida\n");
							break;//da ne vrti bezveze cijelu petlju
						}
					}
					if (dodatno==0){
						nemoguce=0;
						//printf("ipak nije (razlomljena duzina)\n");
					}
					if (nemoguce==1){
						break;//da ne vrti cijelu petlju jer je vec nasao stvarni brid ispred skocnog i od ovog nema doprinosa, i da se ne resetira zastavica zbog razlomljene duzine
					}
				}


			}
			}//if 0

			if (nemoguce==0) {
				//ako je sve ok

				//izracunaj kut i dodaj u sumator
				kut=kut+izracunajKut2(globskocni[i].goal,globskocni[j]);
			}
			else { if (i==j) {
				printf("desilo se za %d %d\n", i,j);
				// 				cout <<"desilo se za" <<i <<j <<"\n";
				//globskocni.erase(globskocni.begin()+i);
				//i--;
				kut=0;
				break;
			}
			}

		}  //petlja po j
		if ((kut>0.)&&(globskocni[i].put<OBSTACLE)){ //da ne uzme obrisanog ako ima manji put
			//if ((kut/7.0+70./globskocni[i].put)>max) {
			if ((kut/globskocni.size()+70./globskocni[i].put)>max) {
				//max=kut/7.0+70./globskocni[i].put;
				max=kut/globskocni.size()+70./globskocni[i].put;
				najboljiSkocni=i;

			}
		}
		printf("angle for %d. jump edge is %f, and path length %d, criterion2=%f, criterion3=%f \n", i,kut, globskocni[i].put, kut/7.0+70./globskocni[i].put,kut/globskocni.size()+70./globskocni[i].put);

	}
		//medo trazi najblizeg ako je najbolji van dometa lasera
		if (globskocni[najboljiSkocni].put>70){
			int min=OBSTACLE;
			for (int i=0; i<globskocni.size(); i++){
				if (globskocni[i].put<min){
					najboljiSkocni=i;
					min=globskocni[i].put;
				}
			}

		}
	return najboljiSkocni;
}

int kriterij3 (vector<Duzina> stvarni, vector<Duzina> & globskocni)
{
	float max=0.;
	int najboljiSkocni=0;
	int najboljiput=0;
int min=OBSTACLE;
	float kut;
	float xpresjek3,ypresjek3;
	int nemoguce,dodatno;
	int vidljivnajblizi=0;//provjera je li vidljiv najblizi brid iz onog s najvecom heuristikom
	int vidljivost=0;//jos jedna ista takva
			for (int i=0; i<globskocni.size(); i++){
				if (globskocni[i].put<min){
					najboljiput=i;
					min=globskocni[i].put;
				}
			}
	//
	for (int i=0; i<globskocni.size(); i++){  //petlja po svim skocnim bridovima
		kut=0;
		vidljivnajblizi=0;
		//printf("i=%d\t",i);
		for (int j=0; j<globskocni.size(); j++){

			//provjera da li je moguce vidjeti sa pozicije globskocni[i].goal brid globskocni[j]
			Pravac p3;
			nemoguce=0;
			// odredivanje pravaca koji spajaju .goal i krajnje tocke skocnog brida

			if (1){
			p3=p3.kroz2tocke(globskocni[i].goal,globskocni[j].poloviste());

			for (int k=0;k<stvarni.size();k++)
	{
	//printf("k=%d\t",k);
		if ((stvarni[k].stvarni==1)){
		//printf("stvarni\t");

				ypresjek3=(p3.vratiR()/cos(p3.vratiTheta())*cos(stvarni[k].vratiPravac().vratiTheta())-stvarni[k].vratiPravac().vratiR())/(tan(p3.vratiTheta())*cos(stvarni[k].vratiPravac().vratiTheta())-sin(stvarni[k].vratiPravac().vratiTheta()));     // presjek stvarnog pravca u prostoru i pravaca vidljivosti
				xpresjek3=(p3.vratiR()-ypresjek3*sin(p3.vratiTheta()))/cos(p3.vratiTheta());
//#if (ISPIS==1)
				//printf("presjek %d - te duzine sa spojnicom izmedju %d-tog kandidata i %d-tim skocnim bridom T=[%f,%f]\n",k,i,j,xpresjek3,ypresjek3);
//#endif
				if  ( ((Tocka(xpresjek3,ypresjek3)-Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste() < globskocni[j].poloviste() - Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste())) && ((Tocka(xpresjek3,ypresjek3)-stvarni[k].poloviste())< (stvarni[k].vratiA()-stvarni[k].poloviste())))
					//if (((xpresjek3 <= globskocni[i].goal.vratiX() && xpresjek3 >= globskocni[j].poloviste().vratiX() ) || ((xpresjek3 <= globskocni[j].poloviste().vratiX() && xpresjek3 >= globskocni[i].goal.vratiX() ))) && (ypresjek3 <= globskocni[i].goal.vratiY() && ypresjek3 >= globskocni[j].poloviste().vratiY() || ypresjek3 <= globskocni[j].poloviste().vratiY() && ypresjek3 >= globskocni[i].goal.vratiY()))
				{
					//printf("P=[%f,%f] J=[%f,%f] G=[%f,%f], P-T=%f, J-P=%f\n", Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste().vratiX(),Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste().vratiY(),globskocni[j].poloviste().vratiX(),globskocni[j].poloviste().vratiY(),globskocni[i].goal.vratiX(),globskocni[i].goal.vratiY(),((Tocka(xpresjek3,ypresjek3)-Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste()),(globskocni[j].poloviste() - Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste())),(globskocni[j].poloviste() - Duzina(globskocni[j].poloviste(),globskocni[i].goal).poloviste()));
					nemoguce=1;
					//printf("stvarni brid ispred skocnog!\n");
					dodatno=1;//ako ne udje u donju petlju
					if (nemoguce==1){
						break;//da ne vrti cijelu petlju jer je vec nasao stvarni brid ispred skocnog i od ovog nema doprinosa, i da se ne resetira zastavica zbog razlomljene duzine
					}
				}


			}
			}
			}//if 0

			if (nemoguce==0) {
				//ako je sve ok

				//izracunaj kut i dodaj u sumator
				kut=kut+izracunajKut2(globskocni[i].goal,globskocni[j]);
				if (j==najboljiput){
					vidljivnajblizi=1;
				}
			}


		}  //petlja po j
		if ((kut>0.)&&(globskocni[i].put<OBSTACLE)){ //da ne uzme obrisanog ako ima manji put
			//if ((kut/7.0+70./globskocni[i].put)>max) {
			if ((kut/globskocni.size()+70./globskocni[i].put)>max) {
				//max=kut/7.0+70./globskocni[i].put;
				max=kut/globskocni.size()+70./globskocni[i].put;
				najboljiSkocni=i;
				vidljivost=vidljivnajblizi;

			}
		}
		printf("angle for %d. jump edge is %f, and path length %d, criterion2=%f, criterion3=%f \n", i,kut, globskocni[i].put, kut/7.0+70./globskocni[i].put,kut/globskocni.size()+70./globskocni[i].put);
	}
		//medo trazi najblizeg ako je najbolji van dometa lasera
		if (vidljivost==0){
		printf("nije vidljiv najblizi, uzimam najblizeg\n");
		najboljiSkocni=najboljiput;
		}
		if (1){
		if (globskocni[najboljiSkocni].put>70){
			najboljiSkocni=najboljiput;
		}
		}
	return najboljiSkocni;
}

void SnimiPoligon (string podaci, gpc_polygon *p)
{

	// Funkcija snima poligon p u file zadan sa podaci
	int c,v;

	ofstream file_op(podaci.c_str());
	if( !file_op.is_open() )
	{

		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}

	file_op << "figure \n hold on \n rezultat = [";

	for (c= 0; c < p->num_contours; c++)
	{
		// Prolazak kroz sve konture
		for (v= 0; v < p->contour[c].num_vertices-1; v++)
		{
			// Kroz sve vrhove
			file_op << p->contour[c].vertex[v].x << " "<<p->contour[c].vertex[v].y << ";"<< p->contour[c].vertex[v+1].x << " "<<p->contour[c].vertex[v+1].y << ";";
		}
		v = p->contour[c].num_vertices-1;
		file_op << p->contour[c].vertex[v].x << " " << p->contour[c].vertex[v].y <<";"<< p->contour[c].vertex[0].x << " "<<p->contour[c].vertex[0].y << ";";

		file_op << "] \n crtaj;\n rezultat = [";;


	}
	file_op << "]"<< endl;;
	file_op.close();
}

void gpc2DuzinaPolygon(gpc_polygon *p, vector<Duzina> & polygon)
{
	// Funkcija pretvara gpcPoligon u poligon zadan sa vektorom duzina
	polygon.clear();



	int c, v;
	for (c= 0; c < p->num_contours; c++)
	{
		for (v= 0; v < p->contour[c].num_vertices-1; v++)
		{
			polygon.push_back(Duzina(Tocka(p->contour[c].vertex[v].x, p->contour[c].vertex[v].y),Tocka(p->contour[c].vertex[v+1].x, p->contour[c].vertex[v+1].y)));
		} // end for v
		v= p->contour[c].num_vertices-1;
		polygon.push_back(Duzina(Tocka(p->contour[c].vertex[v].x, p->contour[c].vertex[v].y),Tocka(p->contour[c].vertex[0].x, p->contour[c].vertex[0].y)));
	}


}

void gpc2DuzinaPolygonBrisi(gpc_polygon *p, vector<Duzina> & polygon)
{
	// Funkcija brise zatvorene konture nestvarnih iz polygona
	int c, v, nasaostvarnog;
	for (c= 0; c < p->num_contours; c++)
	{
		nasaostvarnog=0;
		for (v= 0; v < p->contour[c].num_vertices-1; v++)
		{
			Duzina AB=Duzina(Tocka(p->contour[c].vertex[v].x, p->contour[c].vertex[v].y),Tocka(p->contour[c].vertex[v+1].x, p->contour[c].vertex[v+1].y));
			for (uint brojac1 = 0; brojac1 < polygon.size();brojac1++)
			{
				if ((AB - polygon[brojac1] < 0.000001) && (fabs(AB.duljina()-polygon[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
				{
					if (polygon[brojac1].stvarni==1){
						nasaostvarnog=1;
					}
					break;
				}
			}
			if (nasaostvarnog==1)
				break;

		} // end for v

		if (nasaostvarnog==0){
			v= p->contour[c].num_vertices-1;
			Duzina AB=Duzina(Tocka(p->contour[c].vertex[v].x, p->contour[c].vertex[v].y),Tocka(p->contour[c].vertex[0].x, p->contour[c].vertex[0].y));
			for (uint brojac1 = 0; brojac1 < polygon.size();brojac1++)
			{
				if ((AB - polygon[brojac1] < 0.000001) && (fabs(AB.duljina()-polygon[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
				{
					if (polygon[brojac1].stvarni==1) {
						nasaostvarnog=1;
					}else{
						polygon.erase(polygon.begin()+brojac1);
						printf("nestvarna kontura: obrisan je %d -i brid iz polygon-a\n",brojac1);
					}
					break;
				}
			}
			if (nasaostvarnog==0){//obrisi cijelu konturu
				for (v= 0; v < p->contour[c].num_vertices-1; v++)
				{
					Duzina AB=Duzina(Tocka(p->contour[c].vertex[v].x, p->contour[c].vertex[v].y),Tocka(p->contour[c].vertex[v+1].x, p->contour[c].vertex[v+1].y));
					for (uint brojac1 = 0; brojac1 < polygon.size();brojac1++)
					{
						if ((AB - polygon[brojac1] < 0.000001) && (fabs(AB.duljina()-polygon[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
						{
							polygon.erase(polygon.begin()+brojac1);
							printf("nestvarna kontura: obrisan je %d -i brid iz polygon-a\n",brojac1);
							break;
						}
					}

				} // end for v
			}
		}
	}//po conturama


}

void gpc_add_DuzinaPolygon(gpc_polygon *p, vector<Duzina> & polygon)
{
	// Funkcija poligon zadan vektorom Duzina pretvara u gpc_poligon
	int c, v;

	p->num_contours=1;
	p->hole = new  int[p->num_contours];
	p->contour = new  gpc_vertex_list[p->num_contours];

	for (c= 0; c < p->num_contours; c++)
	{
		p->contour[c].num_vertices=polygon.size();
		p->hole[c]= 0; //FALSE; /* Assume all contours to be external */

		p->contour[c].vertex = new  gpc_vertex[p->contour[c].num_vertices];
		Tocka minTocka;
		double minD ;
		for (v= 0; v < p->contour[c].num_vertices-1; v++)
		{

			minTocka=polygon[v].vratiA();
			minD = abs(polygon[v].vratiA()-polygon[v+1].vratiA());
			if (abs(polygon[v].vratiB()-polygon[v+1].vratiA()) < minD)
			{
				minD = abs(polygon[v].vratiB()-polygon[v+1].vratiA());
				minTocka = polygon[v].vratiB();
			}
			if (abs(polygon[v].vratiA()-polygon[v+1].vratiB()) < minD)
			{
				minD = abs(polygon[v].vratiA()-polygon[v+1].vratiB());
				minTocka = polygon[v].vratiA();
			}
			if (abs(polygon[v].vratiB()-polygon[v+1].vratiB()) < minD)
			{
				minD = abs(polygon[v].vratiB()-polygon[v+1].vratiB());
				minTocka = polygon[v].vratiB();
			}

			p->contour[c].vertex[v].x=minTocka.vratiX();
			p->contour[c].vertex[v].y=minTocka.vratiY();

		} // end for v
		v = p->contour[c].num_vertices-1;
		/*
		   std::cout << "v "<< v;
		   std::cout << "c "<< c;
		   polygon[v].vratiA().print();
		   polygon[v].vratiB().print();

		   polygon[0].vratiA().print();
		   polygon[0].vratiB().print();
		   */
		minD = abs(polygon[v].vratiA()-polygon[0].vratiA());
		minTocka = polygon[v].vratiA();
		if (abs(polygon[v].vratiB()-polygon[0].vratiA()) < minD)
		{
			minD = abs(polygon[v].vratiB()-polygon[0].vratiA());
			minTocka = polygon[v].vratiB();
		}
		if (abs(polygon[v].vratiA()-polygon[0].vratiB()) < minD)
		{
			minD = abs(polygon[v].vratiA()-polygon[0].vratiB());
			minTocka = polygon[v].vratiA();
		}
		if (abs(polygon[v].vratiB()-polygon[0].vratiB()) < minD)
		{
			minD = abs(polygon[v].vratiB()-polygon[0].vratiB());
			minTocka = polygon[v].vratiB();
		}

		p->contour[c].vertex[v].x=minTocka.vratiX();
		p->contour[c].vertex[v].y=minTocka.vratiY();


	} // end for c
}

bool pointInPolygon(gpc_polygon *polygon, Tocka T)
{
	// Vraca da li se tocka nalazi u poligonu (da li pravac s obje strane sijece neparan broj bridova)
	//Tocka T = (*AB).poloviste();
	double x = T.vratiX();
	double y = T.vratiY();
	bool  oddNodes=0;
	for (c= 0; c < polygon->num_contours; c++)
	{
		int j=polygon->contour[c].num_vertices-1 ;

		for (b=0; b<polygon->contour[c].num_vertices; b++)
		{
			if (polygon->contour[c].vertex[b].y<y && polygon->contour[c].vertex[j].y>=y
					||  polygon->contour[c].vertex[j].y<=y && polygon->contour[c].vertex[b].y>y)
			{
				if (polygon->contour[c].vertex[b].x+(y-polygon->contour[c].vertex[b].y)/(polygon->contour[c].vertex[j].y-polygon->contour[c].vertex[b].y)*(polygon->contour[c].vertex[j].x-polygon->contour[c].vertex[b].x)<x)
				{
					oddNodes=!oddNodes;
				}

			}
			j=b;
		}
	}

	return oddNodes;
	//return 1;
}


/* moj::moj() {

	goal.x=MAP_GOAL_POSITION_X;
	goal.y=MAP_GOAL_POSITION_Y;
	subgoal=goal;
	start.x=MAP_START_POSITION_X;
	start.y=MAP_START_POSITION_Y;
	start.th=MAP_START_POSITION_TH;
	wld=WORLD;
	LB.scan_count=0;
	LB2.scan_count=0;
}

*/
void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{

	v=odom->twist.twist.linear.x;
	omega=odom->twist.twist.angular.z;

	//ROS_INFO("I heard: v [%f] i w [%f]", v, omega );
}

void obrada(const sensor_msgs::LaserScanConstPtr& laser_scan, const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{

	ros::Duration vrijeme;
	//ROS_INFO("Vrijeme dohvacanja pozicije iznosi %f ", (float)pose->header.stamp.nsec);
	//ROS_INFO("Vrijeme dohvacanja lasera iznosi %f ", (float)laser_scan->header.stamp.nsec);
	vrijeme=pose->header.stamp - laser_scan->header.stamp;
	//ROS_INFO("Pozvana funkcija sa razlikom u vremenu %f ", (float)vrijeme.nsec);

}



void laser(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
      int i_LS;
      int i_LSsize=laser_scan->ranges.size();

#if LASER_INVERTED
        for(i_LS=0;i_LS<i_LSsize;i_LS++)
        {
          ranges[i_LS]=laser_scan->ranges[i_LSsize-1-i_LS];
          if (ranges[i_LS]<0.5 || (ranges[i_LS]>10)) ranges[i_LS]=0;//IGNORE OBJECTS ON THE ROBOT
        }

#else
        for(i_LS=0;i_LS<i_LSsize;i_LS++)
        {
          ranges[i_LS]=laser_scan->ranges[i_LS];
          if ((ranges[i_LS]<0.5) || (ranges[i_LS]>10)) ranges[i_LS]=0;//IGNORE OBJECTS ON ROBOT
        }
#endif
        lasercount=laser_scan->ranges.size();
        angle_min=laser_scan->angle_min;
        angle_increment=laser_scan->angle_increment;


}


void laser2_room(const sensor_msgs::LaserScanConstPtr& laser_scan){
	int i_LS;
    //ROS_INFO("angle_min %f angle_max %f angle_increment %f",laser_scan->angle_min,laser_scan->angle_max,laser_scan->angle_increment);
//if ((laser_scan->ranges[0]<0.5))
//	                        ranges2[0]=10;
    for(i_LS=0;i_LS<laser_scan->ranges.size();i_LS++){
//              ROS_INFO("range[%d]=%f",i_LS,laser_scan->ranges[i_LS]);
//              uzeti angle_min za kut i dalje angle=angle+angle_increment
            ranges3[i_LS]=laser_scan->ranges[i_LS];

            if(ranges3[i_LS]>10)
		ranges3[i_LS]=10.;//ranges2[i_LS-1];
    }
    lasercount3=laser_scan->ranges.size();
    angle_min3=laser_scan->angle_min+2*M_PI/3;
    angle_increment3=laser_scan->angle_increment;


}


void laser2(const sensor_msgs::LaserScanConstPtr& laser_scan)
{

		//cout<<"stigla poruka za 2D laser " <<endl;
		//getchar();
        //ROS_INFO("\n\n2D laser from riegel %f scan_time %f", (float)laser_scan->header.stamp.nsec, laser_scan->scan_time);
        int i_LS;
        //ROS_INFO("angle_min %f angle_max %f angle_increment %f",laser_scan->angle_min,laser_scan->angle_max,laser_scan->angle_increment);
	//if ((laser_scan->ranges[0]<0.5))
	//	                        ranges2[0]=10;
        for(i_LS=0;i_LS<laser_scan->ranges.size();i_LS++){
//              ROS_INFO("range[%d]=%f",i_LS,laser_scan->ranges[i_LS]);
//              uzeti angle_min za kut i dalje angle=angle+angle_increment
                ranges2[i_LS]=laser_scan->ranges[i_LS];
		//if ((ranges2[i_LS]<0.5)&&(i_LS>0))
			if(ranges2[i_LS]<0.4)
			                        ranges2[i_LS]=10.;//ranges2[i_LS-1];
        }
        lasercount2=laser_scan->ranges.size();
        angle_min2=laser_scan->angle_min+2*M_PI/3;
        angle_increment2=laser_scan->angle_increment;
}

#if 1
void laserRiegel(const std_msgs::Int8ConstPtr& state)
{
	ROS_INFO("thermostate");
	//getchar();
	go=true;
}
#endif

bool voznja=false;// ako je true onda ide do jednog cilja pa do drugog

void laser3D (const sensor_msgs::PointCloudConstPtr& oldpoint){

	kukoc=true;
    go=true;


	bool angle_check3[721];
	bool angle_check2[721];
	 // reset ranges to 100
	 for (int j=0;j<LASER_POINTS;j++){
		 ranges3[j]=0;
		 angle_check3[j]=false;
		 angle_check2[j]=false;
		 ranges2[j]=100;
	 }




     tf::StampedTransform transform;
     float poz_x, poz_y;
 	 br_pokretanja++;
         sensor_msgs::PointCloud copy;
	 sensor_msgs::PointCloud dscan;
	 copy=*oldpoint;

	 tf::StampedTransform trans;
	 ros::Time vrijeme=ros::Time::now();
	 copy.header.stamp=vrijeme;
	 //ros::Time now=0;
	 //ros::Time now = ros::Time::now();
	 try {
	 listener->waitForTransform("/map", "/base_link",vrijeme, ros::Duration(15.0));
	 listener->lookupTransform("/map", "/base_link", vrijeme, transform);
	 //listener->lookupTransform("map", "riegl", vrijeme, trans);
	 //listener->transformPointCloud("map",trans,vrijeme,copy,dscan);
	 listener->transformPointCloud("/map",copy,dscan);
	 }
	 catch (tf::TransformException ex){
	 	 	 ROS_ERROR("%s",ex.what());
	 	 	 cout << "number of points in point cloud " << oldpoint->points.size() << endl;
	 	 	 cout << "current " << ros::Time::now()<<endl;
	 	 	 cout << "time point cloud  " << oldpoint->header.stamp << endl;
	 	 	 exit(1);
	 	 	  }

	 cout<<"stigla poruka za 3D laser " <<endl;

	poz_x=transform.getOrigin().x();
	poz_y=transform.getOrigin().y();

	detroom=true;
	//write points in file
	ofstream dpoints;
	ofstream scan_file;
	string pomstr="./df/scan"+IntToString(br_pokretanja)+".3d";           //points for 3dexploration
	string scan_show="./3dtk_scan/scan00"+IntToString(br_pokretanja)+".3d";  // points for 3dtk

        dpoints.open(pomstr.c_str());
        scan_file.open(scan_show.c_str());

       if (!scan_file.is_open())
       ROS_ERROR("cannot open scan00x file in 3dtk_scan/");
       if (!dpoints.is_open())
       ROS_ERROR("cannot open scanx file in df/");


	for (int jo=0;jo<dscan.points.size(); jo++){
	    float tempand=sqrt((dscan.points[jo].x-poz_x)*(dscan.points[jo].x-poz_x)+(dscan.points[jo].y-poz_y)*(dscan.points[jo].y-poz_y) + (dscan.points[jo].z*dscan.points[jo].z));  // distance from the laser
  		float tempana=sqrt((dscan.points[jo].x-poz_x)*(dscan.points[jo].x-poz_x)+(dscan.points[jo].y-poz_y)*(dscan.points[jo].y-poz_y)); // distance according to the x y plane
	    float con_angle=acos(tempana/tempand);

	    if (con_angle>60*3.1415/180) // virtual laser constraints
	        continue;

		dpoints<<dscan.points[jo].x <<" " << dscan.points[jo].y << " " <<dscan.points[jo].z<<endl;

	}

        dpoints.close();



        //write points in the file which can be used in 3dtk and prepare for room detection

        for (int jo=0;jo<copy.points.size(); jo++){

        	//collect points with height level 2,50 m
        	float kut=atan2(copy.points[jo].y,copy.points[jo].x)*180/3.14159;

   		 		if ((copy.points[jo].z<1.7) && (copy.points[jo].z>1.5)){ // && (kut<135) && (kut>-135)) {
        	 		                       //std::cout << "usao za visinu " << copy.points[jo].z << " i kut" << kut << std::endl;

  		 			int tempi3=floor((kut+180)*2);   // point index must be positive number (+135 or +180)
   	 		 		if (angle_check3[tempi3]==false){
        	 		 angle_check3[tempi3]=true;
        	 		 ranges3[tempi3]=sqrt((copy.points[jo].y)*(copy.points[jo].y)+(copy.points[jo].x)*(copy.points[jo].x));

        	 		 			}



        	 		 		}


		scan_file<<-100*copy.points[jo].y <<" " <<100* copy.points[jo].z << " " <<100*copy.points[jo].x<<endl;

	}

        scan_file.close();

//getchar();
}



bool kriterij(Duzina a,Duzina b)
{
	// 	 	 return a.duljina()/7.+ 7./a.put > b.duljina()/7.+ 7./b.put;//duljina puta je u m
	//zbog racunanja puta D* onda ide drukcije
	return a.duljina()/7.+ 70./a.put > b.duljina()/7.+ 70./b.put;//duljina puta je u dm
}




int update_Map(vector<Duzina> stvarni, GridMap *GM)
{
	double orientation, x_next, y_next;
	int ok=1;
	int te=0;









	          //clear polygon in old map 
	for (int i=0; i < GM->Map_Dim_X; i++)
		{
			for (int j=0; j < GM->Map_Dim_Y; j++){ 


				//GM->Map[i][j].static_cell=false; 
				//GM->Map[i][j].occupancy=0;
				//GM->Map[i][j].time_stamp=0;

				if (GM->Map[i][j].obstacle==3){

					GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1+5;
					//GM->Map[i][j].static_cell=false;
					GM->Map[i][j].time_stamp=0;
				}else{
					GM->Map[i][j].obstacle=0; 
					GM->Map[i][j].static_cell=false;
					GM->Map[i][j].time_stamp=0;
				}
			}
		}







	//write laser readings in grid map

	for(int i=0;i<M->LB.scan_count;i++){  

	if (M->LB.scan[i].r < 4000){

	GM->mapper_point_temp.x=M->LB.point[i].x;
	GM->mapper_point_temp.y=M->LB.point[i].y;

	if (GM->check_point(GM->mapper_point_temp)) 
	{
		GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1+5;
		GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].obstacle=3;
		GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true;
		GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].x=GM->mapper_point_temp.x;
		GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].y=GM->mapper_point_temp.y;
	}
	else {
	printf("izvan mape sick (%f,%f)\n",GM->mapper_point_temp.x, GM->mapper_point_temp.y);
	ok=0;
	}
	}
	}





	for (uint i=0;i<stvarni.size();i++)
	{



		orientation = atan2((stvarni[i].vratiB().vratiY() - stvarni[i].vratiA().vratiY()),(stvarni[i].vratiB().vratiX() - stvarni[i].vratiA().vratiX()));

		//first write endpoints of the line

		GM->mapper_point_temp.x = stvarni[i].vratiA().vratiX()*1000.;
		GM->mapper_point_temp.y = stvarni[i].vratiA().vratiY()*1000.;

		te=GM->check_point(GM->mapper_point_temp);
		if (te)
		{


			if (stvarni[i].stvarni==1) {
							GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1 + 5;
							GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true;


			}

			else {
							GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1;
							GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true;
			}

			GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].x=GM->mapper_point_temp.x;
			GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].y=GM->mapper_point_temp.y;
		}


		else {

			ok=0;
		}





		x_next = GM->mapper_point_temp.x + (GM->Map_Cell_Size/2) * cos(orientation);
		y_next = GM->mapper_point_temp.y + (GM->Map_Cell_Size/2) * sin(orientation);


		GM->mapper_point_temp.x = stvarni[i].vratiB().vratiX()*1000.;
		GM->mapper_point_temp.y = stvarni[i].vratiB().vratiY()*1000.;

		if (GM->check_point(GM->mapper_point_temp))
		{

			if (stvarni[i].stvarni==1) {
										GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1 + 5;
										GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true;


						}

						else {
										GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1;
										GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true;
						}

			GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].x=GM->mapper_point_temp.x;
			GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].y=GM->mapper_point_temp.y;
		}

		else {

			ok=0;
		}



		if (stvarni[i].duljina()>GM->Map_Cell_Size/2./1000.){
		while (sqrt((x_next - stvarni[i].vratiB().vratiX()*1000.) * (x_next - stvarni[i].vratiB().vratiX()*1000.) + (y_next - stvarni[i].vratiB().vratiY()*1000.) * (y_next - stvarni[i].vratiB().vratiY()*1000.)) > (GM->Map_Cell_Size/2))
		{
			GM->mapper_point_temp.x = x_next;
			GM->mapper_point_temp.y = y_next;
			if (GM->check_point(GM->mapper_point_temp))
			{

				if (stvarni[i].stvarni==1) {
											GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1 + 5;
											GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true;


							}

							else {
											GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1;
											GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true;
							}

				GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].x=GM->mapper_point_temp.x;
				GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].y=GM->mapper_point_temp.y;


				x_next += (GM->Map_Cell_Size/2) * cos(orientation);
				y_next += (GM->Map_Cell_Size/2) * sin(orientation);
// 				printf("%f %f\t",x_next,y_next);

			}
			else
			{

				ok=0;
				break;
			}
		}   //gotov while
		}

	}
	return ok;
}




void explore_2D(gpc_polygon &room_poly, bool vect, vector<Duzina> & sobni ,bool still_room, double* najbolji_Hp, Razlomljena_Duzina* novakarta, int br_duzina_nova, Razlomljena_Duzina* karta,gpc_polygon & gpc_globalni_poligon , int *br_duzina, Tocka G2, double fi2, Tocka Xp2, NEWMAT::Matrix Pg1, double* nGMP, vector<Duzina> & globskocni, vector<Duzina> & skocniIzMaxRange,vector<Duzina> & novi_polygon, int promjena)
{



	R_point start, goal, goaledge, next, pot, nextedge, nextgoal, temp_r;
	double slobodni;
	I_point start_i, goal_i;

	double Hp[4];
	double orientation2,orientation;
	vector <Duzina> novi_polygon2;
	novi_polygon2.reserve(560);
	vector <Duzina> potglobskocni;
	potglobskocni.reserve(560);
	vector <Duzina> stariskocni;
	stariskocni.reserve(560);
	//		vector <Duzina> skocni;
	//	skocni.reserve(560);
	vector <Duzina> stvarni;
	vector <Duzina> skocni;
	stvarni.reserve(560);
	skocni.reserve(560);


	Duzina ulazni;
	gpc_polygon gpc_novi_poligon;



	//Razlomljena_Duzina* novakarta; // Pointer gdje ce se spremati karta iz sadasnjih ocitanja

	if (promjena == 1)
	{

		// Iz nove karte nalazi skocne bridove
		for (int i=0;i<br_duzina_nova;i++)  //petlja po broju duzina
		{
			//novakarta[i].print();
			for (uint j = 0;j<novakarta[i].A.size();j++)              //petlja po svakoj razlomljenoj duzini unutar duzine
			{

				stvarni.push_back(novakarta[i].vratiDuzinu(j));
				stvarni[stvarni.size()-1].stvarni = 1;
			}
		}

		int prazna=0;
		if ((stvarni.size()==0))             //ako nije pronađen niti jedan stvarni brid
		{
			stvarni.insert(stvarni.end(), skocniIzMaxRange.begin(), skocniIzMaxRange.end());               //ubacuje se skocni iz maksRange
			prazna = 1;
		}


		sort(stvarni.begin(),stvarni.end(),comp);

		vector <Duzina> polygon;   //poligon koji ce sadrzavati sve stvarne i skocne bridove (bez obzira na velicinu skocnog)
		polygon.reserve(520);

		//polygon.insert(polygon.end(), stvarni.begin(), stvarni.end());
		gpc_polygon gpc_novi_poligon;
		int ii=0;
		Duzina SK1, SK2;
		SK2=Duzina(Tocka(0,0),Tocka(0,0));

		for (uint i=0;i<stvarni.size()-1;i++)     //petlja po svim stvarnim bridovima od prvog do predzadnjeg po kutu
		{

			//dodajem provjeru da li postoji brid kojem je tocka B s manjim kutom nego predhodnom bridu tocka B
			printf("i=%d, stvarni=A(%f,%f),B(%f,%f), kut(A)=%f, kut(B)=%f\n",i,stvarni[i].vratiA().vratiX(),stvarni[i].vratiA().vratiY(),stvarni[i].vratiB().vratiX(),stvarni[i].vratiB().vratiY(),stvarni[i].vratiA().kut(Tocka(0,0))*RuS,stvarni[i].vratiB().kut(Tocka(0,0))*RuS);

				Duzina AB1=Duzina(stvarni[i+1].vratiA(),stvarni[i+1].vratiB());//ponovo stvaram duzinu zato sto nisu dobro postavljeni A i B ne znam zasto, konstruktor ih dobro postavi, a ne znam ko ih pokvari, mozda razlomduzina... dobri su sad popravljena duzina!!! ovo je nepotrebno ali neka sad
				Duzina CD1=Duzina(stvarni[ii].vratiA(),stvarni[ii].vratiB());
				if (((AB1.vratiB().kut(Tocka(0,0))<CD1.vratiB().kut(Tocka(0,0))) && (fabs(AB1.vratiB().kut(Tocka(0,0))-CD1.vratiB().kut(Tocka(0,0)))<PI)) || ((AB1.vratiB().kut(Tocka(0,0))>CD1.vratiB().kut(Tocka(0,0))) && (fabs(AB1.vratiB().kut(Tocka(0,0))-CD1.vratiB().kut(Tocka(0,0)))>PI))) {
					if (0){//necu ih upisat, ipak zelim ovaj lijepi dulji brid
					if (ii==i)
						SK1=Duzina(CD1.vratiA(),AB1.vratiA());
					if (ii<i)
						SK1=Duzina(stvarni[i].vratiB(),AB1.vratiA());
					if (SK1.duljina() > 0)            // ako nije slucaj da su dva susjedna brida spojena (ne postoji skocni brid) onda se u polygon ubacuje skocni brid sa pripadnom oznakom stvarni=0
					{
						polygon.push_back(SK1);
						polygon[polygon.size()-1].stvarni = 0;

					}
					polygon.push_back(stvarni[i+1]); // u polygon se ubacuje stvarni brid
					polygon[polygon.size()-1].stvarni = 1;
					printf("ubacuje skocni brid izmedju glavnog ii=%d i malog i+1=%d i ubacuje malog\n",ii,(i+1));
					SK2=Duzina(AB1.vratiB(),CD1.vratiB());

					}
					printf("jos trazi s kim bi spojio ii=%d i ignorira malog i+1=%d\n",ii,(i+1));
			//takvu ne upisujemo u poligon
					continue;
				}


			Duzina potskocni = stvarni[ii].MaliRazmak(stvarni[i+1]); //nalazi skocni brid izmedu dvije susjedne duzine (tj. dva najbliza stvarna brida). Bridovi su sortirani prije toga po kutu

			polygon.push_back(stvarni[ii]); // u polygon se ubacuje stvarni brid
			ii=i+1; //indeks sljedeceg koji ce biti upisan
			polygon[polygon.size()-1].stvarni = 1;  // u poligon se ubacuje oznaka da je brid stvarni (tj. nije skocni) stvarni=1
			if (potskocni.duljina() > 0)            // ako nije slucaj da su dva susjedna brida spojena (ne postoji skocni brid) onda se u polygon ubacuje skocni brid sa pripadnom oznakom stvarni=0
			{
				polygon.push_back(potskocni);
				polygon[polygon.size()-1].stvarni = 0;

			}
			//polygon.push_back(stvarni[i+1]);

			if (potskocni.duljina()> MINSKOCNI) //trebadeltas
			{
				Duzina AB=Duzina(potskocni.vratiA(),potskocni.vratiB());
				AB.prvi = 0;
				AB.kut = atan2(AB.vratiA().vratiY()-AB.vratiB().vratiY(),AB.vratiA().vratiX()-AB.vratiB().vratiX())+PI/2+fi2;
				skocni.push_back(AB);              // ako skocni brid zadovoljava uvjet minimalne duljine brida onda se ubacuje u vector skocnih bridova "skocni"
				skocni[skocni.size()-1].odabran=0;

			}

			if (i == stvarni.size()-2)             //in the last step
			{
				polygon.push_back(stvarni[ii]);//ubacuje se zadnji stvarni brid u polyogon ili zadnji normalni brid (ii==i+1 ako nije bilo nekih ficleka s malim kutevima)
				polygon[polygon.size()-1].stvarni = 1;

				Duzina CD1=Duzina(stvarni[ii].vratiA(),stvarni[ii].vratiB());
				int num=0, flag=0;
				while (flag==0){
					Duzina AB1=Duzina(stvarni[num].vratiA(),stvarni[num].vratiB());
					if (((AB1.vratiB().kut(Tocka(0,0))<CD1.vratiB().kut(Tocka(0,0))) && (fabs(AB1.vratiB().kut(Tocka(0,0))-CD1.vratiB().kut(Tocka(0,0)))<PI)) || ((AB1.vratiB().kut(Tocka(0,0))>CD1.vratiB().kut(Tocka(0,0))) && (fabs(AB1.vratiB().kut(Tocka(0,0))-CD1.vratiB().kut(Tocka(0,0)))>PI))){
						printf("stvarni brid %d po redu ima manji kut za B tocku od zadnjeg\n",num);
						if ((num==0) && 0){
							SK1=Duzina(CD1.vratiA(),AB1.vratiA());
							if (SK1.duljina() > 0)            // ako nije slucaj da su dva susjedna brida spojena (ne postoji skocni brid) onda se u polygon ubacuje skocni brid sa pripadnom oznakom stvarni=0
							{
								polygon.push_back(SK1);
								polygon[polygon.size()-1].stvarni = 0;

							}//sad bi mozda trebalo obrisati taj veci stvarni brid ii (gore isto)
						}
						//trebalo bi obrisati sve ove male (za tu varijantu se odlucujem)
						for (uint brojac1 = 0; brojac1 < polygon.size();brojac1++)
						{
							if ((AB1 - polygon[brojac1] < 0.000001) && (fabs(AB1.duljina()-polygon[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
							{
								polygon.erase(polygon.begin()+brojac1);
								printf("obrisan je %d -i brid iz polygon-a\n",brojac1);
								break;
							}
						}
						if (num>0){
						SK2=stvarni[num-1].MaliRazmak(stvarni[num]);
						for (uint brojac1 = 0; brojac1 < skocni.size();brojac1++)
						{
							if ((SK2 - skocni[brojac1] < 0.000001) && (fabs(SK2.duljina()-skocni[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
							{
								skocni.erase(skocni.begin()+brojac1);
								printf("obrisan je %d -i skocni brid\n",brojac1);
								break;
							}
						}
						for (uint brojac1 = 0; brojac1 < polygon.size();brojac1++)
						{
							if ((SK2 - polygon[brojac1] < 0.000001) && (fabs(SK2.duljina()-polygon[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
							{
								polygon.erase(polygon.begin()+brojac1);
								printf("obrisan je %d -i brid iz polygon-a\n",brojac1);
								break;
							}
						}
						}

						SK2=Duzina(AB1.vratiB(),CD1.vratiB());//ovo ne trebam vise
						num++;
						if (num==stvarni.size()-2){
							flag=1;
							num=0;//za svaki slucaj ako nesto ne stima
						}
					}else{
						flag=1;
					}
				}


				//potskocni = stvarni[i+1].VelikiKutMaliRazmak(stvarni[0]);
				potskocni = stvarni[ii].MaliRazmak(stvarni[num]);//bilo je 0
				//potskocni = Duzina(stvarni[i+1].vratiB(),stvarni[0].vratiA());
				Duzina A0 = Duzina (Tocka(0,0), potskocni.vratiA());
				Duzina B0 = Duzina (Tocka(0,0), potskocni.vratiB());

				//medo: uvijek spoji prvi i zadnji stvarni brid preko skocnog
				if (potskocni.duljina()> MINSKOCNI) //trebadeltas
				{
					Duzina AB=Duzina(potskocni.vratiA(),potskocni.vratiB());
					AB.prvi = 0;
					AB.kut = atan2(AB.vratiA().vratiY()-AB.vratiB().vratiY(),AB.vratiA().vratiX()-AB.vratiB().vratiX())+PI/2+fi2;
					skocni.push_back(AB);              // ako skocni brid zadovoljava uvjet minimalne duljine brida onda se ubacuje u vector skocnih bridova "skocni"
					skocni[skocni.size()-1].odabran=0;

				}

				polygon.push_back(potskocni);
				polygon[polygon.size()-1].stvarni = 0;

				if (num>0){//sad treba obrisati onaj krivi skocni iz skocnih i iz poligona
				SK2=stvarni[num-1].MaliRazmak(stvarni[num]);
				for (uint brojac1 = 0; brojac1 < skocni.size();brojac1++)
				{
					if ((SK2 - skocni[brojac1] < 0.000001) && (fabs(SK2.duljina()-skocni[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
					{
						skocni.erase(skocni.begin()+brojac1);
						printf("obrisan je %d -i skocni brid\n",brojac1);
						break;
					}
				}
				for (uint brojac1 = 0; brojac1 < polygon.size();brojac1++)
				{
					if ((SK2 - polygon[brojac1] < 0.000001) && (fabs(SK2.duljina()-polygon[brojac1].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu
					{
						polygon.erase(polygon.begin()+brojac1);
						printf("obrisan je %d -i brid iz polygon-a\n",brojac1);
						break;
					}
				}
				}




			}
		}

		if ((prazna == 1))        //ako su stvarni bridovi iz novog ocitanja bili prazni onda dodaj skocneIzMaxRange
		{
			polygon.insert(polygon.end(), skocniIzMaxRange.begin(), skocniIzMaxRange.end());
			skocni.insert(skocni.end(), skocniIzMaxRange.begin(), skocniIzMaxRange.end());
			// tu treba pripaziti da bude stvarni = 0;
			skocniIzMaxRange.clear();                    //zasto se brise skocni iz maxrange
		}
		// Transformira stvarne bridove i nadjene skocne u g. k.s

		//sort(polygon.begin(),polygon.end(),comp);

		SnimiSkocne ("lokpolygon.m", polygon);
		polygon=transformiraj3(polygon, G2,fi2,&Xp2,Pg1); //u globalnim koordinatama
		// only real edges for room detection
		sobni.clear();
		for (int real=0;real<polygon.size();real++){
			if (polygon.at(real).stvarni==1)
			{
				sobni.push_back(polygon[real]);
			}
		}
		SnimiSkocne ("globpolygon.m", polygon);         //snima sve bridove od zadnjeg scana u globalnom k.s.
		gpc_add_DuzinaPolygon(&gpc_novi_poligon,polygon);  //pretvara poligon u gpc format, ima broj kontura 1
		// Stvoren novi poligon iz trenutnih ocitanja
		vector<Duzina> transformirani=transformiraj3(skocni, G2,fi2,&Xp2,Pg1);     //skocni bridovi u glob. koor. sustavu





		// if a jump edge is the same as real line from previous map then discard it

		for (uint brojac1 = 0; brojac1 < transformirani.size();brojac1++)
				{

					for (uint brojac2 = 0; brojac2 < novi_polygon2.size();brojac2++)
					{

		if (transformirani[brojac1] - novi_polygon2[brojac2] < 0.01) {
			transformirani.erase(transformirani.begin()+brojac1);
			brojac1--;


				}
					}
						}




		stariskocni.clear();
		stariskocni.insert(stariskocni.end(), globskocni.begin(),globskocni.end());
		globskocni.insert (globskocni.end(), transformirani.begin(),transformirani.end());    //jump edges in global coordinate frame
		//na globskocni iz prethodnog koraka nadovezuju se skocni iz trenutnog ocitanja
		printf("num_contours: gpc_globalni_poligon %d, gpc_novi_poligon %d\n",gpc_globalni_poligon.num_contours,gpc_novi_poligon.num_contours);
		//if (gpc_globalni_poligon.num_contours>100)
		//	gpc_globalni_poligon.num_contours=0;

                SnimiPoligon ("staripoligon.m", &gpc_globalni_poligon);
		SnimiPoligon ("novipoligon.m", &gpc_novi_poligon);








		if (gpc_globalni_poligon.num_contours > 0)   //ako postoji gpc_globalni_poligon onda se radi unija
		{
			gpc_polygon_clip(GPC_UNION, &gpc_novi_poligon, &gpc_globalni_poligon, &gpc_globalni_poligon);

		}
		else
		{
			gpc_globalni_poligon = gpc_novi_poligon;

		}
		//		 void gpc2DuzinaPolygon(gpc_polygon *p, vector<Duzina> & polygon)

		gpc2DuzinaPolygon(&gpc_globalni_poligon, novi_polygon2);
		SnimiSkocne ("unija.m", novi_polygon2);
		novi_polygon.insert(novi_polygon.end(), polygon.begin(), polygon.end());

		//novi_polygon je sve nalijepljeno skupa: novi_polygon iz prosle iteracije plus polygon iz trenutnog ocitanja, novi_polygon2 je lijepa unija trenutnog poligona i prethodnog, gpc_novi_poligon je napravljen iz polygon od trenutnog ocitanja (u glob. k.s.), a gpc_globalni_poligon je iz proslih iteracija i tu je sad vec spremljen najnoviji poligon
		//lijepa unija novi_polygon2 mu sluzi da zna koji brid treba maknut iz karte i zato usporedjuje nalijepljeni novi_polygon s poligonom dobivenim unijom novi_polygon2

		SnimiSkocne ("prijeunija.m", novi_polygon); //sve skup nalijepljeno

		cout << "proslo unije i sve " << endl;
		//getchar();

		for (uint brojac1 = 0; brojac1 < novi_polygon.size();brojac1++)         //petlja po svim bridovima za jedno snimanje (u globalnom k.s.)
		{
			int za_brisat = 1;
			Razlomljena_Duzina AB;
			for (uint brojac2 = 0; brojac2 < novi_polygon2.size();brojac2++)      //petlja po svim bridovima za cijelu kartu (a ne samo za jedno snimanje) u globalnom k.s.
			{
				//zapravo trazi koji su zajednicki bridovi u nalijepljenom poligonu i u lijepom unija poligonu - te ostavlja, a ostale brise iz novi_polygon
				if (novi_polygon[brojac1] - novi_polygon2[brojac2] < 0.00001) //s ovim uvjetom nadje sve paralelne bridove i trpa ih u AB, zato je zakomentiran break dolje
				//if ((novi_polygon[brojac1] - novi_polygon2[brojac2] < 0.00001)&& (fabs(novi_polygon[brojac1].duljina()-novi_polygon2[brojac2].duljina())<0.00001) ) //micem razlomduzinu i dodajem ekstra uvjet za nalazenje istih linija
				{
					//printf("nadjen isti brid!\n");
					za_brisat = 0;
					AB.dodaj(novi_polygon2[brojac2]);                        //dodaje se duzina iz globalne karte u AB
//prepisujem je li brid stvaran za upis u GM
				        novi_polygon2[brojac2].stvarni = novi_polygon[brojac1].stvarni;             // oznaka da li je brid stvarni ili skocni ostaje
				        //AB1=Duzina(novi_polygon2[brojac2].vratiA(),novi_polygon2[brojac2].vratiB());
					//break;//odkomentiravam break jer je samo jedna ista duzina (bilo je zakomentirano)
				}
			}
			if (za_brisat == 1)               //ukoliko nije pronađena niti jedna bliska duzina
			{
				//printf("nema ovog, brise ga!\n");
				novi_polygon.erase(novi_polygon.begin()+brojac1);
				brojac1--;                                              //brojac pokazuje na sljedecu duzinu zbog brisanja prethodne
			}
			else                                //ukoliko je pronađena ista duzina
			{
				int stvarni1 = novi_polygon[brojac1].stvarni;
				if (1){//zakomentiravam ovo, vec postoji brid u novi_polygon
				novi_polygon[brojac1]=AB.vratiDuzinu(0);              // prepisuje iz unije u ovu, iako je to isto
				novi_polygon[brojac1].stvarni = stvarni1;             // oznaka da li je brid stvarni ili skocni ostaje
				}
				if (stvarni1 == 0) //bez provjere za duljinu brida && novi_polygon[brojac1].duljina() > MINSKOCNI)
					potglobskocni.push_back(novi_polygon[brojac1]);
				if (1){//isto, nema vise razlom duzine
				for (uint brojac3=1;brojac3<AB.A.size();brojac3++)
				{
					novi_polygon.push_back(AB.vratiDuzinu(brojac3));
					novi_polygon[novi_polygon.size()-1].stvarni = stvarni1;
					if (stvarni1 == 0) //bez provjere za duljinu && novi_polygon[brojac1].duljina() > MINSKOCNI)
						potglobskocni.push_back(novi_polygon[novi_polygon.size()-1]);
				}
				}
			}
		}
		SnimiSkocne ("potglobskocni.m", potglobskocni);
//trazenje koji skocni bridovi su sad izmisljeni - oni koji se ne nalaze u globskocni, a nalaze se u potglobskocni i treba izbrisati iz potglobskocni i to postaje globskocni na kraju
		for (uint brojac1 = 0; brojac1 < potglobskocni.size();brojac1++)
		{
			int za_brisat = 1;
			for (uint brojac2 = 0; brojac2 < globskocni.size();brojac2++)
			{
				if ((globskocni[brojac2] - potglobskocni[brojac1] < 0.000001))// && (fabs(potglobskocni[brojac1].duljina()-globskocni[brojac2].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu i imaju zajednicku tocku
				{
					potglobskocni[brojac1].kut = globskocni[brojac2].kut;
					potglobskocni[brojac1].odabran=globskocni[brojac2].odabran;

					za_brisat=0;
				}


			}
			//usporedjuje je li skocni brid zapravo stvarni brid iz karte (svi poligoni su samo vanjstina, a karta sadrzi sve stvarne bridove)
			// Ovo ce ubrzavati male sobe, a usporavati cijelu kartu
			if (za_brisat==0 && vect){
			for (int K1 = 0;K1<*br_duzina;K1++)
			{

				for (uint K = 0; K<karta[K1].A.size();K++)
				{
//					if (potglobskocni[brojac1]-karta[K1].vratiDuzinu(K) < 0.01 && potglobskocni[brojac1].duljina()<=karta[K1].vratiDuzinu(K).duljina())
					if ((potglobskocni[brojac1]-karta[K1].vratiDuzinu(K).vratiA()<0.2) &&(potglobskocni[brojac1]-karta[K1].vratiDuzinu(K).vratiB()<0.2) && (karta[K1].vratiDuzinu(K)-potglobskocni[brojac1].vratiA()<0.2) && (karta[K1].vratiDuzinu(K)-potglobskocni[brojac1].vratiB()<0.2) && (fabs(potglobskocni[brojac1].duljina()-karta[K1].vratiDuzinu(K).duljina())<0.2))
					{
						za_brisat = 1;
						printf("brise skocni brid jer je to stvarni brid u karti\n");
						break;
					}
				}
			}
			}

			if (za_brisat == 1)
			{
//				printf("brise skocni brid jer je to stvarni brid u karti\n");


				potglobskocni.erase(potglobskocni.begin()+brojac1);
				brojac1--;
			}


		}
    //na kraju brisem premale bridove (u eksperimentu je ostao jedan mali brid, ne
    //znam
    //zasto)

		// remove jump edges inside detected room since 3D exploration takes care of them
		if (still_room && vect){
		for (int jump=0;jump<potglobskocni.size();jump++){
			if (pointInPolygon(&room_poly, potglobskocni[jump].poloviste())){
					potglobskocni.erase(potglobskocni.begin()+jump);
			jump--;
			}
		}
		}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//ovdje dodati brisanje skocnih bridova unutar detektirane sobe
/*    for (int jump2;jump2<jump_erase.size();jump2++){
		for (int jump=0;jump<potglobskocni.size();jump++){
			if (jump_erase[jump2] - potglobskocni[jump] < 0.001){
					potglobskocni.erase(potglobskocni.begin()+jump);
					break;
			}
		}
		}
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		globskocni.clear();//samo se potglobskocni upisuju
		globskocni.insert(globskocni.end(), potglobskocni.begin(),potglobskocni.end());


    //je bio krivi i sad ga postavljam na globalni poligon
		SnimiSkocne ("posunija.m", novi_polygon); //trebalo bi biti jednako unija.m jer je novi_polygon sad jednak novi_polygon2 jer se obrisalo sve sto nije isto - ali nije isti jer nesto ne stima s razlomduzinom
                		//gpc2DuzinaPolygon(&gpc_globalni_poligon, novi_polygon);//medo, novi_polygon
                //brisanje zatvorenih kontura samo od nestvarnih bridova
                gpc2DuzinaPolygonBrisi(&gpc_globalni_poligon, novi_polygon2);

    novi_polygon.clear();//medo i linija ispod
    novi_polygon.insert(novi_polygon.end(),novi_polygon2.begin(),novi_polygon2.end());
		SnimiPoligon ("poligon.m", &gpc_globalni_poligon);
		SnimiSkocne ("unijabrisani.m", novi_polygon);

		SnimiSkocne ("skocniPB.m", potglobskocni);//izbaceni skocni unutar poligona (isto ko i globskocni.m samo su ti dolje posortirani)





		stvarni=transformiraj3(stvarni, G2,fi2,&Xp2,Pg1);
		skocni=transformiraj3(skocni, G2,fi2,&Xp2,Pg1);

		SnimiSkocne ("stvarni.m", stvarni);
		SnimiSkocne ("skocni.m", skocni);//skocni bridovi samo od ovog mjerenja
		SnimiSkocne ("globskocniprijebrisanja.m", globskocni);
		SnimiSkocne ("stariskocni.m", stariskocni);

               //medo novi_polygon2 je lokalna varijabla sluzi za upis u GM i tu cu zamaskirati stvarni=0 u 1 za sve one koji nisu globskocni -zasto? - to mi je trebalo prije dok jos nisam upisivala u gm sve i stvarne i skocne
               if (1){//ukljucujem ponovo jer se odabrala mjerna pozicija preblizu zidu (ocito je bio proglasen stvarni==0)
                for (uint brojac1 = 0; brojac1 < novi_polygon2.size();brojac1++)
                {
                        int stvarni1=1;
                        for (uint brojac2 = 0; brojac2 < globskocni.size();brojac2++)
                        {
                                if ((globskocni[brojac2] - novi_polygon2[brojac1] < 0.000001)&& (fabs(novi_polygon2[brojac1].duljina()-globskocni[brojac2].duljina())<0.00001)) //dodajem ovo za duljinu jer je prvi uvjet ispunjen za sve duzine koje su na istom pravcu mozda sad to nece biti dobro ako u globskocni i novi_polygon2 nisu isti bridovi nego razlicitih duljina a na istim pravcima
                                {


                                        stvarni1=0;
                                        break;
                                }


                        }
                        novi_polygon2[brojac1].stvarni = stvarni1;
                }
                }//end if 0




		//upis stvarnih bridova u gridmapu za planiranje, upisuju se kao staticke prepreke, a sve dinamicko se brise (ono skupljeno za vrijeme voznje, okretanja)
		 if ((vect)&&(promjena==1)){
		//GM->reset();//obavezan je!!!!! resetira pokretne prepreke, inace se dogodi da se nesto isprazni iz dstar mape

		if (update_Map(novi_polygon2, GM))//bio je prije stvarni))
		{
			printf("jupi\n");
		}else{
			printf("couldn't fill grid map\n");
//			exit(1);
		}
		}

	} // Kraj if-a ako ima promjene
	//ovaj dio ponavljam dolje ali s resetom! prvo ce provjeriti vidljivost a onda dohvatljivost (zbog pogreske lokalizacije da losa ocitanja ne zablokiraju put)
		if ((promjena==0) && (vect) ){
		GM->reset(); //reset cuva staticke prepreke, obrise sve ostalo
		}
		if (vect){
		DS->reset();//isprazni cijelu mapu
		printf("DS->prviput=%d\n",DS->prviput);
#if RECTANGULAR
    GM->reset(); //treba da se upisu prepreke u cspace
#endif		
		PL->reset();//ovdje puni DStar mapu preprekama, prepisuje iz GM i prosiruje za masku
		int path_found;
		DS->prviput=2;//sad je tu cijeli dio koji je dolje zakomentiran, u sekvenci_izvodjenja
		//odometrija - preuzima iz klase M
		WH->Ucitaj_stanje_odometrije(); //mora se ovo napraviti jer postavlja WH->RB koristi PL
		//
		//      //ocitanja lasera - preuzima iz klase M
		WH->Ucitaj_stanje_lasera(); //ako se zeli jos ekstra upisati ocitanja lasera
		path_found=PL->Sekvenca_izvodjenja();

	/*	if(path_found!=1){//(!(GM->fill_Map())){
			printf("Ne radi nesto s planerom!");
			exit(1);
		}
		*/



	start.x=1000*G2.vratiX();//ovo je pozicija lasera u g.k.s
	start.y=1000*G2.vratiY();
	GM->check_point(start);
	start_i=GM->cell_point_temp;

	printf("start od G2 (%f,%f) int (%d,%d) Start iz PL (%d,%d)\n",start.x,start.y,start_i.x,start_i.y,DS->Start.x,DS->Start.y);
	start_i=DS->Start;
		} //end vect

//dodajem dodatnu provjeru je li stvarni brid ispred skocnog, ili iza, preko laserskih ocitanja i postotka vidljivosti brida iz paralelnog brida, trazenja sjecista okomitih spojnica
//brisanje skocnih koji se preklapaju s ocitanjima lasera
if (vect){
        for (uint i=0;i<globskocni.size();i++)   //petlja za svaki globalni skocni brid
        {
		int notintersect=0; //zastavica je li postoji spojnica koja nema laser point
		int nmbintersect=0;
		int intersect=0;
		Duzina A1B = globskocni[i];
		double kut = globskocni[i].vratiPravac().vratiTheta();
		double distmpa=100.; //kolko je udaljen paralelni brid
		Tocka T = A1B.poloviste() + Tocka(0.01*cos(kut),0.01*sin(kut));
		if (pointInPolygon(&gpc_globalni_poligon, T))         //ako je tocka unutar poligona
		{
			globskocni[i].kut = kut+PI;
		}
		else
		{
			globskocni[i].kut = kut;
		}
				Hp[0]=1000*globskocni[i].vratiA().vratiX();
				Hp[1]=1000*globskocni[i].vratiA().vratiY();
				Hp[2]=1000*globskocni[i].vratiB().vratiX();
				Hp[3]=1000*globskocni[i].vratiB().vratiY();
		next.x=Hp[0];
		next.y=Hp[1];
		nextedge.x=next.x-distmpa*cos(globskocni[i].kut);
		nextedge.y=next.y-distmpa*sin(globskocni[i].kut);
		if (GM->check_point(next)){           // if the point is within map dimensions
			orientation = atan2((Hp[3] - Hp[1]),(Hp[2] - Hp[0]));
			orientation2 = globskocni[i].kut;

				printf("next=[%f,%f], nextedge=[%f,%f], orientation=%f deg, orientation2=%f deg\n",next.x,next.y,nextedge.x,nextedge.y,orientation*RuS,orientation2*RuS);


				while (sqrt((next.x - Hp[0]) * (next.x - Hp[0]) + (next.y - Hp[1]) *
        (next.y - Hp[1])) < globskocni[i].duljina()*1000.)
				{

					if (1)
					{
					//prvo potrazimo intersect izmedju next i nextedge
					intersect=0;//resetiram za sljedecu potragu
					temp_r.x = nextedge.x;// + (GM->Map_Cell_Size/2) * cos(orientation2); //tocka se pomice
					temp_r.y = nextedge.y;// + (GM->Map_Cell_Size/2) * sin(orientation2);
					//printf("spojnica next nextedge: temp_r=[%f,%f], orientation=%f deg\n",temp_r.x,temp_r.y,orientation2*RuS);
					while (sqrt((temp_r.x - nextedge.x) * (temp_r.x - nextedge.x) + (temp_r.y - nextedge.y) *
		(temp_r.y - nextedge.y)) < distmpa+1*GM->Map_Cell_Size)//tolko su udaljeni uvijek plus jos 3 polja s druge strane (iza)
					{
						if (GM->check_point(temp_r))  // is within map size
						{
							if((GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy>GRID_MAP_OCCUPANCY_TRESHOLD +5))//&& (GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true))
							{
								intersect=1;
								nmbintersect++;
								printf("found laser point between the next point of the paralel edge and of the next on the edge\n");
								break;
							}
						}
						temp_r.x += (GM->Map_Cell_Size/2) * cos(orientation2);
						temp_r.y += (GM->Map_Cell_Size/2) * sin(orientation2);
						//printf("temp_r=[%f,%f]\t",temp_r.x,temp_r.y);
					}

					if (intersect==0)
						notintersect++;//postaviti da postoji zraka bez prepreke
					}//kraj if 0


					next.x += (GM->Map_Cell_Size/2) * cos(orientation);
					next.y += (GM->Map_Cell_Size/2) * sin(orientation);
					nextedge.x=next.x-distmpa*cos(globskocni[i].kut);
					nextedge.y=next.y-distmpa*sin(globskocni[i].kut);

					//printf("next=[%f,%f], nextedge=[%f,%f]\t",next.x,next.y,nextedge.x,nextedge.y);
				} // end while
		}//checkpoint
				slobodni=(double)notintersect/((double)nmbintersect+notintersect)*globskocni[i].duljina();
				printf("nmbintersect=%d, notintersect=%d, slobodni=%f\n",nmbintersect,notintersect,slobodni);

			if (slobodni<0.5*globskocni[i].duljina()) //MINSKOCNI) //0.5
			{
				printf("\n\n too much obstacle in front of the jump edge length=%f, deleting. notintersect=%d, nmbintersect=%d, slobodni=%f\n",globskocni[i].duljina(),notintersect,nmbintersect,slobodni);
				globskocni.erase(globskocni.begin()+i);
				i = i-1;
			}
		} // end for po globskocnima
	} //end if 1
	//kraj brisanja skocnih koji se preklapaju s ocitanjima lasera
	if (vect){
	for (uint i=0;i<globskocni.size();i++)	 //petlja za svaki globalni skocni brid
		// 	 {
		// 		  globskocni[i].put = globskocni[i].poloviste()-G2;
		// 	 }
		// 	for (int i=0;i<0;i++)
	{
	//ovo mi je provjera jel radi dobro udaljenost bridova za brisanje dolje kad je put obstacle
		int newjump=0;
		int nasao=0;

		if (promjena==1){//treba da bi se nasao koji je novi
			int nasaostarog=0;


			for (uint J=0; J<stariskocni.size(); J++){
				printf("stari skocni duljina=%f brid=[%f,%f;%f,%f], udaljenost duzina=%f, udaljenost glob.tocke A=%f, udaljenost glob.tocke B=%f\n",stariskocni[J].duljina(),stariskocni[J].vratiA().vratiX(), stariskocni[J].vratiA().vratiY(), stariskocni[J].vratiB().vratiX(), stariskocni[J].vratiB().vratiY(),(globskocni[i] - stariskocni[J]),(stariskocni[J]-globskocni[i].vratiA()),(stariskocni[J]-globskocni[i].vratiB()));
				if ((globskocni[i] - stariskocni[J] < 0.000001)&&(stariskocni[J]-globskocni[i].vratiA()<0.00001) &&(stariskocni[J]-globskocni[i].vratiB()<0.00001) && (globskocni[i]-stariskocni[J].vratiA()<0.00001) && (globskocni[i]-stariskocni[J].vratiB()<0.000001))
				{
					nasaostarog=1;
					break;
				}

			}
			if (nasaostarog==0){
					printf("evo ga, to je novi skocni\n");
					newjump=1;
			}
		}
		int notintersect=0; //zastavica je li postoji spojnica koja nema laser point
		int nmbintersect=0;
		Duzina A1B = globskocni[i];
		double kut = globskocni[i].vratiPravac().vratiTheta();
		double distmp=1200; //distance from the jump edge
		Tocka T = A1B.poloviste() + Tocka(0.01*cos(kut),0.01*sin(kut));
		if (pointInPolygon(&gpc_globalni_poligon, T))         //ako je tocka unutar poligona
		{
			globskocni[i].kut = kut+PI;
		}
		else
		{
			globskocni[i].kut = kut;
		}
    goal.th=globskocni[i].kut;
    next.th = goal.th;
    nextgoal.th = goal.th;
		//odabir mjerne pozicije pola metra ispred polovista brida
		goal.x=1000*globskocni[i].poloviste().vratiX()-distmp*cos(globskocni[i].kut);
		goal.y=1000*globskocni[i].poloviste().vratiY()-distmp*sin(globskocni[i].kut);
		goaledge.x=1000*globskocni[i].poloviste().vratiX();
		goaledge.y=1000*globskocni[i].poloviste().vratiY();
				Hp[0]=1000*globskocni[i].vratiA().vratiX();
				Hp[1]=1000*globskocni[i].vratiA().vratiY();
				Hp[2]=1000*globskocni[i].vratiB().vratiX();
				Hp[3]=1000*globskocni[i].vratiB().vratiY();

		//prije je bilo cisto poloviste
		// 		 goal.x=1000*globskocni[i].poloviste().vratiX();
		// 		 goal.y=1000*globskocni[i].poloviste().vratiY();
		if (GM->check_point(goal)){
			goal_i=GM->cell_point_temp;
			printf("\n%d. jump edge, length=%f m, goal=[%f,%f,%f] goal_i=[%d,%d,%d] odabran=%d\n",i,globskocni[i].duljina(),goal.x,goal.y,goal.th,goal_i.x,goal_i.y,goal_i.th,globskocni[i].odabran);
//ovdje provjera jel goal goaledge sijeku laser point - ako sijeku onda trazi dalje, zastavica notintersect==1 ako ima negdje rupe izmedju next i nextedge inace obrisi brid. ne smije se uzeti mp na mjestu gdje je intersect==1
			int intersect=0;

#if DSTAR3D
	if (DS->PathCostOri(goal_i)<OBSTACLE && (DS->howmanyOriCollides(goal_i)==0) &&(DS->map[goal_i.x][goal_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#else
				if((DS->map[goal_i.x][goal_i.y].tag!=NEW)&&(DS->map[goal_i.x][goal_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
				{
					nasao=1;
					nextgoal=next;
					printf("nasao! PathCostOri=%d\n",DS->PathCostOri(goal_i));
				}

			//if(DS->IsValid(GM->cell_point_temp.x, GM->cell_point_temp.y)==2)
#if DSTAR3D
	if (DS->PathCostOri(goal_i)==OBSTACLE ) // && (DS->howmanyOriCollides(goal_i)>0))
#else
			if((DS->map[goal_i.x][goal_i.y].tag==NEW)||(DS->map[goal_i.x][goal_i.y].traversal_cost>=EMPTYC+COST_MASK-2))// || (intersect==1)|| 1)//uvijek , ne vise
#endif
			{
				int nasaopot=0,nasaopotint=0;
				printf("no path to the new measurement position. searching on the left side of the edge\n");
				 printf("brid=[%f,%f; %f,%f]\n",Hp[0],Hp[1],Hp[2],Hp[3]);

				//orientation = atan2((Hp[1] - goal.y),(Hp[0] - goal.x));
				orientation = globskocni[i].kut+PI/2;//paralelno skocnom bridu
				next.x = goal.x + (GM->Map_Cell_Size/2) * cos(orientation); //tocka se pomice
				next.y = goal.y + (GM->Map_Cell_Size/2) * sin(orientation);
				nextedge.x = goaledge.x + (GM->Map_Cell_Size/2) * cos(orientation); //tocka se pomice ovo mi ne treba
				nextedge.y = goaledge.y + (GM->Map_Cell_Size/2) * sin(orientation);//samo za provjeru
				printf("next=[%f,%f], nextedge=[%f,%f], orientation=%f deg\n",next.x,next.y,nextedge.x,nextedge.y,orientation*RuS);


				//while (sqrt((next.x - Hp[0]) * (next.x - Hp[0]) + (next.y - Hp[0+1]) * (next.y - Hp[0+1])) > 3.0*(GM->Map_Cell_Size))
				while (sqrt((next.x - goal.x) * (next.x - goal.x) + (next.y - goal.y) *
        (next.y - goal.y)) < globskocni[i].duljina()/2.*1000.)
				{


					if (((intersect==0)||1) && (nasao==0) && (GM->check_point(next)))
					{
      			goal_i=GM->cell_point_temp;

#if DSTAR3D
	if (DS->PathCostOri(goal_i)<OBSTACLE  && (DS->howmanyOriCollides(goal_i)==0) &&(DS->map[goal_i.x][goal_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#else
						if
            ((DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag!=NEW)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
						{
							nasao=1;
							nextgoal=next;
							printf("nasao!\n");
							//break;
						} // end if
#if DSTAR3D
	if ((nasaopot==0) && (nasao==0) && (DS->PathCostOri(goal_i)<OBSTACLE) && (DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost<EMPTYC+COST_MASK+1))
#else
						if ((nasaopot==0) && (nasao==0) && (DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag!=NEW) && (DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif						
						{

							pot=next;
							nasaopot=1;
							printf("potencijalni, nije bas odmaknut: pot(%f,%f)\n",pot.x,pot.y);
						}
						if (nasao==0){//zavrti okolinu
							I_point temp_i, pot_i;
								temp_i.th=goal_i.th;
							for (int d=0;d<8;d++)
							{
								temp_i.x=GM->cell_point_temp.x+xofs[d];
								temp_i.y=GM->cell_point_temp.y+yofs[d];
#if DSTAR3D
	if (DS->PathCostOri(temp_i)<OBSTACLE && (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#else
								if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
								{
									GM->cell_point_temp.x=temp_i.x;
									GM->cell_point_temp.y=temp_i.y;
									nasao=1;
									printf("found free distanced for 1 cell\n");
									break;
								}
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0)&&(DS->PathCostOri(temp_i)<OBSTACLE) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#else
								if ((nasaopot==0) && (nasaopotint==0)&& (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
								{
									pot_i=temp_i;
									nasaopotint=1;
									printf("potencijalni okolo\n");
								}
							}
							if (nasao==0)
							{
								for (int d=0;d<16;d++)
								{
									temp_i.x=GM->cell_point_temp.x+x2ofs[d];
									temp_i.y=GM->cell_point_temp.y+y2ofs[d];
									//      if (DS->IsValid(temp_i.x, temp_i.y)==1)
#if DSTAR3D
	if (DS->PathCostOri(temp_i)<OBSTACLE && (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK -2))
#else
									if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
									{
										GM->cell_point_temp.x=temp_i.x;
										GM->cell_point_temp.y=temp_i.y;
										nasao=1;
										printf("found free distanced for 2 cells\n");
										break;
									}
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0)&&(DS->PathCostOri(temp_i)<OBSTACLE) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#else
									if ((nasaopot==0) && (nasaopotint==0)&& (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
									{
										pot_i=temp_i;
										nasaopotint=1;
										printf("potencijalni okolo\n");
									}

								}
							}
              if (nasao==0)
              {
                for (int d=0;d<24;d++)
                {
                  temp_i.x=GM->cell_point_temp.x+x3ofs[d];
                  temp_i.y=GM->cell_point_temp.y+y3ofs[d];

#if DSTAR3D
	if (DS->PathCostOri(temp_i)<OBSTACLE && (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK -2))
#else
                  if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
                  {
                    GM->cell_point_temp.x=temp_i.x;
                    GM->cell_point_temp.y=temp_i.y;
                    nasao=1;
                    printf("found free distanced for 3 cells\n");
                    break;
                  }
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0)&&(DS->PathCostOri(temp_i)<OBSTACLE) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1) )
#else
		  if ((nasaopot==0) &&(nasaopotint==0)&& (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
		  {
			  pot_i=temp_i;
			  nasaopotint=1;
			  printf("potencijalni okolo\n");
		  }

                }
              }
              if (nasao==0)
              {
                for (int d=0;d<32;d++)
                {
                  temp_i.x=GM->cell_point_temp.x+x4ofs[d];
                  temp_i.y=GM->cell_point_temp.y+y4ofs[d];

#if DSTAR3D
	if (DS->PathCostOri(temp_i)<OBSTACLE && (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#else
                  if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
                  {
                    GM->cell_point_temp.x=temp_i.x;
                    GM->cell_point_temp.y=temp_i.y;
                    nasao=1;
                    printf("found free distanced for 4 cells\n");
                    break;
                  }
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0)&&(DS->PathCostOri(temp_i)<OBSTACLE)  &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#else
		  if ((nasaopot==0) && (nasaopotint==0) && (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
		  {
			  pot_i=temp_i;
			  nasaopotint=1;
			  printf("potencijalni okolo\n");
		  }

                }
              }

							if ((nasao==1))//da se dobiju realne koordinate
							{
								nextgoal.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
								nextgoal.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								printf("goal(%f,%f)\n",nextgoal.x,nextgoal.y);
								//break;
							}else if ((nasaopot==0) && (nasaopotint==1)){
								pot.x=pot_i.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
								pot.y=pot_i.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								printf("pot(%f,%f)\n",pot.x,pot.y);
								nasaopot=1;

							}
						}//end zavrti okolinu
					} // end if checkpoint

					//tu sad pitati vidljivost za nadjenog da mu damo sansu da nadje novog, ali ne jos
					if (nasao==1){//vracam break da ne radi bezveze
						break;
					}

					next.x += (GM->Map_Cell_Size/2) * cos(orientation);
					next.y += (GM->Map_Cell_Size/2) * sin(orientation);
//					printf("next=[%f,%f]\t",next.x,next.y);
				} // end while


				if ((nasao==0) || 0) //stavljam 0 umjesto 1 da ne radi bezveze
				{
					if (nasao==0)
					printf("Left not found, searching on the right side.\n");
					//orientation = atan2((Hp[0+3] - goal.y),(Hp[0+2] - goal.x));
					orientation = globskocni[i].kut-PI/2;//paralelno skocnom bridu

					next.x = goal.x + (GM->Map_Cell_Size/2) * cos(orientation);
					next.y = goal.y + (GM->Map_Cell_Size/2) * sin(orientation);
					printf("next=(%f,%f), orientation=%f deg\n",next.x,next.y,orientation*RuS);

				//	while (sqrt((next.x - Hp[0+2]) * (next.x - Hp[0+2]) + (next.y - Hp[0+3]) * (next.y - Hp[0+3])) > 3.0*(GM->Map_Cell_Size))
					while (sqrt((next.x - goal.x) * (next.x - goal.x) + (next.y - goal.y)
          * (next.y - goal.y)) < globskocni[i].duljina()/2.*1000.)
					{
						if (0) //necu to
						{
						//prvo potrazimo intersect izmedju next i nextedge
						intersect=0;//resetiram za sljedecu potragu
						temp_r.x = next.x + (GM->Map_Cell_Size/2) * cos(orientation2); //tocka se pomice
						temp_r.y = next.y + (GM->Map_Cell_Size/2) * sin(orientation2);
						printf("spojnica next nextedge: temp_r=(%f,%f), orientation=%f deg\n",temp_r.x,temp_r.y,orientation2*RuS);
						while (sqrt((temp_r.x - next.x) * (temp_r.x - next.x) + (temp_r.y - next.y) *
			(temp_r.y - next.y)) < distmp+2*GM->Map_Cell_Size)//tolko su udaljeni uvijek plus polje da se uzme i to
						{
							if (GM->check_point(temp_r))
							{
								if((GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy>GRID_MAP_OCCUPANCY_TRESHOLD +5))//&& (GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true))
								{
									intersect=1;
									nmbintersect++;
									printf("found laser point between the next point of the paralel edge and of the next on the edge\n");
									break;
								}
							}
							temp_r.x += (GM->Map_Cell_Size/2) * cos(orientation2);
							temp_r.y += (GM->Map_Cell_Size/2) * sin(orientation2);
							//printf("temp_r=[%f,%f]\t",next.x,next.y);
						}
						if (intersect==0)
							notintersect++;//postaviti da postoji zraka bez prepreke
					}//od if 0
						if (((intersect==0)||1) && (nasao==0) && (GM->check_point(next)))
						{
						      			goal_i=GM->cell_point_temp;
#if DSTAR3D
	if (DS->PathCostOri(goal_i)<OBSTACLE && (DS->howmanyOriCollides(goal_i)==0) &&(DS->map[goal_i.x][goal_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#else
						//	if (DS->IsValid(GM->cell_point_temp.x, GM->cell_point_temp.y)==1)
							if((DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag!=NEW)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
							{
								nasao=1;
								nextgoal=next;
								//break;
							}
#if DSTAR3D
	if ((nasaopot==0) && (nasao==0) &&(DS->PathCostOri(goal_i)<OBSTACLE) && (DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost<EMPTYC+COST_MASK+1) )
#else
              if ((nasaopot==0) && (nasao==0) && (DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag!=NEW)&& (DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
          {

			      pot=next;// novi goal
			      nasaopot=1;
			      printf("potencijalni, nije bas odmaknut: pot(%f,%f)\n",pot.x,pot.y);
		      }
	      if (nasao==0){//zavrti okolinu
		      I_point temp_i,pot_i;
		      temp_i.th=goal_i.th;
		      for (int d=0;d<8;d++)
		      {
			      temp_i.x=GM->cell_point_temp.x+xofs[d];
			      temp_i.y=GM->cell_point_temp.y+yofs[d];
#if DSTAR3D
	if ((DS->PathCostOri(temp_i)<OBSTACLE)&& (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2) )
#else
			      //if (DS->IsValid(temp_i.x, temp_i.y)==1)
			      if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
			      {
				      GM->cell_point_temp.x=temp_i.x;
				      GM->cell_point_temp.y=temp_i.y;
				      nasao=1;
				      printf("found free distanced for 1 cell\n");
				      break;
			      }
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0) &&(DS->PathCostOri(temp_i)<OBSTACLE) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#else
                  if ((nasaopot==0) && (nasaopotint==0)&& (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
                  {
                          pot_i=temp_i;
                          nasaopotint=1;
                          printf("potencijalni okolo\n");
                  }

		      }
		      if (nasao==0)
		      {
			      for (int d=0;d<16;d++)
			      {
				      temp_i.x=GM->cell_point_temp.x+x2ofs[d];
				      temp_i.y=GM->cell_point_temp.y+y2ofs[d];
#if DSTAR3D
	if ((DS->PathCostOri(temp_i)<OBSTACLE) && (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#else
				      //      if (DS->IsValid(temp_i.x, temp_i.y)==1)
				      if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
				      {
					      GM->cell_point_temp.x=temp_i.x;
					      GM->cell_point_temp.y=temp_i.y;
					      nasao=1;
					      printf("found free distanced for 2 cells\n");
					      break;
				      }
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0) &&(DS->PathCostOri(temp_i)<OBSTACLE) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#else
                  if ((nasaopot==0) && (nasaopotint==0)&& (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
                  {
                          pot_i=temp_i;
                          nasaopotint=1;
                          printf("potencijalni okolo\n");
                  }

			      }
		      }
              if (nasao==0)
              {
                for (int d=0;d<24;d++)
                {
                  temp_i.x=GM->cell_point_temp.x+x3ofs[d];
                  temp_i.y=GM->cell_point_temp.y+y3ofs[d];
#if DSTAR3D
	if ((DS->PathCostOri(temp_i)<OBSTACLE) && (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK -2))
#else
                  //      if (DS->IsValid(temp_i.x, temp_i.y)==1)
                  if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
                  {
                    GM->cell_point_temp.x=temp_i.x;
                    GM->cell_point_temp.y=temp_i.y;
                    nasao=1;
                    printf("found free distanced for 3 cells\n");
                    break;
                  }
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0) &&(DS->PathCostOri(temp_i)<OBSTACLE) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1) )
#else
		  if ((nasaopot==0) && (nasaopotint==0)&& (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
		  {
			  pot_i=temp_i;
			  nasaopotint=1;
			  printf("potencijalni okolo\n");
		  }

		}
              }
              if (nasao==0)
              {
                for (int d=0;d<32;d++)
                {
                  temp_i.x=GM->cell_point_temp.x+x4ofs[d];
                  temp_i.y=GM->cell_point_temp.y+y4ofs[d];
                  //      if (DS->IsValid(temp_i.x, temp_i.y)==1)
#if DSTAR3D
	if ((DS->PathCostOri(temp_i)<OBSTACLE) && (DS->howmanyOriCollides(temp_i)==0) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK - 2) )
#else
                  if((DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK-2))
#endif
                  {
                    GM->cell_point_temp.x=temp_i.x;
                    GM->cell_point_temp.y=temp_i.y;
                    nasao=1;
                    printf("found free distanced for 4 cells\n");
                    break;
                  }
#if DSTAR3D
	if ((nasaopot==0) && (nasaopotint==0) &&(DS->PathCostOri(temp_i)<OBSTACLE) &&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1) )
#else
                  if ((nasaopot==0) && (nasaopotint==0)&& (DS->map[temp_i.x][temp_i.y].tag!=NEW)&&(DS->map[temp_i.x][temp_i.y].traversal_cost<EMPTYC+COST_MASK+1))
#endif
                  {
                          pot_i=temp_i;
                          nasaopotint=1;
                          printf("potencijalni okolo\n");
                  }

                }
              }

		      if ((nasao==1))//da se dobiju realne koordinate
		      {
			      nextgoal.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
			      nextgoal.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
			      printf("goal(%f,%f)\n",nextgoal.x,nextgoal.y);
			      //break;
		      }else if ((nasaopot==0) && (nasaopotint==1)){
			      pot.x=pot_i.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
			      pot.y=pot_i.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
			      printf("pot(%f,%f)\n",pot.x,pot.y);
			      nasaopot=1;
		      }

	      }//okolina

						}//checkpoint
						if (nasao==1){//vracam break
							break;
						}
						next.x += (GM->Map_Cell_Size/2) * cos(orientation);
						next.y += (GM->Map_Cell_Size/2) * sin(orientation);
//						printf("next=[%f,%f]\t",next.x,next.y);
					} // end while
				} // end if nasao = 0
				if (nasao==1){
					goal=nextgoal;// novi goal
					printf("goal(%f,%f)\n",goal.x,goal.y);
				}else if (nasaopot==1){
					nasao=1;
					goal=pot;
					printf("uzima potencijalnog goal(%f,%f)\n",goal.x,goal.y);
				}





			} // end is not reachable cell-point temp


			if ((nasao==1)||(newjump==1)||1) //stari ce imati nesto upisano, a novi moraju dobiti nesto
			{
			//upis kandidata u brid
			globskocni[i].goal=Tocka(goal.x/1000.,goal.y/1000.);//sad je u metrima
			//ako nije nasao slobodnu mjernu poziciju ostat ce zapisana ona prva

			GM->check_point(goal);//ko bi to pratio, ponovo postavljam
			goal_i=GM->cell_point_temp;//ak ne nadje slobodnu tu ce bit ona prva pozicija i put ce bit OBSTACLE
			// 		 DS->resetzasearch();

			if (DS->Init(goal_i.x, goal_i.y, start_i.x, start_i.y)==1){//okrenula start cilj
				// 		 if (DS->Init(start_i.x, start_i.y, goal_i.x, goal_i.y)==1)
				// 			 DS->SearchPath();//nije potrebno pretrazivati
#if DSTAR3D
	if ((DS->PathCostOri(goal_i)==OBSTACLE)) // && (DS->howmanyOriCollides(goal_i)>0))
#else
				if((PL->path=(DS->getPath()))==NULL) //prviput==2 dio se izvrsava
#endif
				{
					printf("Nema puta!\n");
					globskocni[i].put = OBSTACLE;
				}
				else
				{
#if DSTAR3D
	        globskocni[i].put = DS->PathCostOri(goal_i)/COSTSTRAIGHT;
	        printf("goal=[%f,%f,%f] goal_i=[%d,%d,%d], s koliko orijentacija je u sudaru %d, cijena puta %d\n",goal.x,goal.y,goal.th,goal_i.x,goal_i.y,goal_i.th, DS->howmanyOriCollides(goal_i), DS->PathCostOri(goal_i));
#else
					globskocni[i].put = DS->getPathLength();//u broju polja (odn. decimetrima)
#endif
				}
			}
			else
			{
				//duljina_mp=OBSTACLE;
				globskocni[i].put = OBSTACLE;
			}
			}
			// 		 DS->resetzasearch();
		}//od GM->checkpoint
		else //ako je van karte
		{
			globskocni[i].put=OBSTACLE;
		}

		//tu ga brisemo ako nema slobodnog sjecista
		//ako nema slobodnog dijela brida duljine MINSKOCNI

		//slobodni=(double)notintersect/((double)nmbintersect+notintersect)*globskocni[i].duljina();
		//printf("nmbintersect=%d, notintersect=%d, slobodni=%f\n",nmbintersect,notintersect,slobodni);
		//if (slobodni<0.5) //MINSKOCNI)
		//{
		//	printf("\n\nto much obstacle infront of jump edge length=%f, deleting. notintersect=%d, nmbintersect=%d, slobodni=%f\n",globskocni[i].duljina(),notintersect,nmbintersect,slobodni);
		//	globskocni.erase(globskocni.begin()+i);
		//	i = i-1;
		//}else if (globskocni[i].put<OBSTACLE) {
		if ((nasao==0)&&1){
		goal.x= globskocni[i].goal.vratiX()*1000.;
		goal.y= globskocni[i].goal.vratiY()*1000.;
		}
		if ((nasao==1)||(newjump==1)||1) {
		//provjera vidljivosti odabrane mjerne pozicije i brida
		//opet brojim sve isto
			nmbintersect=0;
			notintersect=0;
			orientation = atan2((Hp[3] - Hp[1]),(Hp[2] - Hp[0]));

				next.x = Hp[0] + (GM->Map_Cell_Size/2) * cos(orientation); //tocka se pomice
				next.y = Hp[1] + (GM->Map_Cell_Size/2) * sin(orientation);
				double distancenext;

			while (sqrt((next.x - Hp[0]) * (next.x - Hp[0]) + (next.y - Hp[1]) *
        (next.y - Hp[1])) < globskocni[i].duljina()*1000.)
				{

					//prvo potrazimo intersect izmedju next i nextedge
					int intersect=0;//resetiram za sljedecu potragu
					orientation2 = atan2((next.y - goal.y),(next.x - goal.x));
//					temp_r.x = goal.x + (GM->Map_Cell_Size/2) * cos(orientation2); //tocka se pomice
//					temp_r.y = goal.y + (GM->Map_Cell_Size/2) * sin(orientation2);
					temp_r.x = goal.x; //pocetna
					temp_r.y = goal.y;
					distancenext=sqrt((next.x - goal.x)*(next.x - goal.x)+(next.y - goal.y)*(next.y - goal.y))+ 2*GM->Map_Cell_Size;
					printf("spojnica next nextedge: temp_r=[%f,%f], next=[%f,%f], orientation=%f deg\n",temp_r.x,temp_r.y,next.x,next.y, orientation2*RuS);
					while (sqrt((temp_r.x - goal.x) * (temp_r.x - goal.x) + (temp_r.y - goal.y) *
		(temp_r.y - goal.y)) < distancenext)//tolko su udaljeni plus malo vise da se uhvati zid iza ako ga ima
					{
						if (GM->check_point(temp_r))
						{
							if((GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].occupancy>GRID_MAP_OCCUPANCY_TRESHOLD +5))//&& (GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].static_cell=true))
							{
								intersect=1;
								nmbintersect++;
								printf("found laser point between the next point on the edge and the candidate goal position: temp_r=[%f,%f]\t",temp_r.x,temp_r.y);
								break;
							}
						}
						temp_r.x += (GM->Map_Cell_Size/2) * cos(orientation2);
						temp_r.y += (GM->Map_Cell_Size/2) * sin(orientation2);
						//printf("temp_r=[%f,%f]\t",next.x,next.y);
					}

					if (intersect==0)
						notintersect++;//postaviti da postoji zraka bez prepreke

						next.x += (GM->Map_Cell_Size/2) * cos(orientation);
						next.y += (GM->Map_Cell_Size/2) * sin(orientation);
						//printf("next=[%f,%f]\t",next.x,next.y);
			}//end while
				slobodni=(double)notintersect/((double)nmbintersect+notintersect)*globskocni[i].duljina();
				printf("nmbintersect=%d, notintersect=%d, slobodni=%f\n",nmbintersect,notintersect,slobodni);

			if (slobodni<0.5*globskocni[i].duljina()) //MINSKOCNI)
			{
				printf("\n\nto much obstacle infront of jump edge length=%f, deleting. notintersect=%d, nmbintersect=%d, slobodni=%f\n",globskocni[i].duljina(),notintersect,nmbintersect,slobodni);
				globskocni.erase(globskocni.begin()+i);
				i = i-1;
			}
		}

	} // end for po globskocnima

// ako je bila losa lokalizacija, resetiraj ocitanja, ostavi samo poligon i ponovo izracunaj put do ciljeva
#if 0
		GM->reset(); //reset cuva staticke prepreke, obrise sve ostalo
		DS->reset();//isprazni cijelu mapu
		printf("DS->prviput=%d\n",DS->prviput);
		//PL->reset();//ovdje puni DStar mapu preprekama, prepisuje iz GM i prosiruje za masku
		int path_found;
		DS->prviput=2;//iscrpno do starta
		path_found=PL->Sekvenca_izvodjenja();
		if(path_found!=1){//(!(GM->fill_Map())){
			printf("Ne radi nesto s planerom!");
			exit(1);
		}
		for (uint i=0;i<globskocni.size();i++)
		{
			goal.x= globskocni[i].goal.vratiX()*1000.;
			goal.y= globskocni[i].goal.vratiY()*1000.;
			goal.th= globskocni[i].kut;
			GM->check_point(goal);
			goal_i=GM->cell_point_temp;

			if (DS->Init(goal_i.x, goal_i.y, start_i.x, start_i.y)==1){//okrenula start cilj
#if DSTAR3D
	if ((DS->PathCostOri(goal_i)==OBSTACLE))
#else
				if((PL->path=(DS->getPath()))==NULL) //prviput==2 dio se izvrsava
#endif
				{
					printf("Nema puta!\n");
					globskocni[i].put = OBSTACLE;
				}
				else
				{
#if DSTAR3D
	        globskocni[i].put =DS->PathCostOri(goal_i)/COSTSTRAIGHT;
#else
					globskocni[i].put = DS->getPathLength();//u broju polja (odn. decimetrima)
#endif
				}
			}
			else
			{
				globskocni[i].put = OBSTACLE;
			}
		}
#endif

	//WH->Logger();//medo ocu vidjeti te strelice
	//DS->resetzasearch();//resetira stvari za pretrazivanje
	//	 double nGMP[3]; // Odabrana mjerna pozicija
  DW->naCilju=false; //postavio ga je Planner.cpp zbog starta jednako cilju

	sort(globskocni.begin(),globskocni.end(),kriterij);
	//SnimiSkocne ("globskocni.m", globskocni);//svi skocni bridovi posortirani po kriteriju
	for (int I=0;I<globskocni.size();I++)
	{

		//double goal_tolerance=sqrt((globskocni[I].goal.vratiX()-WH->RB.x/1000.)*(globskocni[I].goal.vratiX()-WH->RB.x/1000.)+(globskocni[I].goal.vratiY()-WH->RB.y/1000.)*(globskocni[I].goal.vratiY()-WH->RB.y/1000.));//jer je G2 pozicija senzora, treba biti prava pozicija, i treba biti udaljeno dvije goal_tol jer ce ga prebaciti u promjena==0 ako se nije pomakao za jednu goal_tol
		double old_goal_tolerance=sqrt((globskocni[I].goal.vratiX()-nGMP[0]/1000.)*(globskocni[I].goal.vratiX()-nGMP[0]/1000.)+(globskocni[I].goal.vratiY()-nGMP[1]/1000.)*(globskocni[I].goal.vratiY()-nGMP[1]/1000.));//novi cilj ne smije bit pre blizu trenutnoj poziciji, kao sto put mora bit manji od 7 tako i ovo recimo pol metra

		if ((old_goal_tolerance<GOAL_POSITION_TOLERANCE/1000.)&&(promjena==0)){//necemo ga brisati nego samo postaviti u beskonacno
			globskocni[I].put=OBSTACLE;
		}
		if (   (globskocni[I].duljina()<MINSKOCNI) ) { // || ((promjena==1))){
			//brisanje
			//printf("not good jump edge duljina=%f, MINSKOCNI=%f, put=%d, goal_tolerance=%f\n",globskocni[I].duljina(),MINSKOCNI,globskocni[I].put,goal_tolerance);
			globskocni.erase(globskocni.begin()+I);
			//globskocni[I].put=OBSTACLE;
			I = I-1;
			continue;
		}
	/*	for (int m=0; m<WH->num_mp; m++){    // mp array of old measurement positions
			goal_tolerance=sqrt((globskocni[I].goal.vratiX()-WH->mp[m].x/1000.)*(globskocni[I].goal.vratiX()-WH->mp[m].x/1000.)+(globskocni[I].goal.vratiY()-WH->mp[m].y/1000.)*(globskocni[I].goal.vratiY()-WH->mp[m].y/1000.));
			if (goal_tolerance<2*GOAL_POSITION_TOLERANCE/1000.){
				printf("too close to old mp, deleting duljina=%f, MINSKOCNI=%f, put=%d, goal_tolerance=%f\n",globskocni[I].duljina(),MINSKOCNI,globskocni[I].put,goal_tolerance);
				globskocni.erase(globskocni.begin()+I);
				I = I-1;
				continue;
			}
		}
	*/
		if (globskocni[I].odabran==2){
			globskocni.erase(globskocni.begin()+I);
			I = I-1;
			continue;
		}
		if ((globskocni[I].put==OBSTACLE)&& (promjena==1) && 0){//nove skocne brisemo
			int za_brisat = 0;
			for (uint J=0; J<stariskocni.size(); J++){
				if ((globskocni[I] - stariskocni[J] < 0.000001)&&(stariskocni[J]-globskocni[I].vratiA()<0.00001) &&(stariskocni[J]-globskocni[I].vratiB()<0.00001) && (globskocni[I]-stariskocni[J].vratiA()<0.00001) && (globskocni[I]-stariskocni[J].vratiB()<0.000001))
				{
					za_brisat=1;
					break;
				}

			}
			if (za_brisat==0){
				//brisanje
				globskocni.erase(globskocni.begin()+I);
				I = I-1;
			}

		}
	}
	SnimiSkocne ("globskocni.m", globskocni);//svi skocni bridovi posortirani po kriteriju

	int najbolji;
	//najbolji=kriterij2(karta, br_duzina, globskocni);
	//printf("najbolji za kriterij2 =%d\n",najbolji);
	najbolji=kriterij3(novi_polygon2, globskocni);
	printf("najbolji za kriterij3 =%d\n",najbolji);
	printf("broj skocnih%d\n",globskocni.size());
	//provjera da li se trenutno ne moze do najboljeg (ako je stari brid i zaklonjen zbog pogresne lokalizacije)
	//ne treba ovo jer se u HALT resetira
	if (globskocni.size()>0){
	nGMP[0]=globskocni[najbolji].goal.vratiX()*1000.;
	nGMP[1]=globskocni[najbolji].goal.vratiY()*1000.;
	nGMP[2] = globskocni[najbolji].kut;
	if ((globskocni[najbolji].odabran==1) && (still_room==false)){
		globskocni[najbolji].odabran=2;
	}else if (still_room==false) {
		globskocni[najbolji].odabran=1;
	}

	najbolji_Hp[0]=1000*globskocni[najbolji].vratiA().vratiX();
	najbolji_Hp[1]=1000*globskocni[najbolji].vratiA().vratiY();
	najbolji_Hp[2]=1000*globskocni[najbolji].vratiB().vratiX();
	najbolji_Hp[3]=1000*globskocni[najbolji].vratiB().vratiY();
	}
  	if ((globskocni.size()==0)||(globskocni[najbolji].put==OBSTACLE)){
		nGMP[0]=0.0;
		nGMP[1]=0.0;
		nGMP[2]=0.0;
		najbolji_Hp[0]=0.0;
		najbolji_Hp[1]=0.0;
		najbolji_Hp[2]=0.0;
		najbolji_Hp[3]=0.0;
	}
	//if (globskocni.size()==0) exit(1);
	if (0 && (globskocni[najbolji].put==OBSTACLE)){
		printf("no free measurement positions\n");
		exit(1);
	}



	}
}














int promjena=1;
bool stigao_nGMP=true;

int main(int argc, char** argv)
{

 	  struct xyzpoint {
  	    int x,y,z;
  	    } ;
  	  struct xypoint {
  	    int x,y;
  	    } ;

  	  struct xyzpoint_float {
  	    float x,y,z;
  	    } ;


  	//xyzpoint rp,np,np_old;  //robot position and NBV

	//int brojacPP=0;
        //xypoint PParray[2500];       // potential position voxels
	ofstream upis;
        ofstream dt_pose;
	//CvScalar im;

	Tocka tempT;


	ROOM temp_room;
	vector <Duzina> det_room1;
	det_room1.reserve(100);
 	vector <Duzina> det_room;                        //detected room
	det_room.reserve(100);
Razlomljena_Duzina* nova_karta3;  //karta za vektorizaciju
Razlomljena_Duzina* nova_karta32;  //karta za vektorizaciju
  int br_duzina_nova=0;
  int br_duzina_nova2=0;
  vector <Duzina> sobaMatlab;
  sobaMatlab.reserve(200);
  vector <Duzina> soba;
  	soba.reserve(500);
  //od cuda:
  Tocka G,stariG,Xp,Xp2;
  //double starifi;
  stariG = Tocka(-1,-1);
  //int stariGprvi = 1;
  vector <Duzina> globskocni; // skocni bridovi u glob. k.s
  vector <Duzina> globskocni2;
  globskocni.reserve(560); // Ovo treba odrediti eksperimentalno kompromis - zauzece mem / brzina
  globskocni2.reserve(560);
  vector <Duzina> novi_polygon;
  novi_polygon.reserve(560);

  vector <Duzina> novi_polygon2a;
    novi_polygon2a.reserve(560);

  gpc_polygon gpc_globalni_poligon;
  gpc_globalni_poligon.num_contours=0;
  gpc_polygon gpc_globalni_poligon2;
  gpc_globalni_poligon2.num_contours=0;
  vector <Duzina> skocniIzMaxRange; // Stvarni skocni bridovi dobiveni iz max dometa lasera          -cudan komentar
  skocniIzMaxRange.reserve(100);
  Tocka skocniIzMaxRangePrvi;
  Tocka skocniIzMaxRangeZadnji;
  //int skocniIzMaxRangeNadjen=0;
  //int skocniIzMaxRangeBrojac=0;
  vector <Duzina> sobni;
  vector <Duzina> sobni2;
  sobni.reserve(50);
  sobni2.reserve(50);
  //int ciklus=0;
  //int metric=1000;//da bi bilo u milimetrima (za planiranje putanje)
  //int br_pokretanja=0; //koliko puta je mjerio
  Razlomljena_Duzina* karta;
  bool novicilj=false;
  bool zavrsio=false;
  bool first=true;
  double ro, pi, fi, tx, ty;

  //bool stigao_nGMP=true;
  //FILE	*logfile, *lkbrid;

  double nGMP[3]={0}; //tu se zapisuje nova mjerna pozicija
  double nGMP2[3]={0};
  double oldnGMP[3]={0}; //tu se zapisuje stara mjerna pozicija

  //float minbax,minbay;  //min of detected room

  ifstream myfile;
  ifstream pose;
  NEWMAT::Matrix Pg(3,3);



 /* //ray tracing algorithm parameters
      int temp,brojac=0;
	  int length,xi,yi,zi;
	  int si,sj,sk;
	  int max=0;
	  int zastavica;
	  short int stepX,stepY,stepZ,X,Y,Z;
	  float tMaxX,tMaxY,tMaxZ,tDeltaX,tDeltaY,tDeltaZ;
	  time_t start,end;
	  double diff;
	  float pomz,pomx,pomy;
	  int counter=0;
    float pozicijax,pozicijay,pozicijat;
    xyzpoint BUarray[20000];      //boundary unseen voxels

*/

  double najbolji_Hp[4]={0}; //tu se zapisuje najbolji skocni brid
  double najbolji_Hp2[4]={0};
  //int br_GLp=0,br_GHp=0,ki,kj;
  int br_duzina=0;
  int br_duzina2=0;
  Tocka *ocitanja = new Tocka[741];
  Tocka *ocitanja2 = new Tocka[741];
  int indeks_ocitanja=0;
  int old_path_length=0;
		   //int promjena=1;
  //IplImage* img1=cvCreateImage(cvSize(1000,400),IPL_DEPTH_8U,1);  //create grayscale image with 640x480 resolution


  //od rosa
  ros::init(argc, argv, "tmsp");

  ros::NodeHandle nh;

  listener=new tf::TransformListener(ros::Duration(60));
  M = new moj(nh);
  M->initializePlanner();
  //ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  VisualizationPublisher visual(nh);




  ros::Subscriber sub2 = nh.subscribe("/lidar/scan", 1, laser);

  //ros::Subscriber sub3 = nh.subscribe("/base_scan", 10, laser);
  //ros::Subscriber sub6 = nh.subscribe("room", 1000, laser2_room);
  ros::Subscriber sub6 = nh.subscribe("/lidar/scan", 1, laser2_room);
  //ros::Subscriber sub4 = nh.subscribe("thermostat", 1000, laserRiegel);
  //subscriber for pointCloud
  //ros::Subscriber sub5 = nh.subscribe("/3D_pointcloud",1, laser3D);
  ros::Subscriber sub = nh.subscribe("/odom", 1, odomCallback);





  ros::Rate rate(10.0);
  geometry_msgs::Twist vel;
  std_srvs::Empty e;
  tf::StampedTransform transform;


  ros::Time now = ros::Time::now();
    listener->waitForTransform("map", "base_link", now, ros::Duration(5.0));

  while (nh.ok()) {


	  	ros::spinOnce();

//drawing in rviz	  
		if(M->voznja)
		{
			visual.visualizationduringmotion();	
		}


  	//M->RB.x=tfx*1000;
    	//M->RB.y=tfy*1000;
  	//M->RB.th=yaw;  //radijani


  	//ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.2));
      M->executeMotion();

      if (M->ciklus > 11) {



	  if (first==true){
	  ros::service::call("/start3D_scan",e); // start scan for the first time
	  first=false;
	  }








	      //detroom=false;
		  double fi;
		  double minx = 0;
		  double miny = 0;
		  double minx2 = 0;
		  double miny2 = 0;


		  double Px=0;
		  double Py=0;
		  double Pth=0;



// load ocitanja2 for room detection and room vectorization
		  for (int kr=0;kr<ROOM_POINTS_SIZE;kr++){
		  ocitanja2[kr]=Tocka(ranges3[kr]*cos(0*PI/180+kr*0.5*PI/180), ranges3[kr]*sin(0*PI/180+kr*0.5*PI/180));

		  if (ocitanja2[kr].vratiX()<minx2) minx2 = ocitanja2[kr].vratiX();
		  if (ocitanja2[kr].vratiY()<miny2) miny2 = ocitanja2[kr].vratiY();
		  }
		  Xp2 = Tocka(-minx2,-miny2);



		  for (int j=0; j<lasercount; j++)
		  {

			  // KOMUNIKACIJA MOJ - CUDO
			  //stigao_nGMP je zastavica koja se postavlja kad robot stigne u mjernu poziciju

			  if ((stigao_nGMP))
			  {




				  if (promjena)

				  {//ako nije promjena ocitanja s riegla su zastarjela


					  // Ako nije ukljucen AMCL stavljamo fiksnu vrijednost matrice nesigurnosti
					  Px = 0.005;
					  Py = 0.005;
					  Pth = 0.01;


				  Pg << Px<< 0 << 0
					  << 0  << Py <<0
					  << 0  << 0 <<Pth;


				  ocitanja[j]=Tocka(ranges[j]*cos(angle_min+j*angle_increment), ranges[j]*sin(angle_min+j*angle_increment));  // in laser frame


				  // Ogranicavamo zraku lasera na 8 m
				  //if (((skocniIzMaxRangeNadjen == 1) && (M->LB.scan[j].r < 7999)&&(M->LB.scan[j].r>30)) || (skocniIzMaxRangeBrojac == (lasercount-1)/2-1))
/*				  if (((skocniIzMaxRangeNadjen == 1) && (M->LB2.scan[j].r < 7999)) || (skocniIzMaxRangeBrojac == (M->LB2.scan_count)/4))
				  {
					  if ((skocniIzMaxRangeBrojac == (M->LB2.scan_count)/4)){
					  printf("M->LB2.scan_count=%d M->LB2.scan_count/4=%d skocniIzMaxRangeBrojac=%d\t",M->LB2.scan_count, M->LB2.scan_count/4, skocniIzMaxRangeBrojac);

					  Duzina AB = Duzina(skocniIzMaxRangePrvi,skocniIzMaxRangeZadnji);
					  AB.kut = atan2(AB.vratiA().vratiY()-AB.vratiB().vratiY(),AB.vratiA().vratiX()-AB.vratiB().vratiX())+PI/2+fi;
					  AB.stvarni = 0;
					  skocniIzMaxRange.push_back(AB);
					  }
					  skocniIzMaxRangeBrojac = 0;
					  skocniIzMaxRangeNadjen = 0;
				  }
				  //if ((M->LB.scan[j].r > 7999))
				  if ((M->LB2.scan[j].r > 7999))
				  {
					  if (skocniIzMaxRangeNadjen == 1)
					  {
						  skocniIzMaxRangeBrojac++;
						  //printf("M->LB2.scan_count=%d M->LB2.scan_count/4=%d skocniIzMaxRangeBrojac=%d\t",M->LB2.scan_count, M->LB2.scan_count/4, skocniIzMaxRangeBrojac);
						  skocniIzMaxRangeZadnji=ocitanja[j];
					  }
					  if (skocniIzMaxRangeNadjen == 0)
					  {
						  skocniIzMaxRangeNadjen = 1;
						  skocniIzMaxRangePrvi = ocitanja[j];
						  skocniIzMaxRangeZadnji=ocitanja[j];
						  skocniIzMaxRangeBrojac = 0;
					  }

					  // "brisemo" ocitanja
            if (j>0){
              ocitanja[j] = ocitanja[j-1];
            }

				  }
*/
				  // Trazimo min x i min y (treba zbog pomaka ks u sliku)
				  if (ocitanja[j].vratiX()<minx) minx = ocitanja[j].vratiX();
				  if (ocitanja[j].vratiY()<miny) miny = ocitanja[j].vratiY();



				  } // od if promjena

				  //
				  if (j == lasercount-1)    // in the last step
				  {


					  // pomak k.s za sliku za H.T.

					  Razlomljena_Duzina  *nova_karta2;
					  if (promjena){        // if the robot has reached the new NBV point

				      Xp = Tocka(-minx,-miny);

					  //fi = M->RB.th;



					 // find the laser position in the map frame
					  try{
					      now = ros::Time::now();

					     listener->waitForTransform("map", "base_laser_link",now, ros::Duration(4.0));
					     listener->lookupTransform("map", "base_laser_link", now, transform);
					    }
						  catch (tf::TransformException ex){
							  ROS_ERROR("%s",ex.what());
						  }



					      transform.getBasis().getRPY(ro, pi, fi);
						  tx=transform.getOrigin().x();
						  ty=transform.getOrigin().y();
						  //fi=fi*180/M_PI;
					  G = Tocka(tx, ty);// laser position in the map frame in m

					  // transform ocitanja in base_link


						  cout << "Vectorize...";
							 // karta=update_karte(&karta[0], &br_duzina,&ocitanja[0],G,fi,Xp,Pg);
	//=======================================================================================================================================
	// Funkcija stvara kartu iz predanih joj ocitanja i spaja tu kartu sa postojecom u globalnom k.s.


	printf("update_karte: vectorizing start\n");
	br_pokretanja++;
	nova_karta3 =  vektoriziraj(&ocitanja[0],Xp, &br_duzina_nova, 540);  // in the laser coordinate frame, for 2D exploration

	// save detected lines with time stamp

	Save_With_Time_Stamp("map_lines_in_local_frame" +IntToString(br_pokretanja)+".txt", nova_karta3, br_duzina_nova);

	Snimi ("map" +IntToString(br_pokretanja)+".m", nova_karta3,br_duzina_nova);
	nova_karta32 =  vektoriziraj(&ocitanja2[0],Xp2, &br_duzina_nova2, 540); // in the laser coordinate frame, for room detection
    Snimi_ocitanja("laser_scan_room", ocitanja2, &G, &fi, 1, 1, 1);
    Snimi_ocitanja("laser_scan"+IntToString(br_pokretanja)+".txt", ocitanja, &G, &fi, 1, 1, 1);
	printf("update_karte: vectorized\n");
        Snimi ("vectorized_laser_scan" +IntToString(br_pokretanja)+".m", nova_karta3,br_duzina_nova);
	nova_karta2= new Razlomljena_Duzina[br_duzina_nova];

	//make a copy
	for (int i=0; i<br_duzina_nova;i++){
	 nova_karta2[i]=nova_karta3[i];
	}


	transformiraj2(nova_karta2,br_duzina_nova,G,fi,&Xp,Pg);   // transform karta into global coordinate frame




	karta=duzinacat2(karta, br_duzina, nova_karta2,br_duzina_nova,&br_duzina);
	delete[] nova_karta2;
	//=======================================================================================================================================


        string temp_pose;
        string temp_s;

	//write scan pose in scan.pose
        if (br_pokretanja==-1){
	temp_pose= "./3dtk_scan/scan00"+IntToString(br_pokretanja+1)+".pose";
	temp_s= "./df/scan"+IntToString(br_pokretanja+1)+".pose";
        }

        else{
	temp_pose= "./3dtk_scan/scan00"+IntToString(br_pokretanja)+".pose";
	temp_s= "./df/scan"+IntToString(br_pokretanja)+".pose";
	}


        upis.open(temp_s.c_str());
        dt_pose.open(temp_pose.c_str());

        if (!dt_pose.is_open())
        ROS_ERROR("cannot open scan00x.pose file used in 3dtk");
	 if (!upis.is_open())
        ROS_ERROR("cannot open df/scanx.pose file used in df");

        upis << G.vratiX() <<" " << G.vratiY() << " " <<fi <<endl;


          //float fi_deg=fi*180/3.14152;
          dt_pose << -100*G.vratiY() <<" " << 0 << " " <<100*G.vratiX() <<"\n"<<0 << " " << -fi << " " << 0 << endl;
	  upis.close();
          dt_pose.close();
	Snimi ("skriptalok.m", nova_karta32,br_duzina_nova);
						  Snimi ("skriptaNS.m", karta,br_duzina);

					  }  // promjena==1


					  if (promjena == 0 && globskocni.size()==0)            // end of exploration in 2D
					  {
						  cout << "finished";
						  cout << "Reducing the number of lines...";
						  poklopi_kartu2(karta,br_duzina);

						  Snimi ("final_map.m", karta, br_duzina);

						  return 1;
					  }




					  // always run NBV_explore, if promjena==0 && promjena==1
					  explore_2D(Rfind.gpcpoly, false,sobni2,explore_in_3D.still_room,najbolji_Hp2, nova_karta32,br_duzina_nova2,karta,gpc_globalni_poligon2, &br_duzina2, G, fi,Xp2, Pg,&nGMP2[0], globskocni2, skocniIzMaxRange,novi_polygon2a,promjena);
					  explore_2D(Rfind.gpcpoly, true ,sobni, explore_in_3D.still_room, najbolji_Hp, nova_karta3,br_duzina_nova,karta,gpc_globalni_poligon, &br_duzina , G, fi,Xp, Pg,&nGMP[0], globskocni, skocniIzMaxRange,novi_polygon,promjena);






					  if (promjena==1) {      // room detection only if promjena

						  //before change in soba structure save detected room from previous step
						  					  det_room1.clear();
						  					  for (int broj3=0;broj3<Rfind.R.size();broj3++)  {          //It will probably be only one room detected at once

						  					               for (int broj=0; broj<Rfind.R[broj3].size(); broj++){

						  					               det_room1.push_back(Rfind.soba[Rfind.R[broj3][broj]].line); }
						  					  }
						  				SnimiSkocne("det_soba1_" +IntToString(br_pokretanja)+".m",det_room1);

						 SnimiSkocne("ROOM_plane"+IntToString(br_pokretanja)+".m",novi_polygon2a);
                                                 det_room.insert(det_room.end(),det_room1.begin(),det_room1.end());         //lines from all detected rooms
						 SnimiSkocne("det_soba" +IntToString(br_pokretanja)+".m",det_room);

						 //prepare lines for room detection
						  soba.clear();
						  int flagroom;
						  for (uint brojac2 = 0; brojac2 < novi_polygon2a.size();brojac2++)  {
							  flagroom=0;
						  	  for (uint brojac1=0;brojac1<globskocni2.size();brojac1++)                   //petlja po svim bridovima nastalih unijom
						  		{
								  if (novi_polygon2a[brojac2] - globskocni2[brojac1] < 0.00001){
                                       flagroom=1;
								  }}
						  			//if ((novi_polygon[brojac2].stvarni==1) || (novi_polygon[brojac2].duljina() < MINSKOCNI)){
						  			if (flagroom==0)
						  				soba.push_back(novi_polygon2a[brojac2]);    //soba contains all edges without jump edges
						  			//}


						  }
						 SnimiSkocne("soba.m",soba);



						 //copy soba to Rfind.soba(added flag oznaka)
						 Rfind.soba.clear();

                         temp_room.oznaka=1;
                         temp_room.inroom=0;
					 for (int counter=0; counter < soba.size(); counter++) {

							 temp_room.line=soba[counter];
							 Rfind.soba.push_back(temp_room);
							 //Rfind.soba[brojac1].line=soba[brojac1];
							 //Rfind.soba[brojac1].oznaka=1;
						 }

				                      cout << "broj linija u soba je "<< soba.size() << " a broj linija u Rfind.soba je " << Rfind.soba.size()<<endl;




                      //remove detected room lines from the previous step !! not important now since we reset room map
                      for (int broj=0; broj<det_room.size(); broj++){
                    	  for (int broj2=0; broj2<Rfind.soba.size(); broj2++){


                    	  if ((Rfind.soba[broj2].line - det_room[broj]) < 0.2){   // max distance between endpoints of two lines
                    		  Rfind.soba.erase(Rfind.soba.begin()+broj2);
                    	 }
                      }
                      }



                      //write Rfind.soba in Matlab
                      sobaMatlab.clear();
                      for (int counter=0; counter < Rfind.soba.size(); counter++) {
                    	  sobaMatlab.push_back(Rfind.soba[counter].line);
                      }

                      SnimiSkocne("Rfind_soba" +IntToString(br_pokretanja)+".m",sobaMatlab);


                      //vector < vector < int > > Droom=Rfind.Room_detector();
                      //Rfind.Room_detector();
                     // Rfind.soba.clear();
                      Rfind.sobni.clear();
                      Rfind.sobni.insert(Rfind.sobni.begin(),sobni2.begin(),sobni2.end());
                      SnimiSkocne("Rfind_sobni.m",Rfind.sobni);
                      cout << "usao u detektiranje sobe " << endl;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//if (explore_in_3D.still_room==false)
//Rfind.Room_detector(gpc_globalni_poligon2,globskocni2);                   //main function for room detection


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                     cout << " number of detected rooms is " << Rfind.R.size() <<endl;


                      //write detected room in Matlab environment (only the first room)
                      					  sobaMatlab.clear();
                      					  if (Rfind.R.size()!=0){
                      					  for (uint brojac1=0; brojac1<Rfind.R[0].size(); brojac1++){
                      						  cout << "element " << Rfind.R[0][brojac1]<<endl;
                      						  sobaMatlab.push_back(Rfind.soba[Rfind.R[0][brojac1]].line);
                      						  }
                      					  }
                      					  SnimiSkocne("sobaMatlab.m",sobaMatlab);





//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//novi kod
                                          	// room detection in 3D

                                      if   (Rfind.R.size()!=0){         // if the room has been detected
                                   explore_in_3D.explore_room(br_pokretanja, Rfind, DS, GM);
                                          	}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                      //DS->resetzasearch();//resetira stvari za pretrazivanje
					  SnimiSkocne2("./Nocitanja/poligon"+IntToString(indeks_ocitanja),novi_polygon);
					  SnimiSkocne2("./Nocitanja/globskocni"+IntToString(indeks_ocitanja),globskocni);

					  SnimiSkocneGoal2("./Nocitanja/globskocnigoal"+IntToString(indeks_ocitanja),globskocni);
					  // snimanje ocitanja
					  Snimi_ocitanja("./Nocitanja/podaci"+IntToString(indeks_ocitanja),&ocitanja[0], &G, &fi,Px,Py,Pth);


					  //WH->Logmeasure(indeks_ocitanja);
			  		  //WH->old_cycle=WH->cycle_number;


					  string poz2="./Nocitanja/goal"+IntToString(indeks_ocitanja);
					  ofstream file_op2(poz2.c_str());
					  if( !file_op2.is_open() )
					  {
						  cout << "Ne mogu otvoriti datoteku\n";
					  }
					  file_op2 << nGMP[0] << " "<<nGMP[1]<<"\n";
					  file_op2.close();

					  string poz3="./Nocitanja/najboljiskocni"+IntToString(indeks_ocitanja);
					  ofstream file_op3(poz3.c_str());
					  if( !file_op3.is_open() )
					  {
						  cout << "Ne mogu otvoriti datoteku\n";
					  }
					  file_op3 << najbolji_Hp[0] << " "<<najbolji_Hp[1]<<"\n"<< najbolji_Hp[2] << " "<<najbolji_Hp[3]<<"\n";
					  file_op3.close();

					  indeks_ocitanja++;
					  }
					  // snimanje karte
					  Snimi ("./Nocitanja/skripta.m", karta, br_duzina);



				  } // the last step
			  } // if stigao

		  }// end for j ocitanja lasera






		  if (stigao_nGMP)
		  {

			  cout << "usao sam u voznju i resetirao zastavice " << endl;

			  //detroom=false;
			  voznja=true;
			  //go=false;
			  //pomnovicilj=true;
			  novicilj=true;
			  stigao_nGMP=false;



			  printf("I did everything %d. times\n",br_pokretanja+1);
			  if (promjena){
			  oldnGMP[0]=M->RB.x; oldnGMP[1]=M->RB.y; oldnGMP[2]=M->RB.th;
			 // 	WH->mp[WH->num_mp].x=M->RB.x;
			  //	WH->mp[WH->num_mp].y=M->RB.y;
			  //	WH->num_mp++;
			  }
			  printf("Nova MP = %f %f %f\n",nGMP[0],nGMP[1],nGMP[2]);


		  } // end if stigao && go && detroom







/*
			  M->start.x=M->RB.x;
			  M->start.y=M->RB.y;
			  M->start.th=M->RB.th;
			  printf("M->start.x:%.3f\tM->start.y:%.3f\tgoal=(%.3f,%.3f)\n",M->start.x,M->start.y,M->goal.x,M->goal.y);
*/


		  if (voznja)
		  {
			  printf("Usao sam u voznju\n");
			  if (novicilj){
				  novicilj=false;

				  //set new goal pose from 3D SPA in mm
				  if (explore_in_3D.still_room==true){

					  //write old NBV
					  explore_in_3D.np_old.x=explore_in_3D.np.x;
					  explore_in_3D.np_old.y=explore_in_3D.np.y;
					  explore_in_3D.np_old.z=explore_in_3D.np.z;

					  //set NBV from 3D
					  int goalx, goaly, goalth;
					  goalx=(explore_in_3D.np.x*0.2+explore_in_3D.minbax)*1000;
					  goaly=(explore_in_3D.np.y*0.2+explore_in_3D.minbay)*1000;
					  goalth=nGMP[2];//-180*SuR
					  nGMP[0]=(explore_in_3D.np.x*0.2+explore_in_3D.minbax)*1000;
					  nGMP[1]=(explore_in_3D.np.y*0.2+explore_in_3D.minbay)*1000;


					  // write in a file
            R_point tem;
            tem.x=goalx; tem.y=goaly; tem.th=goalth;
            M->gotogoal(tem);

#if 0
					  ofstream goal ("koordinate.txt", ios::app);
					    if (goal.is_open())
					    {
					      goal << "1.53.68.233 > " << goalx <<" " << goaly << " " << goalth <<std::endl;

					      goal.close();
					    }
					    else cout << "Unable to open file koordinate.txt";
#endif


					  ofstream poset;
						string temp_s1= "./3dpose/pose"+IntToString(br_pokretanja-1)+".3dpose";
						poset.open(temp_s1.c_str());

						  poset <<M->goal.x <<" " << M->goal.y << " " <<M->goal.th<<endl;
						  poset.close();

				  }
				  else{
					  //set NBV from 2D
					  //write in a file

            R_point tem;
            tem.x=nGMP[0]; tem.y=nGMP[1]; tem.th=nGMP[2];
            M->gotogoal(tem);
#if 0            
					  ofstream goal ("koordinate.txt", ios::app);
					  if (goal.is_open())
					  	  {
						  goal << "1.53.68.233 > " << nGMP[0] <<" " << nGMP[1] << " " << nGMP[2] <<std::endl;

						  goal.close();
					  	  }
					      else cout << "Unable to open file koordinate.txt";
#endif
				/*  M->goal.x=nGMP[0];
				  M->goal.y=nGMP[1];
				  M->goal.th=nGMP[2];//-180*SuR
				  */
				  }



				  //noj provjera nalazi li se nova mjerna pozicija na prepreci, ako da onda trazi po odabranom jump edgeu slobodnu poziciju (to je najbolji_Hp)


//				  M->subgoal=M->goal;
//				  if (WH->processState==NO_PATH){
//					  M->subgoal=WH->global_goal_workhorse; //postavljen u planneru na neku slobodnu
//				  }
//				  WH->new_global_goal=false;
				  printf("U novicilj Novi goal (%f,%f,%f)\n",M->goal.x,M->goal.y,M->goal.th);
				  zavrsio=false;


  if ((nGMP[0]==0.0)&&(nGMP[1]==0.0)&&(nGMP[2]==0.0)){
			  	  cout << "finished";
				  cout << "Reducing the number of lines...";
				  poklopi_kartu2(karta,br_duzina);
				  Snimi ("final_map.m", karta, br_duzina);
				  break;//voznja=false;    EXIT PROGRAM
			  }
        }
			  //WH->process();//pozivanje procesa koji sve racuna
			  promjena=1;


			  old_path_length=DS->PathLength;

		  } // if voznja




		  // if the robot has reached the goal position

		  double new_goal_tolerance=sqrt((M->RB.x-M->goal.x)*(M->RB.x-M->goal.x)+(M->RB.y-M->goal.y)*(M->RB.y-M->goal.y));
		  if ((M->voznja==false)&&(M->slam_loop==0)&&(WH->processState==HALT)&&(WH->no_path_counter==0)){ //  && (new_goal_tolerance<500.)){   //kad dodje do cilja udje u HALT mod
			  cout << "usao sam u taj HALT mode " << endl;
			  //getchar();
//			  WH->Logger();
			  voznja=false;//vise nejde
			  novicilj=true;
			  stigao_nGMP=true;
			  printf("Dosao sam do cilja, nalazim se na %f %f\n",M->RB.x,M->RB.y);
			  printf("PathCost=%d\n",DS->PathCost);

#if DSTAR3D
			  if (DS->PathCostOri(DS->Start)==OBSTACLE)
#else
			  if (DS->PathCost==OBSTACLE)
#endif
			  {         // if the reason for HALT is no path then promjena=0
				  promjena=0;
			  }

			  double old_goal_tolerance=sqrt((M->RB.x-oldnGMP[0])*(M->RB.x-oldnGMP[0])+(M->RB.y-oldnGMP[1])*(M->RB.y-oldnGMP[1]));
			 if (old_goal_tolerance<GOAL_POSITION_TOLERANCE){//ako je preblizu staroj mjernoj poziciji nemoj mjeriti!
			  	promjena=0;
			  }


			  if (promjena==1){
				  //set a new position in SLAM

				M->snimanje=1;

			  ros::service::call("/start3D_scan",e);
			  kukoc=true; //false;
			  //go on when "laser3D" callback function is called
					  while (kukoc==false){
					  ros::spinOnce();
					  }
			  //okinuo=true;
			  } else {

				  go=true;//inace nece otic u odabir nove pozicije
				  detroom=true;
				  cout <<"postavio se onaj glupi else " << endl;
			  }


			  if (zavrsio)
				  break;
		  }




//		  if (WH->processState==NO_PATH){//dabar
//			  novicilj=true; //neka izracuna pomaknuti cilj
//		  }
  } 
	  	  rate.sleep();
  }//while (rosnode.ok())

  return 0;
}



