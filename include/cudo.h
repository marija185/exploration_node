#ifndef CUDO_H
#define CUDO_H
#include "razlomduzina.h"


//void explore_2D(gpc_polygon &room_poly, bool vect, vector<Duzina> & sobni ,bool still_room, double* najbolji_Hp, Razlomljena_Duzina* novakarta, int br_duzina_nova, Razlomljena_Duzina* karta,gpc_polygon & gpc_globalni_poligon , int *br_duzina, Tocka G2, double fi2, Tocka Xp2, NEWMAT::Matrix Pg1, double* nGMP, vector<Duzina> & globskocni, vector<Duzina> & skocniIzMaxRange,vector<Duzina> & novi_polygon, int promjena);
void transformiraj2(Razlomljena_Duzina *src,int br_duzina, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1);
// Transformacija koordinatnih sustava za razlomljenu_duzinu
vector<Duzina> transformiraj3(vector<Duzina> & src, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1);
// Transformacija koordinatnih sustava za sve duzine u vectoru duzina 
Razlomljena_Duzina* vektoriziraj(Tocka *ocitanja, Tocka Xp, int *duljina, int br_ocitanja);
// Iz ocitanja lasera vraca polje razlomljenih duzina
Razlomljena_Duzina* update_karte(Razlomljena_Duzina* karta,int *br_duzina, Tocka *ocitanja,Tocka G, double fi, Tocka Xp, NEWMAT::Matrix Pg);
// Updatea (prosiruje postojecu kartu), vektorizacijom iz laserskih ocitanja, G je pozicija robota, fi orjentacija, Pg varijanca, Xp (tocka koja odredjuje pomak za sliku)
void Snimi (string podaci, Razlomljena_Duzina *rez, int duljina);
void Snimi_ocitanja(string podaci, Tocka *ocitanja, Tocka *G, double *fi, double Px, double Py, double Pth);
string IntToString ( int number );
int glavni(void); // Funkcija koja je bila main za cudo, sluzi za offline vektorizaciju
Razlomljena_Duzina transformiraj_razl_duzinu(Razlomljena_Duzina AB, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1); // Trasnsformacija KS za jednu razl_duzinu
bool comp (Duzina a,Duzina b); // Funkcija usporedbe za sort bridova
void SnimiSkocne (string podaci, vector<Duzina> rez); // snima skocne bridove
void SnimiSkocne2 (string podaci, vector<Duzina> rez); // snima skocne bridove
void SnimiSkocneGoal2 (string podaci, vector<Duzina> rez); // snima skocne bridove
Duzina transformiraj_duzinu(Duzina AB, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1); // transformacija KS jedne duzine
void poklopi_kartu2(Razlomljena_Duzina *karta, int duljina); // vrsi spajanje 
Razlomljena_Duzina *duzinacat2(Razlomljena_Duzina *src1, int duljina1, Razlomljena_Duzina *src2, int duljina2,int *duljina);
void Save_With_Time_Stamp(string file_name, Razlomljena_Duzina *line_map, int number_of_lines);
#endif
