
// Testiramo amcl lokalizaciju -16856.000000 ,873.000000 ,-2.538965

// bin/moj -s -16856.000000 873.000000 214.5280

// Treba eliminirati utjecaj visestrukog spajanja istih linija!!!
// Treba osigurati da linefit izracuna pravac, te da na temelju zajednickog pravca i estimacije odsjece linije
// Za pocetak cemo onemoguciti da spajanje spaja sam sebe !
// Linefit trenutno ne racuna bas najbolje varijance linija :()	
// Trenutno imamo losije rezultate sa razlomljenom duzinom :(
// 1.Pojednostavni treba reviziju jer prema ovome ne radi
// 2.Spoji - spajanje u centru rot nesigurnosti
// 3. Chi squred - prvo sa manjim D !
// 4. predikcija mjerenja 
// 5. rekurzivni HT

/*
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif*/

#include <iostream>
#include <algorithm>
#ifdef use_namespace
using namespace NEWMAT;              // access NEWMAT namespace
#endif
#include <fstream>
#include <sstream>  
#include <vector>
# define BR_OCITANJA 570
# define DISK 70
# define maxRUPA 100 
# define maxRUPA2 15 

# define glasackiPRAG 15

			 /*
# define PODACI "podaci1.txt"

# define PODACI "podaci57.txt"
			 57 *
			 274*
# define PODACI "podaci117.txt"
# define PODACI "podaci224.txt"
# define PODACI "podaci274.txt"
# define PODACI "podaci324.txt"
# define PODACI "podaci421.txt"
# define PODACI "podaci521.txt"

*/
//# define PODACI "podaci637.txt"
# define  BLIZINA 3
# define  BLIZINA_GROUP 10
# define  BLIZINA2 3

#include "cudo.h" 
using namespace std;
	 
	 
	 
string IntToString ( int number )
{
ostringstream oss;
oss<< number;
return oss.str();
}	 
	 
	 
	 
void SnimiSkocne (string podaci, vector<Duzina> rez)
{
	/*
	ofstream fout("sample.txt");
	fout << "WWW" << endl;
	fout.close();
	*/
	ofstream file_op(podaci.c_str());
	if( !file_op.is_open() )
	{
		
		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}        
	file_op << "rezultat = [";
	int i;
	for (int i = 0;i<rez.size();i++)
	{
				file_op << rez[i].vratiA().vratiX() << " " << rez[i].vratiA().vratiY() << ";"<<rez[i].vratiB().vratiX() << " " << rez[i].vratiB().vratiY() << ";" ;
		
	}
	file_op << "]"<< endl;;
	file_op.close();
}
	 
void SnimiSkocne2(string podaci, vector<Duzina> rez)
{
	/*
	ofstream fout("sample.txt");
	fout << "WWW" << endl;
	fout.close();
	*/
	ofstream file_op(podaci.c_str());
	if( !file_op.is_open() )
	{
		
		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}        
	//file_op << "rezultat = [";
	int i;
	for (int i = 0;i<rez.size();i++)
	{
				file_op << rez[i].vratiA().vratiX() << " " << rez[i].vratiA().vratiY() << "\n"<<rez[i].vratiB().vratiX() << " " << rez[i].vratiB().vratiY() << "\n" ;
		
	}
	//file_op << "]"<< endl;;
	file_op.close();
}

void SnimiSkocneGoal2(string podaci, vector<Duzina> rez)
{
	/*
	ofstream fout("sample.txt");
	fout << "WWW" << endl;
	fout.close();
	*/
	ofstream file_op(podaci.c_str());
	if( !file_op.is_open() )
	{
		
		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}        
	//file_op << "rezultat = [";
	int i;
	for (int i = 0;i<rez.size();i++)
	{
				file_op << rez[i].goal.vratiX()*1000. << " " << rez[i].goal.vratiY()*1000. << "\n" ;
		
	}
	//file_op << "]"<< endl;;
	file_op.close();
}





void Save_With_Time_Stamp(string file_name, Razlomljena_Duzina *line_map, int number_of_lines){

	int line_counter=0;
	ofstream file_op(file_name.c_str());
		if( !file_op.is_open() )
		{

			cout << "Cannot open file \n";

		}

		else {



			for (int i = 0;i<number_of_lines;i++)
				{
					for (int j = 0;j<line_map[i].A.size();j++)
					{

						Duzina tmp=line_map[i].vratiDuzinu(j);
						if (tmp.duljina()>0.005)   // in case when the size of line is not 0
						file_op << tmp.vratiA().vratiX() << " " << tmp.vratiA().vratiY() << " "<<tmp.vratiB().vratiX() << " " << tmp.vratiB().vratiY() << endl ;

					}
				}
			// write time and the number of lines





		}

}



	 
void Snimi (string podaci, Razlomljena_Duzina *rez, int duljina)
{
	/*
	ofstream fout("sample.txt");
	fout << "WWW" << endl;
	fout.close();
*/
	ofstream file_op(podaci.c_str());
	if( !file_op.is_open() )
	{
		
		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}        
	file_op << "rezultat = [";
	int i;
	for (int i = 0;i<duljina;i++)
	{
		for (int j = 0;j<rez[i].A.size();j++)
		{
			
			Duzina tmp=rez[i].vratiDuzinu(j);
			if (tmp.duljina()>0.005)
			file_op << tmp.vratiA().vratiX() << " " << tmp.vratiA().vratiY() << ";"<<tmp.vratiB().vratiX() << " " << tmp.vratiB().vratiY() << ";" ;
		}
	}
	file_op << "]"<< endl;;
	file_op.close();
}
	 


NEWMAT::Matrix ocitaj_varijancu(string podaci)
{
	double Px, Py, Pth;
	ifstream file_op(podaci.c_str());
	if( !file_op.is_open() )
	{
		cout << "Ne mogu otvoriti datoteku\n";
		Px = 0; Py=0;Pth=0;
	}        
	else	
	{
	file_op >> Px>>Py>>Pth;
	file_op.close();
	}	
	NEWMAT::Matrix P(3,3);
	P << Px << 0 << 0 
	  << 0 << Py << 0
	  << 0 << 0 << Pth;
	return P;				
	
}

void ocitaj(string podaci, Tocka *ocitanja,Tocka *Xp, Tocka *G, double *fi)
{
// Funkcija ocitava file PODACI  (txt) te iz njega vadi ocitanja u polje tocaka Tocka(x,y) (361 ocitanje)
// min vrijednosti za koje treba pomaknuti zbog PHT-a sprema u polje tocaka xp i yp (negativne)
// Ocitava trenutni polozaj robota (362 ocitanje) i njegovo usmjerenje (broj na kraju) 
// Umjesto standardnih 361 korisit se BR_OCITANJA

// PAZNJA Xp se vraca kao pozitivna vrijednost	
int i;	
double loklaserx;
double loklasery;
double xp=0, yp=0;
	ifstream file_op(podaci.c_str());
	if( !file_op.is_open() )
	{
		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}        
	for (i = 0;i<BR_OCITANJA;i++)
	{
	file_op >> loklaserx>>loklasery;
	
		if (loklaserx < xp) xp = loklaserx;
		if (loklasery < yp) yp = loklasery;
		ocitanja[i]=Tocka(loklaserx,loklasery);
	}
	file_op >> loklaserx>>loklasery;
	*G = Tocka(loklaserx,loklasery);
	file_op >> *fi;
	
	file_op.close();
	*Xp = Tocka(abs(xp),abs(yp));
}
	 
void Snimi_ocitanja(string podaci, Tocka *ocitanja, Tocka *G, double *fi, double Px, double Py, double Pth)
{
// Funkcija snima ocitanja u datoteku

// PAZNJA Xp se vraca kao pozitivna vrijednost	
	int i;	
	string podaci1 = podaci+".txt";
	ofstream file_op(podaci1.c_str());
	if( !file_op.is_open() )
	{
		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}        
	for (i = 0;i<BR_OCITANJA;i++)
	{
		file_op << ocitanja[i].vratiX()<<" "<<ocitanja[i].vratiY()<<"\n";
	}
	
	file_op << (*G).vratiX() << " "<<(*G).vratiY()<<"\n";
	file_op << *fi;
	file_op.close();
	
	podaci1 = podaci+".dat";
	ofstream file_op1(podaci1.c_str());
	if( !file_op1.is_open() )
	{
		cout << "Ne mogu otvoriti datoteku\n";
		return;
	}
	file_op1 << Px << " "<<Py<<" "<<Pth<<"\n";
	file_op1.close();

	
}
	
	 
IplImage* napravi(vector<Tocka> &ocitanja, Tocka Xp)
{
// Funkcija napravi vraća pointer na IplImage, koji stvara iz laserskih ocitanja
// Pripadnim tockama (diskretiziranim dodaje vrijednost 255) (pomaknuto je za Xp)
 
	IplImage* src=cvCreateImage(cvSize(4*DISK*DOMET,4*DISK*DOMET),IPL_DEPTH_8U,1);
	CvScalar s;
	s.val[0]=255;

// Funkcija ocitava laserska ocitanja iz datoteke PODACI
// Koristi fju ocitaj te tockama dodaje Xp da bi pomaknula koordinatni sustav tako da par najmanjih neg vrijednosti (x_min, y_min) preslikava u (0,0)

// Zatim stvara mrezastu kartu zauzetosti (IplImage) 

	int i;
	Tocka G;
				
	//Ocitavam podatke i dobivam pomak i poziciju robota (koja mi zasad ne treba)

	//ocitaj(&ocitanja[0],Xp, &G, &fi);

	cvSetZero(src);
	for (i = 0;i<ocitanja.size();i++)
	{
	// Pomice dobivena ocitanja za xp, yp
		//ocitanja[i] = ocitanja[i] + *Xp;
	// Stvara mrezastu kartu zauzetosti 
		cvSet2D(src,int(((ocitanja[i]+Xp).vratiX())*DISK),int(((ocitanja[i]+Xp).vratiY())*DISK),s);   
	};
	return src;
}

Razlomljena_Duzina spoji2(Razlomljena_Duzina AB, Razlomljena_Duzina CD)
{
	
	Razlomljena_Duzina EF;
	NEWMAT::Matrix B(2,2), K(2,3), varijanca(2,2), deltaL(2,1), pomocna(1,1),L1(2,1), L2(2,1),Lm(2,1),PL1(2,2), PL2(2,2), Pm(2,2); 

	double ro = (AB).vratiPravac().vratiR();
	double alpha = (AB).vratiPravac().vratiTheta();
	NEWMAT::Matrix Ptmp(2,2);
	Ptmp = (AB).vratiPravac().vratiVarijanca()+(CD).vratiPravac().vratiVarijanca();
	double deltaPsi=-Ptmp(1,2)/Ptmp(2,2);
	Tocka gij = Tocka((ro*cos(alpha)-deltaPsi*sin(alpha)), (ro*sin(alpha)+deltaPsi*cos(alpha))); //centar rotacijske nesigurnosti
	NEWMAT::Matrix Pg1(3,3);
	Pg1<<0<<0<<0<<0<<0<<0<<0<<0<<0;
	double fi = 0;
	Tocka Xp = Tocka(0,0);
	AB=transformiraj_razl_duzinu(AB, gij, fi,&Xp,Pg1); 
	Xp = Tocka(0,0);
	CD=transformiraj_razl_duzinu(CD, gij, fi,&Xp,Pg1);
	
	
				
	L1 << (AB).vratiPravac().vratiR() << (AB).vratiPravac().vratiTheta(); // pravac u glob koordinatnom sustavu
	L2 << (CD).vratiPravac().vratiR() << (CD).vratiPravac().vratiTheta();
        // u ovom dijelu provjerava se da li su sve vrijednosti za pravac ispravne
	while (L2(1,1) < 0) {L2(1,1)=-L2(1,1); L2(2,1)=L2(2,1) + PI;}  
	while (L1(1,1) < 0) {L1(1,1)=-L1(1,1); L1(2,1)=L1(2,1) + PI;}
	
	while (L2(2,1) < 0) L2(2,1)=L2(2,1) + 2*PI;
	while (L1(2,1) < 0) L1(2,1)=L1(2,1) + 2*PI;
	while (L2(2,1) > 2*PI) L2(2,1)=L2(2,1) - 2*PI;
	while (L1(2,1) > 2*PI) L1(2,1)=L1(2,1) - 2*PI;
	
	
	PL1 = AB.vratiPravac().vratiVarijanca();
	PL2 = CD.vratiPravac().vratiVarijanca();
	deltaL = L1-L2;
	pomocna = (deltaL.t()*(PL1+PL2).i()*deltaL);
	double pomocna1 = abs(pomocna(1,1));  //mislim da je nepotrebno jer je ta vrijednost uvijek pozitivna
	
	if (L1==L2)
		cout << "Hipoteza";
	if (pomocna1 < BLIZINA || L1==L2)    //zasto je potrebno provjeravati L1==L2 ?
	{
		cout << "Potvrda\n";
		Pm=(PL1.i()+PL2.i()).i();
		Lm=Pm*((PL1).i()*L1+(PL2).i()*L2);
		
		for (unsigned int i=0;i<AB.A.size();i++)
		{
		if (EF.A.size()==0)
			
			{
				EF = Razlomljena_Duzina(najblizaPravcu(AB.A[i],AB.B[i],Pravac(Lm(1,1),Lm(2,1), Pm)));
			}
			else
			{
			EF.dodaj(najblizaPravcu(AB.A[i],AB.B[i],Pravac(Lm(1,1),Lm(2,1), Pm)));
			}
		}
		
		for (unsigned int i=0;i<CD.A.size();i++)
		{
			if (EF.A.size()==0)
			
			{
				EF = Razlomljena_Duzina(najblizaPravcu(CD.A[i],CD.B[i],Pravac(Lm(1,1),Lm(2,1), Pm)));
			}
			else
			{
				EF.dodaj(najblizaPravcu(CD.A[i],CD.B[i],Pravac(Lm(1,1),Lm(2,1), Pm)));
			}
		}
				
	
	}
	Xp = Tocka(0,0);
	fi = 0;
	gij=Tocka(-gij.vratiX(),-gij.vratiY());
	EF=transformiraj_razl_duzinu(EF, gij, fi,&Xp,Pg1);
	return EF;
}

Duzina spoji(Duzina *AB, Duzina *CD, Tocka gij=Tocka(0,0), double gamma=0,double sigma_d=0.005)
{

// VRATITI Tu treba napraviti uvjet za duge linije da ne moze biti veci razmak od max rupe!!! 

// Ovo treba jos malo promisliti

	NEWMAT::Matrix B(2,2), K(2,3), varijanca(2,2), deltaL(2,1), pomocna(1,1),L1(2,1), L2(2,1),Lm(2,1),PL1(2,2), PL2(2,2), Pm(2,2); 

	L1 << (*AB).vratiPravac().vratiR() << (*AB).vratiPravac().vratiTheta();

	L2 << (*CD).vratiPravac().vratiR() << (*CD).vratiPravac().vratiTheta();

	PL1 = (*AB).vratiPravac().vratiVarijanca();
	PL2 = (*CD).vratiPravac().vratiVarijanca();


	Tocka pocetna=(*AB).VelikiRazmak(*CD).vratiA();
	Tocka zavrsna=(*AB).VelikiRazmak(*CD).vratiB();

// Ako se nije pomakao ide sljedeci kod !

	deltaL = L1-L2;
	pomocna = (deltaL.t()*(PL1+PL2).i()*deltaL);


//if ((AB-CD<BLIZINA2*sigma_d) ||  (uvjet_parc_preklapajuce(AB, CD,BLIZINA*sigma_d)))// || 

//cout << "\n" << uvjet_parc_preklapajuce(AB, CD,3*sigma_d);
	double PRRm,PAAm;			
	double pomocna1 = abs(pomocna(1,1));
	if (pomocna1 < BLIZINA) 
	{
		
	if ((((*AB).MaliRazmak(*CD).vratiA()-(*AB).MaliRazmak(*CD).vratiB())*DISK < maxRUPA2)) //||  (((*AB)-(*CD)<BLIZINA2*sigma_d)))
		{
			
			Pm=(PL1.i()+PL2.i()).i();
			PRRm=Pm(1,1);
			PAAm=Pm(2,2);
	
			Lm=Pm*((PL1).i()*L1+(PL2).i()*L2);
			Duzina najbliza1 = najblizaPravcu(pocetna,zavrsna,Pravac(Lm(1,1),Lm(2,1), Pm));
			Pm=najbliza1.vratiPravac().vratiVarijanca();
			PRRm=Pm(1,1);
			PAAm=Pm(2,2);
	
			return najbliza1;
		}
		else
		{
			Pm=(PL1.i()+PL2.i()).i();
			Lm=Pm*((PL1).i()*L1+(PL2).i()*L2);
		
			PRRm=Pm(1,1);
			PAAm=Pm(2,2);

			(*AB) = najblizaPravcu((*AB).vratiA(),(*AB).vratiB(),Pravac(Lm(1,1),Lm(2,1), Pm));
			(*CD) = najblizaPravcu((*CD).vratiA(),(*CD).vratiB(),Pravac(Lm(1,1),Lm(2,1), Pm));
			Pm=(*AB).vratiPravac().vratiVarijanca();
			PRRm=Pm(1,1);
			PAAm=Pm(2,2);

		}

	}
	/*
	if (AB.vratiA()!=AB.vratiB() && CD.vratiA()!=CD.vratiB())
	{
	cout << "\nNisu spojene\n";
	AB.print();
	cout << "\n";
	CD.print();
}
	*/
	return Duzina(Tocka(1,1), Tocka(1,1));

}

void razvrstaj1(Duzina AB, vector<Tocka> & ocitanja, int *razvrstane,int *velicina,double 
		sigma_d=0.005) 
{
	// Funkcija vraca polje od 0 ili 1-ica prema tome da li je tocka "dovoljno blizu" duzine AB ili nije
	// funkcija je napisana da sluzi funkciji razvrstaj a i za smanjenje slozenosti kod razvrstavanja tocaka koje pripadaju vise duzina
	// povecava brojac da zna rezervirati memoriju za tocke koje su blizu
	// PAZNJA funkcija vraca 1 samo tamo di je blizu, a gdje nije tamo ne mjenja vrijednost *razvrstane;
	*velicina = 0;
	int brojac=0,i;
	for (i=0;i<ocitanja.size();i++)
	{
		if (AB-ocitanja[i]<(BLIZINA_GROUP)*sigma_d) // tu ide 4sigma zbog diskretizacije
		{
			brojac++;
	// tamo gdje je razvrstane[i]=1 ta tocka pripada odredjenoj liniji
			razvrstane[i]=1;
		}

	}
	
	*velicina = brojac;
	return; 
}

Tocka* razvrstaj(Duzina AB, vector<Tocka> & ocitanja, int *velicina,int br_tocaka, double sigma_d=0.005) 
{
// Fja vraca razvrstane tocke koje su "blizu" odredjene linije
// Tijekom rada postavlja polje razvrstane na 1 (vraca pomocne fje razvrstaj1)); 
// Vraca velicinu polja koje treba rezervirati preko *velicina		
	//int lagano=br_tocaka;
        int razvrstane[720]={0};
        //int *razvrstane= new int[720];
	int i,brojac=0;
//	AB.vratiPravac().print();
	razvrstaj1(AB,ocitanja,razvrstane,velicina);

	Tocka *razvrstana_ocitanja = new Tocka[*velicina];	
//*velicina = brojac;
	brojac = 0;
	for (i=0;i<br_tocaka;i++)
	{
		if (razvrstane[i]==1)
		{
			razvrstana_ocitanja[brojac++]=ocitanja[i];
		}
	}
      

	return razvrstana_ocitanja;
	
}

Razlomljena_Duzina pojednostavni(Duzina *AB1, Duzina *CD1,Tocka *ocitanja,double sigma_d=0.005)
{

	Duzina AB = *AB1;
	Duzina CD = *CD1;
	int razvrstane[361]={0};

	int i,velicina1=0,velicina2=0,brojac;

	NEWMAT::Matrix B(2,2), K(2,3), varijanca(2,2), deltaL(2,1), pomocna(1,1),L1(2,1), L2(2,1),Lm(2,1),PL1(2,2), PL2(2,2), Pm(2,2); 

	
	Tocka pocetna=AB.VelikiRazmak(CD).vratiA();
	Tocka zavrsna=AB.VelikiRazmak(CD).vratiB();


	double razlika;
	razlika = AB-CD;
	double parc_preklop;
	parc_preklop=uvjet_parc_preklapajuce(AB, CD,BLIZINA*sigma_d);
		
		
	if ((AB-CD<BLIZINA2*sigma_d) ||  (uvjet_parc_preklapajuce(AB, CD,BLIZINA2*sigma_d)))// || 
	{


	//void razvrstaj1(AB,*ocitanja,*razvrstane,*velicina) 
		// NETESTIRANO
		vector <Tocka> ocitanja1; ocitanja1.assign(ocitanja,ocitanja+BR_OCITANJA);
		razvrstaj1(AB,ocitanja1, &razvrstane[0],&velicina1); 
		razvrstaj1(CD,ocitanja1, &razvrstane[0],&velicina2);

		if (velicina1 + velicina2 > minLINIJA)
		{
			brojac=0;
			Tocka *tmp = new Tocka[velicina1+velicina2];
			for (i = 0;i<361;i++)
			{
				if (razvrstane[i]==1) 
					tmp[brojac++]=ocitanja[i];
			}

			vector<Tocka> beskorisne;
			return linefit3(Duzina(pocetna,zavrsna),&tmp[0],&brojac,Duzina(pocetna,zavrsna).vratiPravac().vratiR(),Duzina(pocetna,zavrsna).vratiPravac().vratiTheta(),0.005,0.0001,1);
		}
	}
 
	return Razlomljena_Duzina();
}

void crtaj(char *Caption, int duljina, Duzina *fitani, Tocka Xp)
{
	IplImage* color_dst = cvCreateImage(cvSize(4*DISK*DOMET,4*DISK*DOMET), 8, 3);
	int i;
	
	for (i = 0;i<duljina;i++)
	{
		cvLine( color_dst, (fitani[i].vratiA()+Xp).convert2cvPoint(DISK), (fitani[i].vratiB()+Xp).convert2cvPoint(DISK), CV_RGB(255,0,0), 1, 8 );
	}
	cvNamedWindow(Caption, 1 );
	cvShowImage(Caption, color_dst );
	cvReleaseImage( &color_dst);
} 

Razlomljena_Duzina* vektoriziraj(Tocka *ocitanja, Tocka Xp, int *duljina, int br_ocitanja)
{
// Poziva se svaki puta HT! .. 
double pocSize=0;
	// Prepisujemo tocke da mozemo micati upotrebljene
			
	vector<Tocka> Ocitanja1;Ocitanja1.reserve(br_ocitanja);
	vector<Tocka> beskorisne;
	for (int i=0; i<br_ocitanja;i++)
	{
		if ((i==0) || (i>0 && ocitanja[i]!=ocitanja[i-1]))
		Ocitanja1.push_back(ocitanja[i]);
	}
	Razlomljena_Duzina *fitani2 = new Razlomljena_Duzina[120];
		
	int zastavica=0; int brojac=0;
	int MAXvelicina = 1;

	CvSeq* lines = 0;
	CvSize SrcSize;

//	while (MAXvelicina > 0)
	//{
	pocSize = Ocitanja1.size();
	// main radi hafovu transformaciju nad IplImage 
	IplImage* src;	
	CvScalar s; 
	s.val[0]=255;
	int i;
	Duzina *linije;
	Duzina *fitani;
	//Tocka ocitanja[361];
	//Tocka Xp;
	src=napravi(Ocitanja1,Xp);
	SrcSize = cvGetSize(src);
	// Paznja dobivamo stvarna ocitanja i pomaknute diskretizirane tocke za PHT (Zbog OpenCV-a) 	
	//CvSeq* lines = 0;

		IplImage* dst = cvCreateImage( SrcSize, 8, 1 );
		IplImage* color_dst = cvCreateImage(  SrcSize, 8, 3 );
		CvMemStorage* storage = cvCreateMemStorage(0);
		

		cvCanny( src, dst, 50, 200, 3 );
		cvCvtColor( dst, color_dst, CV_GRAY2BGR );
	

		cvReleaseImage( &src);
		
		double minLINIJA2=minLINIJA;//5
		double glasackiPRAG2=glasackiPRAG;//15
		double maxRUPA3=maxRUPA;//max gap between line segments to join them
		
		do
		{
		lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/360, glasackiPRAG2, minLINIJA2, maxRUPA3);
		printf("PHT lines->total=%d Ocitanja1.size()=%d glasackiPRAG2=%f minLINIJA2=%f maxRUPA3=%f\n",lines->total, Ocitanja1.size(), glasackiPRAG2, minLINIJA2, maxRUPA3);
		if (glasackiPRAG2 > 5)
		{
		glasackiPRAG2 -=5;
		}
		else if (glasackiPRAG2 > 2)
		{
		glasackiPRAG2 --;
		}
		minLINIJA2=3;
		maxRUPA3=30;
		} while (lines->total < 1 && glasackiPRAG2>2);
		cvClearMemStorage( storage );
		cvReleaseImage( &dst);
		cvReleaseImage( &color_dst);
		
		// linije = new Duzina[lines->total];
		//fitani = new Duzina[lines->total];
		
		
				
	//odk	
	/*
	
		// Dobivene linije[i] su vracene u originalni KS u kojemu su i ocitanja
		
		Duzina *linije2 = new Duzina [lines->total];
		for (int a = 0; a<lines->total;a++)
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,a);
		linije2[a]=Duzina(Tocka(double(line[0].y)/DISK,double(line[0].x)/DISK)+Xp*(-1),Tocka(double(line[1].y)/DISK,double(line[1].x)/DISK)+Xp*(-1));
		
		}
			
		// Prikaz slike dobivene PHT-om
		crtaj("Rezultat PHT-a",lines->total,&linije2[0],Xp);
		cvWaitKey(0);
	//krajodk
	*/

	//vector <Razlomljena_Duzina> fitani2(lines->total);
	

	int velicina=0;
	*duljina = 0;

	Tocka *razvrstana_ocitanja;

	
// Pocetak bloka za ispis

	
		//fitani2.clear();
		
	/*	MAXvelicina=0;
		int MAXi=-1; // Potraziti cemo najvecu velicinu
		printf("broj linija=%d",lines->total);
		for (i = 0;i<lines->total;i++)
		{
			
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			Duzina linije=Duzina(Tocka(double(line[0].y)/DISK,double(line[0].x)/DISK)+Xp*(-1),Tocka(double(line[1].y)/DISK,double(line[1].x)/DISK)+Xp*(-1));
			razvrstana_ocitanja=razvrstaj(linije, Ocitanja1,&velicina, br_ocitanja, 0.005);
//			printf("broj razvrstanih tocaka =%d",velicina);
			// Linije za koje je glasalo vise od minLINIJA tocaka idu na fitanje
			if (velicina > minLINIJA)
			{
				if (velicina > MAXvelicina)
				{
				MAXvelicina = velicina;
				MAXi = i;
				}
				int indeks=0;
				zastavica=0; 
	
				//Razlomljena_Duzina tmp1 = 		fitani2[brojac++]=linefit3(linije[i],&razvrstana_ocitanja[indeks],&velicina,linije[i].vratiPravac().vratiR(),linije[i].vratiPravac().vratiTheta(),0.005,0.0001,1);

				

			};
		};
*/
	for (i = 0;i<lines->total;i++)
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			Duzina linije=Duzina(Tocka(double(line[0].y)/DISK,double(line[0].x)/DISK)+Xp*(-1),Tocka(double(line[1].y)/DISK,double(line[1].x)/DISK)+Xp*(-1));
			linije.vratiPravac().print();
			razvrstana_ocitanja=razvrstaj(linije, Ocitanja1,&velicina, br_ocitanja, 0.005);
			printf("za brisanje: broj razvrstanih tocaka =%d, R=%f, theta=%f\n",velicina,linije.vratiPravac().vratiR(),linije.vratiPravac().vratiTheta());
            if (velicina > 0)
			Razlomljena_Duzina tmp1 = fitani2[brojac++]=linefit3(linije,&razvrstana_ocitanja[0],&velicina,linije.vratiPravac().vratiR(),linije.vratiPravac().vratiTheta(),0.005,0.0001,1);
			//tmp1.print();

		
		
			
			
			
			
			
			
			
			
			vector <Tocka> za_brisati; 
			for (int I=0; I<velicina;I++)
			{
				
			for (int iO1 = Ocitanja1.size()-1; iO1 >= 0;iO1--)
				if (Ocitanja1[iO1]==razvrstana_ocitanja[I])
				{
//					if ((I==0)||(I==velicina-1)||1){
//						printf("%d-ta tocka brisanja: (%f,%f) ",I,razvrstana_ocitanja[I].vratiX(),razvrstana_ocitanja[I].vratiY());
//					}
					Ocitanja1.erase(Ocitanja1.begin()+iO1);
				}
		
			}
		
						
		

			


		
		*duljina =brojac;
		//Razlomljena_Duzina tmp;

		}

	return fitani2; //zakom
}



Duzina transformiraj_duzinu(Duzina AB, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1)    
{
// Treba jos napraviti transformaciju varijanci
	
	
	double alpha,R,delta_psi;
	double xi,yi;
	xi = gij.vratiX();
	yi = gij.vratiY();
	alpha = AB.vratiPravac().vratiTheta();
	R = AB.vratiPravac().vratiR();
	delta_psi = yi*cos(alpha+fi)-xi*sin(alpha+fi);
	
	Tocka A, B;
	A=AB.vratiA();
	B=AB.vratiB();
	double Ra, Rb;
	Ra = A-Tocka(0,0);
	Rb = B-Tocka(0,0);
	A = Tocka(Ra*cos(A.kut(Tocka(0,0))+fi)+xi,Ra*sin(A.kut(Tocka(0,0))+fi)+yi);
	B = Tocka(Rb*cos(B.kut(Tocka(0,0))+fi)+xi,Rb*sin(B.kut(Tocka(0,0))+fi)+yi);
	if (A.vratiX() < (*Xp).vratiX())
	*Xp = Tocka(A.vratiX(),(*Xp).vratiY());
	if (A.vratiY() < (*Xp).vratiY())
	*Xp = Tocka((*Xp).vratiX(),A.vratiY());
	if (B.vratiX() < (*Xp).vratiX())
		*Xp = Tocka(B.vratiX(),(*Xp).vratiY());
	if (B.vratiY() < (*Xp).vratiY())
		*Xp = Tocka((*Xp).vratiX(),B.vratiY());
	
	// Treba jos transformirati kovarijance!!!
	
	NEWMAT::Matrix P1(2,2), H1(2,2), P0(2,2), Ki(2,3), Pg0(2,2);
	H1 << 1 << 0 << delta_psi << 1;
	P1 = AB.vratiPravac().vratiVarijanca();
	P0=(H1*P1)*H1.t();
	Ki << 0<< 0 << 1 << cos(alpha+fi) << sin(alpha+fi) << 0;
	Pg0=(Ki*Pg1)*Ki.t();
	
	return Duzina(A,B,P0+Pg0);
}

Razlomljena_Duzina transformiraj_razl_duzinu(Razlomljena_Duzina AB, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1) 
{
	Razlomljena_Duzina CD;
	Duzina tmp;
	for (unsigned int i=0;i<AB.A.size();i++)
	{
		tmp = transformiraj_duzinu(Duzina(AB.A.at(i),AB.B.at(i),AB.vratiPravac().vratiVarijanca()), gij,fi,Xp,Pg1);
		if (i==0)
		{
			CD=Razlomljena_Duzina(tmp);
		}
		else
		{	
			CD.dodaj(tmp);
		}
	}
	CD.postaviVar(tmp.vratiPravac().vratiVarijanca());
	return CD;
}

void transformiraj2(Razlomljena_Duzina *src,int br_duzina, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1)
{
	*Xp = Tocka(0,0);
	for (unsigned int i=0;i<br_duzina;i++)
	{
		src[i]=transformiraj_razl_duzinu(src[i],gij,fi,Xp,Pg1);
	}
return;
}
		

Duzina *transformiraj(Duzina *src, int duljina, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1)
{
*Xp = Tocka(0,0);
int i=0;
Duzina *dest = new Duzina[duljina];	
	for (i=0;i<duljina;i++)
	{
	dest[i]=transformiraj_duzinu(src[i],gij,fi,Xp,Pg1);
	}
	*Xp = Tocka(abs(Xp->vratiX()),abs(Xp->vratiY()));
return dest;
}



vector<Duzina> transformiraj3(vector<Duzina> & src, Tocka gij, double fi,Tocka *Xp, NEWMAT::Matrix Pg1)
{
	*Xp = Tocka(0,0);
	int i=0;
	vector<Duzina> dest;
	dest.reserve(src.size());	
	for (i=0;i<src.size();i++)
	{
		dest.push_back(transformiraj_duzinu(src[i],gij,fi,Xp,Pg1));
		dest[i].kut = src[i].kut;
		dest[i].stvarni = src[i].stvarni;
		dest[i].odabran = src[i].odabran;
	}
	*Xp = Tocka(abs(Xp->vratiX()),abs(Xp->vratiY()));
	return dest;
}



Razlomljena_Duzina *duzinacat2(Razlomljena_Duzina *src1, int duljina1, Razlomljena_Duzina *src2, int duljina2,int *duljina)

{
	Razlomljena_Duzina  *rez = new Razlomljena_Duzina[duljina1+duljina2];
	*duljina=duljina1+duljina2;
	int i;
	for (i = 0;i < duljina1; i++)
		rez[i]=src1[i];
	for (i = duljina1;i < duljina1+duljina2; i++)
		rez[i]=src2[i-duljina1];
	return rez; 
}

Duzina *duzinacat(Duzina *src1, int duljina1, Duzina *src2, int duljina2,int *duljina)

{
Duzina *rez = new Duzina[duljina1+duljina2];
*duljina=duljina1+duljina2;
int i;
for (i = 0;i < duljina1; i++)
 rez[i]=src1[i];
for (i = duljina1;i < duljina1+duljina2; i++)
	rez[i]=src2[i-duljina1];
return rez; 
}

void poklopi_kartu2(Razlomljena_Duzina *karta, int duljina)
{
	for (unsigned int i = 0;i<duljina;i++)
	{
		cout << (i*1.00/duljina)*100 << "% finished\n";
		for (unsigned int j=0;j<duljina;j++)
		if (i!=j && karta[i].A.size()>0 && karta[j].A.size()>0) // if lines are not the same and no error in line size
		{
				
		
							
				//cout << "Spajam";
				Razlomljena_Duzina tmp = spoji2(karta[i],karta[j]);
				//cout << "Uspjelo";
				if (tmp.A.size()>0)				
				{
					karta[i].B.clear();
					karta[i].A.clear();
					karta[j]=tmp;
					//*duljina--;
				}
		
		}
	}	

}

Duzina *poklopi_kartu(Duzina *rez, int duljina,int duljina_nove)
{
// Funkcija poklapa slicne duzine na karti i vraca kartu	
// duljina je duljina trenutne karte (gdje su uklopljene nove linje)
// duljina_nove je duljina nove karte koja je prethodno uklopljena
	int j,i;
	Duzina tmp;
		
	for (i = 0;i<duljina-duljina_nove;i++)
	{
		for (j=duljina_nove;j<duljina;j++)
			if (i!=j && rez[i].prvi == 1 && rez[j].prvi == 1) // osigurava da se samo prvi kandidati spoje
		{
			if (rez[i].vratiA() != rez[i].vratiB())
				if (rez[j].vratiA() != rez[j].vratiB())
			{
				tmp = spoji(&rez[i],&rez[j]);
				if (rez[i].vratiPravac()==rez[i].vratiPravac())
				{
				// Ako su se spojili fitani[i] i fitani[j]
					int J = j;
					while (rez[J].slijedeci!=j)
					{
						// Prolazi kroz sve duzine na praavcu sa fitani[j] i updatea ih
						int tmpprvi=rez[J].prvi, tmpslj=rez[J].slijedeci, tmppr=rez[J].prethodni;
						rez[J]=najblizaPravcu(rez[J].vratiA(),rez[J].vratiB(),rez[j].vratiPravac());
						rez[J].prvi=tmpprvi; rez[J].slijedeci=tmpslj; rez[J].prethodni=tmppr;
	
						
						J=rez[J].slijedeci;
						if (J==-2) break;
					}
					int I=i;
					while (rez[I].slijedeci!=i)
					{
						// Prolazi kroz sve duzine na praavcu sa fitani[j] i updatea ih
						int tmpprvi=rez[I].prvi, tmpslj=rez[I].slijedeci, tmppr=rez[I].prethodni;
						rez[I]=najblizaPravcu(rez[I].vratiA(),rez[I].vratiB(),rez[i].vratiPravac());
						rez[I].prvi=tmpprvi; rez[I].slijedeci=tmpslj; rez[I].prethodni=tmppr;
						
						int J = j;
						while (rez[J].slijedeci!=j)
						{
							Duzina tmpI, tmpJ;
							tmpI = rez[I];
							tmpJ = rez[J];
							tmp = spoji(&tmpI,&tmpJ); 
							J=rez[J].slijedeci;
							if (tmp.vratiA()!=tmp.vratiB())
							{
								
								rez[rez[I].prethodni].slijedeci=rez[I].slijedeci;
								rez[rez[I].slijedeci].prethodni=rez[I].prethodni;
								rez[I]=Duzina(Tocka(1,1),Tocka(1,1));
								rez[I].prethodni = -2;
								rez[I].slijedeci = -2;
								
						
								int tmpprvi=rez[J].prvi, tmpslj=rez[J].slijedeci, tmppr=rez[J].prethodni;
								rez[J]=Duzina(tmp.vratiA(),tmp.vratiB(),rez[i].vratiPravac().vratiVarijanca());
								rez[J].prvi=tmpprvi; rez[J].slijedeci=tmpslj; rez[J].prethodni=tmppr;
	
	
							}
							if (J==-2) break;
						}	
						
						I=rez[i].slijedeci;
						if (I==-2) break;
					}
				}				
					
				
			}
		}
	}
return rez;
}

Razlomljena_Duzina* update_karte(Razlomljena_Duzina* karta,int *br_duzina, Tocka *ocitanja,Tocka G, double fi, Tocka Xp, NEWMAT::Matrix Pg)
{
// Funkcija stvara kartu iz predanih joj ocitanja i spaja tu kartu sa postojecom u globalnom k.s.

int br_duzina_nova;
vector <Razlomljena_Duzina> nova_karta2;
printf("update_karte: vectorizing start\n");
Razlomljena_Duzina* nova_karta3 =  vektoriziraj(&ocitanja[0],Xp, &br_duzina_nova, 570);
printf("update_karte: vectorized\n");
// dobivena je karta u odnosu na lok. k.s
transformiraj2(nova_karta3,br_duzina_nova,G,fi,&Xp,Pg);


// karta je prebacena u globalni k.s

karta=duzinacat2(karta, *br_duzina, nova_karta3,br_duzina_nova,br_duzina);
return karta;
//poklopi_kartu2(karta,*br_duzina);
}
bool comp (Duzina a,Duzina b) 
{ 
	if (a.vratiA() == Tocka(0,0) || b.vratiA() == Tocka(0,0))
	return (a.vratiB().kut(Tocka(0,0))<b.vratiB().kut(Tocka(0,0))); 
	return (a.vratiA().kut(Tocka(0,0))<b.vratiA().kut(Tocka(0,0))); 

}

int glavni(void)//(int argc, char *argv[])
{	
	NEWMAT::Matrix Pg1(3,3);
	double Px=0.005;
	double Py=0.005;
	double Pth=0.001;
	Pg1 << Px<< 0 << 0
			<< 0  << Py <<0 
			<< 0  << 0 <<Pth;

	Tocka ocitanja[361], Xp1, G1,Xp2, G2,Xp21;
	double fi2;
	
	
	Razlomljena_Duzina* karta;
	Razlomljena_Duzina* tmp;

	int br_duzina=0;
	
	//karta.clear();
	
	int k;

//karta.clear();
	vector <Duzina> globskocni;
	globskocni.reserve(360);
for (k = 1; k<650 ; k++)
if (k==1 || k==57 ||k==117 || k==224 ||k==274 || k==324 ||k==421 || k==521 ||k==637)
//if (k%2==0)
{	
	
	cout << "k:" << k << "\n";
		//Pg1 = ocitaj_varijancu("./Nocitanja/podaci"+IntToString(k)+".dat");
		ocitaj("./ocitanja/podaci"+IntToString(k)+".txt",&ocitanja[0],&Xp2, &G2, &fi2);
		karta=update_karte(karta,&br_duzina,&ocitanja[0],G2,fi2,Xp2,Pg1);
		
		int duljinatmp = 0;
		tmp = vektoriziraj(&ocitanja[0],Xp2, &duljinatmp, 570);
		Snimi ("skripta"+IntToString(k)+".m", tmp, duljinatmp);

		// U tmp se nalaze stvarni bridovi kao razlomljene duzine: 
		
		vector <Duzina> stvarni;
		vector <Duzina> skocni;
		stvarni.reserve(360);
		skocni.reserve(360);
			
		for (int i=0;i<duljinatmp;i++)
		{
			for (int j = 0;j<tmp[i].A.size();j++)
			{
			
				stvarni.push_back(tmp[i].vratiDuzinu(j));
				
					
			}
		}
		
		sort(stvarni.begin(),stvarni.end(),comp);
		//SnimiSkocne ("stvarni"+IntToString(k)+".m", stvarni);
		for (int i=0;i<stvarni.size()-1;i++)
		{
			
			Duzina potskocni = stvarni[i].MaliRazmak(stvarni[i+1]);
			
			if (potskocni.duljina()> 0.7) //trebadeltas
				skocni.push_back(potskocni);
			if (i == stvarni.size()-2)
			{
			potskocni = stvarni[i+1].VelikiKutMaliRazmak(stvarni[0]);
			if (potskocni.duljina()> 0.7) //trebadeltas
				skocni.push_back(potskocni);
			}
		}
		
		vector<Duzina> transformirani=transformiraj3(skocni, G2,fi2,&Xp2,Pg1);
		globskocni.insert (globskocni.end(), transformirani.begin(),transformirani.end());
		 
		
		stvarni=transformiraj3(stvarni, G2,fi2,&Xp2,Pg1);
		
		
		 // eliminacija onih kojima jedan brid lezi na novoj karti
		 
	// Prolazi kroz sve skocne bridove
	// Prolazi kroz cijelu kartu
		
		for (int I = 0; I<globskocni.size();I++)
		{
			
			for (int j = 0; j<br_duzina;j++)
			{
				for (int K = 0; K<karta[j].A.size();K++)
				{
					
					Tocka T; 
					if (karta[j].vratiDuzinu(K) - globskocni[I].vratiA() < karta[j].vratiDuzinu(K) - globskocni[I].vratiB()) 
					{
						T=globskocni[I].vratiA();
					}
					else
					{
						T=globskocni[I].vratiB();
					}
					
					if (karta[j].vratiDuzinu(K) - T <0.15 && T- karta[j].vratiDuzinu(K).vratiB()>0.05 && T- karta[j].vratiDuzinu(K).vratiA()>0.05 > 0.05)
						{
							globskocni[I]=Duzina(Tocka(1,1),Tocka(1,1));
						};
									       
						
				}	
			}
		}		
	
		

		 /*
		for (acskocni=0;brojacskocni<globskocni.size();brojacskocni++)
			 for (brojacstvarni=0;brojacstvarni<karta.size();brojacstvarni++)
			for (int brojackarta=0;brojacskarta<karta[brojacstvarni].A.size();brojackarta++))
				
		 		{
					if (karta[brojacstvarni].vratiDuzina(brojackarta)-globskocni[brojacskocni].vratiA() < 0.1 || stvarni[brojac.stvarni]-globskocni[brojacskocni].vratiB() < 0.1)
						globskocni[brojacskocni]=Duzina(1,1);
		 		}
	*/	
		
		
		
		SnimiSkocne ("skocni"+IntToString(k)+".m", skocni);
		// Ovo je dodano i prevelike je slozenosit !!!
}
SnimiSkocne ("globskocni.m", globskocni);
Snimi ("skriptaNS.m", karta, br_duzina);
poklopi_kartu2(karta,br_duzina);

Snimi ("skripta.m", karta, br_duzina);
return EXIT_SUCCESS;
}	
