//
// C++ Implementation: razlomljeni_pravac
//
// Description: 
//
//
// Author: Marija Seder,,, <marija@marija-laptop>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "razlomduzina.h"
void Razlomljena_Duzina::print(void)
{
	for (int i=0;i<A.size();i++) 
	{
		vratiDuzinu(i).print();
		cout << " ";
	}
}
void Razlomljena_Duzina::postaviVar(NEWMAT::Matrix P)
{
	p.postaviVar(P);
}
void Razlomljena_Duzina::dodaj(Duzina CD)
{

// Dodaje duzinu na razlomljenu duzinu
// Provjerava da li se duzine preklapaju 
// 2. sortira duzine tako da da je A lijeva a B desna tocka

	double epsilon = 0.15;
	int i;
	Tocka T1 = CD.vratiA();
	Tocka T2 = CD.vratiB();

	int indT1 = -1, indT2=-1;
	Tocka TminT, TmaxT;

	if (T1-Tocka(-10000,-10000) < (T2-Tocka(-10000,-10000)))
	{
		TminT = T1;
		TmaxT = T2;
	}
	else
	{
		TminT = T2;
		TmaxT = T1;
	}
	
	for (i=0;i<A.size();i++)
	{
		if (A.at(i)-Tocka(-10000,-10000) < (TminT-Tocka(-10000,-10000)))
		{
			TminT = A.at(i);
		}
		if (A.at(i)-Tocka(10000,10000) < (TmaxT-Tocka(10000,10000)))
		{
			TmaxT = A.at(i);
		}
		if (abs(((T1-A.at(i)) + (T1-B.at(i))-(A.at(i)-B.at(i))) < epsilon)) indT1 = i;
		if (abs(((T2-A.at(i)) + (T2-B.at(i))-(A.at(i)-B.at(i))) < epsilon)) indT2 = i;

	}
	
	if ((TminT == T1 && TmaxT == T2) || (TminT == T2 && TmaxT == T1) )
	{
		A.clear();
		B.clear();
		A.push_back(TminT);
		B.push_back(TmaxT);
		return;
	}
	
	if (indT1==-1 && indT2==-1)
	{
		if (T1 - Tocka(-10000,-10000) < T2 - Tocka(-10000,-10000))
		{
			A.push_back(T1);
			B.push_back(T2);
		}
		else 
		{
			A.push_back(T2);
			B.push_back(T1);
		}	
	}
	
	if (indT1!=-1 && indT2==-1)
	{
		if (T2 - Tocka(-10000,-10000) < A.at(indT1) - Tocka(-10000,-10000))
		{
			Tocka B1 = B.at(indT1);
			A.erase(A.begin()+indT1);
			B.erase(B.begin()+indT1);
		
			A.push_back(T2);
			B.push_back(B1);
		}
		else 
		{
			Tocka A1 = A.at(indT1);
			A.erase(A.begin()+indT1);
			B.erase(B.begin()+indT1);
		
			A.push_back(A1);
			B.push_back(T2);
		}
	}	
	
	if (indT1==-1 && indT2!=-1)
	{
		if (T1 - Tocka(-10000,-10000) < A.at(indT2) - Tocka(-10000,-10000))
		{
			Tocka B1 = B.at(indT2);
			A.erase(A.begin()+indT2);
			B.erase(B.begin()+indT2);
		
			A.push_back(T1);
			B.push_back(B1);
		}
		else 
		{
			Tocka A1 = A.at(indT2);
			A.erase(A.begin()+indT2);
			B.erase(B.begin()+indT2);
		
			A.push_back(A1);
			B.push_back(T1);
		}
	}	
	
	if (indT1!=-1 && indT2!=-1 &&  (indT1 != indT2))
	{
		Tocka A1,B1;
			
		if (A.at(indT1)- Tocka(-10000,-10000) < A.at(indT2)- Tocka(-10000,-10000))
		{
			A1=A.at(indT1);
			B1=B.at(indT2);
		}
		else
		{
			A1=A.at(indT2);
			B1=B.at(indT1);
		}
		A.erase(A.begin()+indT2);
		B.erase(B.begin()+indT2);
	
	
		A.erase(A.begin()+indT1);
		B.erase(B.begin()+indT1);
		
		A.push_back(A1);
		B.push_back(B1);
	
	}
	nerazlomljena=Duzina(TminT,TmaxT);
};	

Duzina Razlomljena_Duzina::vratiDuzinu(int i)
{
	return Duzina(A.at(i),B.at(i),p.vratiVarijanca());
}

Razlomljena_Duzina::Razlomljena_Duzina(Duzina AB)
{
// 	A.reserve(100);
	dodaj(AB);
	p = AB.vratiPravac();
};

Razlomljena_Duzina::Razlomljena_Duzina() {A.reserve(100);};
		// Defaultni destruktor
Razlomljena_Duzina::~Razlomljena_Duzina() {};
	


Pravac Razlomljena_Duzina::vratiPravac(void) 
{
	return p;
};


Razlomljena_Duzina linefit3(Duzina ref, Tocka *ocitanja,int *duljina, double R_estim1,double alpha_estim1, double sigma_d, double sigma_fi,int detectrupa) 
{
	vector <int> beskorisna;
	beskorisna.reserve(20);
	//cout << "\nNova linija\n";
	CvPoint2D32f* points = (CvPoint2D32f*)malloc( *duljina * sizeof(points[0]));
	NEWMAT::Matrix P(2,2);
	P << 0 << 0 << 0 << 0;	
	double dk,fik, ck,sk,bk,suma_brojnik,suma_nazivnik;
	double Pk, sumaPRR=0,PRR,sumaR=0,P_Ra,P_aa;
	double R_estim=R_estim1, alpha_estim=alpha_estim1;//nove varijable
	int k,zastavica=0;	
	float *line;
	
	double psiP,suma_psiP;

	line = new float[4*(*duljina)];
	suma_psiP=0;	
	
	for (k = 0; k<(*duljina);k++)
	{
 		/*
		double kut2;
		if (k>0)
			kut2 = abs(ref.vratiPravac().najbliza(ocitanja[k-1]).kut(Tocka(0,0)) - ref.vratiPravac().najbliza(ocitanja[k]).kut(Tocka(0,0)));
		if (k==0 || kut2 > PI/720)
		*/
		points[k]=ocitanja[k].convert2cvPointFloat();
		dk = ocitanja[k] - Tocka(0,0);
		fik = ocitanja[k].kut(Tocka(0,0));
		sk = sin(alpha_estim - fik); 
		bk = pow(sigma_d*ck,2) + pow(sigma_fi*dk*sk,2);
		suma_psiP = suma_psiP + (dk*sk/bk);
		sumaPRR = sumaPRR + pow(bk,-1);
	}
	
	
	PRR = pow(sumaPRR,-1);
	psiP = suma_psiP*PRR;

	CvMat point_mat = cvMat( 1,(*duljina),  CV_32FC2 , points );
	cvFitLine( &point_mat, CV_DIST_L2 , 0,(0.000005), (0.0000001),line); //least square method
	double k1, l1;
	
	if (line[0]!=0)
	{
		k1 = line[1]/ line[0];  
		l1 = line[3]- k1*line[2];
		R_estim = l1*sin(atan(-1/k1)); 
		alpha_estim = atan(-1/k1);//ovdje se ne pridruzuju vrijednosti jer su to ulazi  
	}
	else 
	{
		alpha_estim = 0;
	}
	
	while (alpha_estim>2*PI) alpha_estim= alpha_estim - 2*PI;
	
	// Izracunat pravac;
	Razlomljena_Duzina AB;
	AB.A.reserve(50);
	AB.B.reserve(50);
	suma_brojnik=0; suma_nazivnik=0; sumaR=0;
    
	double deltaPsi_k; sumaPRR=0;
	double sumaP_Ra1=0, sumaP_aa1=0,minA=DOMET+1, minB=DOMET+1;
	Tocka najblizaA, najblizaB;
	
	
	zastavica = -2;
	Pravac p = Pravac(R_estim,alpha_estim,P);
	double L1;
	double L2;
/*		
	for (k=0;k<(*duljina);k++)
	{   
		
		// Odredjujem stvarni razmak izmedju tocaka
		Tocka Tocka1, Tocka2;
		if (k>0)
		{
			// Na ovaj nacin predajem prosli d_k i fi_k mijenjajuci ih poslije ifa
			Tocka1 = Pravac(R_estim,alpha_estim).najbliza(ocitanja[k]);
			Tocka2 = Pravac(R_estim,alpha_estim).najbliza(ocitanja[k-1]);
			
			//cout << "\nPoc\n";
			//ocitanja[k-1].print();
			//cout << "\nStvarna\n";
			//ocitanja[k].print();
			//
			L1=L(dk,fik,alpha_estim,R_estim);
         		L2 = Tocka1-Tocka2;
			//if (L2>1.5*L1) cout << L1 << "vs"<<L2<< Tocka1.kut(Tocka(0,0))-Tocka2.kut(Tocka(0,0))<<"\n" ; 
		
		}
		
		
		dk = ocitanja[k] - Tocka(0,0);
		fik = ocitanja[k].kut(Tocka(0,0));
	
	
		// Odredjujem kada treba razlomiti duzinu
		
		if (zastavica==1 || (zastavica==-2 && k==0))
		{
			najblizaA = ocitanja[k];
			najblizaB = ocitanja[k];
			minA =ocitanja[k]-ref.vratiA();
			minB = ocitanja[k]-ref.vratiB();			
		}
		
		double kut; 
		kut = abs(ocitanja[k].kut(Tocka(0,0))-ocitanja[k-1].kut(Tocka(0,0)));
		
		//(((L2)>(L1)) && L2!=0)
		if (k>0 && (L2>10 )  && detectrupa == 1 )//|| L2 < 0.5*L1))
		{
			Duzina CD = Duzina(p.najbliza(najblizaA),p.najbliza(najblizaB),P);
			if (CD.duljina() > 0) 
			{
				if (zastavica == -2)
				{
					AB = Razlomljena_Duzina(CD);
				}
				else
				{
					AB.dodaj(CD);
				}
			}
			zastavica=1; // Odredjujem da treba naci nove najblize
		
		}
		if (zastavica == -2 && k==*duljina-1)
		{
			Duzina CD = Duzina(p.najbliza(najblizaA),p.najbliza(najblizaB),P);
			AB = Razlomljena_Duzina(CD);
		}
		// Odredjuje tocke najblize referentnim
		if (ocitanja[k]-ref.vratiA() < minA)
		{
			minA = ocitanja[k]-ref.vratiA();
			najblizaA = ocitanja[k];
		}
		if (ocitanja[k]-ref.vratiB() < minB)
		{
			minB = ocitanja[k]-ref.vratiB();
			najblizaB = ocitanja[k];
		}
		
		ck = cos(alpha_estim - fik); 
		sk = sin(alpha_estim - fik); 
		bk = pow(sigma_d*ck,2) + pow(sigma_fi*dk*sk,2);
		deltaPsi_k = dk*sk - psiP;       
		Pk = bk;
		sumaP_aa1 = sumaP_aa1 + (pow((deltaPsi_k),2)/Pk);
		sumaP_Ra1 = sumaP_Ra1 + (deltaPsi_k/Pk);
		sumaPRR = sumaPRR + pow(bk,-1);
	}
	*/	
	AB.A.clear();
	AB.B.clear();
	
	for (int i=0;i<*duljina;i++)
	{
	
		dk = ocitanja[i] - Tocka(0,0);
		fik = ocitanja[i].kut(Tocka(0,0));
		ck = cos(alpha_estim - fik); 
		sk = sin(alpha_estim - fik); 
		bk = pow(sigma_d*ck,2) + pow(sigma_fi*dk*sk,2);
		deltaPsi_k = dk*sk - psiP;       
		Pk = bk;
		sumaP_aa1 = sumaP_aa1 + (pow((deltaPsi_k),2)/Pk);
		sumaP_Ra1 = sumaP_Ra1 + (deltaPsi_k/Pk);
		sumaPRR = sumaPRR + pow(bk,-1);
	
		L1=5*L(dk,fik,alpha_estim,R_estim);
		L2=(ocitanja[i])-(ocitanja[i-1]);
		double kut,kut2;
		if (i==0)
		{
			najblizaA = ocitanja[0];
			najblizaB = ocitanja[0];
			continue;
		}
		if (i>0)
		{
			kut = abs(ocitanja[i-1].kut(Tocka(0,0)) - ocitanja[i].kut(Tocka(0,0)));//kut moze bit veci od PI
		 	kut2 = abs(Pravac(R_estim, alpha_estim).najbliza(ocitanja[i-1]).kut(Tocka(0,0)) - Pravac(R_estim, alpha_estim).najbliza(ocitanja[i]).kut(Tocka(0,0)));//kut moze bit veci od PI
//		 	printf("PI/360+5*sigma_fi=%f kut=%f kut2=%f\n",(PI/360+5*sigma_fi)*180/PI, kut*180./PI, kut2*180./PI);
		 	if (kut>PI) kut=abs(kut-2*PI);
		 	if (kut2>PI) kut2=abs(kut2-2*PI);
		}
		
		if ((i>0 && i<=*duljina-1) && (kut > 2*PI/360 + 5*sigma_fi || kut < PI/360 - 5*sigma_fi || kut2 < PI/2000 ) || L2 > MINSKOCNI) //dodajem <= zbog ovog dolje uvjeta jer inace ne upise brid kojem je zadnja tocka odmaknuta
		{
		najblizaB=ocitanja[i-1];
		if (najblizaA!=najblizaB)
		{
			najblizaA = Pravac(R_estim, alpha_estim).najbliza(najblizaA);
			najblizaB = Pravac(R_estim, alpha_estim).najbliza(najblizaB);
			AB.dodaj(Duzina(najblizaA,najblizaB));
		}
		else
		{
			//ocitanja[i] = Tocka(DOMET+1,DOMET+1);
			beskorisna.push_back(i);
		}
		najblizaA=ocitanja[i];
		najblizaB=ocitanja[i];
		}
		if ((i==*duljina-1) &&  (kut <= 2*PI/360 + 5*sigma_fi && kut >= PI/360 - 5*sigma_fi))//ne radi dobro ako bas zadnja tocka ima razliku u kutu (desi se da ne upise onda brid ako nema razlomljenih duzina)
 		{
			najblizaB=ocitanja[i];
			najblizaA = Pravac(R_estim, alpha_estim).najbliza(najblizaA);
			najblizaB = Pravac(R_estim, alpha_estim).najbliza(najblizaB);
		
		if (najblizaA!=najblizaB)
		{
			AB.dodaj(Duzina(najblizaA,najblizaB));
		}
		else
		{
			//ocitanja[i] = Tocka(DOMET+1,DOMET+1);
			beskorisna.push_back(i);
		}
		}
	}
	
	for (int i=0;i<beskorisna.size();i++)
	{
	ocitanja[beskorisna[i]]=Tocka(DOMET+1,DOMET+1);
	}
	PRR = pow(sumaPRR,-1);
	P_aa = 1/sumaP_aa1;
	P_Ra = - sumaP_Ra1*PRR*P_aa;

	//P << PRR << (P_Ra) << (P_Ra) << P_aa;
	P << 0 << 0 << 0 << 0;    //no uncertainty in line fitting
	AB.postaviVar(P);

	//if (AB.duljina() < (minLINIJA/100)) AB = Duzina(Tocka(1,1),Tocka(1,1),P);
	return AB;

}
