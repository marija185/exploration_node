//
// C++ Implementation: duzina
//
// Description: 
//
//
// Author:  <>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "duzina.h"
// Konstruktor 

	
Duzina::Duzina(Tocka Aa, Tocka Bb) 
	{
	if (fabs(Aa.kut(Tocka(0,0)) - Bb.kut(Tocka(0,0))) <PI) {
		if ((Aa.kut(Tocka(0,0)) < Bb.kut(Tocka(0,0)))) 
		{
			A = Aa;
			B = Bb;
		}
		else 
		{
			B = Aa;
			A = Bb;
		}	
//		printf("kut(A)=%f, kut(B)=%f\n",A.kut(Tocka(0,0))*180./PI,B.kut(Tocka(0,0))*180./PI);
	}else{//za one preko PI vrijedi obrnuta logika
		if ((Aa.kut(Tocka(0,0)) > Bb.kut(Tocka(0,0)))) 
		{
			A = Aa;
			B = Bb;
		}
		else 
		{
			B = Aa;
			A = Bb;
		}
		//printf("kut(A)=%f, kut(B)=%f\n",A.kut(Tocka(0,0))*180./PI,B.kut(Tocka(0,0))*180./PI);
	}
	NEWMAT::Matrix P(2,2); P=0.0;
	p=p.kroz2tocke(Aa,Bb); p.postaviVar(P); 
	}


///////////////////////////////////////////////////////////////
Duzina::Duzina(Tocka Aa, Tocka Bb, bool sort)
	{
	
			A = Aa;
			B = Bb;
		//p=p.kroz2tocke(Aa,Bb); p.postaviVar(P); 
	}
///////////////////////////////////////////////////////////////



Duzina::Duzina(Tocka Aa, Tocka Bb, NEWMAT::Matrix P)
	{
	if (fabs(Aa.kut(Tocka(0,0)) - Bb.kut(Tocka(0,0)))<PI) {
		if ((Aa.kut(Tocka(0,0)) < Bb.kut(Tocka(0,0)))) 
		{
			A = Aa;
			B = Bb;
		}
		else 
		{
			B = Aa;
			A = Bb;
		}	
	}else{//za one preko PI vrijedi obrnuta logika
		if ((Aa.kut(Tocka(0,0)) > Bb.kut(Tocka(0,0)))) 
		{
			A = Aa;
			B = Bb;
		}
		else 
		{
			B = Aa;
			A = Bb;
		}
		//printf("matrixP kut(A)=%f, kut(B)=%f\n",A.kut(Tocka(0,0))*180./PI,B.kut(Tocka(0,0))*180./PI);
	}
		p=p.kroz2tocke(Aa,Bb); p.postaviVar(P); 
	}
Duzina::Duzina() { };

// Destruktor 

Duzina::~Duzina() {};

double Duzina::operator-(Tocka T) 
{
// Vraca udaljenost najblize tocke od duzine
	Tocka T1 = p.najbliza(T);
	if (fabs(((T1-A) + (T1-B)-(A-B))) < 0.0000001)
	{
	//printf("muuu\n");
		return (T1-T);
	}
	else
	{
		if ((T-A) < (T-B))
		{
			return T-A;
		}
		else
		{
			return T-B;
		}
	}
};

bool Duzina::operator==(Duzina druga)
{
	return ((A ==  druga.vratiA() && B ==  druga.vratiB() )|| (B ==  druga.vratiA() && A ==  druga.vratiB() ));
};
double Duzina::operator-(Duzina druga) 
{
// Metoda vraca udaljenost dvije duzine u smislu maksimalne udaljenosti njihovih krajnjih tocaka od duzine 
// ali tako da vrijedi komutativnost, znaci ako se duzine preklapaju vratiti ce najvecu udaljenost one krajnje tocke koja se preklapa sa drugom duzinom	
	
	double kandidat1, kandidat2;
	
	if (fabs(*this-druga.vratiA()) > fabs(*this-druga.vratiB())) 
	{
		kandidat1 = fabs(*this-druga.vratiA())  ;
	}
	else
	{
		kandidat1 = fabs(*this-druga.vratiB())  ;
	}
	
	if (druga-(*this).vratiA() > druga-(*this).vratiB()) 
	{
		kandidat2 = fabs(druga-(*this).vratiA())  ;
	}
	else
	{
		kandidat2 = fabs(druga-(*this).vratiB())  ;
	}
	
	if (kandidat1 < kandidat2) return kandidat1;
	else return kandidat2;				
};



Pravac Duzina::vratiPravac(void) 
{
	return p;
};

double Duzina::duljina(void)
{
// vraca duljinu duzine
	return A-B;
}

Duzina Duzina::VelikiRazmak(Duzina druga) 
{

// Metoda vraca duzinu koja je najdulja medju svim kombinacijama spajanja tocaka
	double max;
	Duzina tmp;
	max = A - druga.A;
	tmp = Duzina(A,druga.A);

	if (A-B > max)
	{
		max=A-B;
		tmp = Duzina(A,B);
	}


	if (druga.A-druga.B > max)
	{
		max=druga.A-druga.B;
		tmp = Duzina(druga.A,druga.B);
	}

	if (A - druga.B > max)
	{
		max = A - druga.B; 
		tmp = Duzina(A,druga.B);
	}
	if (B - druga.A > max)
	{
		max = B - druga.A;
		tmp = Duzina(B,druga.A);
	}
	if (B - druga.B > max)
	{
		max = B - druga.B;
		tmp = Duzina(B,druga.B);
	}

	if (A - druga.A > max)
	{
		max = A - druga.A;
		tmp = Duzina(A,druga.A);
	}

	return tmp;
};

Tocka Duzina::vratiA(void) 
{
	return A;
}
Tocka Duzina::vratiB(void) 
{
	return B;
}

Duzina Duzina::MaliRazmak(Duzina druga) 
{
//falilo je svodjenje na -PI,PI
	// Vraca duzinu odredjenom najblizim kutem 
	// Vraca manji od dva razmaka medju tockama npr AB i CD vraca najmanji od AC AD i BC i BD
	double min;
	Duzina tmp;
	min = fabs(A.kut(Tocka(0,0)) - druga.A.kut(Tocka(0,0)));
	if (min>PI)
	min=fabs(min-2*PI);
	
	tmp = Duzina(A,druga.A);
	double kut =  fabs(A.kut(Tocka(0,0)) - druga.B.kut(Tocka(0,0)));
	if (kut>PI)
	kut=fabs(kut-2*PI);
	if (kut < min)
	{
		min = kut; 
		tmp = Duzina(A,druga.B);
	}
	kut =  fabs(B.kut(Tocka(0,0)) - druga.A.kut(Tocka(0,0)));
	if (kut>PI)
	kut=fabs(kut-2*PI);
	if (kut < min)
	{
		min = kut;
		tmp = Duzina(B,druga.A);
	}
	kut =  fabs(B.kut(Tocka(0,0)) - druga.B.kut(Tocka(0,0)));
	if (kut>PI)
	kut=fabs(kut-2*PI);
	if (kut < min)
		tmp = Duzina(B,druga.B);
	return tmp;

};

Duzina Duzina::VelikiKutMaliRazmak(Duzina druga) 
{
	// Vraca duzinu odredjenom najdaljim kutem 
	// Vraca manji od dva razmaka medju tockama npr AB i CD vraca najmanji od AC AD i BC i BD
	double min;
	Duzina tmp;
	min = fabs(A.kut(Tocka(0,0)) - druga.A.kut(Tocka(0,0)));
	if (min>PI)
	min=fabs(min-2*PI);
	
	tmp = Duzina(A,druga.A);
	double kut =  fabs(A.kut(Tocka(0,0)) - druga.B.kut(Tocka(0,0)));
	if (kut>PI)
	kut=fabs(kut-2*PI);
	
	if (kut > min)
	{
		min = kut; 
		tmp = Duzina(A,druga.B);
	}
	kut =  fabs(B.kut(Tocka(0,0)) - druga.A.kut(Tocka(0,0)));
	if (kut>PI)
	kut=fabs(kut-2*PI);
	
	if (kut > min)
	{
		min = kut;
		tmp = Duzina(B,druga.A);
	}
	kut =  fabs(B.kut(Tocka(0,0)) - druga.B.kut(Tocka(0,0)));
	if (kut>PI)
	kut=fabs(kut-2*PI);
	
	if (kut > min)
		tmp = Duzina(B,druga.B);
	return tmp;

};

void Duzina::print(void) 
{
	A.print();
	cout << "-";
	B.print();
	//cout << "\n Varijanca (*10^-5): \n";
	//cout << 100000*p.vratiVarijanca();
	//cout << "\n";    
};


// Kraj klase dodatak nekih funkcija 

Duzina najblizaPravcu(Tocka A, Tocka B, Pravac p1) 
{
	Tocka A1=p1.najbliza(A);
	Tocka B1=p1.najbliza(B);
	return Duzina(A1,B1,p1.vratiVarijanca());
}

Duzina preklapajuse(Duzina AB, Duzina CD)
{
 // Vraca duzinu odredjenu sa projekcijama preklapajucih tocaka na suprotne duzine
// Zbog numericke nepreciznosti trebam definirati epsilon npr 10^-7
	Tocka A=AB.vratiA();
	Tocka B=AB.vratiB();
	Tocka C=CD.vratiA();
	Tocka D=CD.vratiB();
	Tocka ABC=Tocka(1,1);
	Tocka CDB=Tocka(1,1);
	int zastavica=0;

	Tocka T1 = AB.vratiPravac().najbliza(C);
	if (fabs(((T1-A) + (T1-B)-(A-B))) < 0.0000001)
	{
		ABC =T1;
		zastavica = 1;
	}
	T1 = AB.vratiPravac().najbliza(D);
	if (fabs(((T1-A) + (T1-B)-(A-B))) < 0.0000001)
	{
		ABC =T1;
		zastavica = 1;
	}

	if (zastavica == 1)
	{

		T1 = CD.vratiPravac().najbliza(A);
		if (fabs(((T1-C) + (T1-D)-(C-D))) < 0.0000001)

		{
			zastavica = 2;
			CDB =T1;
		}
		T1 = CD.vratiPravac().najbliza(B);


		if (fabs(((T1-C) + (T1-D)-(C-D))) < 0.0000001)
		{
			zastavica =2;
			CDB =T1;
		}
	}
	Duzina Preklapajuca;
	if (zastavica == 2)
	{
		Preklapajuca=Duzina(ABC,CDB);
	}
	else
	{
		Preklapajuca=Duzina(Tocka(1,1),Tocka(1,1));
	}
	return Preklapajuca;
}

bool uvjet_parc_preklapajuce(Duzina AB, Duzina CD,double uvjet)
{
	Duzina klop = preklapajuse(AB,CD);
	if (klop.vratiA() != klop.vratiB()) 
	{
		if (CD-najblizaPravcu(klop.vratiA(),klop.vratiB(),AB.vratiPravac())<uvjet && klop.duljina() > 0.03) return 1;
	}
	return 0;
}

Tocka Duzina::poloviste(void)
{
	return Tocka((A.vratiX()+B.vratiX())/2,(A.vratiY()+B.vratiY())/2);
};

double L(double dk,double fi,double alpha,double R,double sigma_d, double sigma_fi)
{
	/*
	// Funkcija vraća max razmak koji je dopušten za rupu
	
	// Očitanja svakih 0.5 stupanj
	double pi = PI;
	double x1 = -R*tan(pi/2+fi)/(sin(alpha)-cos(alpha)*tan(pi/2+fi));
	double y1 =  R/(sin(alpha)-cos(alpha)*tan(pi/2+fi));
	fi= fi -2*PI/360; 	
	double x2 = -R*tan(pi/2+fi)/(sin(alpha)-cos(alpha)*tan(pi/2+fi));
	double y2 =  R/(sin(alpha)-cos(alpha)*tan(pi/2+fi));
	double dk2 = Tocka(x2,y2)-Tocka(0,0);
	double fik2 = Tocka(x2,y2).kut(Tocka(0,0));

	//if (Tocka(x1,y1)-Tocka(0,0) > dk)
	//{
		dk2 = dk2-5*sigma_d;
	//}
	//if (Tocka(x1,y1)-Tocka(0,0) < dk)
	//{
		dk2 = dk2+5*sigma_d;
		
	//}
	
	x2 = dk2*cos(fik2-5*sigma_fi);	
	y2 = dk2*sin(fik2-5*sigma_fi);
	*/
	double pi = PI;
	double x1 = dk*cos(fi);
	double y1 = dk*sin(fi);
	
	fi= fi+(PI/360+10*sigma_fi); 	
	double x2 = R/(cos(alpha)+sin(alpha)*tan(fi));
	double y2 = R*tan(fi)/(cos(alpha)+sin(alpha)*tan(fi));
	double dk2 = Tocka(x2,y2)-Tocka(0,0);
	x2 = (dk2+10*sigma_d)*cos(fi);	
	y2 = (dk2+10*sigma_d)*sin(fi);
	
	/*
	cout << "\nUlazna\n";
	Tocka(x1,y1).print();
	cout << "\nEstimirana\n";
	Tocka(x2,y2).print();
	Pravac(R,alpha).najbliza(Tocka(x2,y2)).print();
	*/			
	return (Tocka(x1,y1))-(Tocka(x2,y2));
}











Duzina linefit2(Duzina ref, Tocka *ocitanja,int *duljina, double R_estim,double alpha_estim, double sigma_d, double sigma_fi,int detectrupa) 
{

	
	CvPoint2D32f* points = (CvPoint2D32f*)malloc( *duljina * sizeof(points[0]));
	NEWMAT::Matrix P(2,2);
	
	double dk,fik, ck,sk,bk,suma_brojnik,suma_nazivnik;
	double Pk, sumaPRR=0,PRR,sumaR=0,P_Ra,P_aa;
	int k,zastavica=0;	
	float *line;
	
	double psiP,suma_psiP;

	line = new float[4*(*duljina)];
	suma_psiP=0;	
	int nova_duljina;
	for (k = 0; k<(*duljina);k++)
	{
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
	cvFitLine( &point_mat, CV_DIST_L2 , 0,(0.000005), (0.0000001),line);
	double k1, l1;
	
	if (line[0]!=0)
	{
		k1 = line[1]/ line[0];  
		l1 = line[3]- k1*line[2];
		R_estim = l1*sin(atan(-1/k1)); 
		alpha_estim = atan(-1/k1);  
	}
	else 
	{
		alpha_estim = 0;
	}
	
	suma_brojnik=0; suma_nazivnik=0; sumaR=0;
    
	double deltaPsi_k; sumaPRR=0;
	double sumaP_Ra1=0, sumaP_aa1=0,minA=DOMET+1, minB=DOMET+1;
	Tocka najblizaA, najblizaB,najblizaA1, najblizaB1;
	for (k=0;k<(*duljina);k++)
	{   
		dk = ocitanja[k] - Tocka(0,0);
		fik = ocitanja[k].kut(Tocka(0,0));
		
		// Trazim najblize tocke referentnim tockama
		Pravac p1 = Pravac(R_estim,alpha_estim);
		double L1=L(dk,fik,alpha_estim,R_estim);
		//if (L1<0.5) 
		double L2;
		Tocka Tocka1, Tocka2;
		if (k>0)
		{
			Tocka1 = ocitanja[k];
			Tocka2 = ocitanja[k-1];
			L2 = Tocka1-Tocka2;
		//Tocka1 = p1.najbliza(Tocka1);
		//Tocka2 = p1.najbliza(Tocka2);
		//if (L2 > Tocka1-Tocka2)	L2 = Tocka1 - Tocka2;
		}
		if (k>0 && (((L2)>(1.5*L1) || (L2 < 0.1*L1)) && L1!=L2) && detectrupa == 1 && zastavica == 0)//|| L2 < 0.5*L1))
		{
			nova_duljina=(*duljina)-k;
			zastavica = 1;
			najblizaA1 = najblizaA;
			najblizaB1 = najblizaB;
		}
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
		
	PRR = pow(sumaPRR,-1);
	P_aa = 1/sumaP_aa1;
	P_Ra = - sumaP_Ra1*PRR*P_aa;

	P << PRR << (P_Ra) << (P_Ra) << P_aa; 
    
	while (alpha_estim>2*PI) alpha_estim= alpha_estim - 2*PI;

	Pravac p = Pravac(R_estim,alpha_estim,P);
	Duzina AB;
	if (zastavica==1)
	{
		najblizaA = najblizaA1;
		najblizaB = najblizaB1;
		*duljina = nova_duljina;
		ocitanja=&ocitanja[(k)];
	}
	
	AB = Duzina(p.najbliza(najblizaA),p.najbliza(najblizaB),P);
	//if (AB.duljina() < (minLINIJA/100)) AB = Duzina(Tocka(1,1),Tocka(1,1),P);
	return AB;

};
