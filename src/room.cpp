#include "room.h"




//constructor

room::room(){
	LR.reserve(100);                 // reserve room storage
 	NP.reserve(100);                 // reserve NP storage
 	soba.reserve(500);
 	R.reserve(500);              // reserve 50 by 100 vector array
 	soba.reserve(500);
 	sobni.reserve(60);
 	poly.reserve(50);
 	brojanje=0;

}

//void room::Room_detector(){
void room::Room_detector(gpc_polygon &gpc_globalni_poligon, vector<Duzina> & globskocni){

	Tocka T;
	vector <Duzina> pomPoligon, sobatemp;
	int dalje=0;
	float Area_temp;
	int d1,d2,flagn1,flagn2;
	R.clear();
    LR.clear();
    Duzina pom;
    int det=0;


    for (int i=0;i<soba.size();i++){
    	if (det==1)
    		break;
dalje=1;
for (int real=0; real<sobni.size();real++){                            //start only from new lines
	if (soba[i].line-sobni[real]<0.01){
		dalje=0;
		break;
	}
}



		if ((soba.at(i).inroom==1) || (dalje==1))  // if the line has already been detected as room_line
		continue;

/*
    flagn1=0;   // flags for what
		flagn2=0;


		// if the second nearest neighbor is not directly connected
	for (int M=0; M<soba.size();M++){
		if (M==i)
			continue;



	if ( ( (soba.at(i).line.vj	ratiA()-soba.at(M).line.vratiA())<0.01) || ( (soba.at(i).line.vratiA()-soba.at(M).line.vratiB())<0.01)){
		flagn1=1;
	}

	if ( ( (soba.at(i).line.vratiB()-soba.at(M).line.vratiA())<0.01) || ( (soba.at(i).line.vratiB()-soba.at(M).line.vratiB())<0.01)){
			flagn2=1;
		}

	}


	if (flagn1==1 && flagn2==1)
		continue;

*/


	NP.clear();
	LR.clear();
	//cout <<"starting line " << i << endl;
	LR.push_back(i);



	while (1){

		vrati_najblizu();


		//check crossing unexplored region
		if (najbliza!=-1){ //if the nearest line exist
               // cout << "ipak je pronaslo susjeda"<< endl;
		if (soba.at(LR.back()).oznaka==1){
			//if the line is the same as globalskocni jump edge then skip


				 if (soba.at(najbliza).oznaka==1)
					 pom=Duzina(soba.at(LR.back()).line.vratiA(),soba.at(najbliza).line.vratiB());
				 else
					 pom=Duzina(soba.at(LR.back()).line.vratiA(),soba.at(najbliza).line.vratiA());
		 }
		 else {
			 if (soba.at(najbliza).oznaka==1)
			 			 pom=Duzina(soba.at(LR.back()).line.vratiB(),soba.at(najbliza).line.vratiB());
			 		 else
			 			 pom=Duzina(soba.at(LR.back()).line.vratiB(),soba.at(najbliza).line.vratiA());
		 }


		int zastava=0;
					for (int z=0;z<globskocni.size();z++){
						if ((pom-globskocni[z])<0.01){ 
								zastava=1;
                                                               // cout << "veza je globalni skocni"<<endl;
                                                 }
					}

                
			if (zastava==0 && udaljenost>0.01){   //if the nearest line is not connected by jump edge it could cross unexplored area

		if (!pointInPolygon(&gpc_globalni_poligon,pom.poloviste())){
			 //cout << "ova tocka nije u polygonu " <<pom.poloviste().vratiX()<<" "<<pom.poloviste().vratiY()<<endl;
				 najbliza=-1;
		 }
		
                }
	

        	}

    

		if ((najbliza==LR.front())  && (LR.size()>15)){  //room is detected    // && (d2<d1)





				Area_temp=polygon_area();
        SnimiSkocne("InRoom.m",poly);
				cout << "The room is detected, press any key the room area is "<<Area_temp << "and number of breakes is " <<broj_prekida<< endl;
				if (Area_temp<MIN_AREA || (broj_prekida>7)){
				/*	for (uint brojac1=0; brojac1<R[0].size(); brojac1++){
					            cout << "element " << LR[brojac1]<<endl;
					          sobatemp.push_back(soba[LR[brojac1]].line);
					                 }

					SnimiSkocne("losa_soba"+IntToString(brojanje++)+".m",sobatemp);
				*/
                                NP.push_back(LR.back());    // the line in LR is put in NP
			if (LR.size()==1)           // if the starting line doesn't have a neighbor
				break;                  // break while loop

			LR.pop_back();              // delete last  element

                                     	continue;
				}
		
				 //====================================================================================
				//there should be another check point for detected room
				// choose one random point near the first line
				float kut=soba[LR.front()].line.vratiPravac().vratiTheta();
				T=soba[LR.front()].line.poloviste()+Tocka(0.01*cos(kut),0.01*sin(kut));
				//check whether the point is inside the detected room and if so check the main polygon belonging
                gpc_add_DuzinaPolygon(&gpcpoly,poly);  // from duzina polygon to gpc polygon
				int good=0;
        cout<< "the first point T is "<<T.vratiX() <<" " << T.vratiY()<<endl;
				if (pointInPolygon(&gpcpoly, T)) {   // if the point is inside the new room then check belonging to exploration polygon
        cout <<"the first point T is inside the polygon" <<endl;
					if (pointInPolygon(&gpc_globalni_poligon, T)){
        cout <<"the first point T is inside the main polygon" <<endl;
						good=1;
					}
				}
				else {
      
					T=soba[LR.front()].line.poloviste()+Tocka(-0.01*cos(kut),-0.01*sin(kut));
		 cout<< "the second point T is "<<T.vratiX() <<" " << T.vratiY()<<endl;
    if (pointInPolygon(&gpc_globalni_poligon, T)){
    cout << "the second point is inside the main polygon"<<endl;
						good=1;
					}
				}


				if ((good==0)) {
					cout << "the detected room is outside explored region" << endl;
					break;
				}

				det=1;                            //set flag - room is detected

				getchar();
				R.push_back(LR);                 // put detected room in Room buffer

			for (int I=0;I<LR.size();I++) {
				soba[LR[I]].inroom=1;
				//soba[I].line=Duzina(Tocka(0,0),Tocka(0,0));   // line in detected room is deleted (set to (0,0))
			}

			break;
		//}

	}
		else if (najbliza==-1) {        // if the nearest line doesn't exist
                       // cout << "najbliza linija nema susjeda i jbg to je tako " << endl;
                       // cout << "nema susjeda " << soba.at(LR.back()).line.vratiB().vratiX() << " " <<  soba.at(LR.back()).line.vratiB().vratiY()<< " "<<soba.at(LR.back()).line.vratiA().vratiX() << " "<< soba.at(LR.back()).line.vratiA().vratiY()<<endl;
			NP.push_back(LR.back());    // the line in LR is put in NP
			if (LR.size()==1)           // if the starting line doesn't have a neighbor
				break;                  // break while loop

			LR.pop_back();              // delete last  element
		}

		else {                                    // the nearest line has been found
			soba.at(najbliza).oznaka=oznaka;      // mark the end of line which is going to be tested next
			LR.push_back(najbliza);
		}

		}//while

}//for

}//room_detector




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void room::vrati_najblizu(){


	float temp1,temp2,temp;
	oznaka=0;
	najbliza=-1;
	udaljenost=10000;  //some large number
	int oz=0;



	for (int j=0; j<soba.size();j++)
	{

		if (LR.back()==j || soba.at(j).inroom==1) // if the line already belongs to some other room or checking the nearest line with the same line
		continue;

	//for every line find distance for both line ends
	if (soba.at(LR.back()).oznaka==1){
	temp1=soba.at(LR.back()).line.vratiA()- soba.at(j).line.vratiA(); //distance between last line in LR and other lines
	temp2=soba.at(LR.back()).line.vratiA()- soba.at(j).line.vratiB(); //
	}

	else {
		temp1=soba.at(LR.back()).line.vratiB()- soba.at(j).line.vratiA(); //distance between last line in LR and other lines
		temp2=soba.at(LR.back()).line.vratiB()- soba.at(j).line.vratiB(); //
	}

	temp=temp1;
	oz=3;
	if (temp2<temp1){
	temp=temp2;
		oz=1;
	}


	// if the current line is not in the set NP nor LR
	int zastavica1=0;
	int zastavica2=0;


	for (int i =1; i<LR.size(); i++)

	{
		                if (LR[i]==j){
				zastavica1=1;
                                break;
                                 }
                    
	}




	for (int i =0; i<NP.size(); i++)

	{
		if (NP[i]==j) {
		zastavica2=1;
                break;
                }
	}



	if ((temp<udaljenost) && (temp<prag) && (zastavica1==0) && (zastavica2==0))

	{
		udaljenost=temp;
		najbliza=j;
                //cout << "najbliza variable has been updated and the value is  " << j<< endl;
		oznaka=oz;
	}
    if (udaljenost==0)
    	break;
	}



/*
 // check whether the other line end of which we search for the nearest is maybe nearer (wtf?)
 if (najbliza!=-1){
 if (oznaka==3){
 if
 ((udaljenost>(soba.at(LR.back()).line.vratiB()-soba.at(najbliza).line.vratiA())) &&
 (soba.at(LR.back()).oznaka==1))
    najbliza=-1;
 
else
if
 ((udaljenost>(soba.at(LR.back()).line.vratiA()-soba.at(najbliza).line.vratiA())) &&
 (soba.at(LR.back()).oznaka==3))
 najbliza=-1;
 }
else {
 if
 ((udaljenost>(soba.at(LR.back()).line.vratiB()-soba.at(najbliza).line.vratiB())) &&
 (soba.at(LR.back()).oznaka==1))
    najbliza=-1;
 
else
if
 ((udaljenost>(soba.at(LR.back()).line.vratiA()-soba.at(najbliza).line.vratiB())) &&
 (soba.at(LR.back()).oznaka==3))
 najbliza=-1;
}

 }

*/
  // if najbliza==-1 try to find the nearest line from the other endpoint 
  // repeat the whole procedure
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  /*
 if (najbliza==-1){
 for (int j=0; j<soba.size();j++)
	{

		if (LR.back()==j || soba.at(j).inroom==1) // if the line already belongs to some other room
		continue;

	//for every line find distance for both line ends
	if (soba.at(LR.back()).oznaka==3){
	temp1=soba.at(LR.back()).line.vratiA()- soba.at(j).line.vratiA(); //distance between last line in LR and other lines
	temp2=soba.at(LR.back()).line.vratiA()- soba.at(j).line.vratiB(); //
	}

	else {
		temp1=soba.at(LR.back()).line.vratiB()- soba.at(j).line.vratiA(); //distance between last line in LR and other lines
		temp2=soba.at(LR.back()).line.vratiB()- soba.at(j).line.vratiB(); //
	}

	temp=temp1;
	oz=3;
	if (temp2<temp1){
	temp=temp2;
		oz=1;
	}


	// if the current line is not in the set NP nor LR
	int zastavica1=0;
	int zastavica2=0;


	for (int i =1; i<LR.size(); i++)

	{
		if (LR[i]==j)
				zastavica1=1;
	}




	for (int i =0; i<NP.size(); i++)

	{
		if (NP[i]==j)
		zastavica2=1;
	}



	if ((temp<udaljenost) && (temp<prag) && (zastavica1==0) && (zastavica2==0))

	{
		udaljenost=temp;
		najbliza=j;
		oznaka=oz;
	}

	}
  }
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  */
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


float room::polygon_area(){
	poly.clear();
	float x1,x2,y1,y2,temp;
	float area=0;
	broj_prekida=0;
	for (int p=0;p<LR.size();p++){
		if (soba[LR[p]].oznaka==1){
			x1=soba[LR[p]].line.vratiB().vratiX();
			y1=soba[LR[p]].line.vratiB().vratiY();
			x2=soba[LR[p]].line.vratiA().vratiX();
			y2=soba[LR[p]].line.vratiA().vratiY();
		}
		else {
			x1=soba[LR[p]].line.vratiA().vratiX();
			y1=soba[LR[p]].line.vratiA().vratiY();
			x2=soba[LR[p]].line.vratiB().vratiX();
			y2=soba[LR[p]].line.vratiB().vratiY();
		}


		poly.push_back(Duzina(Tocka(x1,y1),Tocka(x2,y2),true));   // create poylgon for room check
		area=area+(y1-y2)*(x1+x2)/2;



		//check whether the next line is directly connected with previous line
		if (p!=LR.size()-1){
		if (!((soba[LR[p+1]].line.vratiA()==soba[LR[p]].line.vratiA()) || (soba[LR[p+1]].line.vratiA()==soba[LR[p]].line.vratiB()) || (soba[LR[p+1]].line.vratiB()==soba[LR[p]].line.vratiA()) || (soba[LR[p+1]].line.vratiB()==soba[LR[p]].line.vratiB()) ))
		{ //cout << "detektiran prekid " << endl;
		broj_prekida++;
			if (soba[LR[p]].oznaka==1){
					x1=soba[LR[p]].line.vratiA().vratiX();
					y1=soba[LR[p]].line.vratiA().vratiY();
				}
				else {
					x1=soba[LR[p]].line.vratiB().vratiX();
					y1=soba[LR[p]].line.vratiB().vratiY();
				}
				if (soba[LR[p+1]].oznaka==1){
						x2=soba[LR[p+1]].line.vratiB().vratiX();
						y2=soba[LR[p+1]].line.vratiB().vratiY();
					}
					else {
						x2=soba[LR[p+1]].line.vratiA().vratiX();
						y2=soba[LR[p+1]].line.vratiA().vratiY();
					}


				area=area+(y1-y2)*(x1+x2)/2;
				poly.push_back(Duzina(Tocka(x1,y1),Tocka(x2,y2),true));   // create polygon for inside room check

		}
		}

	}


	if (!((soba[LR.front()].line.vratiA()==soba[LR.back()].line.vratiA()) || (soba[LR.front()].line.vratiA()==soba[LR.back()].line.vratiB()) || (soba[LR.front()].line.vratiB()==soba[LR.back()].line.vratiA()) || (soba[LR.front()].line.vratiB()==soba[LR.back()].line.vratiB()) ))
		broj_prekida++;
		//final step
	if (soba[LR.back()].oznaka==1){
		x1=soba[LR.back()].line.vratiA().vratiX();
		y1=soba[LR.back()].line.vratiA().vratiY();
	}
	else {
		x1=soba[LR.back()].line.vratiB().vratiX();
		y1=soba[LR.back()].line.vratiB().vratiY();
	}
 
 
 
 if (soba[LR[1]].oznaka==1){ //find point in the first line which is
                                  //farthest from the second line  
    if
    ((soba[LR[1]].line.vratiB()-soba[LR.front()].line.vratiA())<(soba[LR[1]].line.vratiB()-soba[LR.front()].line.vratiB()))
    {
      x2=soba[LR.front()].line.vratiB().vratiX();
			y2=soba[LR.front()].line.vratiB().vratiY();
}
else {
      x2=soba[LR.front()].line.vratiA().vratiX();
			y2=soba[LR.front()].line.vratiA().vratiY();

}
  }
  else {
  if
  ((soba[LR[1]].line.vratiA()-soba[LR.front()].line.vratiA())<(soba[LR[1]].line.vratiA()-soba[LR.front()].line.vratiB())) 
    {
      x2=soba[LR.front()].line.vratiB().vratiX();
			y2=soba[LR.front()].line.vratiB().vratiY();
}
else {
      x2=soba[LR.front()].line.vratiA().vratiX();
			y2=soba[LR.front()].line.vratiA().vratiY();

}
  }
        

	area=area+(y1-y2)*(x1+x2)/2;
	poly.push_back(Duzina(Tocka(x1,y1),Tocka(x2,y2),true));  

	return fabs(area);
}

