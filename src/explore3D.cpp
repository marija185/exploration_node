#include "explore3D.h"



// constructor

explore3D::explore3D() {
	still_room=false;
	brojacPP=0, brojac=0;
	max=0;
	vo.reserve(100);
	vo={'U'};
	dv.reserve(100);
	dv={vo};
	tv.reserve(20);
	tv={dv};
};



void explore3D::explore_room(int br_pokretanja, room Rfind, DStar *DS, GridMap *GM) {


cout << "usao sam u sobu" <<endl;
// getchar();
              	if (still_room==false){


              		brojacPP=0;
              		//initialize old NBV to zero
              		np_old.x=0;
              		np_old.y=0;
              		np_old.z=0;

              		minbax=Rfind.soba[Rfind.R[0][0]].line.vratiA().vratiX();
            	  minbay=Rfind.soba[Rfind.R[0][0]].line.vratiA().vratiY();
              // find the minimum coordinates of the room
            	 for (int ba=0; ba<Rfind.R[0].size();ba++){
            		 if (Rfind.soba[Rfind.R[0][ba]].line.vratiA().vratiX()<minbax)
            			 minbax=Rfind.soba[Rfind.R[0][ba]].line.vratiA().vratiX();
            		 if (Rfind.soba[Rfind.R[0][ba]].line.vratiA().vratiY()<minbay)
            			 minbay=Rfind.soba[Rfind.R[0][ba]].line.vratiA().vratiY();
            		 if (Rfind.soba[Rfind.R[0][ba]].line.vratiB().vratiX()<minbax)
            			 minbax=Rfind.soba[Rfind.R[0][ba]].line.vratiB().vratiX();
            		 if (Rfind.soba[Rfind.R[0][ba]].line.vratiB().vratiY()<minbay)
            			 minbay=Rfind.soba[Rfind.R[0][ba]].line.vratiB().vratiY();
            	 }


            	 //getchar();

            	 minbax=minbax-1;
            	 minbay=minbay-1;
            	cout << "min x i y od pronadene sobe " << minbax <<" "<< minbay<<endl;
				for(int i=0;i<=tv.size()-1;i++){
		for(int p=0;p<=tv[i].size()-1;p++){
			for(int m=0;m<=tv[i][p].size()-1;m++){
				tv[i][p][m]='U'; }}}  // initialize voxel map




            	  // open stream to read from file
            	  //xyzpoint_float *podaci=new xyzpoint_float[423494];
            	  	for (int runi=0; runi<=br_pokretanja; runi++){ //br_pokretanja
            	  		//read pose and if scan was taken inside the room update 3D map
                  	  string pomsrt="./df/scan"+IntToString(runi)+".pose";
                  	  pose.open (pomsrt.c_str());
                      	if(!pose) { // file couldn't be opened
                      			  	      cerr << "Error: file scan"<<runi<<".pose could not be opened" << endl;
                      			  	      exit(-1);
                      			  	   }

                  	  pose >> pozicijax >> pozicijay >> pozicijat;



bool smece=pointInPolygon(&Rfind.gpcpoly,Tocka(pozicijax,pozicijay));
if   (!smece) { //   (!pointInPolygon(&Rfind.gpcpoly,Tocka(pozicijax,pozicijay)))  {    //if the scanning position was inside a detected room
pose.close();
continue;
}






            	  pomsrt="./df/scan"+IntToString(runi)+".3d";
            	  myfile.open (pomsrt.c_str());         // open file

            	if(!myfile) { // file couldn't be opened
            			  	      cerr << "Error: file scan"<< runi  <<".3d could not be opened" << endl;
            			  	      exit(-1);
            			  	   }




rp.x=floor((pozicijax-minbax)/0.2);
rp.y=floor((pozicijay-minbay)/0.2);
rp.z=floor(0.62/0.2);
cout <<"trenutna pozicija robota " << rp.x <<" " << rp.y << " " << rp.z<<endl;
float delta_t;
tv[rp.z][rp.y][rp.x]='E';

//for every laser_data point mark ray intersecting voxels
int brojPo=0;
time(&start);
while (!(myfile.eof())){


      //read from file
      myfile >>pomx >>pomy >>pomz;
      xi=floor((pomx-minbax)/0.2);
      yi=floor((pomy-minbay)/0.2);
      zi=floor((pomz+0.55)/0.2);
      //cout << xi << " " << yi<<" "<<zi<<endl;
      //make sure that indexes are within the boundary and discard some weird obstacles near to the robot
      //tu je kljuc sad
      float distance=sqrt((rp.x-xi)*(rp.x-xi) + (rp.y-yi)*(rp.y-yi) + (rp.z-zi)*(rp.z-zi));


      if (!((zi<0 || zi >tv.size()-1) || (yi<0 || yi>tv[0].size()-1) || (xi<0 || xi>tv[0][0].size()-1)))
      tv[zi][yi][xi]='O';
      if (distance<4)
      continue;

// initial parameters of ray tracing algorithm
if (xi-rp.x==0) {
stepX=0;
tMaxX=100;               //some number higher than 1
}	else {
stepX=(xi-rp.x)/abs(xi-rp.x);
tMaxX=abs(0.5/(xi-rp.x));
}

if (yi-rp.y==0) {
stepY=0;         // doesn't have impact on ray propagation
tMaxY=100;
} else {
stepY=(yi-rp.y)/abs(yi-rp.y);
tMaxY=abs(0.5/(yi-rp.y));
}

if (zi-rp.z==0) {
stepZ=0;
tMaxZ=100;
} else {
stepZ=(zi-rp.z)/abs(zi-rp.z);
tMaxZ=abs(0.5/(zi-rp.z));
}

tDeltaX=2*tMaxX;
tDeltaY=2*tMaxY;
tDeltaZ=2*tMaxZ;
X=rp.x;
Y=rp.y;
Z=rp.z;



while (!(X==xi && Y==yi && Z==zi)) {
if(tMaxX < tMaxY) {
if(tMaxX < tMaxZ) {
      X= X + stepX;
      tMaxX= tMaxX + tDeltaX;
} else {
      Z= Z + stepZ;
      tMaxZ= tMaxZ + tDeltaZ;
}
} else {
if(tMaxY < tMaxZ) {
      Y= Y + stepY;
      tMaxY= tMaxY + tDeltaY;
} else {
      Z= Z + stepZ;
      tMaxZ= tMaxZ + tDeltaZ;
 }
}

if ((Z<0 || Z >tv.size()-1) || (Y<0 || Y>tv[0].size()-1) || (X<0 || X>tv[0][0].size()-1) )
break;

if (tv[Z][Y][X]=='U' || tv[Z][Y][X]=='B') {
tv[Z][Y][X]='E';
//cout << "empty voxel is " << X << " " << Y << " " << Z << endl;
//getchar();
}
//else if  (tv[Z][Y][X]=='O')
//break;


}
}

cout << "ray intersecting points marked" <<endl;
brojac=0;






//cout << "broj occupied " <<brojacU <<" i broj postavljanja occupied je" <<brojPo << endl;
myfile.close();
pose.close();
            	  	} //loop for reading points and map updating

            	} // if (still_room==false)



///////////////////////////////////////////////////////////////////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

              	else {      // still_room==true
              		//update map with the newest scan
              		cout << "usao sam u else " << endl;
              		//getchar();
                                	  string pomsrt="./df/scan"+IntToString(br_pokretanja)+".pose";
                                	  pose.open (pomsrt.c_str());
                                    	if(!pose) { // file couldn't be opened
                                    			  	      cerr << "Error: file scan"<<br_pokretanja<<".pose could not be opened" << endl;
                                    			  	      exit(-1);
                                    			  	   }

                                	  pose >> pozicijax >> pozicijay >> pozicijat;
                                	//if (!pointInPolygon(&Rfind.gpcpoly,Tocka(pozicijax,pozicijay))) //if the scanning position was inside a detected room
                                		//continue;


                          	  pomsrt="./df/scan"+IntToString(br_pokretanja)+".3d";
                          	  myfile.open (pomsrt.c_str());         // open file

                          	if(!myfile) { // file couldn't be opened
                          			  	      cerr << "Error: file scan"<< br_pokretanja  <<".3d could not be opened" << endl;
                          			  	      exit(-1);
                          			  	   }


                        						rp.x=floor((pozicijax-minbax)/0.2);
                                              	rp.y=floor((pozicijay-minbay)/0.2);
                                              	rp.z=floor(0.62/0.2);
                                              	cout <<"trenutna pozicija robota " << rp.x <<" " << rp.y << " " << rp.z<<endl;
                                              	float delta_t;
                                              	tv[rp.z][rp.y][rp.x]='E';

                                              	//for every laser_data point mark ray intersecting voxels
                                              	int brojPo=0;
                                              	time(&start);
                                            	while (!(myfile.eof())){


                                                                                      //read from file
                                                                                      myfile >>pomx >>pomy >>pomz;
                                                                                      //coordinates in voxel map
                                                                                      //cout << pomz << " " << G.vratiX()<<" " << minbax<< endl;
                                                                                     /* xi=floor((pomz/100.*cos(pozicijat)-pomx/100.*sin(pozicijat)+G.vratiX()-minbax)/0.2);
                                                                                      yi=floor((pomx/100.*cos(pozicijat)+pomz*sin(pozicijat)+G.vratiY()-minbay)/0.2);
                                                                                      zi=floor((pomy/100.+0.4325)/0.2);  */
                                                                                      xi=floor((pomx-minbax)/0.2);
                                                                                      yi=floor((pomy-minbay)/0.2);
                                                                                      zi=floor((pomz+0.55)/0.2);
                                                                                      //cout << xi << " " << yi<<" "<<zi<<endl;
                                                                                      //make sure that indexes are within the boundary and don't update for objects near the robot
                                                                                      float distance2=sqrt((rp.x-xi)*(rp.x-xi) + (rp.y-yi)*(rp.y-yi) + (rp.z-zi)*(rp.z-zi));
                                                                                      if (!((zi<0 || zi >tv.size()-1) || (yi<0 || yi>tv[0].size()-1) || (xi<0 || xi>tv[0][0].size()-1)))
                                                                                      tv[zi][yi][xi]='O';
                                                                                      if (distance2<4)
                                                                                      continue;
                                                                                      //cout << "occupied is " << xi <<" " << yi <<" "<< zi << endl;
                                              	// initial parameters of ray tracing algorithm
                                              						if (xi-rp.x==0) {
                                              						    stepX=0;
                                              	                        tMaxX=100;               //some number higher than 1
                                              						}	else {
                                              					        stepX=(xi-rp.x)/abs(xi-rp.x);
                                              					        tMaxX=abs(0.5/(xi-rp.x));
                                              						}

                                              						if (yi-rp.y==0) {
                                              							stepY=0;         // doesn't have impact on ray propagation
                                              							tMaxY=100;
                                              						} else {
                                              							stepY=(yi-rp.y)/abs(yi-rp.y);
                                              							tMaxY=abs(0.5/(yi-rp.y));
                                              						}

                                              						if (zi-rp.z==0) {
                                              							stepZ=0;
                                              							tMaxZ=100;
                                              						} else {
                                              	                       stepZ=(zi-rp.z)/abs(zi-rp.z);
                                              					   	   tMaxZ=abs(0.5/(zi-rp.z));
                                              						}

                                              					tDeltaX=2*tMaxX;
                                              					tDeltaY=2*tMaxY;
                                              					tDeltaZ=2*tMaxZ;
                                              					X=rp.x;
                                              					Y=rp.y;
                                              					Z=rp.z;



                                              					while (!(X==xi && Y==yi && Z==zi)) {
                                              	                   if(tMaxX < tMaxY) {
                                              	                       if(tMaxX < tMaxZ) {
                                              	                                    X= X + stepX;
                                              	                                    tMaxX= tMaxX + tDeltaX;
                                              	                       } else {
                                              	                                    Z= Z + stepZ;
                                              	                                    tMaxZ= tMaxZ + tDeltaZ;
                                              	                              }
                                              	               } else {
                                              	                     if(tMaxY < tMaxZ) {
                                              	                                    Y= Y + stepY;
                                              	                                    tMaxY= tMaxY + tDeltaY;
                                              	                       } else {
                                              	                                    Z= Z + stepZ;
                                              	                                    tMaxZ= tMaxZ + tDeltaZ;
                                              	                               }
                                              	                          }

                                                  	               if ((Z<0 || Z >tv.size()-1) || (Y<0 || Y>tv[0].size()-1) || (X<0 || X>tv[0][0].size()-1) )
                                                  	            	   break;

                                              							   if (tv[Z][Y][X]=='U' || tv[Z][Y][X]=='B') {
                                              						   tv[Z][Y][X]='E';
                                              							 //cout << "empty voxel is " << X << " " << Y << " " << Z << endl;
                                              							//getchar();
                                              							   }
                                              							  // else if (tv[Z][Y][X]=='O'){
                                              								   //cout << "usao u break " << endl;
                                              								//   break; }



                                              	}
                                              	}

                                              	cout << "ray intersecting points marked" <<endl;
                                              	brojac=0;






                                              	//cout << "broj occupied " <<brojacU <<" i broj postavljanja occupied je" <<brojPo << endl;


                                              	                                          	  	myfile.close();
                                              	                                          	  	pose.close();


              	}

/////////////////////////////////////////////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






            	int brojacU=0;
//mark boundary unseen voxels and potential position voxels
int brojacB=0;
brojacPP=0;
ofstream show;
string pomstr="./df/scan00"+IntToString(br_pokretanja)+".3d";
show.open(pomstr.c_str());

for(int i=0;i<=tv.size()-1;i++){
for(int p=0;p<=tv[i].size()-1;p++){
for(int m=0;m<=tv[i][p].size()-1;m++){



if ((i==rp.z) && tv[rp.z][p][m]=='E'){    // if the voxel is empty save the position

//cout << "ima " <<m <<" " <<p << endl;
//getchar();
//check if the position is empty in the gridmap
////////////////////////////////////////////////
R_point laog;
I_point i_goal;
laog.x=(m*0.2+minbax)*1000;      //potential goal position in global coordinates
laog.y=(p*0.2+minbay)*1000;
if (GM->check_point(laog)){             // whether the potential position is within map constraints
i_goal=GM->cell_point_temp;    //potential goal position in the grid map

if((DS->map[i_goal.x][i_goal.y].tag!=NEW))// &&(DS->map[i_goal.x][i_goal.y].traversal_cost<EMPTYC+COST_MASK-2))
{
//			if (pointInPolygon(&Rfind.gpcpoly,Tocka((m*0.2+minbax),(p*0.2+minbay)))){
	//printf("nasao!\n");


PParray[brojacPP].x=m;
PParray[brojacPP].y=p;
brojacPP++;
show<<-100*p <<" " << 100*i << " " <<100*m<< " " <<0 << " " << 0 << " " <<255 <<endl;
	}
//		}
}
}
//if unseen voxel has at least one adjacent 'E' voxel than mark it as 'B' (boundary unseen)

//				cout << "smece iznosi " << i << " "<< p << " " <<m<< endl;
if (tv[i][p][m]=='B')
{
BUarray[brojacB].x=m;
BUarray[brojacB].y=p;
BUarray[brojacB].z=i;
brojacB++;
goto insideloop;
}
else	if (tv[i][p][m]=='U' && pointInPolygon(&Rfind.gpcpoly,Tocka((m*0.2+minbax),(p*0.2+minbay)))){

//cout <<"unseen je " <<m <<" "<<p <<" "<<i<<endl;
//petlja za susjede
for (si=i-1;si<i+2;si++) {
for (sj=p-1;sj<p+2;sj++){
	for(sk=m-1;sk<m+2;sk++){
// if one of the index is outside of map boundary or current voxels considers itselfs

			if (si>tv.size()-1 || si<0 || sj>tv[i].size()-1 || sj<0 || sk>tv[i][p].size()-1 || sk<0 || (si==0 && sj==0 && sk==0)) {

				continue; }

			else {
				if (tv[si][sj][sk]=='E'){    // if exists at least one adjacent voxels with status EMPTY


			tv[i][p][m]='B';

			BUarray[brojacB].x=m;
			BUarray[brojacB].y=p;
			BUarray[brojacB].z=i;
			brojacB++;
			goto insideloop;}
		}  //else
	}

}
}
}
insideloop:        ;
}
}
}


//number of boundary-unseen voxels
//ofstream binfile;
//pomstr="./df/databin.bin";
//binfile.open (pomstr.c_str(), ios::out | ios::binary);
brojac=0;
int brojacBNOVI=0;
//ofstream show;
//pomstr="./df/scan00"+IntToString(br_pokretanja)+".3d";
//show.open(pomstr.c_str());
for(int i=0;i<=tv.size()-1;i++){
for(int p=0;p<=tv[i].size()-1;p++){
for(int m=0;m<=tv[i][p].size()-1;m++){
double xa,ya,za;
if (tv[i][p][m]=='O'){
show<<-100*p <<" " << 100*i << " " <<100*m<< " " <<255 << " " << 0 << " " <<0 <<endl;

//binfile.write((char*) &m,sizeof(m));
//binfile.write((char*) &p,sizeof(p));
//binfile.write((char*) &i,sizeof(i));
}


if (tv[i][p][m]=='B'){
brojacBNOVI++;
show<<-100*p <<" " << 100*i << " " <<100*m<< " " <<0 << " " << 255 << " " <<0 <<endl;
}

if ((i==rp.z) && tv[rp.z][p][m]=='E') {
show<<-100*p <<" " << 100*i << " " <<100*m<< " " <<0 << " " << 0 << " " <<255 <<endl;
                    			 }


}
}
}

//binfile.close();
//write as binary



cout << "number of boundary-unseen voxels is " <<brojacBNOVI << '\n' ;
cout << "number of potential position voxels is " <<brojacPP << '\n' ;


/*if (brojacB>Benough)
still_room=true;
else still_room=false; */

//for every potential position scan position count number of boundary unseen voxels

max=0;


/////////////////////////////////////////////       count the number of boundary unseen voxels from each potential scan position
for(int j=0;j<brojacPP;j++){
brojac=0;            //variable that counts boundary unseen voxels visible from potential scan position

for (int z=0;z<brojacB;z++){                //check every boundary unseen voxel
zastavica=0;
//take laser constraints into account
float tempand=sqrt((PParray[j].x-BUarray[z].x)*(PParray[j].x-BUarray[z].x)+ (PParray[j].y-BUarray[z].y)*(PParray[j].y-BUarray[z].y) + (rp.z-BUarray[z].z)*(rp.z-BUarray[z].z));  // distance from the laser
float tempana=sqrt((PParray[j].x-BUarray[z].x)*(PParray[j].x-BUarray[z].x)+ (PParray[j].y-BUarray[z].y)*(PParray[j].y-BUarray[z].y)); // distance according to the x y plane
float con_angle=acos(tempana/tempand);
//no sensor constraints
if ((rp.z-BUarray[z].z)<0)
{
if (con_angle>60.0*3.1415/180.0) //more than 60 degrees
continue;
}
else {
if (con_angle>60.0*3.1415/180.0) // more than -40 degrees
continue;
}

//

// initial parameters of ray tracing algorithm
if (BUarray[z].x-PParray[j].x==0) {     //the same x coordinate of starting and goal voxel
stepX=0;
tMaxX=100;               //some number higher than 1
}	else {
stepX=(BUarray[z].x-PParray[j].x)/abs(BUarray[z].x-PParray[j].x);  //step is either -1 or 1
tMaxX=abs(0.5/(BUarray[z].x-PParray[j].x));  // time when the ray crosses the first vertical boundary
}

if (BUarray[z].y-PParray[j].y==0) {        //the same x coordinate of starting and goal voxel
stepY=0;
tMaxY=100;
} else {
stepY=(BUarray[z].y-PParray[j].y)/abs(BUarray[z].y-PParray[j].y);
tMaxY=abs(0.5/(BUarray[z].y-PParray[j].y));
}

if (BUarray[z].z-rp.z==0) {
stepZ=0;
tMaxZ=100;
} else {
stepZ=(BUarray[z].z-rp.z)/abs(BUarray[z].z-rp.z);
tMaxZ=abs(0.5/(BUarray[z].z-rp.z));
}

tDeltaX=2*tMaxX;             //time to pass one voxel in x direction
tDeltaY=2*tMaxY;
tDeltaZ=2*tMaxZ;
X=PParray[j].x;
Y=PParray[j].y;
Z=rp.z;

//cout << "provjera za " << PParray[j].x << " " << PParray[j].y<< " " << BUarray[z].x << " "<< BUarray[z].y << " " << " " << BUarray[z].z << endl;
while (!(X==BUarray[z].x && Y==BUarray[z].y && Z==BUarray[z].z)) {
if(tMaxX < tMaxY) {
if(tMaxX < tMaxZ) {
      X= X + stepX;
      tMaxX= tMaxX + tDeltaX;
} else {
      Z= Z + stepZ;
      tMaxZ= tMaxZ + tDeltaZ;
}
} else {
if(tMaxY < tMaxZ) {
      Y= Y + stepY;
      tMaxY= tMaxY + tDeltaY;
} else {
      Z= Z + stepZ;
      tMaxZ= tMaxZ + tDeltaZ;
 }
}


//cout << "setnja za " << X << " " << Y << " " << Z<< endl;
//getchar();
if ((Z>tv.size()-1 || Z<0 || Y>tv[0].size()-1 || Y<0 || X>tv[0][0].size()-1 || X<0 )){
zastavica=1;
break; }
if (tv[Z][Y][X]=='O'){// || tv[Z][Y][X]=='U'){ // || tv[Z][Y][X]=='B') {

zastavica=1;

break; }

}  //while


if (zastavica==0){   // if ray from potential position to boundary unseen voxel doesn't intersect "O" or "U"
show<<-100*BUarray[z].y+20<<" " << 100*BUarray[z].z+20 << " " <<100*BUarray[z].x+20<< " " <<255 << " " << 255 << " " <<255 <<endl;
brojac++;
}
} // B unseen


//cout << "brojac koji je mozda kriv za pojedini voxel " <<brojac<<endl;
//getchar();
float  NBV_distance=sqrt((np_old.x-PParray[j].x)*(np_old.x-PParray[j].x)+((np_old.y-PParray[j].y))*((np_old.y-PParray[j].y)));
if ((brojac>max) && (NBV_distance>=10)){

max=brojac;
np.x=PParray[j].x;
np.y=PParray[j].y;
np.z=rp.z;
}
}



///////////////////////////////////////////////////end of counting for the next best view in 3D
cout << "number of BU voxels seen from the best position is " << max << endl;
cout << "The best next view is " << np.x <<' '<<np.y <<' '<< np.z<<'\n';
cout << "old NBV was " << np_old.x<<' ' << np_old.y << ' '<< np_old.z << endl;
if (max>Benough)
still_room=true;
else {
still_room=false;
//reset room detection map
//gpc_globalni_poligon2.num_contours=0;
}

int brojaCSVE=0;
brojacU=0;
brojacB=0;
int	brojacE=0;
int	brojacO=0;


for(int i=0;i<=tv.size()-1;i++){
for(int p=0;p<=tv[0].size()-1;p++){
for(int m=0;m<=tv[0][0].size()-1;m++){
brojaCSVE++;
if (tv[i][p][m]=='U')
brojacU++;
if (tv[i][p][m]=='E')
{
brojacE++;
//cout << "empty is " << m << " " << p << " " << i<< endl;
//getchar();
}
if (tv[i][p][m]=='O')
brojacO++;


}
}
}


time (&end);
diff = difftime (end,start);
cout<<"It took me " <<diff<<" seconds to read from file" <<'\n';

cout << "broj occupied voxela je " << brojacO << endl;
cout << "broj empty voxela je " << brojacE << endl;
cout << "broj unseen voxela je " << brojacU << endl;
cout << "broj svih voxela je " << brojaCSVE << endl;
show.close ();
getchar();

}
