

#include <iostream>
#include <fstream>
#include "cudo.h"

int main(int number_of_scan_positions) {


	NEWMAT::Matrix Pg(3,3);
	NEWMAT::Matrix P(2,2);
	double tempAx, tempAy, tempBx, tempBy;
	double Gx, Gy, fi;
	double Px, Py, Pfi;
	Tocka Xp, G;
	Razlomljena_Duzina *map_in_local_frame = new Razlomljena_Duzina[120];
	Razlomljena_Duzina* map= new Razlomljena_Duzina[400];
	int line_counter=0;
	int number_of_lines_in_map=0;
	for (int i=0; i<number_of_scan_positions; i++){

		line_counter=0;
		ifstream file_line ("map_lines_in_local_frame" +IntToString(i)+".txt");

		P << 0 << 0 << 0 << 0;        // set line variance in local frame to 0

		while(!file_line.eof()){

			file_line >> tempAx >> tempAy >> tempBx >> tempBy;

			map_in_local_frame[line_counter].A.push_back(Tocka(tempAx, tempAy));
			map_in_local_frame[line_counter].B.push_back(Tocka(tempBx, tempBy));
			map_in_local_frame[line_counter].postaviVar(P);

			line_counter++;
		}




		file_line.close();

		ifstream file_pose ("pose_with_covariance" +IntToString(i)+".txt");
		file_pose >> Gx >> Gy >> fi;
		G=Tocka (Gx, Gy);

		file_pose >> Px  >> Py >> Pfi;

		Pg << Px<< 0 << 0
		   << 0  << Py <<0
		   << 0  << 0 <<Pfi;

		file_pose.close();




		transformiraj2(map_in_local_frame,line_counter,G,fi,&Xp,Pg);

		map=duzinacat2(map, number_of_lines_in_map, map_in_local_frame, line_counter, &number_of_lines_in_map);


	}

	poklopi_kartu2(map,number_of_lines_in_map);
	Snimi ("final_map.m", map, number_of_lines_in_map);



}
