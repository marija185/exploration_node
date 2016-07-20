metric=0.001;
%  footprint=[0.17, 0.27;-0.15, 0.27;-0.48, 0.17;-0.48,-0.17;-0.15, -0.27;0.17, -0.27;0.17, 0.27];
 footprint=[0.26/2, 0.52/2;-0.15, 0.52/2;-0.48, 0.17;-0.48,-0.17;-0.15, -0.52/2;0.26/2, -0.52/2; 0.26/2 -0.102/2; (0.12+0.105/2) -0.102/2; (0.12+0.105/2) 0.102/2; 0.26/2 0.102/2; 0.26/2, 0.52/2];
 kutija=[(-0.58+0.26/2) 0.52/2; (-0.58+0.26/2) -0.52/2; (0.26/2) -0.52/2; (0.26/2) 0.52/2; (-0.58+0.26/2) 0.52/2];
 udaljenosti=sqrt(footprint(:,1).^2+footprint(:,2).^2);

	TB_flag=load('log_TB_flag.dat');
	LOG=load('log.dat'); %[indeks v, indeks w, dimenzija v, dimenzija w, trenutna brzina v, trenutna brzina w]
	M_putanje_x=load('log_putanje_x.dat'); %cijele krivulje (dimenzija v)x(dimenzija w)x30
	M_putanje_y=load('log_putanje_y.dat');
	M_breakage_putanje_x=load('log_breakage_putanje_x.dat'); %krivulje za brzo zaustavljanje
	M_breakage_putanje_y=load('log_breakage_putanje_y.dat');
    M_breakage_point_x=load('log_breakage_point_x.dat'); %konacna tocka krivulje zaustavljanja
    M_breakage_point_y=load('log_breakage_point_y.dat');
    M_obstacle_point_x=load('log_obstacle_point_x.dat');
    M_obstacle_point_y=load('log_obstacle_point_y.dat');
    M_breadage_point_th=load('log_breakage_point_th.dat');
	M_robot_position=load('log_robot_position.dat'); %pozicija robota [x y th]
	M_laser_obstacles=load('log_laser_obstacles.dat'); %prepreke [x1 y1;x2 y2;x3 y3;...]
    gridmapcolmov_x=load('wh_gridmap_colmov_x.dat');
    gridmapcolmov_y=load('wh_gridmap_colmov_y.dat');
    oldgridmapcolmov_x=load('wh_gridmap_colmov_old_x.dat');
    oldgridmapcolmov_y=load('wh_gridmap_colmov_old_y.dat');
    linbrzine=load('log_lin_brzine.dat');
    rotbrzine=load('log_rot_brzine.dat');
    TB_v=linbrzine;
    TB_w=rotbrzine;
    
    if (length(oldgridmapcolmov_x)>0 || length(gridmapcolmov_x)>0)
        gridmapcolmov_x
        break;
    end
	pokretne_prepreke_x=load('log_pokretne_prepreke_point_x.dat');
  	pokretne_prepreke_y=load('log_pokretne_prepreke_point_y.dat');
    moving_x=load('wh_gridmap_moving_x.dat');
    moving_y=load('wh_gridmap_moving_y.dat');
  	pokretne_prepreke_indeks=load('log_pokretne_prepreke_indeks.dat');
WH_planner_globalna_putanja_x=load('global_planner_path_x.dat');
WH_planner_globalna_putanja_y=load('global_planner_path_y.dat');
Dstar_path_x=load('wh_dstar_path_x.dat');
Dstar_path_y=load('wh_dstar_path_y.dat');
	dstar_path_putanje_x=load('log_dstar_path_putanje_x.dat');
	dstar_path_putanje_y=load('log_dstar_path_putanje_y.dat');

	%efektivni path (10 tocaka)
	M_path_putanje_x=load('log_path_putanje_x.dat');
	M_path_putanje_y=load('log_path_putanje_y.dat');
    M_tocka_infleksije=load('log_tocka_infleksije.dat'); % [x y]
    	figure;hold on;
        
	%polumjer robota (mm)
% 	rr=260;
  rr=290*metric;
%     rr=udaljenosti(3);
	fi=linspace(0,2*pi,300);
	x_temp=M_robot_position(1)*metric;y_temp=M_robot_position(2)*metric;th_temp=M_robot_position(3);
%     th_temp=th_temp+192*pi/180;
% th_temp=-1.330893;
	if (1)
    for i=1:300
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
    end
	plot(x_circle,y_circle,'k--');
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');
    end
    plot((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2)),(y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2)),'k');
%     plot((x_temp+cos(th_temp)*kutija(:,1)-sin(th_temp)*kutija(:,2)),(y_temp+sin(th_temp)*kutija(:,1)+cos(th_temp)*kutija(:,2)),'r');
%      pomak=-350*metric;
%      pomak=-300*metric;
     pomak=-298*metric;
    x_temp=x_temp+pomak*cos(th_temp);
    y_temp=y_temp+pomak*sin(th_temp);
%         rr=200*metric;
%         rr=220*metric;
    rr=udaljenosti(3)+pomak;
    for i=1:300
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
    end
	plot(x_circle,y_circle,'k--');
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');

    
    
    %nacrtat cu mjesto riegla
    pomak=-135;
    xrig=M_robot_position(1)+pomak*cos(M_robot_position(3));
    yrig=M_robot_position(2)+pomak*sin(M_robot_position(3));
    plot(xrig*metric,yrig*metric,'bo')
    
    %nacrtat cu mjesto sicka
    pomak=120;
    xrig=(M_robot_position(1)+pomak*cos(M_robot_position(3)))*metric;
    yrig=(M_robot_position(2)+pomak*sin(M_robot_position(3)))*metric;
    plot(xrig,yrig,'bo')
    sick=[0.105/2 0.102/2; -0.105/2 0.102/2; -0.105/2 -0.102/2; 0.105/2 -0.102/2; 0.105/2 0.102/2];
%     plot((xrig+cos(th_temp)*sick(:,1)-sin(th_temp)*sick(:,2)),(yrig+sin(th_temp)*sick(:,1)+cos(th_temp)*sick(:,2)),'r');


    plot(M_laser_obstacles(:,1)*metric,M_laser_obstacles(:,2)*metric,'k.');
%     plot(pokretne_prepreke_x*metric,pokretne_prepreke_y*metric,'gd');
% 	plot(x_circle*metric, y_circle*metric,'r','Linewidth',2);
	plot([M_robot_position(1) M_robot_position(1)+rr*cos(M_robot_position(3))]*metric, [M_robot_position(2) M_robot_position(2)+rr*sin(M_robot_position(3))]*metric,'r','Linewidth',2);
    %crtanje efektivne putanje
	plot(M_path_putanje_x*metric,M_path_putanje_y*metric,'m-', M_path_putanje_x*metric,M_path_putanje_y*metric, 'm.');
	plot(M_tocka_infleksije(1)*metric,M_tocka_infleksije(2)*metric,'bd');
    
    
    
    if (0)
    %provjera NaN u kodu
    path_rx=Dstar_path_x;
    path_ry=Dstar_path_y;
    RBx=M_robot_position(1);
    RBy=M_robot_position(2);
    max_path=150;
    delta_x=(path_rx(2)-RBx);
			delta_y=(path_ry(2)-RBy);
	path_length_a1=(delta_x*delta_x+delta_y*delta_y);
    delta_x=(path_rx(1)-RBx);
			delta_y=(path_ry(1)-RBy);
			path_length_a2=delta_x*delta_x+delta_y*delta_y;
       
            path_length_a1=sqrt(path_length_a1)
				path_length_a2=sqrt(path_length_a2)     
    i=1;
    da12=(path_rx(i)-path_rx(i+1))*(path_rx(i)-path_rx(i+1))+(path_ry(i)-path_ry(i+1))*(path_ry(i)-path_ry(i+1)) %kvadratna udaljenost izmedju dvije tocke
		da12=sqrt(da12)
		kuta1=acos(((RBx-path_rx(i+1))*(path_rx(i)-path_rx(i+1))+(RBy-path_ry(i+1))*(path_ry(i)-path_ry(i+1)))/(path_length_a1*da12));
kuta1
		kutc=asin((path_length_a1/max_path)*sin(kuta1)) %paznja kod asin, mogu bit dva rjesenja kutc i PI-kutc
		%zato gledamo oba rjesenja i odabiremo ono ciji je komadx manji
		kutb1=pi-kuta1-kutc
		komadx2=(max_path)*sin(kutb1)/sin(kuta1)
		kutb1=-kuta1+kutc;
		komadx=(max_path)*sin(kutb1)/sin(kuta1)
		if (komadx>komadx2)
			komadx=komadx2
        end
		tocka_infleksijex=komadx/da12*path_rx(i)+(da12-komadx)/da12*path_rx(i+1)
		tocka_infleksijey=komadx/da12*path_ry(i)+(da12-komadx)/da12*path_ry(i+1)
    end
    
    %crtanje globalnog puta
%     plot(WH_planner_globalna_putanja_x(1:end)*metric,WH_planner_globalna_putanja_y(1:end)*metric,'b');
%     plot(Dstar_path_x(1:end)*metric,Dstar_path_y(1:end)*metric,'b');
    plot(dstar_path_putanje_x(1:end)*metric,dstar_path_putanje_y(1:end)*metric,'b');
    
    
        %parametri iz Params.h i DynamicWindow.h
        RR=290;
    SC1=10;
    SC2=500;
    SC_W=0;
    V_MAX=500;
    W_MAX=100;

        TB_v
    TB_w
    disp('stupci su w, a redci su v')
    TB_flag

	for i=1:LOG(3)
	     for j=1:LOG(4)
                            limit_distance=((RR+SC1+(SC2-SC1)*TB_v(i)/V_MAX)+SC_W*TB_w(j)/W_MAX)*metric;
%                 limit_distancec2=((RR2+SC1+(SC2-SC1)*TB_v(i)/V_MAX)+SC_W*TB_w(j)/W_MAX)*metric; %security distance za drugu kruznicu

		index=(i-1)*LOG(4)+j;%po redu par brzina, v fiksiran, w sece pa do 25. para
		if (TB_flag(i,j)==2) %clear
		plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'g');
		elseif (TB_flag(i,j)==1) %has obstacle
		plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'b');
        plot(M_obstacle_point_x(index)*metric,M_obstacle_point_y(index)*metric,'xk');
        elseif (TB_flag(i,j)==-2) %non admissible
		plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'k');
%         pause
        plot(M_obstacle_point_x(index)*metric,M_obstacle_point_y(index)*metric,'xk','LineWidth',2);
%         pause
		elseif (TB_flag(i,j)==-3) %kinematic constraint
		plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m');
		elseif (TB_flag(i,j)==-4) %circular
		plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m');
		end
		
	     end
	end
	%optimalnu putanju crtamo posebno
	if((LOG(1)>=1)&(LOG(2)>=1))
	index=(LOG(1)-1)*LOG(4)+LOG(2);
 	plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'r-',M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'rx');
    end
    
    NKL=61; %broj tocaka na trajektorijama
    od=1;
    brboje=0;
    boja=hsv(8);
for i=1:length(pokretne_prepreke_indeks)
    brboje=pokretne_prepreke_indeks(i);
    brboje=mod(brboje,8)+1;
    do=od+NKL-1;
        plot(pokretne_prepreke_x(od:do)*metric,pokretne_prepreke_y(od:do)*metric,'o','Color',boja(brboje,:));
        text(pokretne_prepreke_x(od)*metric,pokretne_prepreke_y(od)*metric,mat2str(pokretne_prepreke_indeks(i)));
        
        od=do+1;

end
      plot(moving_x*metric,moving_y*metric,'b.');
  
teziste_x=load('wh_teziste_x.dat');
teziste_y=load('wh_teziste_y.dat');
tezistepointeri=load('wh_tezistepointeri.dat');
brboje=0;boja=prism(50);
od=1;
for i=1:max(size(tezistepointeri)),
brboje=mod(brboje,50)+1;
do=od+tezistepointeri(i)-1;
if (i==max(size(tezistepointeri))) || 0
ruka=plot(teziste_x(od:do)*metric,teziste_y(od:do)*metric,'.');
set(ruka,'Color',boja(brboje,:));
end
od=do+1;
% pause
end


%provjera kruzne trajektorije
if 0
KORAK=0.1;
v=linbrzine(end);
w=rotbrzine(1);
klx=ones(1,30);
kly=klx;
klth=klx;
klx(1)=M_robot_position(1);
kly(1)=M_robot_position(2);
klth(1)=M_robot_position(3);
for i=2:61
			klth(i)=klth(i-1)+w*KORAK;
			klx(i)=klx(i-1)+v/w*(sin(klth(i))-sin(klth(i-1)));
			kly(i)=kly(i-1)-v/w*(cos(klth(i))-cos(klth(i-1)));
end
plot(klx*metric,kly*metric,'b-o')
end

	axis equal tight;
xlabel('x [m]', 'fontsize',16,'fontname', 'times');
ylabel('y [m]', 'fontsize',16,'fontname', 'times');
%crtanje svih laserskih ocitanja
% svilaseri_x=load('wh_svilaseri_x.dat');
% svilaseri_y=load('wh_svilaseri_y.dat');
% svilaserpointeri=load('wh_svilaserpointeri.dat');
% brboje=0;boja=prism(50);
% od=1;
% for i=1:max(size(svilaserpointeri)),
% brboje=mod(brboje,50)+1;
% do=od+svilaserpointeri(i)-1;
% ruka=plot(svilaseri_x(od:do)*metric,svilaseri_y(od:do)*metric,'.');
% od=do+1;
% set(ruka,'Color',boja(brboje,:));
% end


