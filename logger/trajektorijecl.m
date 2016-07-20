% close all
%parametri
obicni=0;
plus=0;
minus=0;
minus2=0;
plus1=0;
plus2=0;
plus2nonadmissible=1;
plus1nonadmissible=1;
allnonadmissible=0;
optold_col_draw=0;
break_col_draw=0;
opt_col_draw=0;
col_traj_draw=0;
col_traj_col_draw=0;%obicni
col_trajplus_col_draw=0;
draw_last_pt_plus=0;
draw_last_pt_plus1=0;
draw_last_pt_plus2=0;
draw_old=0;
optdraw=0;
draw_break=0;
draw_pointers=1;
draw_dstarpath=1;
draw_dstarpathinit=1;
drawori=0;
draw_cspacepath=0;
scale1=1;
nonadm_traj_col_draw=0;
discr=0;
draw_last_traj_pt=0;
draw_pose_while_driving=0;
costmask=0;%-1; %put -1 for paper
drawmatrix=1;%0-for the paper, 1 else
drawobstenl=1;
drawcostmask=0;
drawmovingcells=0;
plotonlytrajectories=0; %without paths
plotlasers=0;
plotoncurrentfig=0;
debel=0;
costobstenl=14; %14 je broj koje ima prosirenje prepreke za costmask 2 i debel 10, a 15 za cm 3 debel 10
nepoznata=0;%kad imas nepoznate prepreke stavi 1 inace 0
if (nepoznata)
    costmask=costmask+1;
end
origin=load('origin.dat');
Map_Home_x=origin(1)
Map_Home_y=origin(2)

% cell=50;
cell=load('cell_size.dat');
angularresolution=1;
% angularresolution=load('angularresolution.dat');
maxori=ceil(2*pi/angularresolution);
metric=0.001;
duljina=1.; %in m
  sirina=0.6;
  robot_shape=load('robot_shape.dat');
  duljina=robot_shape(1);
  sirina=robot_shape(2);
%   pom=sirina; sirina=duljina; duljina=pom;
  duljina=duljina/cell/metric; %in pixels
  sirina=sirina/cell/metric;
  duljina=2*2.555;%pioneer - the largest x coordinate
  rr=duljina/2;
%   footprint=[-duljina/2 -sirina/2; duljina/2 -sirina/2; duljina/2 sirina/2; -duljina/2 sirina/2; -duljina/2 -sirina/2]; 
footprint=load('robot_footprint.dat')/cell;
rrmm=duljina/2*cell;
footprintmm=footprint*cell;
  
WH_dstar_cost_map=load('wh_dstar_cost_map.dat');
WH_globalna_putanja_th=load('robot_globalna_putanja_th.dat');
WH_globalna_putanja_x=load('robot_globalna_putanja_x.dat');
WH_globalna_putanja_y=load('robot_globalna_putanja_y.dat');
travcost=load('travcost_putanja.dat');
travcost3d=load('travcost_putanja3d.dat');
traversalcostpath=load('traversalcostpath');
newgoals=load('newgoals');
a=size(traversalcostpath);
if a(2)==4
    orisearch=1;
else
    orisearch=0;
end
% 	pokretne_prepreke_x=(load('log_pokretne_prepreke_point_x.dat')-Map_Home_x)/cell;
%   	pokretne_prepreke_y=(load('log_pokretne_prepreke_point_y.dat')-Map_Home_y)/cell;
    WH_gridmap_x=(load('wh_gridmap_x.dat')-Map_Home_x)/cell+1;%zbog indeksa, matlab ima indekse od 1
	WH_gridmap_y=(load('wh_gridmap_y.dat')-Map_Home_y)/cell+1;
staticna=(load('wh_gridmap_static_cell.dat'));
DS_occ_i=(load('wh_dstar_occ_i.dat'))+1; %maltab ima indekse od 1
DS_occ_j=(load('wh_dstar_occ_j.dat'))+1;

[sizex,sizey]=size(WH_dstar_cost_map);


xmin=1; xmax=sizex+1; ymin=1; ymax=sizey+1;
% xmin=1; xmax=32; ymin=86; ymax=115;
xorigin=origin(1);
yorigin=origin(2);

veca=zeros(sizex+10,sizey+10);
Xos=[0:1:sizex-1];
Yos=[0:1:sizey-1];
xv=[0:1:sizex+9];
yv=[0:1:sizey+9];


  osi=[474 505 44 67];
 osi=[85+80 168+80 48 90];
 osi=[165 230 50 78];  
 osi=[110 180 49 80];
 osi=[145 180 62 78];
 osi=[271 315 50 70];
 osi=[235 275 50 70];
 osi=[60 120 10 50];%cilj lijevo kod mene
 osi=[35 101 27 70];%phd cijela putanja
 osi=[55 141 27 70];%cilj peric
 osi=[250 300 27 100];%iz 0,1
 osi=[255 285 67 100];%iz 0,1 dio, cell=100
%  osi=[520 570 140 190];%iz 0,1 dio, cell=50
%  osi=[70 120 130 170];%spirala
%  osi=[65 210 50 150];%replaniranje phd
%  osi=[67 120 70 115];%replaniranje phd prepreka1
%  osi=[85 99 33 54];%phd trajektorije
%  osi=[85 100 35 48];%clanak trajektorije
%  osi=[72 75 35 38];%clanak exit
%  osi=[140 300 50 120];%cilj desno kod Andreje
%  osi=[50 200 30 115];%cilj lijevo kod Blanke
% osi=[100 300 10 148];
 osi=[xmin xmax ymin ymax];




k=[1:-0.1:0];
k=[1:-0.2:0];
k=[1 0.3+3*0.7/4 0.3+0.7/2 0.3+0.7/4 0.3 0];
k=[1 1 1 1 1 0];%kad neces shadeove
if (costmask==1)
k=[1 1 1;0.7 0.7 0.7;0 0 0;1 0 0];
if (debel==1)
k=[1 1 1;1 0.8 0.8;1 0.8 0.8;0.5 0.5 0.5;0 0 0];
end
end
if (costmask==-1)
   k=[1 1 1;1 1 1;0 0 0];
end
if (costmask==0)
   k=[1 1 1;0.5 0.5 0.5;0 0 0];
end
if (costmask==2)
   k=[1 1 1;1 0.8 0.8;0.9 0.6 0.6;0.5 0.5 0.5;0 0 0];
end
if (costmask==3)
   k=[1 1 1;1 0.8 0.8;0.9 0.6 0.6;0.8 0.4 0.4;0.5 0.5 0.5;0 0 0];
end
if (costmask==4)
   k=[1 1 1;1 0.8 0.8;0.9 0.6 0.6;0.8 0.4 0.4;0.7 0.3 0.3;0.5 0.5 0.5;0 0 0];
end
if (debel)
    k=[ones(debel,3);k];
end


costmapa=WH_dstar_cost_map';
maxcost=max(max(costmapa));
WH_gridmap_x_new=WH_gridmap_x;
WH_gridmap_y_new=WH_gridmap_y;

for i=1:max(size(WH_gridmap_x))
    pomx=floor( WH_gridmap_x(i));
    pomy=floor( WH_gridmap_y(i));
    if pomx>=1 && pomx<=sizex && pomy>=1 && pomy<=sizey
        costmapa(pomy,pomx)=maxcost+2-nepoznata;
        if staticna(i)==2
            WH_gridmap_x(i)=0;
            WH_gridmap_y(i)=0;
            costmapa(pomy,pomx)=maxcost+2;
        else
            WH_gridmap_x_new(i)=0;
            WH_gridmap_y_new(i)=0;
            
        end
    end
end
costmaskax=[];
costmaskay=[];
costmaskax2=[];
costmaskay2=[];
costmaskax3=[];
costmaskay3=[];
for pomx=1:sizex
    for pomy=1:sizey
        if costmapa(pomy,pomx)==costobstenl
            costmaskax=[costmaskax; pomx];
            costmaskay=[costmaskay; pomy];
        end
        if costmapa(pomy,pomx)==10
            costmaskax2=[costmaskax2; pomx];
            costmaskay2=[costmaskay2; pomy];
        end
        if costmapa(pomy,pomx)==100
            costmaskax3=[costmaskax3; pomx];
            costmaskay3=[costmaskay3; pomy];
        end
    end
end
if 0
for i=1:max(size(DS_occ_i))
    pomx=( DS_occ_i(i));
    pomy=( DS_occ_j(i));
        costmapa(pomy,pomx)=maxcost+3;
end
end

veca(1:sizex,1:sizey)=costmapa';
kartica=veca;
 Xos=xv(osi(1):osi(2))+1;
Yos=yv(osi(3):osi(4))+1;
kartica=kartica(Xos,Yos);
Xos_real=(Xos-1)*cell+xorigin;
Yos_real=(Yos-1)*cell+yorigin;

% costmap=costmapa(Yos,Xos);
if plotoncurrentfig==0
 figure
end
 if costmask~=-1
 k=summer(maxcost);
%  k=white(maxcost)*0.85;
%  k(:,1)=1; %k(:,3)=1;
 k=[1 1 1;k;0 0 0];
 if ~isempty(DS_occ_i) && 0
     k=[1 1 1;spring(maxcost);0 0 0;0 1 1];
 end
 end
%  k=lines(maxcost+2);
 colormap(k)
%  surf(Xos,Yos,costmap*0,costmap,'EdgeColor',[0.7 0.7 0.7])
%  stare_moving=[2392.0281721999859, 3173.8397744780345; 2284.4227513688766, 3175.1182441551146;2183.5816598133792,  3176.3165096112857;2414.4822345936032, 3147.0365746522762;2420.792818742947, 3207.2513391968305;2393.0142365900738, 3207.5813738655834;2286.4830188448832, 3208.8473348997431;2186.580407507302,3210.0342671805406;2203.7763428132557, 3409.8440528105243;2178.9207587036717, 3410.1393384486441;2179.0461425900758, 3510.1450428003386;2199.6740812659823, 3609.9070137685085;2204.2278837577969, 3330.3425420519902;2205.9474819166021, 3630.3433648194373;2193.3007006889056, 3380.4518879373159;2230.8391536657737, 3509.481416629329;2099.3856795969987, 3111.6707135962438;2099.0517379568141,3209.7382582883802;2098.7419384987033, 3300.7609277262477;2098.3620403720524, 3412.3271646341136;2098.0549363950913, 3502.5240014173223;2097.666907460291, 3616.5000259797416;2137.8111707312428, 3090.0522022833406];
stare_moving_x=load('wh_gridmap_stare_moving_x.dat');
nove_moving_x=load('wh_gridmap_nove_moving_x.dat');
stare_moving_y=load('wh_gridmap_stare_moving_y.dat');
nove_moving_y=load('wh_gridmap_nove_moving_y.dat');
WH_dstar_open_x=load('wh_dstar_open_x.dat');
WH_dstar_open_y=load('wh_dstar_open_y.dat');
WH_dstar_closed_x=load('wh_dstar_closed_x.dat');
WH_dstar_closed_y=load('wh_dstar_closed_y.dat');
WH_dstar_cspace_x=load('wh_dstar_cspace_x.dat');
WH_dstar_cspace_y=load('wh_dstar_cspace_y.dat');
	M_laser_obstacles=load('log_laser_obstacles.dat'); %prepreke [x1 y1;x2 y2;x3 y3;...] %ove ce kasnit jedan korak u slucaju no_path-a

if drawmatrix
 surf(Xos_real*metric,Yos_real*metric,kartica'*0,kartica','EdgeColor',[0.7 0.7 0.7])
 grid off
 view([0 90])
%  title('cost grid map','fontsize',16);
maxcost=max(max(kartica));
caxis([1 maxcost])

axis([0.8 4.5 1.3 3.3])
axis([0.8 3 1.3 3.3])
 axis equal tight;
else
%    plot(((floor(WH_gridmap_x)-0.5)*cell+xorigin)*metric,((floor(WH_gridmap_y)-0.5)*cell+yorigin)*metric,'k.',...
%                 'MarkerSize',2);%4);%5); %1);
   plot(((floor(WH_gridmap_x)-0.5)*cell+xorigin)*metric,((floor(WH_gridmap_y)-0.5)*cell+yorigin)*metric,'ks','MarkerEdgeColor','none',...
                'MarkerFaceColor',[0 0 0],...
                'MarkerSize',18);%18);%4);%5); %1);
            hold on
            if drawobstenl
   plot(((floor(costmaskax)-0.5)*cell+xorigin)*metric,((floor(costmaskay)-0.5)*cell+yorigin)*metric,'ks','MarkerEdgeColor',[0.5 0.5 0.5],...
                'MarkerFaceColor',[0.5 0.5 0.5],...
                'MarkerSize',18);%4);%5); %1);
            end
            if drawcostmask
   plot(((floor(costmaskax2)-0.5)*cell+xorigin)*metric,((floor(costmaskay2)-0.5)*cell+yorigin)*metric,'ks','MarkerEdgeColor',[1 0. 0.],...
                'MarkerFaceColor','none',...
                'MarkerSize',8);%4);%5); %1);
   plot(((floor(costmaskax3)-0.5)*cell+xorigin)*metric,((floor(costmaskay3)-0.5)*cell+yorigin)*metric,'ks','MarkerEdgeColor','none',...
                'MarkerFaceColor',[1 0 0],...
                'MarkerSize',8);%4);%5); %1);
            end
            if drawmovingcells
            plot(((floor(WH_gridmap_x_new)-0.5)*cell+xorigin)*metric,((floor(WH_gridmap_y_new)-0.5)*cell+yorigin)*metric,'ks','MarkerEdgeColor',[0.5 0.5 0.5],...
                'MarkerFaceColor',[0.5 0.5 0.5],...
                'MarkerSize',4);%4);%5); %1);
   plot(((stare_moving_x))*metric,((stare_moving_y))*metric,'ks','MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',4);%4);%5); %1);
   plot((nove_moving_x)*metric,(nove_moving_y)*metric,'ks','MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',4);%4);%5); %1);
            end
%  axis equal tight;

end

 hold on

 if costmask~=-1
 plot(((WH_gridmap_x-1)*cell+xorigin)*metric,((WH_gridmap_y-1)*cell+yorigin)*metric,'m.');
 end

if costmask~=-1
    if (isempty(M_laser_obstacles)==0)
    plot(M_laser_obstacles(:,1)*metric,M_laser_obstacles(:,2)*metric,'g.');
    end

plot(stare_moving_x*metric,stare_moving_y*metric,'yo')
plot(nove_moving_x*metric,nove_moving_y*metric,'bd')
% plot(WH_dstar_closed_x*metric,WH_dstar_closed_y*metric,'cs')
plot(WH_dstar_cspace_x*metric,WH_dstar_cspace_y*metric,'ko','MarkerSize',4.5)
end
    
if draw_pointers
%svi pointeri
    WH_dstar_x=load('wh_dstar_x.dat')+0*cell;
    WH_dstar_y=load('wh_dstar_y.dat')+0*cell;
    WH_dstar_dx=load('wh_dstar_dx.dat');
    WH_dstar_dy=load('wh_dstar_dy.dat');

    
%     kviv=quiver(WH_dstar_x*metric,WH_dstar_y*metric,WH_dstar_dx*metric,WH_dstar_dy*metric,0.1,'filled','Color',[0.6 0.5 0.7]);
    kviv=quiver(WH_dstar_x*metric,WH_dstar_y*metric,WH_dstar_dx*metric,WH_dstar_dy*metric,0.52,'Color',[0 0 0]);
    set(kviv,'MaxHeadSize',0.4)
end


    M_putanje_x=load('log_putanje_x.dat')+0*cell; %cijele krivulje (dimenzija v)x(dimenzija w)x30
	M_putanje_y=load('log_putanje_y.dat')+0*cell;
	M_putanje_th=load('log_putanje_th.dat');
	M_putanje_th_plus=load('log_putanje_th_plus.dat');
	M_putanje_th_plus1=load('log_putanje_th_plus1.dat');
	M_putanje_th_plus2=load('log_putanje_th_plus2.dat');
	M_putanje_th_minus=load('log_putanje_th_minus.dat');
	M_putanje_th_minus2=load('log_putanje_th_minus2.dat');
    M_putanje_x_plus=load('log_putanje_x_plus.dat')+0*cell; %cijele krivulje (dimenzija v)x(dimenzija w)x30
	M_putanje_y_plus=load('log_putanje_y_plus.dat')+0*cell;
    M_putanje_x_plus1=load('log_putanje_x_plus1.dat')+0*cell; %cijele krivulje (dimenzija v)x(dimenzija w)x30
	M_putanje_y_plus1=load('log_putanje_y_plus1.dat')+0*cell;
    M_putanje_x_plus2=load('log_putanje_x_plus2.dat')+0*cell; %cijele krivulje (dimenzija v)x(dimenzija w)x30
	M_putanje_y_plus2=load('log_putanje_y_plus2.dat')+0*cell;
    M_putanje_x_minus=load('log_putanje_x_minus.dat')+0*cell; %cijele krivulje (dimenzija v)x(dimenzija w)x30
	M_putanje_y_minus=load('log_putanje_y_minus.dat')+0*cell;
    M_putanje_x_minus2=load('log_putanje_x_minus2.dat')+0*cell; %cijele krivulje (dimenzija v)x(dimenzija w)x30
	M_putanje_y_minus2=load('log_putanje_y_minus2.dat')+0*cell;
    kl_x_old=load('log_putanje_x_old.dat')+0*cell;%stara optimalna u novom ciklusu (dodana tocka na kraj)
	kl_y_old=load('log_putanje_y_old.dat')+0*cell;
	kl_th_old=load('log_putanje_th_old.dat');
    kl_x_break=load('log_putanje_x_break.dat')+0*cell;%putanja za kocenje po rampi
	kl_y_break=load('log_putanje_y_break.dat')+0*cell;
	kl_th_break=load('log_putanje_th_break.dat');
    kl_x=load('log_opt_putanje_x.dat')+0*cell;%optimalna u ovom ciklusu
	kl_y=load('log_opt_putanje_y.dat')+0*cell;
	kl_th=load('log_opt_putanje_th.dat');
    M_obstacle_point_x=load('log_obstacle_point_x.dat')+0*cell;
    M_obstacle_point_y=load('log_obstacle_point_y.dat')+0*cell;
    TB_flag=load('log_TB_flag.dat');
	 TB_flag_plus=load('log_TB_flag_plus.dat');
	 TB_flag_plus1=load('log_TB_flag_plus1.dat');
	 TB_flag_plus2=load('log_TB_flag_plus2.dat');
	 TB_flag_minus=load('log_TB_flag_minus.dat');
	 TB_flag_minus2=load('log_TB_flag_minus2.dat');
		LOG=load('log.dat'); %[indeks v, indeks w, dimenzija v, dimenzija w, trenutna brzina v, trenutna brzina w]
% LOG=load('log.dat'); %[indeks v, indeks w, dimenzija v, dimenzija w, trenutna brzina v, trenutna brzina w, trenutna brzina vy, indeks vy, dimenzija vy]
	M_putanje_v=load('log_putanje_v.dat'); %cijele krivulje (dimenzija v)x(dimenzija w)x30

    

    
    
    if(1)
%      for k=1:LOG(9) 
    for i=1:LOG(3)
%         ii=(k-1)*LOG(3)+i;
        ii=i;
	    for j=1:LOG(4)
% 		index=(k-1)*LOG(3)*LOG(4)+(i-1)*LOG(4)+j;%po redu par brzina, v fiksiran, w sece pa do 25. para
		index=(i-1)*LOG(4)+j;%po redu par brzina, v fiksiran, w sece pa do 25. para
        
        if (plus)
				if (TB_flag_plus(ii,j)==2) %clear
		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'m.-','Color',[0 0 0.8]);
        if draw_last_pt_plus
            jj=length(M_putanje_x(index,:));
%             scale1=0.01;
                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*scale1*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*scale1*sin(th_temp)]*cell+yorigin)*metric,'b-','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprint(:,1)-sin(th_temp)*scale1*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*scale1*footprint(:,1)+cos(th_temp)*scale1*footprint(:,2))*cell+yorigin)*metric,'b'); 
                        else
                            th_temp=M_putanje_th_plus(index,jj);
                            x_temp=M_putanje_x_plus(index,jj); y_temp=M_putanje_y_plus(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b-','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'b'); 
                        end
                    end
        end
%         pause
%         delete(h);
                elseif (TB_flag_plus(ii,j)==1) && col_traj_draw %has obstacle
		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'b-*','Color',[0 0 0.5]);
            if index>=0 && col_trajplus_col_draw
%                 discr=1;
%                 for jj=1:length(M_putanje_x(index,:))
                    jj=length(M_putanje_x(index,:));

                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*scale1*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*scale1*sin(th_temp)]*cell+yorigin)*metric,'b-','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprint(:,1)-sin(th_temp)*scale1*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*scale1*footprint(:,1)+cos(th_temp)*scale1*footprint(:,2))*cell+yorigin)*metric,'b'); 
                        else
                            th_temp=M_putanje_th_plus(index,jj);
                            x_temp=M_putanje_x_plus(index,jj); y_temp=M_putanje_y_plus(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b-','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'b'); 
                        end
                    end
%                 end
   
%                 return
%                 pause
%                 delete(h1,h2);

            end
        elseif (TB_flag_plus(ii,j)==-2) && allnonadmissible %non admissible
		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'k.-','Color',[0.1 0.1 0.1]);
            if index>0 && nonadm_traj_col_draw
%                 discr=1;
%                 for jj=1:length(M_putanje_x(index,:))
                    jj=length(M_putanje_x(index,:));

                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th_plus(index,jj);
                            x_temp=M_putanje_x_plus(index,jj); y_temp=M_putanje_y_plus(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
%                 end
   
%                 return
                pause
                delete(h1,h2);
            end
%         pause
%         delete(h);
% 		elseif (TB_flag_plus(ii,j)==-1) %dynamic constraint
% 		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'y-*');
%         pause
%         delete(h);
%         elseif (TB_flag_plus(ii,j)==-4) %GOAL_CONSTRAINTS
% 		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'m-*','Color',[0.8 0.2 0.2]);
%         pause
%         delete(h);
                end
         end
        if (plus1)
				if (TB_flag_plus1(ii,j)==2) %clear
		h=plot(M_putanje_x_plus1(index,:)*metric,M_putanje_y_plus1(index,:)*metric,'m.-','Color',[0.8 0 0]);
        if draw_last_pt_plus1
            jj=length(M_putanje_x(index,:));
%             scale1=0.01;
                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus1(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus1(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus1(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*scale1*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*scale1*sin(th_temp)]*cell+yorigin)*metric,'b-','Color',[0.8 0 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprint(:,1)-sin(th_temp)*scale1*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*scale1*footprint(:,1)+cos(th_temp)*scale1*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th_plus1(index,jj);
                            x_temp=M_putanje_x_plus1(index,jj); y_temp=M_putanje_y_plus1(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b-','Color',[0.8 0 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
        end
%         pause
%         delete(h);
                elseif (TB_flag_plus1(ii,j)==1) && col_traj_draw %has obstacle
		h=plot(M_putanje_x_plus1(index,:)*metric,M_putanje_y_plus1(index,:)*metric,'b-*','Color',[0.5 0 0]);
            if index>=0 && col_trajplus_col_draw
%                 discr=1;
%                 for jj=1:length(M_putanje_x(index,:))
                    jj=length(M_putanje_x(index,:));

                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus1(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus1(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus1(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*scale1*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*scale1*sin(th_temp)]*cell+yorigin)*metric,'b-','Color',[0.8 0 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprint(:,1)-sin(th_temp)*scale1*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*scale1*footprint(:,1)+cos(th_temp)*scale1*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th_plus1(index,jj);
                            x_temp=M_putanje_x_plus1(index,jj); y_temp=M_putanje_y_plus1(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b-','Color',[0.8 0 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
%                 end
   
%                 return
%                 pause
%                 delete(h1,h2);

            end
        elseif (TB_flag_plus1(ii,j)==-2)  && plus1nonadmissible %non admissible
		h=plot(M_putanje_x_plus1(index,:)*metric,M_putanje_y_plus1(index,:)*metric,'k.-','Color',[0.1 0.1 0.1]);
        
            if index>0 && nonadm_traj_col_draw
%                 discr=1;
%                 for jj=1:length(M_putanje_x(index,:))
                    jj=length(M_putanje_x(index,:));

                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus1(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus1(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus1(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th_plus1(index,jj);
                            x_temp=M_putanje_x_plus1(index,jj); y_temp=M_putanje_y_plus1(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
%                 end
   
%                 return
                pause
                delete(h1,h2);
            end
%         pause
%         delete(h);
% 		elseif (TB_flag_plus(ii,j)==-1) %dynamic constraint
% 		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'y-*');
%         pause
%         delete(h);
%         elseif (TB_flag_plus(ii,j)==-4) %GOAL_CONSTRAINTS
% 		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'m-*','Color',[0.8 0.2 0.2]);
%         pause
%         delete(h);
                end
         end
        if (plus2)
				if (TB_flag_plus2(ii,j)==2) %clear
		h=plot(M_putanje_x_plus2(index,:)*metric,M_putanje_y_plus2(index,:)*metric,'m.-','Color',[0 0.8 0]);
        if draw_last_pt_plus2
            jj=length(M_putanje_x(index,:));
%             scale1=0.01;
                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus2(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus2(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus2(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*scale1*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*scale1*sin(th_temp)]*cell+yorigin)*metric,'b-','Color',[0 0.8 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprint(:,1)-sin(th_temp)*scale1*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*scale1*footprint(:,1)+cos(th_temp)*scale1*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th_plus2(index,jj);
                            x_temp=M_putanje_x_plus2(index,jj); y_temp=M_putanje_y_plus2(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b-','Color',[0 0.8 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
%         pause
%         delete(h,h1,h2);
        end
                elseif (TB_flag_plus2(ii,j)==1) && col_traj_draw %has obstacle
		h=plot(M_putanje_x_plus2(index,:)*metric,M_putanje_y_plus2(index,:)*metric,'b-*','Color',[0 0.5 0]);
            if index>=0 && col_trajplus_col_draw
%                 discr=1;
%                 for jj=1:length(M_putanje_x(index,:))
                    jj=length(M_putanje_x(index,:));

                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus2(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus2(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus2(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*scale1*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*scale1*sin(th_temp)]*cell+yorigin)*metric,'b-','Color',[0 0.8 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprint(:,1)-sin(th_temp)*scale1*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*scale1*footprint(:,1)+cos(th_temp)*scale1*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th_plus2(index,jj);
                            x_temp=M_putanje_x_plus2(index,jj); y_temp=M_putanje_y_plus2(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b-','Color',[0 0.8 0],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
%                 end
   
%                 return
%                 pause
%                 delete(h1,h2);

            end
        elseif (TB_flag_plus2(ii,j)==-2) && plus2nonadmissible %non admissible
		h=plot(M_putanje_x_plus2(index,:)*metric,M_putanje_y_plus2(index,:)*metric,'k.-','Color',[0.1 0.1 0.1]);
            if index>0 && nonadm_traj_col_draw
%                 discr=1;
%                 for jj=1:length(M_putanje_x(index,:))
                    jj=length(M_putanje_x(index,:));

                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th_plus2(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x_plus2(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y_plus2(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th_plus2(index,jj);
                            x_temp=M_putanje_x_plus2(index,jj); y_temp=M_putanje_y_plus2(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
%                 end
   
%                 return
                pause
                delete(h1,h2);
            end
%         pause
%         delete(h);
% 		elseif (TB_flag_plus(ii,j)==-1) %dynamic constraint
% 		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'y-*');
%         pause
%         delete(h);
%         elseif (TB_flag_plus(ii,j)==-4) %GOAL_CONSTRAINTS
% 		h=plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'m-*','Color',[0.8 0.2 0.2]);
%         pause
%         delete(h);
                end
         end
                
                        v=M_putanje_v(index,:);
if(obicni)
		if (TB_flag(ii,j)==2) %&& (v(47)==0) %clear i bez kracenja
		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'g.-','Color',[0.7 0.8 1]);
% 		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'g.-','Color',[1 1 0]);
%         pause
%         delete(h);
		elseif (TB_flag(ii,j)==1) && col_traj_draw %has obstacle
		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'b-*');
            if index>0 && col_traj_col_draw
%                 discr=1;
%                 for jj=1:length(M_putanje_x(index,:))
                  jj=length(M_putanje_x(index,:));
                    if mod(jj,4)==3 || 1
                        if discr
                            th_temp=floor(mod(M_putanje_th(index,jj),2*pi)/(angularresolution))*(angularresolution);
                            x_temp=floor((M_putanje_x(index,jj)-xorigin)/cell)+0.5; y_temp=floor((M_putanje_y(index,jj)-yorigin)/cell)+0.5;
                            h1=plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'k'); 
                        else
                            th_temp=M_putanje_th(index,jj);
                            x_temp=M_putanje_x(index,jj); y_temp=M_putanje_y(index,jj);
                            h1=plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                            h2=plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'k'); 
                        end
                    end
%                 end
   
%                 return
                pause
                delete(h1,h2);
            end

%         h1=plot(M_obstacle_point_x(index)*metric,M_obstacle_point_y(index)*metric,'xk');
%         in=find(M_putanje_x(index,:)==M_obstacle_point_x(index));
% 		h=plot(M_putanje_x(index,in)*metric,M_putanje_y(index,in)*metric,'m-s');
%         pause
%         delete(h);
%         delete(h1);
        elseif (TB_flag(ii,j)==-2) && allnonadmissible %&& (v(47)==0) %non admissible i bez kracenja
		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'k.-','Color',[0.3 0.3 0.3]);
% 		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'k.-','Color',[1 1 0]);
%         pause
%         delete(h);
		elseif (TB_flag(ii,j)==-3) && 0 %kinematic constraint
		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m');
%         pause
%         delete(h);
% 		elseif (TB_flag(ii,j)==-1) %dynamic constraint
% 		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'y-*');
%         pause
%         delete(h);
        elseif (TB_flag(ii,j)==-4) %GOAL_CONSTRAINTS
		h=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m-*','Color',[0.8 0.2 0.2]);
%         pause
%         delete(h);
        end

end     
         if(minus)       
				if (TB_flag_minus(ii,j)==2) %clear
		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'m.-','Color',[1 0 0.2]);
%         pause
%         delete(h);
%         elseif (TB_flag_minus(ii,j)==1) %has obstacle
% 		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'b-*','Color',[0.3 0.3 1]);
%         pause
%         delete(h);
        elseif (TB_flag_minus(ii,j)==-2) && allnonadmissible%non admissible
		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'k.-','Color',[0.5 0.5 0.5]);
%         pause
%         delete(h);
% 		elseif (TB_flag_minus(ii,j)==-1) %dynamic constraint
% 		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'y-*');
%         pause
%         delete(h);
%         elseif (TB_flag_minus(ii,j)==-4) %GOAL_CONSTRAINTS
% 		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'m-*','Color',[0.8 0.2 0.2]);
%         pause
%         delete(h);
                end
         end

                  if(minus2)       
				if (TB_flag_minus2(ii,j)==2) %clear
		h=plot(M_putanje_x_minus2(index,:)*metric,M_putanje_y_minus2(index,:)*metric,'m.-','Color',[1 0.5 0.8]);
%         pause
%         delete(h);
%         elseif (TB_flag_minus(ii,j)==1) %has obstacle
% 		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'b-*','Color',[0.3 0.3 1]);
%         pause
%         delete(h);
        elseif (TB_flag_minus2(ii,j)==-2) && allnonadmissible %non admissible
		h=plot(M_putanje_x_minus2(index,:)*metric,M_putanje_y_minus2(index,:)*metric,'k.-','Color',[0.7 0.7 0.7]);
%         pause
%         delete(h);
% 		elseif (TB_flag_minus(ii,j)==-1) %dynamic constraint
% 		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'y-*');
%         pause
%         delete(h);
%         elseif (TB_flag_minus(ii,j)==-4) %GOAL_CONSTRAINTS
% 		h=plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'m-*','Color',[0.8 0.2 0.2]);
%         pause
%         delete(h);
                end
         end

         end
	     end
%     end
    end
    
    %optimalnu putanju crtamo posebno
	if((LOG(1)>=1)&(LOG(2)>=1)) && 0
	index=(LOG(8)-1)*LOG(3)*LOG(4)+(LOG(1)-1)*LOG(4)+LOG(2);
%    	plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'r-*');
% 	plot(M_putanje_x_minus2(index,:)*metric,M_putanje_y_minus2(index,:)*metric,'r*-');
% 	plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'r*-');  
 	plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'r*-');
    end
if ((LOG(1)>=1) && (LOG(2)>=1)) && optdraw
plot(kl_x*metric,kl_y*metric,'r.-','Color',[1 0.5 0]);
    if opt_col_draw
%         discr=1;
%                 for ii=1:length(kl_x)
            ii=length(kl_x);
                if mod(ii,4)==3 || 1
                    if discr
                th_temp=floor(mod(kl_th(ii),2*pi)/(angularresolution))*(angularresolution);
                x_temp=floor((kl_x(ii)-xorigin)/cell)+0.5; y_temp=floor((kl_y(ii)-yorigin)/cell)+0.5;
                plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[1 0.5 0.],'Linewidth',1);
                plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'r'); 
                    else
                        th_temp=kl_th(ii);
                        x_temp=kl_x(ii); y_temp=kl_y(ii);
                        plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[1 0.5 0.],'Linewidth',1);
                        plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'r'); 
                    end
                end
%                 pause
%                 end
    end

end

if draw_old
plot(kl_x_old*metric,kl_y_old*metric,'c-*');
    if optold_col_draw
%         discr=0;
%                 for ii=1:length(kl_x_old)
ii=length(kl_x_old);
                if mod(ii,4)==3 || 1
                    if discr
                th_temp=floor(mod(kl_th_old(ii),2*pi)/(angularresolution))*(angularresolution);
                x_temp=floor((kl_x_old(ii)-xorigin)/cell)+0.5; y_temp=floor((kl_y_old(ii)-yorigin)/cell)+0.5;
                plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0.8 0.8],'Linewidth',1);
                plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'c'); 
                    else
                        th_temp=kl_th_old(ii);
                        x_temp=kl_x_old(ii); y_temp=kl_y_old(ii);
                        plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[0 0.8 0.8],'Linewidth',1);
                        plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'c'); 
                    end
                end
%                 end
    end

end
if draw_break
plot(kl_x_break*metric,kl_y_break*metric,'k-x');
    if break_col_draw
%         discr=0;
%                 for ii=1:length(kl_x_old)
ii=length(kl_x_break);
                if mod(ii,4)==3 || 1
                    if discr
                th_temp=floor(mod(kl_th_break(ii),2*pi)/(angularresolution))*(angularresolution);
                x_temp=floor((kl_x_break(ii)-xorigin)/cell)+0.5; y_temp=floor((kl_y_break(ii)-yorigin)/cell)+0.5;
                plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0.8 0.8],'Linewidth',1);
                plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'c'); 
                    else
                        th_temp=kl_th_break(ii);
                        x_temp=kl_x_break(ii); y_temp=kl_y_break(ii);
                        plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[0 0.8 0.8],'Linewidth',1);
                        plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'c'); 
                    end
                end
%                 end
    end

end
replan_putanja=load('replan_putanja.dat');

    epoint=[-18017.500000,-2110.000000]+0*cell;
%     plot(epoint(1)*metric,epoint(2)*metric,'r*');
	%pose while driving
    if (draw_pose_while_driving)
    
        for ii=1:length(WH_globalna_putanja_x)
%                 discr=1;

                if mod(ii,50)==0 || 0 || replan_putanja(ii)==2
                    if replan_putanja(ii)==2
                        tempcolor=[0 0 0];
                    else
                        tempcolor=[0.5 0.5 1];
                    end
                    if discr
                th_temp=floor(mod(WH_globalna_putanja_th(ii),2*pi)/(angularresolution))*(angularresolution);
                x_temp=floor((WH_globalna_putanja_x(ii)-xorigin)/cell)+0.5; y_temp=floor((WH_globalna_putanja_y(ii)-yorigin)/cell)+0.5;
                plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0 0.8],'Linewidth',1);
                plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'b'); 
                    else
                        th_temp=WH_globalna_putanja_th(ii);
                        x_temp=WH_globalna_putanja_x(ii); y_temp=WH_globalna_putanja_y(ii);
                        plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b','Color',tempcolor,'Linewidth',1);
                        plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'b','Color',tempcolor); 
                    end
                end
%                 pause
            
           
        end
        
    end
    
    %inicijalna D* putanja
    putanja_x=load('global_planner_path_x.dat');
    putanja_y=load('global_planner_path_y.dat');
    putanja_th=load('global_planner_path_th.dat');
    putanja_travcost=load('global_planner_path_travcost.dat');
    putanja_h=load('global_planner_path_h.dat');
    putanja_intlo=load('global_planner_path_intlo.dat');
    putanja_intup=load('global_planner_path_intup.dat');
    planner_putanja_x=(load('global_planner_path_current_x.dat'));
planner_putanja_y=(load('global_planner_path_current_y.dat'));
planner_putanja_th=(load('global_planner_path_current_th.dat'));
planner_putanja_travcost=load('global_planner_path_current_travcost.dat');
planner_putanja_h=load('global_planner_path_current_h.dat');
planner_putanja_intlo=load('global_planner_path_current_intlo.dat');
planner_putanja_intup=load('global_planner_path_current_intup.dat');
pathpointeri=load('wh_svipathpointeri.dat');
interpolatedcost=load('intcost_putanja.dat');
duljina_wit_puta_um=load('replan_putanja.dat');
duljina_wit_puta_um(1)=0;
timesearch=load('vremena_ds_algoritma.dat')*1000;
exploredD=load('explored_ds_algoritma.dat');
timedw=load('vremena_dw_algoritma.dat');


brboje=0;
boja=hsv(10);
boja(1:3,:)=hsv(3);
brboje=mod(brboje,10)+1;

    if draw_dstarpathinit && plotonlytrajectories==0
    ruk=plot((putanja_x+0*cell)*metric,(putanja_y+0*cell)*metric,'g.-')
        set(ruk,'Color',abs(boja(brboje,:)-0.3));
        smallr=0.2;
        if drawori
                        for ii=1:length(putanja_x)
plot(([putanja_x(ii) putanja_x(ii)+rrmm*smallr*cos(putanja_th(ii))])*metric, ([putanja_y(ii) putanja_y(ii)+rrmm*smallr*sin(putanja_th(ii))])*metric,'b-');%,'Color',abs(boja(brboje,:)-0.3));%,'Linewidth',1);
%                         th_temp=putanja_th(ii);
%                         x_temp=putanja_x(ii); y_temp=putanja_y(ii);
%                         plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[1 0 0],'Linewidth',1);
%                         plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'b','Color',[0.5 0.5 1]); 
                        end
        end
    end
    
    if 0
    plot((A(:,1)+0.5)*cell*metric,(A(:,2)+0.5)*cell*metric,'bs-')
                        for ii=1:length(A(:,1))
plot(([A(ii,1)+0.5 A(ii,1)+0.5+rrmm/cell*smallr*cos(A(ii,3)*angularresolution)])*cell*metric, ([A(ii,2)+0.5 A(ii,2)+0.5+rrmm/cell*smallr*sin(A(ii,3)*angularresolution)])*cell*metric,'b-','Color',[0 0.3 0.3],'Linewidth',1);
%                         th_temp=putanja_th(ii);
%                         x_temp=putanja_x(ii); y_temp=putanja_y(ii);
%                         plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[1 0 0],'Linewidth',1);
%                         plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'b','Color',[0.5 0.5 1]); 
                        end
    return                    
    end
     
%     plot((172*cell+xorigin)*metric,(105*cell+yorigin)*metric,'gs-')
%     text((172*cell+xorigin)*metric,(105*cell+yorigin)*metric,'p3')
%     text((162*cell+xorigin)*metric,(130*cell+yorigin)*metric,'p2')
%     text((105*cell+xorigin)*metric,(81*cell+yorigin)*metric,'p1')

    %trenutna D* putanja
trenutna_planner_putanja_x=load('wh_dstar_path_x.dat');
trenutna_planner_putanja_y=load('wh_dstar_path_y.dat');
trenutna_planner_putanja_th=load('wh_dstar_path_th.dat');
if draw_dstarpath && ~isempty(trenutna_planner_putanja_x)
xt=trenutna_planner_putanja_x(1);
yt=trenutna_planner_putanja_y(1);
plot((trenutna_planner_putanja_x+0*cell)*metric,(trenutna_planner_putanja_y+0*cell)*metric,'rs-','Markersize',4);
                        smallr=0.1;
                        for ii=1:length(trenutna_planner_putanja_x)
plot(([trenutna_planner_putanja_x(ii) trenutna_planner_putanja_x(ii)+rrmm*smallr*cos(trenutna_planner_putanja_th(ii))])*metric, ([trenutna_planner_putanja_y(ii) trenutna_planner_putanja_y(ii)+rrmm*smallr*sin(trenutna_planner_putanja_th(ii))])*metric,'b-','Color',[0.3 0.3 0],'Linewidth',1);
                        end
end
% plot(([trenutna_planner_putanja_x(1) trenutna_planner_putanja_x(1)+cell/2]+cell)*metric,([trenutna_planner_putanja_y(1) trenutna_planner_putanja_y(1)+cell/2]+cell)*metric,'ms-');
% plot(([xt xt+cell/2 xt+cell/2 xt]+cell)*metric,([yt yt+cell/2 yt-cell/2 yt]+cell)*metric,'ks-');

%odvozena trajektorija
% plot(((WH_globalna_putanja_x))*metric,((WH_globalna_putanja_y))*metric,'bo-','Color',[0 0 0.8],'Linewidth',1,'MarkerSize',4)
% plot((putanja_x+cell)*metric,(putanja_y+cell)*metric,'g')

if draw_last_traj_pt
                    if discr
                th_temp=floor(mod(WH_globalna_putanja_th(end),2*pi)/(angularresolution))*(angularresolution);
                x_temp=floor((WH_globalna_putanja_x(end)-xorigin)/cell)+0.5; y_temp=floor((WH_globalna_putanja_y(end)-yorigin)/cell)+0.5;
                plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'b--','Color',[0 0 0.8],'Linewidth',1.5);
                plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'b','Linewidth',1.5); 
                    else

th_temp=WH_globalna_putanja_th(end);
x_temp=WH_globalna_putanja_x(end); y_temp=WH_globalna_putanja_y(end);
plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'k','Color',[0 0 0]);%,'Linewidth',1.5);
plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'k'); 
                    end
end

    
    %parameters
    straightcost=10;
    rotcost=straightcost/pi;%1;
    oricost=0; %1 for 3d search, 0 for 2d search
    orisearch=0; %1 for oriintervals, 0 for full3d

    trajlength=0;
    drivenpathcost=0;
    drivenpathcost3d=0;%interpolatedcost(end);
    trajlength3d=0;
zz=length(WH_globalna_putanja_x);
%    for ii=2:length(WH_globalna_putanja_x)
ii=zz+1;
while ii>2
%     disp('calc cost')
%         disp(interpolatedcost(ii-1))
    ii=ii-1;

        temptrav=sqrt((WH_globalna_putanja_x(ii)-WH_globalna_putanja_x(ii-1))^2+(WH_globalna_putanja_y(ii)-WH_globalna_putanja_y(ii-1))^2)/cell;
        temprot=(abs(WH_globalna_putanja_th(ii)-WH_globalna_putanja_th(ii-1)))/angularresolution;
        temprot=min([abs(temprot-maxori), temprot]);
        trajlength=trajlength+ temptrav*straightcost;
        drivenpathcost=drivenpathcost + temptrav*straightcost*max(travcost(ii),travcost(ii-1));
        if oricost
            temppathcost=(temptrav*straightcost)*travcost3d(ii-1) + temprot*rotcost;
            drivenpathcost3d=drivenpathcost3d + temppathcost;
        else
            temprot=(abs(WH_globalna_putanja_th(ii)-WH_globalna_putanja_th(ii-1)));
            while temprot>pi
                temprot=abs(temprot-2*pi);
            end
            drivenpathcost3d=drivenpathcost3d + temptrav*straightcost*max(travcost(ii),travcost(ii-1))+temprot*rotcost*max(travcost(ii),travcost(ii-1));
%                     disp('accumulated cost')
%         disp(drivenpathcost3d)
%         disp('calc cost')
%         disp(interpolatedcost(ii-1))
%         pause
        end
                trajlength3d=trajlength3d+ temptrav*straightcost +temprot*rotcost;

    end
    disp('cost of the robot trajectory measured with 2D costs, orientation cost is ignored')
    disp(drivenpathcost)
    disp('cost of the robot trajectory measured with 3D costs including the cost of the orientation change')
    disp(drivenpathcost3d)
    temp=-1*abs(diff(interpolatedcost));
    disp('calc from h')
    disp(-1*temp*ones(size(temp))')

    
    pathcost=0;
    pathlength=0;
    pathlength3d=0;
    zz=length(putanja_x);
%    for ii=2:length(putanja_x)
ii=zz+1;
while ii>2
    ii=ii-1;
        temptrav=sqrt((putanja_x(ii)-putanja_x(ii-1))^2+(putanja_y(ii)-putanja_y(ii-1))^2)/cell;
        if temptrav>1.1
            temptrav=1.4;
        end
        if oricost
        temprot=((abs(putanja_th(ii)-putanja_th(ii-1)))/angularresolution);
        temprot=min([abs(temprot-maxori), temprot]);
        else
            temprot=abs(putanja_th(ii)-putanja_th(ii-1));
            while temprot>pi
                temprot=abs(temprot-2*pi);
            end
        end
        if orisearch
%             inta=[traversalcostpath(ii,3) traversalcostpath(ii,4)];
%             intb=[traversalcostpath(ii-1,3) traversalcostpath(ii-1,4)];
            inta=[putanja_intlo(ii) putanja_intup(ii)]
            intb=[putanja_intlo(ii-1) putanja_intup(ii-1)]
            if mod(inta(2)-inta(1)+1,maxori)>0 %|| mod(intb(2)-intb(1),maxori)<maxori 
            tempa=round(putanja_th(ii)/angularresolution);
            tempb=round(putanja_th(ii-1)/angularresolution);
            if tempa < inta(1) || (tempa < intb(1) && intb(1)>intb(2))
                tempa=tempa+maxori;
            end
            if tempb < intb(1) || (tempb < inta(1) && inta(1)>inta(2)) 
                    tempb=tempb+maxori;
            end
            temprot=abs(tempb-tempa);
            end
        end
        pathlength=pathlength+ temptrav*straightcost;
        pathlength3d=pathlength3d+ temptrav*straightcost +temprot*rotcost;
        if oricost
%             disp('traversal cost')
%             disp(putanja_travcost(ii-1))
%             pathcost
%             temppathcost=(temptrav*straightcost)*traversalcostpath(ii-1,1) + temprot*rotcost;
            temppathcost=(temptrav*straightcost)*putanja_travcost(ii-1) + temprot*rotcost;
        pathcost=pathcost + temppathcost;
%         pathcost
%         temprot
%         temptrav
        else
        pathcost=pathcost + temptrav*straightcost*putanja_travcost(ii-1);%max(putanja_travcost(ii),putanja_travcost(ii-1));%+temprot*rotcost*max(putanja_travcost(ii),putanja_travcost(ii-1));
        end
%         disp('accumulated cost')
%         disp(pathcost)
%         disp('calc cost')
%         disp(putanja_h(ii-1))
%         temptrav
%         disp('travcost ii')
%         disp(putanja_travcost(ii))
%         disp('travcost ii-1')
%         disp(putanja_travcost(ii-1))
% pause
   end
    disp('calculated cost on the path')
    if (numel(putanja_h)>0)
    disp(putanja_h(1))
    else
        putanja_h=0;
    end
    disp('measured cost on the path')
    disp(pathcost)
    temp=diff(putanja_h);
    disp('calc from h')
    disp(-1*temp*ones(size(temp))')

 disp('whole trajectory row: traj2D (cm) traj3D (cm) trajlength (cm) trajlength3d (cm) interpolatedcost (average)')
 tabrow1=[drivenpathcost drivenpathcost3d trajlength trajlength3d interpolatedcost*(ones(size(interpolatedcost))')/length(interpolatedcost)]

 disp('first init path row: pathintcost (cm) pathrealcost (cm) pathlength (mm) pathlength3d (cm) pathnumcells (#) cost (average)')
 tabrow2=[putanja_h(1) pathcost pathlength/straightcost*cell pathlength3d length(putanja_x) putanja_h*(ones(size(putanja_h))')/length(putanja_h)]

 
  robot_trans_vel_kal=load('robot_trans_vel_kal.dat');
 robot_trans_vely=load('robot_trans_vely.dat');
v=sqrt(robot_trans_vel_kal.^2+robot_trans_vely.^2);
 
putx=putanja_x;
puty=putanja_y;
putth=putanja_th;
 poc=1;
iii=1;
trajx=[];trajy=[];trajth=[];trajtrav=[];trajtrav3d=[];trajintcost=[];trajv=[];trajsearch=[];trajexpl=[];trajdw=[];
ax=[];ay=[];ath=[];


for ii=1:length(WH_globalna_putanja_x)
    if (duljina_wit_puta_um(ii)==2 || ii==length(WH_globalna_putanja_x))
        if (ii==length(WH_globalna_putanja_x))
            trajx=[trajx WH_globalna_putanja_x(ii)];
            trajy=[trajy WH_globalna_putanja_y(ii)];
            trajth=[trajth WH_globalna_putanja_th(ii)];
            trajtrav=[trajtrav travcost(ii)];
            trajtrav3d=[trajtrav3d travcost3d(ii)];
            trajintcost=[trajintcost interpolatedcost(ii)];
            trajv=[trajv v(ii)];
            trajsearch=[trajsearch timesearch(ii)];
            trajexpl=[trajexpl exploredD(ii)];
            trajdw=[trajdw timedw(ii)];

        end
        ax=[trajx];
        ay=[trajy];
        ath=[trajth];
        atrav=[trajtrav];
        atrav3d=[trajtrav3d];
        aintcost=[trajintcost];
        av=[trajv];
        
        if  1 %((ii)>247) || (ii>371) || 1
            
            boja2=gray(33);
boja2=boja2(1:25,:);
brb=0;
        for kk=[1 length(ax)] %kk=1:length(ax)
                tempcolor=[0.8 0.8 0.8];
                tempcolor=[0 0 0];
                                    th_temp=ath(kk);
                        x_temp=ax(kk); y_temp=ay(kk);
                        plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b','Color',tempcolor,'Linewidth',1);
                        plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'b','Color',tempcolor); 
        end
            if 0
        for kk=1:length(ax)
            if kk~=1 && kk~=length(ax)
tempcolor=abs(boja(brboje,:)-0.3);
tempcolor=[0.5 0.5 0.5];
            else
                tempcolor=[0 0 0];
            end
            if (mod(kk,29)==0 && kk~=116) || kk==1 || kk==length(ax) %for trail1 and trail 3 53 for trail2 33
                kk
    if kk~=length(ax)
    brb=mod(brb-3,25);
tempcolor=abs(boja2(brb,:));
    end
                                    th_temp=ath(kk);
                        x_temp=ax(kk); y_temp=ay(kk);
                        plot(([x_temp x_temp+rrmm*scale1*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*scale1*sin(th_temp)])*metric,'b','Color',tempcolor,'Linewidth',1);
                        plot(((x_temp+cos(th_temp)*scale1*footprintmm(:,1)-sin(th_temp)*scale1*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*scale1*footprintmm(:,1)+cos(th_temp)*scale1*footprintmm(:,2)))*metric,'b','Color',tempcolor,'linewidth',1); 
            end
        end
            end
%         brboje=mod(brboje,10)+1;
        ruk=plot(((ax))*metric,((ay))*metric,'m.-','LineWidth',2)%,'Color',[1 0 0.8],'MarkerSize',0.4);
        set(ruk,'Color',boja(brboje,:));
%         return
        end
if (0 && ii==length(WH_globalna_putanja_x))
        figure %(13)
%         plot(putanja_travcost,'*-')
        plot(puttravcost,'*-')
        hold on
        plot(atrav3d,'mo-')
        plot(atrav,'go-')
        legend('path cost','traj cost')
        figure
        plot(puth,'*-')
        hold on
        plot(aintcost,'mo-')
        legend('path cost h','traj int cost')
%         figure
%         plot(ax(95:115)*metric,ay(95:115)*metric,'o-')
        return
end
        trajlength=0;
    drivenpathcost=0;
    drivenpathcost3d=0;
    trajlength3d=0;
   for iij=2:length(ax)
        temptrav=sqrt((ax(iij)-ax(iij-1))^2+(ay(iij)-ay(iij-1))^2)/cell;
        temprot=(abs(ath(iij)-ath(iij-1)))/angularresolution;
        temprot=min([abs(temprot-maxori), temprot]);
        trajlength=trajlength+ temptrav*straightcost;
        drivenpathcost=drivenpathcost + temptrav*straightcost*max(atrav(iij),atrav(iij-1));
        if oricost
            temppathcost=(temptrav*straightcost)*atrav3d(iij-1) + temprot*rotcost;
            drivenpathcost3d=drivenpathcost3d + temppathcost;
        else
            temprot=(abs(ath(iij)-ath(iij-1)));
            while temprot>pi
                temprot=abs(temprot-2*pi);
            end
        drivenpathcost3d=drivenpathcost3d + temptrav*straightcost*max(atrav(iij),atrav(iij-1))+temprot*rotcost*max(atrav(iij),atrav(iij-1));
        end
        trajlength3d=trajlength3d+ temptrav*straightcost +temprot*rotcost;
    end
    disp('cost of the robot trajectory measured with 2D costs, orientation cost is ignored')
    disp(drivenpathcost)
    disp('cost of the robot trajectory measured with 3D costs including the cost of the orientation change')
    disp(drivenpathcost3d)
    temp=-1*abs(diff(aintcost));
    disp('calc from int cost')
    disp(-1*temp*ones(size(temp))')
 disp('and its trajectory row: traj2D (cm) traj3D (cm) trajlength (mm) trajlength3d (cm) real cost')%interpolatedcost (average)')
 tabrow1=[drivenpathcost drivenpathcost3d trajlength/straightcost*cell trajlength3d (-1*temp*ones(size(temp))')]%aintcost*(ones(size(aintcost))')/length(aintcost)]

if numel(putx)==0
    putx=0;
    puty=0;
    putth=0;
end
goalpostol=200;
goalthtol=180;
for ng=1:size(newgoals,1)
    temp=sqrt((ax(end)-newgoals(ng,2))^2+(ay(end)-newgoals(ng,3))^2);
%     pause
    if goalpostol>temp
        goalpostol=temp;
        goalthtol=abs(newgoals(ng,4)-ath(end));
        while goalthtol>pi
            goalthtol=abs(goalthtol-2*pi);
        end
        goalthtol=goalthtol*180/pi;
    end
end
%  goalpostol=sqrt((putx(end)-ax(end))^2+(puty(end)-ay(end))^2);%/cell*straightcost;
%     goalthtol=abs(putth(end)-ath(end));
%     while goalthtol>pi
%         goalthtol=abs(goalthtol-2*pi);
%     end
%     goalthtol=goalthtol*180/pi;

disp('close to the goal: goalpostol (mm) goalthtol (deg)')
goaltol=[goalpostol goalthtol]

disp('average speed (mm/s) and time (s) to goal')
time_speed=av*ones(size(av))'/length(av);
time_togoal=length(ax)*0.1;
speedgoal=[time_speed time_togoal]

disp('time D*, explored nodes, time dw')
taverage=[trajsearch*(ones(size(trajsearch))')/length(trajsearch) trajexpl*(ones(size(trajexpl))')/length(trajexpl) trajdw*(ones(size(trajdw))')/length(trajdw)]
tmaxexe=[max(trajsearch) max(trajexpl) max(trajdw)]
 
        trajx=[];trajy=[];trajth=[];trajtrav=[];trajtrav3d=[];trajintcost=[];trajv=[];trajsearch=[];trajexpl=[];trajdw=[];
    end
    trajx=[trajx WH_globalna_putanja_x(ii)];
    trajy=[trajy WH_globalna_putanja_y(ii)];
    trajth=[trajth WH_globalna_putanja_th(ii)];
    trajtrav=[trajtrav travcost(ii)];
    trajtrav3d=[trajtrav3d travcost3d(ii)];
    trajintcost=[trajintcost interpolatedcost(ii)];
    trajv=[trajv v(ii)];
            trajsearch=[trajsearch timesearch(ii)];
            trajexpl=[trajexpl exploredD(ii)];
            trajdw=[trajdw timedw(ii)];
        if (iii<=length(pathpointeri))
    if (pathpointeri(iii)~=0) && ((duljina_wit_puta_um(ii)~=0)) && (ii>1)
        kraj=pathpointeri(iii)+poc-1;
        if ~isempty(planner_putanja_x(poc:kraj))
            if duljina_wit_puta_um(ii)==1 && 0 %kad hocu gore malo iznad crtati zadnju putanju
                 putx=planner_putanja_x(poc:kraj);
                puty=planner_putanja_y(poc:kraj);
                putth=planner_putanja_th(poc:kraj);
                puttravcost=planner_putanja_travcost(poc:kraj);
                puth=planner_putanja_h(poc:kraj);
                putintlo=planner_putanja_intlo(poc:kraj);
                putintup=planner_putanja_intup(poc:kraj);
           end
            if (duljina_wit_puta_um(ii)==2 && 1) %|| ii==length(WH_globalna_putanja_x) %|| iii==382
                putx=planner_putanja_x(poc:kraj);
                puty=planner_putanja_y(poc:kraj);
                putth=planner_putanja_th(poc:kraj);
                puttravcost=planner_putanja_travcost(poc:kraj);
                puth=planner_putanja_h(poc:kraj);
                        if orisearch

                putintlo=planner_putanja_intlo(poc:kraj);
                putintup=planner_putanja_intup(poc:kraj);
                        end
if plotonlytrajectories==0                        
%                 plot(putx*metric,puty*metric,'k-');
        ruka=plot(putx*metric,puty*metric,'k.-');
          brboje=mod(brboje,10)+1;
        set(ruka,'Color',abs(boja(brboje,:)-0.3));
                          smallr=0.2;
                          if 1
                        for iij=1:length(putx)
ruk=plot(([putx(iij) putx(iij)+rrmm*smallr*cos(putth(iij))])*metric, ([puty(iij) puty(iij)+rrmm*smallr*sin(putth(iij))])*metric,'b-','Color','k'); %abs(boja(brboje,:)-0.3));%,'Linewidth',1);
                        th_temp=putth(iij);
                        x_temp=putx(iij); y_temp=puty(iij);
%                         plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b--','Color',[1 0 0],'Linewidth',1);
%                         plot(((x_temp+cos(th_temp)*footprintmm(:,1)-sin(th_temp)*footprintmm(:,2)))*metric,((y_temp+sin(th_temp)*footprintmm(:,1)+cos(th_temp)*footprintmm(:,2)))*metric,'b','Color',[0.5 0.5 1]); 
% pause
                        end
                          end
else
              brboje=mod(brboje,10)+1;
end
      pathcost=0;
    pathlength=0;
    pathlength3d=0;
    zz=length(putx);
%    for iij=2:length(putx)
iij=zz+1;
while iij>2
    iij=iij-1;
        temptrav=sqrt((putx(iij)-putx(iij-1))^2+(puty(iij)-puty(iij-1))^2)/cell;
        if temptrav>1.1
            temptrav=1.4;
        end
        if oricost
        temprot=((abs(putth(iij)-putth(iij-1)))/angularresolution);
        temprot=min([abs(temprot-maxori), temprot]);
        else
           temprot=(abs(putth(iij)-putth(iij-1))); 
           while temprot>pi
                temprot=abs(temprot-2*pi);
           end
        end
        if orisearch
%             inta=[traversalcostpath(ii,3) traversalcostpath(ii,4)];
%             intb=[traversalcostpath(ii-1,3) traversalcostpath(ii-1,4)];
            inta=[putintlo(iij) putintup(iij)];
            intb=[putintlo(iij-1) putintup(iij-1)];
            if mod(inta(2)-inta(1)+1,maxori)>0 %|| mod(intb(2)-intb(1),maxori)<maxori 
            tempa=round(putth(iij)/angularresolution);
            tempb=round(putth(iij-1)/angularresolution);
            if tempa < inta(1) || (tempa < intb(1) && intb(1)>intb(2))
                tempa=tempa+maxori;
            end
            if tempb < intb(1) || (tempb < inta(1) && inta(1)>inta(2)) 
                    tempb=tempb+maxori;
            end
            temprot=abs(tempb-tempa);
            end
        end
        pathlength=pathlength+ temptrav*straightcost;
        pathlength3d=pathlength3d+ temptrav*straightcost +temprot*rotcost;
        if oricost
            disp('traversal cost')
            disp(puttravcost(iij-1))
%             pathcost
%             temppathcost=(temptrav*straightcost)*traversalcostpath(ii-1,1) + temprot*rotcost;
            temppathcost=(temptrav*straightcost)*puttravcost(iij-1) + temprot*rotcost;
        pathcost=pathcost + temppathcost;
%         pathcost
%         temprot
%         temptrav
        else
        pathcost=pathcost + temptrav*straightcost*puttravcost(iij-1);%max(puttravcost(iij),puttravcost(iij-1));%+temprot*rotcost*max(puttravcost(iij),puttravcost(iij-1));
        end
%         disp('accumulated cost')
%         disp(pathcost)
%         disp('calc cost')
%         disp(puth(iij-1))
% pause
   end
    disp('calculated cost on the path')
    disp(puth(1))
    disp('measured cost on the path')
    disp(pathcost)
    temp=diff(puth);
    disp('calc from h')
    disp(-1*temp*ones(size(temp))')

 
  disp('next init path row: pathintcost (cm) pathrealcost (cm) pathlength (cm) pathlength3d (cm) pathnumcells (#) cost (average)')
 tabrow2=[puth(1) pathcost pathlength pathlength3d length(putx) puth*(ones(size(puth))')/length(puth)]
                     
                        
          end
        else
            ruka=-1;
        end
        poc=kraj+1;
        iii=iii+1;
    end
        end
end

 
%crtam onu na sredini - ta bi trebala odgovarati staroj osim zavrsetka
    %osim zadnje tocke
% 	index=(3-1)*LOG(4)+4;
%  	plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'r-',M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'rx');

%crtam debug, stara bolja a izabrao bi ovu koja se poklapa sa starom
% ni=1; nj=3;%prepisano iz DW, treba uvecati za 1 zbog matlaba
% index=(ni+1-1)*LOG(4)+nj+1;
%  	plot(M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'m-',M_putanje_x_minus(index,:)*metric,M_putanje_y_minus(index,:)*metric,'m*');
%  	tocka=31;
%     plot(M_putanje_x_minus(index,tocka)*metric,M_putanje_y_minus(index,tocka)*metric,'kd');
% % 
% ni=0; nj=0;%prepisano iz DW, treba uvecati za 1 zbog matlaba
% index=(ni+1-1)*LOG(4)+nj+1;
%  	plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m-',M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m*');
% hold on
%     ni=1; nj=0;%prepisano iz DW, treba uvecati za 1 zbog matlaba
% index=(ni+1-1)*LOG(4)+nj+1;
%  	plot(M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'b-',M_putanje_x_plus(index,:)*metric,M_putanje_y_plus(index,:)*metric,'bx');
% 
    ylabel('y [m]', 'fontsize',12,'fontname', 'times');
    xlabel('x [m]', 'fontsize',12,'fontname', 'times');
    h=gca;
    set(h,'fontsize',12,'fontname','times','box', 'on');
% box off
%     figure
%     plot(kl_x_old*metric,'c-*')
%     hold on
%     plot(M_putanje_x(index,:)*metric,'rx-')

goal=load('wh_goal_position.dat');
% plot((goal(1)+0*cell)*metric,(goal(2)+0*cell)*metric,'b*','Color',[1 0.5 1])
                        th_temp=goal(3);
                        x_temp=goal(1); y_temp=goal(2);
% plot(([x_temp x_temp+rrmm*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*sin(th_temp)])*metric,'b-','Color',[0.8 0 0.8],'Linewidth',1);

    goalpostol=sqrt((WH_globalna_putanja_x(end)-goal(1))^2+(WH_globalna_putanja_y(end)-goal(2))^2);%/cell*straightcost;
    goalthtol=abs(WH_globalna_putanja_th(end)-goal(3));
    while goalthtol>pi
        goalthtol=abs(goalthtol-2*pi);
    end
    goalthtol=goalthtol*180/pi;
disp('close to the goal: goalpostol (mm) goalthtol (deg)')
goaltol=[goalpostol goalthtol]

time_speed=load('time_speed.dat');
disp('average speed (mm/s) and time (s) to goal')
time_speed

if draw_cspacepath
cspath=load('cspath');
plot((cspath(:,1)*cell+cell/2+xorigin)*metric,(cspath(:,2)*cell+cell/2+yorigin)*metric,'cs-')

for i=1:size(cspath,1)
                        th_temp=cspath(i,3); %+angularresolution/2;
                        x_temp=cspath(i,1)*cell+cell/2+xorigin; y_temp=cspath(i,2)*cell+cell/2+yorigin;
                        smallr=0.1;
plot((cspath(i,1)*cell+cell/2+xorigin)*metric,(cspath(i,2)*cell+cell/2+yorigin)*metric,'cs-')
plot(([x_temp x_temp+rrmm*smallr*cos(th_temp)])*metric, ([y_temp y_temp+rrmm*smallr*sin(th_temp)])*metric,'b-','Color',[0 0.3 0.3],'Linewidth',1);
                    if discr
                th_temp=floor(mod(cspath(i,3),2*pi)/(angularresolution))*(angularresolution);
                x_temp=cspath(i,1)+0.5; y_temp=cspath(i,2)+0.5;
                plot(([x_temp x_temp+rr*cos(th_temp)]*cell+xorigin)*metric, ([y_temp y_temp+rr*sin(th_temp)]*cell+yorigin)*metric,'g--','Color',[0.8 0 0],'Linewidth',1);
                plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2))*cell+xorigin)*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2))*cell+yorigin)*metric,'r--'); 
                    end



% pause
end
end

if 0
figure
plot(travcost3d,'.-')
hold on
plot(travcost,'g.-')
legend('3D cost', '2D cost')
end

if plotlasers
    racunajponovo=1;
    if racunajponovo
%     laserx=load('wh_svilaseri_x.dat');
% lasery=load('wh_svilaseri_y.dat');
% load najboljilaseri %prethodno ih snimi u workspace nakon crtajloklaser.m
laserx=globlaserix;
lasery=globlaseriy;
staticlaserx=laserx;
staticlasery=lasery;
movinglaserx=laserx;
movinglasery=lasery;
costmapatemp=costmapa;
for i=1:length(laserx)
        pomx=floor( laserx(i)/cell)+1;
    pomy=floor( lasery(i)/cell)+1;
    staticis=0;
    zapamcen=0;
    if pomx<1 || pomx>sizex || pomy<1 || pomy>sizey
        movinglaserx(i)=0;
        movinglasery(i)=0;
        staticlaserx(i)=0;
        staticlasery(i)=0;
        continue;
    end
    for k=pomx-2:pomx+2
        for l=pomy-2:pomy+2
            if k>=1 && k<=sizex && l>=1 && l<=sizey
                if k==pomx && l==pomy
                    if costmapatemp(l,k)==-1
                    zapamcen=1;
                    else
                        costmapatemp(l,k)=-1;
                    end
                 end
                if costmapa(l,k)==maxcost+2 || zapamcen
                    movinglaserx(i)=0;
                    movinglasery(i)=0;
                    staticis=1;
%                     break;
                end
            end
        end
        if staticis
%             break;
        end
    end
    if staticis==0 || zapamcen
        staticlaserx(i)=0;
        staticlasery(i)=0;
    end
end
    end
insta=find(staticlaserx);
inmov=find(movinglaserx);
plot(staticlaserx(insta)*metric,staticlasery(insta)*metric,'.','MarkerSize',3,'Color',[0 0.6 1]);
plot(movinglaserx(inmov)*metric,movinglasery(inmov)*metric,'.','MarkerSize',3,'Color',[0.5 0. 0.5]);

% plot(laserx*metric,lasery*metric,'.','MarkerSize',0.3,'Color',[0 0.6 1]);
% tezistex=load('wh_teziste_x.dat');
% tezistey=load('wh_teziste_y.dat');
% plot(tezistex*metric,tezistey*metric,'o','MarkerSize',0.6,'Color',[0.5 0. 0.5]);

end

