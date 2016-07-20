% clear
WH_dstar_cost_map=load('wh_dstar_cost_map.dat');
origin=load('origin.dat');
% Map_Home_x=-25000.00000;
% Map_Home_y=-25000.000000;
Map_Home_x=origin(1);
Map_Home_y=origin(2);
xorigin=Map_Home_x;%player-stage
yorigin=Map_Home_y;
metric=0.001;
cell=100;%load('cell_size.dat');
WH_gridmap_x=load('wh_gridmap_x.dat');
	WH_gridmap_y=load('wh_gridmap_y.dat');
     WH_gridmap_static_cell=load('wh_gridmap_static_cell.dat');
     WH_gridmap_x=WH_gridmap_x*metric;
    WH_gridmap_y=WH_gridmap_y*metric;
novex=[];novey=[];starex=[];starey=[];
for i=1:max(size(WH_gridmap_x))
                if(WH_gridmap_static_cell(i)==1)
                    starex=[starex;WH_gridmap_x(i)];
                    starey=[starey;WH_gridmap_y(i)];
               else
                    novex=[novex;WH_gridmap_x(i)];
                    novey=[novey;WH_gridmap_y(i)];
             end
end

WH_planner_globalna_putanja_x=(load('global_planner_path_x.dat')-Map_Home_x)/cell;
WH_planner_globalna_putanja_y=(load('global_planner_path_y.dat')-Map_Home_y)/cell;
    moving_x=load('wh_gridmap_moving_x.dat'); %ovaj je logiran u trenutnom ciklusu uvijek
    moving_y=load('wh_gridmap_moving_y.dat');

cijena_prepreke=max(max(WH_dstar_cost_map));
Dstar_path_x=(load('wh_dstar_path_x.dat')-Map_Home_x)/cell;
Dstar_path_y=(load('wh_dstar_path_y.dat')-Map_Home_y)/cell;

[sizex,sizey]=size(WH_dstar_cost_map);
veca=zeros(sizex,sizey);
Xos=[0:1:sizex-1];
Yos=[0:1:sizey-1];
xv=[0:1:(sizex+9)];
yv=[0:1:(sizey+9)];

% figure
veca(1:sizex,1:sizey)=WH_dstar_cost_map;

xmin=1; xmax=50; ymin=112; ymax=152;
% xmin=55; xmax=120; ymin=80; ymax=105;
xmin=1; xmax=sizex; ymin=1; ymax=sizey;
%ovdje crtamo malu matricu karticu

 osi=[xmin xmax ymin ymax];
kartica=veca;
 Xos=xv(osi(1):osi(2))+1;
Yos=yv(osi(3):osi(4))+1;
% Xos_real=Xos*cell+xorigin;
% Yos_real=Yos*cell+yorigin;
kartica=kartica(Xos,Yos);
Xos=Xos*cell+xorigin-cell;
Yos=Yos*cell+yorigin-cell;
k=linspace(1,0.4,cijena_prepreke-1);
if cijena_prepreke==2
    k=1;
end
   colormap([[k' k' k'];0 0 0]);

% colormap([k]);
% pcolor(x,y,WH_dstar_f_cost_map','EdgeColor','none');
surf(Xos*metric,Yos*metric,kartica'*0,kartica','EdgeColor',[1 0 1])%'none')
% surf(Xos_real*metric,Yos_real*metric,kartica'*0,kartica','EdgeColor',[0.5 0.5 0.5])%,'EdgeColor','none')
if cijena_prepreke>1
caxis([1 cijena_prepreke])
end

% grid off
% view([90 -90]);

% grid off
view([0 90]);
% surf(xv*metric,yv*metric,veca'*0,veca','EdgeColor','none')
% caxis([minff-2 maxff])
hold on

plot(starex,starey,'g.');
 plot(novex,novey,'m.');
axis equal
