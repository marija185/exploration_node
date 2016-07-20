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
WH_planner_globalna_putanja_x=(load('global_planner_path_x.dat')-Map_Home_x)/cell;
WH_planner_globalna_putanja_y=(load('global_planner_path_y.dat')-Map_Home_y)/cell;
	M_laser_obstacles=load('log_laser_obstacles.dat'); %prepreke [x1 y1;x2 y2;x3 y3;...] %ove ce kasnit jedan korak u slucaju no_path-a
    moving_x=load('wh_gridmap_moving_x.dat'); %ovaj je logiran u trenutnom ciklusu uvijek, a ovaj iznad u DW samo pa moze bit iz proslog
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

figure
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
Xos=Xos*cell+xorigin;
Yos=Yos*cell+yorigin;
k=linspace(1,0.4,cijena_prepreke-1);
if cijena_prepreke==2
    k=1;
end
   colormap([[k' k' k'];0 0 0]);

% colormap([k]);
% pcolor(x,y,WH_dstar_f_cost_map','EdgeColor','none');
surf(Xos*metric,Yos*metric,kartica'*0,kartica','EdgeColor','none')
% surf(Xos_real*metric,Yos_real*metric,kartica'*0,kartica','EdgeColor',[0.5 0.5 0.5])%,'EdgeColor','none')
if cijena_prepreke>1
caxis([1 cijena_prepreke])
end

grid off
% view([90 -90]);

grid off
view([0 90]);
% surf(xv*metric,yv*metric,veca'*0,veca','EdgeColor','none')
% caxis([minff-2 maxff])
hold on




% for i=1:length(WH_planner_globalna_putanja_x)
%    plot(((WH_planner_globalna_putanja_x(i)+1)*cell+xorigin)*metric,((WH_planner_globalna_putanja_y(i)+1)*cell+yorigin)*metric,'b-*','LineWidth',1.5)
% pause
% end 

plot(((WH_planner_globalna_putanja_x+1)*cell+xorigin)*metric,((WH_planner_globalna_putanja_y+1)*cell+yorigin)*metric,'b','LineWidth',1.5)
plot(((Dstar_path_x+1)*cell+xorigin)*metric,((Dstar_path_y+1)*cell+yorigin)*metric,'g','LineWidth',1.5)
if (isempty(M_laser_obstacles)==0)
    plot(M_laser_obstacles(:,1)*metric+0.1,M_laser_obstacles(:,2)*metric+0.1,'g.');
end
      plot(moving_x*metric+0.1,moving_y*metric+0.1,'b.');


start=[549 378];%550,366]
goal=[548,353]
plot(((start(1)+1.5)*cell+xorigin)*metric,((start(2)+1.5)*cell+yorigin)*metric,'ys');
plot(((goal(1)+1.5)*cell+xorigin)*metric,((goal(2)+1.5)*cell+yorigin)*metric,'gs');

            if length(pozg)>0
plot((pozg(:,1)+cell)*metric,(pozg(:,2)+cell)*metric,'r*');
            end


if 1    
    %svi pointeri
    WH_dstar_x=(load('wh_dstar_x.dat')+cell);
    WH_dstar_y=(load('wh_dstar_y.dat')+cell);
    WH_dstar_dx=load('wh_dstar_dx.dat');
    WH_dstar_dy=load('wh_dstar_dy.dat');

    quiver(WH_dstar_x*metric,WH_dstar_y*metric,WH_dstar_dx*metric,WH_dstar_dy*metric,0.17,'Color',[0.6 0.5 0.7]);
end

    ylabel('y [m]', 'fontsize',16,'fontname', 'times');
    xlabel('x [m]', 'fontsize',16,'fontname', 'times');
    h=gca;
    set(h,'fontsize',12,'fontname','times','box', 'on');

axis equal tight
xmin=xmin*cell+xorigin;
xmax=xmax*cell+xorigin;
ymin=ymin*cell+yorigin;
ymax=ymax*cell+yorigin;
axis([xmin xmax ymin ymax]*metric)

