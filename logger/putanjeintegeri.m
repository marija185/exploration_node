%parametri
crtajlaser=1;
crtajteziste=1;
crtajdwopttraj=1;
crtajpreslik=0;
crtaj_efput=0;
use_twd=0;

% load tamf
% load tam
% Map_Home_x=-26917.500000;
% Map_Home_y=-8160.000000;
% cell=100;
origin=load('origin.dat');
Map_Home_x=origin(1);
Map_Home_y=origin(2);
metric=0.001;
cell=load('cell_size.dat');
  robot_shape=load('robot_shape.dat');
  duljina=robot_shape(1);
  sirina=robot_shape(2);
%   pom=sirina; sirina=duljina; duljina=pom;
  duljina=duljina/cell/metric; %in pixels
  sirina=sirina/cell/metric;
  rr=duljina/2;
  rrmm=duljina/2;

  footprint=[-duljina/2 -sirina/2; duljina/2 -sirina/2; duljina/2 sirina/2; -duljina/2 sirina/2; -duljina/2 -sirina/2]; 
footprint=load('robot_footprint.dat')/cell;
  
metric=1;
logger_sizes=load('logger_sizes.dat');%broj ciklusa logiranja


%     figure(3);
figure
    WH_gridmap_x=(load('wh_gridmap_x.dat')-Map_Home_x)/cell;%+1;
	WH_gridmap_y=(load('wh_gridmap_y.dat')-Map_Home_y)/cell;%+1;
	hold on
WH_planner_globalna_putanja_x=(load('global_planner_path_x.dat')-Map_Home_x)/cell;%+1;
WH_planner_globalna_putanja_y=(load('global_planner_path_y.dat')-Map_Home_y)/cell;%+1;
WH_globalna_putanja_x=(load('robot_globalna_putanja_x.dat')-Map_Home_x)/cell;%+1;
WH_globalna_putanja_y=(load('robot_globalna_putanja_y.dat')-Map_Home_y)/cell;%+1;
WH_globalna_putanja_th=load('robot_globalna_putanja_th.dat');
tocka_infleksije_x=(load('tocka_infleksije_x.dat')-Map_Home_x)/cell;%+1;
tocka_infleksije_y=(load('tocka_infleksije_y.dat')-Map_Home_y)/cell;%+1;
Dstar_path_x=(load('wh_dstar_path_x.dat')-Map_Home_x)/cell;
Dstar_path_y=(load('wh_dstar_path_y.dat')-Map_Home_y)/cell;


% subplot(2,2,1:2,'position',[0.05 0.3 0.9 0.675])

ruka=plot(WH_planner_globalna_putanja_x*metric,WH_planner_globalna_putanja_y*metric,'k');
    goalx=WH_planner_globalna_putanja_x(end)*metric;
    goaly=WH_planner_globalna_putanja_y(end)*metric;
    newgoalx=goalx
    newgoaly=goaly

% plot(tocka_infleksije_x(2:end)*metric,tocka_infleksije_y(2:end)*metric,'g*-');
% legend('inic','run')
%      scatter(WH_gridmap_x*metric,WH_gridmap_y*metric,4,'r');
plot(WH_gridmap_x,WH_gridmap_y,'rs','Color',[0.6 0.6 0.6])


newgoals=load('newgoals');
newgoals(:,2)=(newgoals(:,2)-Map_Home_x)/cell;
newgoals(:,3)=(newgoals(:,3)-Map_Home_y)/cell;
goalflag=newgoals(:,1);
for i=1:length(goalflag)
                        th_temp=newgoals(i,4);
                        x_temp=newgoals(i,2); y_temp=newgoals(i,3);
                        smallr=8;

    if goalflag(i)==3
        plot(newgoals(i,2)*metric,newgoals(i,3)*metric,'gs','LineWidth',2,'MarkerEdgeColor',[0.7 0.7 0.7],'MarkerFaceColor',[0 1 0],'MarkerSize',8);
        plot(([x_temp x_temp+smallr*cos(th_temp)])*metric, ([y_temp y_temp+smallr*sin(th_temp)])*metric,'b-','Color',[0 1 0],'Linewidth',3);
        text((x_temp+0.3*smallr*cos(th_temp))*metric-2.5*metric,(y_temp+0.3*smallr*sin(th_temp))*metric-0.5*metric,mat2str(i-1))
%        text(newgoals(i,2)*metric-2.5*metric,newgoals(i,3)*metric-0.5*metric,mat2str(i-1))

    end
    if goalflag(i)==2
        plot(newgoals(i,2)*metric,newgoals(i,3)*metric,'go','LineWidth',2,'MarkerEdgeColor',[0.7 0.7 0.7],'MarkerFaceColor',[.49 .8 .93],'MarkerSize',8);
        plot(([x_temp x_temp+smallr*cos(th_temp)])*metric, ([y_temp y_temp+smallr*sin(th_temp)])*metric,'b-','Color',[.49 .8 .93],'Linewidth',3);
        text((x_temp+0.3*smallr*cos(th_temp))*metric+1.5*metric,(y_temp+0.3*smallr*sin(th_temp))*metric-0.5*metric,mat2str(i-1))
    end
    if goalflag(i)==1
        plot(newgoals(i,2)*metric,newgoals(i,3)*metric,'gd','LineWidth',2,'MarkerEdgeColor',[0.7 0.7 0.7],'MarkerFaceColor',[.9 0.5 .8],'MarkerSize',8);
        plot(([x_temp x_temp+smallr*cos(th_temp)])*metric, ([y_temp y_temp+smallr*sin(th_temp)])*metric,'b-','Color',[.9 0.5 .8],'Linewidth',3);
        text((x_temp+0.3*smallr*cos(th_temp))*metric-1.5*metric,(y_temp+0.3*smallr*sin(th_temp))*metric-0.5*metric,mat2str(i-1))
%         text(newgoals(i,2)*metric-1.5*metric,newgoals(i,3)*metric-0.5*metric,mat2str(i-1))
    end
end
%plotanje putanja
trenutna_planner_putanja_x=(load('global_planner_path_current_x.dat')-Map_Home_x)/cell;%+1;%sljedeci Pinitial
trenutna_planner_putanja_y=(load('global_planner_path_current_y.dat')-Map_Home_y)/cell;%+1;
trenutna_planner_putanja_th=(load('global_planner_path_current_th.dat'));%+1;
pathpointeri=load('wh_svipathpointeri.dat');
poc=1;
brboje=0;
boja=hsv(10);
if 0
for i=1:max(size(pathpointeri)),
     if (pathpointeri(i)~=0)
         brboje=mod(brboje,10)+1;
 kraj=pathpointeri(i)+poc-1;
%  if (i==max(size(pathpointeri))-1)
 ruka=plot(trenutna_planner_putanja_x(poc:kraj)*metric,trenutna_planner_putanja_y(poc:kraj)*metric,'b');
                         set(ruka,'Color',boja(brboje,:));
                                                   smallr=0.2;
                          if 1
                              putx=trenutna_planner_putanja_x(poc:kraj);
                              puty=trenutna_planner_putanja_y(poc:kraj);
                              putth=trenutna_planner_putanja_th(poc:kraj);
                              rukth=[];
                        for iij=1:length(putx)
rukth=[rukth plot(([putx(iij) putx(iij)+rrmm*smallr*cos(putth(iij))])*metric, ([puty(iij) puty(iij)+rrmm*smallr*sin(putth(iij))])*metric,'b-','Color',abs(boja(brboje,:)-0.3))];%,'Linewidth',1);
                        end
                          end

%                          trenutna_planner_putanja_x(poc)
%                          trenutna_planner_putanja_y(poc)
%   end
 poc=kraj+1;
  pause
  delete([ruka rukth])
 end
end
end
% plot(trenutna_planner_putanja_x*metric,trenutna_planner_putanja_y*metric,'b')
%     rr=3;%kruzici za crtanje pocetne i trenutne pozicije
% 	fi=linspace(0,2*pi,30);
	x_temp=WH_globalna_putanja_x(1);y_temp=WH_globalna_putanja_y(1);th_temp=WH_globalna_putanja_th(1);
% 	for i=1:30
% 		x_circle(i)=x_temp+rr*cos(fi(i));
% 		y_circle(i)=y_temp+rr*sin(fi(i));
% 	end
% 	plot(x_circle*metric,y_circle*metric,'r--');
	plot([x_temp x_temp+rr*cos(th_temp)]*metric, [y_temp y_temp+rr*sin(th_temp)]*metric,'r--');
    plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2)))*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2)))*metric,'r--'); 

%     plot(Dstar_path_x+1,Dstar_path_y+1,'g','LineWidth',1.5)
% plot(tamx+.5,tamy+.5,'cs')%crtanje timestampova
% plot(tamfx+.5,tamfy+.5,'kd')
% tamxr=(tamx-0.5)*cell+Map_Home_x;
% tamyr=(tamy-0.5)*cell+Map_Home_y;
% tamfxr=(tamfx-0.5)*cell+Map_Home_x;
% tamfyr=(tamfy-0.5)*cell+Map_Home_y;
% xlabel('x [m]', 'fontsize',16,'fontname', 'times');
% ylabel('y [m]', 'fontsize',16,'fontname', 'times');
axis equal
grid on
%     axis([0 50 100 150])   
starttxt=[WH_globalna_putanja_x(1) WH_globalna_putanja_y(1)]*metric;
    goaltxt=[WH_globalna_putanja_x(end) WH_globalna_putanja_y(end)]*metric;
% text(starttxt(1),starttxt(2),'start','Interpreter','latex')%s h

if (crtajlaser)
    svilaseri_x=(load('wh_svilaseri_x.dat')-Map_Home_x)/cell;%+1;
    svilaseri_y=(load('wh_svilaseri_y.dat')-Map_Home_y)/cell;%+1;
    svilaserpointeri=load('wh_svilaserpointeri.dat');
    od=1;
    svilaseri2_x=(load('wh_svilaseri2_x.dat')-Map_Home_x)/cell;%+1;
    svilaseri2_y=(load('wh_svilaseri2_y.dat')-Map_Home_y)/cell;%+1;
    svilaserpointeri2=load('wh_svilaserpointeri2.dat');
    odl2=1;
end

if (crtajteziste)
    teziste_x=(load('wh_teziste_x.dat')-Map_Home_x)/cell;%+1;
    teziste_y=(load('wh_teziste_y.dat')-Map_Home_y)/cell;%+1;
    tezistepointeri=load('wh_tezistepointeri.dat');
    od2=1;
end

if (crtajdwopttraj)
    dwopttraj_x=(load('dw_opt_traj_x.dat')-Map_Home_x)/cell;%+1;
    dwopttraj_y=(load('dw_opt_traj_y.dat')-Map_Home_y)/cell;%+1;
    dwpointeri=load('dw_pointeri.dat');
    odt=1;
end

if (crtajpreslik)
    preslik_x=load('wh_dstar_preslik_x.dat')+0.5;%1.5;
    preslik_y=load('wh_dstar_preslik_y.dat')+0.5;%1.5;
end

if use_twd
duljina_wit_puta_um=load('wh_wit_duljina_puta_um.dat');
else
% duljina_wit_puta_um=load('wh_dstar_duljina_puta_um.dat');
duljina_wit_puta_um=load('replan_putanja.dat');
size(duljina_wit_puta_um)
duljina_puta=load('wh_dstar_duljina_puta.dat');
brit=load('wh_dstar_broj_iteracija.dat');
azur=load('wh_dstar_azurirani.dat');
% plot(0.3*sign(duljina_wit_puta_um),'r*-')
% hold on
% plot(0.1*sign(duljina_puta),'go-')
% plot(0.2*sign(brit),'bs-')
% plot(0.05*sign(azur),'kx-')
% size(duljina_puta)
% ii=find(duljina_puta==0);
% size(ii)
% iii=find(duljina_wit_puta_um==0);
% size(iii)
% iiii=find(brit==0);
% size(iiii)
% duljina_wit_puta_um(ii)=1;
end
duljina_wit_puta_um(1)=0;


poc=1;
brboje=0;
iii=1;
%prvi je D*
if (use_twd)
if isempty(pathpointeri)~=1 && (pathpointeri(iii)~=0)
        brboje=mod(brboje,10)+1;
        kraj=pathpointeri(iii)+poc-1;
        rukaD=plot(trenutna_planner_putanja_x(poc:kraj)*metric,trenutna_planner_putanja_y(poc:kraj)*metric,'b--');
        set(rukaD,'Color',boja(brboje,:));
        poc=kraj+1;
        iii=2;
end
end
    
aa=1;
    promjenacilja=0;

    
for ii=1:length(WH_globalna_putanja_x)
% 	plot(WH_globalna_putanja_x(ii)*metric,WH_globalna_putanja_y(ii)*metric,'bo');
drawth=0;
    if (crtajlaser)
        if ii>1
            if ~isempty(ruklas)
            delete(ruklas);
            end
            if ~isempty(ruklas2)
            delete(ruklas2);
            end
        end
        do=od+svilaserpointeri(ii)-1;
        ruklas=plot(svilaseri_x(od:do)*metric,svilaseri_y(od:do)*metric,'.');
        od=do+1;
        dol2=odl2+svilaserpointeri2(ii)-1;
        ruklas2=plot(svilaseri2_x(odl2:dol2)*metric,svilaseri2_y(odl2:dol2)*metric,'g.');
        odl2=dol2+1;
    end
    if (crtajteziste)
        if ii>1
            delete(ruktez);
        end
        do2=od2+tezistepointeri(ii)-1;
        ruktez=plot(teziste_x(od2:do2)*metric,teziste_y(od2:do2)*metric,'ro');
        od2=do2+1;
    end
    if (crtajdwopttraj)
        if ii>1
            delete(rukdw);
        end
        dot=odt+dwpointeri(ii)-1;
        rukdw=plot(dwopttraj_x(odt:dot)*metric,dwopttraj_y(odt:dot)*metric,'r.-','Color',[1 0.5 0]);
        odt=dot+1;
    end
    if (crtajpreslik)
        if ii>1
            delete(rukpres);
        end
        rukpres=plot(preslik_x(ii)*metric,preslik_y(ii)*metric,'ko');
    end
    
    if ((duljina_wit_puta_um(ii)~=0)|| ((use_twd==0)&&(duljina_puta(ii)==0)&&(brit(ii)~=0))) && (ii>1)
        if use_twd
        delete(rukaD)
        end
        if iii>0 && ruka~=-1
            delete(ruka)
            ruka=-1;
        end
    end
    if (iii<=length(pathpointeri))
    if (pathpointeri(iii)~=0) && ((duljina_wit_puta_um(ii)~=0)|| ((use_twd==0)&&(duljina_puta(ii)==0)&&(brit(ii)~=0))) && (ii>1)
        brboje=mod(brboje,10)+1;
        kraj=pathpointeri(iii)+poc-1;
        if ~isempty(trenutna_planner_putanja_x(poc:kraj))
            if duljina_wit_puta_um(ii)==2
        ruka=plot(trenutna_planner_putanja_x(poc:kraj)*metric,trenutna_planner_putanja_y(poc:kraj)*metric,'r');
            else
        ruka=plot(trenutna_planner_putanja_x(poc:kraj)*metric,trenutna_planner_putanja_y(poc:kraj)*metric,'k');
            end
        else
            ruka=-1;
        end
        putDDx=trenutna_planner_putanja_x(poc:kraj)*metric;
        putDDy=trenutna_planner_putanja_y(poc:kraj)*metric;
                          smallr=0.2;
                          if ruka~=-1
                              putx=putDDx;
                              puty=putDDy;
                              putth=trenutna_planner_putanja_th(poc:kraj);
                              rukth=[];
                              drawth=1;
                        for iij=1:length(putx)
rukth=[rukth plot(([putx(iij) putx(iij)+rrmm*smallr*cos(putth(iij))])*metric, ([puty(iij) puty(iij)+rrmm*smallr*sin(putth(iij))])*metric,'b-','Color',abs(boja(brboje,:)-0.3))];%,'Linewidth',1);
                        end
                          end
        if ~isempty(putDDx)
        if (goalx~=putDDx(end) || goaly~=putDDy(end))
            newgoalx=putDDx(end);
            newgoaly=putDDy(end);
        end
        end
        if (0)
            for jj=1:length(putDDx)
                plot(putDDx(jj),putDDy(jj),'k*');
                pause;
            end
        end
%         set(ruka,'Color',boja(brboje,:));
        poc=kraj+1;
        iii=iii+1;
    end
    if use_twd
    if (pathpointeri(iii)~=0) && (duljina_wit_puta_um(ii)~=0) && (ii>1)
        brboje=mod(brboje,10)+1;
        kraj=pathpointeri(iii)+poc-1;
        rukaD=plot(trenutna_planner_putanja_x(poc:kraj)*metric,trenutna_planner_putanja_y(poc:kraj)*metric,'b--');
        set(rukaD,'Color',boja(brboje,:));
        poc=kraj+1;
        iii=iii+1;
    end
    end
    end
    
    x_temp=WH_globalna_putanja_x(ii);y_temp=WH_globalna_putanja_y(ii);th_temp=WH_globalna_putanja_th(ii);
% 	for i=1:30
% 		x_circle(i)=x_temp+rr*cos(fi(i));
% 		y_circle(i)=y_temp+rr*sin(fi(i));
% 	end
% 	hc=plot(x_circle*metric,y_circle*metric,'b');
	hc=plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2)))*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2)))*metric,'b'); 
    hl=plot([x_temp x_temp+rr*cos(th_temp)]*metric, [y_temp y_temp+rr*sin(th_temp)]*metric,'b');

    if (crtaj_efput)
    efput_x=linspace(x_temp,tocka_infleksije_x(ii),10);
    efput_y=linspace(y_temp,tocka_infleksije_y(ii),10);
    else
        efput_x=[];
        efput_y=[];
    end
hep=plot(efput_x*metric,efput_y*metric,'mo-');
hti=plot(tocka_infleksije_x(ii)*metric,tocka_infleksije_y(ii)*metric,'kd');
% axis([67 175 140 210]) %map 078
% axis([90 250 190 330]) %large hall map 078
pause

% print(gcf,'-dpng',mat2str(aa+10000));
aa=aa+1
if (ii==1) && 0
    brojac=1;
    while brojac<=78
        pause
%     print(gcf,'-dpng',mat2str(aa+10000));
        aa=aa+1
        brojac=brojac+1;
    end
end



if (goalx~=newgoalx || goaly~=newgoaly) && 0
    goalx=newgoalx;
    goaly=newgoaly;
    promjenacilja=promjenacilja+1;
    goalx
    goaly
    brojac=1;
    maxbr=1;
    if promjenacilja==2
    maxbr=3;
    end
    if promjenacilja>2
        maxbr=4.5;
    end
    
    while brojac<=10*maxbr
        pause
%     print(gcf,'-dpng',mat2str(aa+10000));
        aa=aa+1
        brojac=brojac+1;
    end
end

delete(hc,hl,hti,hep);
if drawth==1
    delete(rukth)
end
        
end
plot(WH_globalna_putanja_x*metric,WH_globalna_putanja_y*metric,'m');
    if 1
	x_temp=WH_globalna_putanja_x(end);y_temp=WH_globalna_putanja_y(end);th_temp=WH_globalna_putanja_th(logger_sizes);
% 	for i=1:30
% 		x_circle(i)=x_temp+rr*cos(fi(i));
% 		y_circle(i)=y_temp+rr*sin(fi(i));
% 	end
% 	plot(x_circle*metric,y_circle*metric,'r');
	plot([x_temp x_temp+rr*cos(th_temp)]*metric, [y_temp y_temp+rr*sin(th_temp)]*metric,'r');
    plot(((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2)))*metric,((y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2)))*metric,'r'); 
    end
%     text(goaltxt(1),goaltxt(2),'end','Interpreter','latex')%s h

% print(gcf,'-dpng',mat2str(aa+10000));

