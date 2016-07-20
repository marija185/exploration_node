% close all  
WH_gridmap_x=load('wh_gridmap_x.dat');
	WH_gridmap_y=load('wh_gridmap_y.dat');
     WH_gridmap_static_cell=load('wh_gridmap_static_cell.dat');
    metric=0.001;
WH_gridmap_x=WH_gridmap_x*metric;
    WH_gridmap_y=WH_gridmap_y*metric;
%     figure(1);
    hold on;%grid on;
%     axis equal tight;
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
plot(starex,starey,'g.');
 plot(novex,novey,'m.');
WH_globalna_putanja_x=load('robot_globalna_putanja_x.dat');%odvozena putanja
WH_globalna_putanja_y=load('robot_globalna_putanja_y.dat');
WH_globalna_putanja_th=load('robot_globalna_putanja_th.dat');
fakeloc_x=load('fakelocrobot_globalna_putanja_x.dat');%odvozena putanja iz fakelocalize-a
fakeloc_y=load('fakelocrobot_globalna_putanja_y.dat');
fakeloc_th=load('fakelocrobot_globalna_putanja_th.dat');
goal=load('wh_goal_position.dat');

%plotanje putanja
%  plot(WH_globalna_putanja_x*metric,WH_globalna_putanja_y*metric,'r');
 plot(WH_globalna_putanja_x*metric,WH_globalna_putanja_y*metric,'r.-');
% plot(goal(1)*metric,goal(2)*metric,'b*');
%  plot(fakeloc_x*metric,fakeloc_y*metric,'g.');

    %plotanje pocetne i konacne koordinate robota
    %nacrtajmo robota polumjer robota (mm)
	rr=260*metric;
	fi=linspace(0,2*pi,30);
	x_temp=WH_globalna_putanja_x(1)*metric;y_temp=WH_globalna_putanja_y(1)*metric;th_temp=WH_globalna_putanja_th(1);
	for i=1:30
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
	end
	plot(x_circle,y_circle,'k--');
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');
% 
	x_temp=WH_globalna_putanja_x(end)*metric;y_temp=WH_globalna_putanja_y(end)*metric;th_temp=WH_globalna_putanja_th(end);
	for i=1:30
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
	end
	plot(x_circle,y_circle,'k--');
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');
%     
      if (0)
    kraj=length(WH_globalna_putanja_x)
    for i=1:kraj
        	rr=260*metric;
	fi=linspace(0,2*pi,30);
	x_temp=WH_globalna_putanja_x(i)*metric;y_temp=WH_globalna_putanja_y(i)*metric;th_temp=WH_globalna_putanja_th(i);
	for i=1:30
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
	end
	h1=plot(x_circle,y_circle,'k--');
	h2=plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');
    pause
    delete(h1)
    delete(h2)
    end
    end
   
    %figure(1);
    ylabel('y [m]', 'fontsize',16,'fontname', 'times');
    %title('Robot environment', 'fontsize', 18,'fontname', 'times');
    xlabel('x [m]', 'fontsize',16,'fontname', 'times');
    h=gca;
    set(h,'fontsize',10,'fontname','times','box', 'on');
    axminx=min(min(starex),min(novex));
    axminy=min(min(starey),min(novey));
    axmaxx=max(max(starex),max(novex));
    axmaxy=max(max(starey),max(novey));
%     axis([-26 6 -10 10])   
% axis([axminx axmaxx axminy axmaxy])
starttxt=[WH_globalna_putanja_x(1) WH_globalna_putanja_y(1)]*metric;
    goaltxt=[WH_globalna_putanja_x(end) WH_globalna_putanja_y(end)]*metric;
text(starttxt(1),starttxt(2),'start','Interpreter','latex')%s h
text(goaltxt(1),goaltxt(2),'goal','Interpreter','latex')%s h
%print('-depsc2','ime.eps')%radi super bounding boxove