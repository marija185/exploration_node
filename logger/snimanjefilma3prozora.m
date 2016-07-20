% mov=avifile('film.avi');
% mov.compression='mpeg4';
figure(3);
    WH_gridmap_x=load('wh_gridmap_x.dat');
	WH_gridmap_y=load('wh_gridmap_y.dat');
    WH_gridmap_static_cell=load('wh_gridmap_static_cell.dat');
%  	hold on
% pause on
    WH_start_position=load('wh_start_position.dat');
    metric=0.001;
 	logger_sizes=load('logger_sizes.dat');
   WH_gridmap_x=WH_gridmap_x*metric;
    WH_gridmap_y=WH_gridmap_y*metric;
    WH_start_position=WH_start_position*metric;
WH_globalna_putanja_x=load('robot_globalna_putanja_x.dat');
WH_globalna_putanja_y=load('robot_globalna_putanja_y.dat');
WH_globalna_putanja_th=load('robot_globalna_putanja_th.dat');
staticnax=[];novax=[];
staticnay=[];novay=[];
 for i=1:max(size(WH_gridmap_x))
                 if(WH_gridmap_static_cell(i)==1)
                     staticnax=[staticnax WH_gridmap_x(i)];
                     staticnay=[staticnay WH_gridmap_y(i)];
%                  scatter(WH_gridmap_x(i),WH_gridmap_y(i),2,'k');
               else
                           novax=[novax WH_gridmap_x(i)];
                     novay=[novay WH_gridmap_y(i)];
%                   scatter(WH_gridmap_x(i),WH_gridmap_y(i),2,'m');
              end
 end


laserx=load('wh_svilaseri_x.dat');
lasery=load('wh_svilaseri_y.dat');
laserx=laserx*metric;
lasery=lasery*metric;
pointeri=load('wh_svilaserpointeri.dat');
%crtanje svih tezista
tezistex=load('wh_teziste_x.dat');
tezistey=load('wh_teziste_y.dat');
tezistex=tezistex*metric;
tezistey=tezistey*metric;
tezistepointeri=load('wh_tezistepointeri.dat');
time_ref=load('wh_log_time.dat')/1000;
time=load('wh_log_read_time.dat')/1000;
 robot_trans_vel=load('robot_trans_vel.dat');
 robot_rot_vel=load('robot_rot_vel.dat')*180/pi;
 robot_trans_vel_ref=load('robot_trans_vel_ref.dat');
 robot_rot_vel_ref=load('robot_rot_vel_ref.dat');
boja=hsv(50);
brboje=40;
poc=1;tezpoc=1;pocbrzina=1;
for i=1:max(size(pointeri)),
    brboje=mod(brboje,50);%+1;
  %  hold on
%  if (i>50)
%  pocbrzina=i-10;
%  end
% subplot(2,2,3)
%  subplot('position',[0.05 0.05 0.425 0.2])
grH3=subplot(2,2,3);
set(grH3,'Position',[0.3 0.025 0.1 0.2]);

%colormap([1 1 1;0.5 0.5 0.5;0 0 0])
    ruka=bar(robot_trans_vel(i),'b');%brzina stvarna (s kalibr. odom.)

%  colormap([1 0.2 0.2;0.2 1 0.2]);
%       plot(time(pocbrzina:i), robot_trans_vel(pocbrzina:i));%brzina stvarna (s kalibr. odom.)
                       set(gca,'ylim',[0 600],'fontsize',10,'fontname','times','box', 'on'); 
%  hold on
%      plot(time_ref, robot_trans_vel_ref, 'k');%postavna brzina s ukljucenom kalibr. odom.
%  ylabel('v [mm/sec]', 'fontsize',10,'fontname', 'times');%sve se zblesira s ylabel
%  title('v [mm/sec]');
text(-0.1,160,'v [mm/sec]','rotation',90);
% subplot(2,2,4)
%  subplot('position',[0.525 0.05 0.425 0.2])
grH4=subplot(2,2,4);
set(grH4,'Position',[0.525 0.025 0.225 0.2]);
rot=robot_rot_vel(i)/100;
if (rot>0)
    pie([rot (1-rot)]);%colormap bone%
colormap([0.2 0 0.8;1 1 1]);
%      set(ruk,'Color',[1 1 1;0 0 0]);colormap([1 1 1;0 0 0;0.5 0.5 0.5]);
else
    pie([(1+rot) -rot]);%colormap gray%
colormap([1 1 1;0.2 0 0.8]);
%      set(ruk,'Color',[0 0 0;1 1 1]);colormap([0 0 0;1 1 1;0.5 0.5 0.5]);
end
                       set(gca,'fontsize',10,'fontname','times','box', 'on'); 
%       plot(time(pocbrzina:i), robot_rot_vel(pocbrzina:i));
%                        set(gca,'xlim',[0 102.2],'ylim',[-100 100]); 
%  hold on
%     plot(time_ref, robot_rot_vel_ref, 'k');
%  ylabel('\omega [deg/sec]', 'fontsize',10,'fontname', 'times');
text(-2,-0.9,'\omega/\omega_{max} [%/sec]','rotation',90);
%  title('\omega [deg/sec]');
% subplot(2,2,1:2)
subplot(2,2,1:2,'position',[0.05 0.3 0.9 0.675])
%  xlabel('x [m]', 'fontsize',10,'fontname', 'times');
%  ylabel('y [m]', 'fontsize',10,'fontname', 'times');

    if (i>100)
        brejkanje=1;
    end
     if (pointeri(i)~=0)
 kraj=pointeri(i)+poc-1;
% if (i==max(size(pointeri)))
 ruka=plot(laserx(poc:kraj),lasery(poc:kraj),'.');
 hold on
                         set(ruka,'Color',boja(brboje,:));
%                     end
 poc=kraj+1;
 end
 %crtanje staticnih prepreka
 plot(staticnax,staticnay,'k.');
 hold on
%crtanje svih tezista
if (tezistepointeri(i)~=0)
tezkraj=tezistepointeri(i)+tezpoc-1;
% if (i==max(size(pointeri)))
%  ruka=plot(tezistex(tezpoc:tezkraj),tezistey(tezpoc:tezkraj),'*');
%                          set(ruka,'Color',boja(brboje,:));
%                     end
tezpoc=tezkraj+1;
end
%if (i==1)
rr=260;
	fi=linspace(0,2*pi,30);
	x_temp=WH_globalna_putanja_x(i);y_temp=WH_globalna_putanja_y(i);th_temp=WH_globalna_putanja_th(i);
	for j=1:30
		x_circle(j)=x_temp+rr*cos(fi(j));
		y_circle(j)=y_temp+rr*sin(fi(j));
	end
	ruka=plot(x_circle*metric,y_circle*metric,'g--');
%     hold on
                        set(ruka,'Color',boja(brboje,:));
	ruka=plot([x_temp x_temp+rr*cos(th_temp)]*metric, [y_temp y_temp+rr*sin(th_temp)]*metric,'g--');
                        set(ruka,'Color',boja(brboje,:));
%                         set(gca,'xlim',[5 23],'ylim',[4 16.2],'fontsize',10,'fontname','times','box', 'on'); 
%                         set(gca,'xlim',[-27.5 -14],'ylim',[-1 7.2],'fontsize',10,'fontname','times','box', 'on'); 
                        set(gca,'xlim',[-15 -6],'ylim',[-1 7],'fontsize',10,'fontname','times','box', 'on'); 
%                         set(gca,'xlim',[-21 -13],'ylim',[-1 7],'fontsize',10,'fontname','times','box', 'on'); 
%                         f=getframe(gcf);
%                         mov=addframe(mov,f);
                        print(gcf,'-dpng',mat2str(i+1000));
                        hold off
                        %                   end
%      if (pointeri(i)~=0)
% kraj=pointeri(i)+poc-1;
% % if (i==1)
% indeksi=find(sqrt((laserx(poc:kraj)-x_temp*metric).^2+(lasery(poc:kraj)-y_temp*metric).^2)<3);
% ruka=plot(laserx(indeksi),lasery(indeksi),'o');
%                         set(ruka,'Color',boja(brboje,:));
% %                     end                       
% poc=kraj+1;
% end
 pause
end
% axis equal
col_mov_old_x=load('wh_gridmap_colmov_old_x.dat');
col_mov_old_y=load('wh_gridmap_colmov_old_y.dat');
% plot(col_mov_old_x*metric,col_mov_old_y*metric,'gd');
xlabel('x [m]', 'fontsize',16,'fontname', 'times');
ylabel('y [m]', 'fontsize',16,'fontname', 'times');
% mov=close(mov);