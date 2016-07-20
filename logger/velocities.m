metric=0.001;
duljina=load('ef_path_length.dat');
read_time=load('wh_log_read_time.dat');
 robot_trans_vel=load('robot_trans_vel.dat');
 robot_rot_vel=load('robot_rot_vel.dat');
 robot_trans_vel_ref=load('robot_trans_vel_ref.dat');
 robot_trans_vely=load('robot_trans_vely.dat');
 robot_trans_vely_ref=load('robot_trans_vely_ref.dat');
 robot_rot_vel_ref=load('robot_rot_vel_ref.dat');
 robot_trans_vel_refdin=load('robot_trans_vel_refdin.dat');
 robot_rot_vel_refdin=load('robot_rot_vel_refdin.dat');
 robot_trans_vel_kal=load('robot_trans_vel_kal.dat');
 robot_rot_vel_kal=load('robot_rot_vel_kal.dat');
 robot_trans_vel_des=load('robot_trans_vel_des.dat');
 robot_rot_vel_des=load('robot_rot_vel_des.dat');
 logger_sizes=load('logger_sizes.dat');
WH_globalna_putanja_x=load('robot_globalna_putanja_x.dat');%trenutni put iz planera (od dstara pomaknut za pocetnu poziciju robota od sredine celije)
WH_globalna_putanja_y=load('robot_globalna_putanja_y.dat');
 WH_globalna_putanja_th=load('robot_globalna_putanja_th.dat');
robot_path_size=logger_sizes(1);
 rps=robot_path_size;
 %td=0.1;
 %time=linspace(0,robot_path_size*td,robot_path_size);
 duljina=duljina(1:robot_path_size);
 time=read_time(1:robot_path_size)/100;
 time=[1:robot_path_size];
 robot_trans_vel=robot_trans_vel(1:robot_path_size);
 robot_rot_vel=robot_rot_vel(1:robot_path_size)*180/pi;
 robot_trans_vel_ref=robot_trans_vel_ref(1:(robot_path_size))/metric;
 robot_trans_vely_ref=robot_trans_vely_ref(1:(robot_path_size))/metric;
 robot_rot_vel_ref=robot_rot_vel_ref(1:(robot_path_size))*180/pi;
 robot_trans_vel_refdin=robot_trans_vel_refdin(1:(robot_path_size));
 robot_rot_vel_refdin=robot_rot_vel_refdin(1:(robot_path_size))*180/pi;
 robot_trans_vel_kal=robot_trans_vel_kal(1:(robot_path_size));
 robot_rot_vel_kal=robot_rot_vel_kal(1:(robot_path_size))*180/pi;
 robot_trans_vel_des=robot_trans_vel_des(1:(robot_path_size));
 robot_rot_vel_des=robot_rot_vel_des(1:(robot_path_size))*180/pi;
 predikcija_rot_kp1=robot_rot_vel_kal;
 predikcija_trans_kp1=robot_rot_vel_kal;
predikcija_rot_kp2=robot_rot_vel_kal;
predikcija_trans_kp2=robot_rot_vel_kal;
predikcija_x=robot_rot_vel_kal;
predikcija_y=robot_rot_vel_kal;
predikcija_th=predikcija_y;
%  T1=0.055
%  T2=0.055
%  hc=tf([1],[T1*T2 (T1+T2) 1])
%  hd=c2d(hc,0.1)
%  b=hd.num{1}
%  a=hd.den{1}
%  b1=b(2);
%  b0=b(3);
%  a1=a(2);
%  a0=a(3);
 Tv=0.4;
 Tw=0.4;
predikcija=strcat('predikcija po PT1 s Tv=',num2str(Tv),', Tw=',num2str(Tw));
 for i=1:rps,
 predikcija_rot_kp1(i)=0;
 predikcija_trans_kp1(i)=0;
predikcija_rot_kp2(i)=0;
predikcija_trans_kp2(i)=0;
predikcija_x(i)=0;
predikcija_y(i)=0;
predikcija_th(i)=0;

if(i>2)
     predikcija_rot_kp1(i)=robot_rot_vel(i-1)*exp(-0.1/Tw)+(1-exp(-0.1/Tw))*robot_rot_vel_refdin(i-1);
     predikcija_trans_kp1(i)=robot_trans_vel(i-1)*exp(-0.1/Tv)+(1-exp(-0.1/Tv))*robot_trans_vel_refdin(i-1);
%predikcija v(k+2)
     predikcija_rot_kp2(i)=predikcija_rot_kp1(i-1)*exp(-0.1/Tw)+(1-exp(-0.1/Tw))*robot_rot_vel_refdin(i-2);
     predikcija_trans_kp2(i)=predikcija_trans_kp1(i-1)*exp(-0.1/Tv)+(1-exp(-0.1/Tv))*robot_trans_vel_refdin(i-2);
	deltaxy=robot_trans_vel_refdin(i-1)*0.1+(robot_trans_vel_refdin(i-1)-robot_trans_vel(i-1))*Tv*(exp(-0.1/Tv)-1);
	deltath=robot_rot_vel_refdin(i-1)*0.1+(robot_rot_vel_refdin(i-1)-robot_rot_vel(i-1))*Tv*(exp(-0.1/Tv)-1);
	predikcija_th(i)=deltath*pi/180+WH_globalna_putanja_th(i-1);
	predikcija_x(i)=WH_globalna_putanja_x(i-1)+deltaxy*cos(predikcija_th(i));
	predikcija_y(i)=WH_globalna_putanja_y(i-1)+deltaxy*sin(predikcija_th(i));

end
%ovo je bila pt2 predikcija
%  if (i>2)
%  predikcija_rot2(i)=-a1*robot_rot_vel(i-1)-a0*robot_rot_vel(i-2)+b0*robot_rot_vel_refdin(i-2)+b1*robot_rot_vel_refdin(i-1);
%  predikcija_trans2(i)=-a1*robot_trans_vel(i-1)-a0*robot_trans_vel(i-2)+b0*robot_trans_vel_refdin(i-2)+b1*robot_trans_vel_refdin(i-1);
%  end
 end
figure;
boja=[0 0 0];
 subplot(2,1,1);
 hold on
     grid on;
%    plot(time/10, 
   plot(robot_trans_vel_kal, 'r.-','Linewidth',1,'Color',boja);%brzina iz enkodera
%    plot(time/10, 
%    plot(robot_trans_vely, 'm.-');%side speed
%    plot(sqrt(robot_trans_vel_kal.^2+robot_trans_vely.^2),'b.-')
%            plot([rp(1) rp(1)]/10,[0 robot_trans_vel_kal(rp(1)+1)],'b--')
%                 plot([rp(2) rp(2)]/10,[0 robot_trans_vel_kal(rp(2)+1)],'b--')
%                         plot([rp(3) rp(3)]/10,[0 robot_trans_vel_kal(rp(3)+1)],'b--')
%     plot(time, robot_trans_vel, 'm*-');%brzina stvarna (s kalibr. odom.)
%      plot(robot_trans_vel_ref, 'k*-');%postavna brzina s ukljucenom kalibr. odom.
% plot(robot_trans_vely_ref,'b*-');
%     plot(time(1:rps), robot_trans_vel_refdin(1:rps), 'g*-');%dinamikom odredjena postavna brzina iz zeljene
%      plot(time_ref, robot_trans_vel_des, 'b*-');%zeljena brzina
%     plot(time(2:rps),robot_trans_vel_des(1:rps-1),'b*-');
%     plot(time, predikcija_trans2, 'y');
%      plot(time(1:rps), predikcija_trans_kp1(1:rps), 'c');
%       plot(time(1:rps), predikcija_trans_kp2(1:rps), 'y');
%    plot(time(1:rps), duljina(1:rps)*0.1, 'r');
%     plot(time(1:rps),duljina(1:rps)*0.1,'r*');
%    legend('ocit: enkoderi','stvarna (s kalibr.odom.)','postav: dinamikom iz zeljene i kalibr. odom.','postav:samo dinamikom iz zeljene','zeljena u k+1-vom ciklusu',predikcija);%,'predikcija po pt1 od predikcije: vPred2(k)=f(vPred(k-1),vR(k-2) (treba se slagati sa zeljenom');
%    legend('ocit: enkoderi','stvarna (s kalibr.odom.)','postav: dinamikom iz zeljene i kalibr. odom.','postav:samo dinamikom iz zeljene','zeljena u k+1-vom ciklusu');

% ylabel('v [mm/sec]', 'fontsize',10, 'fontname', 'times');grid on;
% legend('vx','vy','v','vxref','vyref')
  ylabel('v', 'fontsize',16, 'fontname', 'times');grid on;
    h=gca;
    set(h,'fontsize',16,'fontname','times','box', 'on');
%                         set(h,'ylim',[-602 602]);%,'fontsize',10,'fontname','times','box', 'on'); 
    %rotational velocity
      subplot(2,1,2);
      hold on
%     plot(time/10, 
    plot(robot_rot_vel_kal, 'r.-','Linewidth',1,'Color',boja);grid on;
%     plot(time, robot_rot_vel, 'm*-');
%     plot(robot_rot_vel_ref, 'k*-');%postavna brzina s ukljucenom kalibr. odom.
%     plot(time(1:rps), robot_rot_vel_refdin(1:rps), 'g*-');%dinamikom odredjena postavna brzina iz zeljene
%      plot(time_ref, robot_rot_vel_des, 'b');%zeljena brzina
%     plot(time(2:rps),robot_rot_vel_des(1:rps-1),'b*-');
%     plot(time, predikcija_rot2, 'y');
%       plot(time, predikcija_rot_kp1, 'c');
%        plot(time, predikcija_rot_kp2, 'y');
% legend('w','wref')
%    ylabel('w\omega [deg/sec]', 'fontsize',10,'fontname','times');
%     xlabel('time intervals', 'fontsize',10, 'fontname', 'times');

ylabel('w', 'fontsize',16,'fontname','times');
    xlabel('vt', 'fontsize',16, 'fontname', 'times');
    h=gca;
    set(h,'fontsize',16,'fontname','times','box', 'on');
% figure(20)
% %pogreske predikcija
% subplot(2,1,1)
% hold on
% grid on
% plot(time,(robot_trans_vel-predikcija_trans_kp1),'r');
% plot(time,(robot_trans_vel-predikcija_trans_kp2),'b');
% legend('odstupanje vel-predikcija po pt1','odstupanje vel-zeljena u k+2-gom koraku');
% plot(time,(robot_trans_vel-predikcija_trans_kp1),'r*');
% plot(time,(robot_trans_vel-predikcija_trans_kp2),'b*');
% subplot(2,1,2)
% hold on
% grid on
% plot(time,(robot_rot_vel-predikcija_rot_kp1),'r');
% plot(time,(robot_rot_vel-predikcija_rot_kp1),'r*');
% plot(time,(robot_rot_vel-predikcija_rot_kp2),'b');
% plot(time,(robot_rot_vel-predikcija_rot_kp2),'b*');
% figure(2)
% plot(time,WH_globalna_putanja_th)

%diff 2 kad hocu provjeriti ogranicenja po akceleraciji
if 0
figure
plot(diff(robot_trans_vel_kal,1),'r.-')
hold on
plot(diff(robot_trans_vel_ref,1),'k*-')
legend('dvx','dvxref')
title('delta vx')

figure
plot(diff(robot_trans_vely,1),'m.-')
hold on
plot(diff(robot_trans_vely_ref,1),'b*-')
legend('dvy','dvyref')
title('delta vy')

figure
plot(diff(robot_rot_vel_ref,1),'r.-')
hold on
plot(diff(robot_rot_vel_kal,1),'k*-')
legend('dvw','dvwref')
title('delta w')
end