metric=0.001;
read_time=load('wh_log_read_time.dat');%vrijeme citanja i zadavanja reference
 robot_trans_vel_ref=load('robot_trans_vel_ref.dat')/metric;
 robot_rot_vel_ref=load('robot_rot_vel_ref.dat')*180/pi;
 robot_trans_vel_kal=load('robot_trans_vel_kal.dat');
 robot_rot_vel_kal=load('robot_rot_vel_kal.dat')*180/pi;
 logger_sizes=load('logger_sizes.dat');
rps=logger_sizes(1);
 time=read_time/1000;
 predikcija_rot_kp1=robot_rot_vel_kal;
 predikcija_trans_kp1=robot_rot_vel_kal;
predikcija_rot_kp2=robot_rot_vel_kal;
predikcija_trans_kp2=robot_rot_vel_kal;
pt2_trans=robot_rot_vel_kal;
pt2_rot=pt2_trans;
 Tv=0.11;%vremenska konstanta za pt1 translacijska
 Tw=0.09;%rotacijska
 
 s = tf('s');
% hc=1/(1+0.05*s+1/20/20*s^2)
hc=1/(1+0.06*s+1/12/12*s^2)%plava,dobro imitira porast
 hd=c2d(hc,0.1,'zoh')
   a=hd.num{1}
 b=hd.den{1}
 b1=b(2);
 b0=b(3);
 a1=a(2);
 a0=a(3);


 for i=1:rps,
 predikcija_rot_kp1(i)=0;
 predikcija_trans_kp1(i)=0;
predikcija_rot_kp2(i)=0;
predikcija_trans_kp2(i)=0;
pt2_trans(i)=0;

if(i>2)
     predikcija_rot_kp1(i)=robot_rot_vel_kal(i-1)*exp(-0.1/Tw)+(1-exp(-0.1/Tw))*robot_rot_vel_ref(i-1);
     predikcija_trans_kp1(i)=robot_trans_vel_kal(i-1)*exp(-0.1/Tv)+(1-exp(-0.1/Tv))*robot_trans_vel_ref(i-1);
%model pt1
     predikcija_rot_kp2(i)=predikcija_rot_kp2(i-1)*exp(-0.1/Tw)+(1-exp(-0.1/Tw))*robot_rot_vel_ref(i-1);
     predikcija_trans_kp2(i)=predikcija_trans_kp2(i-1)*exp(-0.1/Tv)+(1-exp(-0.1/Tv))*robot_trans_vel_ref(i-1);
     %model pt2osc
          pt2_trans(i)=-b1*pt2_trans(i-1)-b0*pt2_trans(i-2)+a1*robot_trans_vel_ref(i-1)+a0*robot_trans_vel_ref(i-2);
          pt2_rot(i)=-b1*pt2_rot(i-1)-b0*pt2_rot(i-2)+a1*robot_rot_vel_ref(i-1)+a0*robot_rot_vel_ref(i-2);
end
end
figure(1);
 subplot(2,1,1);
 hold on
     grid on;
   plot(time, robot_trans_vel_kal, 'r');%brzina iz enkodera
    plot(time(1:rps), robot_trans_vel_ref(1:rps), 'k');%postavna brzina s ukljucenom kalibr. odom.
%      plot(time, predikcija_trans_kp1, 'c');
     plot(time, predikcija_trans_kp2, 'b');
      plot(time, pt2_trans, 'g');
  legend('stvarna','referenca','vPred(k)=f(vPred(k-1),referenca(k-1)','pt2osc');
     plot(time, robot_trans_vel_kal, 'r*');
    plot(time(1:rps), robot_trans_vel_ref(1:rps), 'k*');%postavna brzina s ukljucenom kalibr. odom.
%     plot(time, predikcija_trans_kp1, 'c*');
     plot(time, predikcija_trans_kp2, 'b*');
      plot(time, pt2_trans, 'g*');
  ylabel('v [mm/sec]', 'fontsize',10, 'fontname', 'times');grid on;
    %rotational velocity
      subplot(2,1,2);
      hold on
    plot(time, robot_rot_vel_kal, 'r');grid on;
    plot(time(1:rps), robot_rot_vel_ref(1:rps), 'k');%postavna brzina s ukljucenom kalibr. odom.
%      plot(time, predikcija_rot_kp1, 'c');
      plot(time, predikcija_rot_kp2, 'b');
        plot(time, pt2_rot, 'g');
plot(time, robot_rot_vel_kal, 'r*');
     plot(time(1:rps), robot_rot_vel_ref(1:rps), 'k*');%postavna brzina s ukljucenom kalibr. odom.
%     plot(time, predikcija_rot_kp1, 'c*');
      plot(time, predikcija_rot_kp2, 'b*');
            plot(time, pt2_rot, 'g*');
  ylabel('\omega [deg/sec]', 'fontsize',10,'fontname','times');
    xlabel('t [sec]', 'fontsize',10, 'fontname', 'times');
