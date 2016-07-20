
dsori=1;

vrijeme=load('wh_dstar_izracuni.dat');
replan=load('replan_putanja.dat');
in=find(replan==0);
in2=find(replan==2);

exploredD=1;
if dsori==1
vrijeme=load('vremena_ds_algoritma.dat')*1000;
exploredD=load('explored_ds_algoritma.dat');
end
izracuniWitkowski=load('wh_wit_izracuniWitkowski.dat');
izracuniWitkowski=izracuniWitkowski(2:end);
vrijemeD=vrijeme(1:end);
vremena_skupine=load('wit_put_i_skupine_vremena.dat');
vremena_skupine=vremena_skupine(2:end);
time=load('wh_log_time.dat');
logger_sizes=load('logger_sizes.dat');
time=time(1:logger_sizes);
%figure(6);
%plot(time)
figure;
hold on
%     inic_x=load('wh_dstar_inicijalni_x.dat');
%     if numel(inic_x)==0 %radi se o witkowskijevom algoritmu
%         plot(izracuniWitkowski,'r*-')
%     else
        plot(vrijemeD,'b*-');
%     end
%bar(vrijeme,'stacked');
%figure(2);

vrijeme=load('vremena_cijelog_algoritma.dat');
% hold on;
plot(vrijeme(1:(logger_sizes)),'m*-');
xlabel('number of steps','fontsize',16);
ylabel('time planning [ms]','fontsize',16);
vrijeme=load('vremena_dw_algoritma.dat');
plot(vrijeme,'g*-');
% plot(vremena_skupine,'k*-');
time=load('wh_log_read_time.dat');%vrijeme citanja i zadavanja reference
stepovi=[diff(time)];
plot(stepovi,'k.-')
% if numel(inic_x)==0
%     legend('Witkowski search time','D* search time','total planning time','dw search time')
%     legend('Witkowski search time','D* search time','convex set time')
% else
%     legend('D* search time','total planning time','dw search time','step duration');
% end
grid on
legend('D*','everything','dw','steps')

figure
plot(vrijemeD,'b*-');
hold on
plot(in,vrijemeD(in),'ro');
plot(in2,vrijemeD(in2),'gs');
title('D*')

figure
plot(exploredD,'b*-');
hold on
plot(in,exploredD(in),'ro');
plot(in2,exploredD(in2),'gs');
title('explored nodes')
% legend('D* reverse','D*','motion planning','DW','convex sets and W path')
% legend('reverse D*','D*','convex sets and W path')
% figure
% plot(vremena_skupine-izracuniWitkowski)