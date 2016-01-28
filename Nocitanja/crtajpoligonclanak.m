% figure(1)
metric=0.001; 
%  footprint=[0.17, 0.27;-0.15, 0.27;-0.48, 0.17;-0.48,-0.17;-0.15, -0.27;0.17, -0.27;0.17, 0.27];
  footprint=[0.26/2, 0.52/2;-0.15, 0.52/2;-0.48, 0.17;-0.48,-0.17;-0.15, -0.52/2;0.26/2, -0.52/2; 0.26/2 -0.102/2; (0.12+0.105/2) -0.102/2; (0.12+0.105/2) 0.102/2; 0.26/2 0.102/2; 0.26/2, 0.52/2]; 
 udaljenosti=sqrt(footprint(:,1).^2+footprint(:,2).^2);
 x_circle=[];
 y_circle=[];
brojac=0;
while(1)
%figure
hold on

if(0)
boja='b'
fi=strcat('poligon',num2str(brojac))
rezultat=load(fi);
brojac_duzina=0;
for i = 1:2:length(rezultat)
    brojac_duzina=brojac_duzina+1;
tmp = [rezultat(i,:);rezultat(i+1,:)];
h = plot(tmp(:,1),tmp(:,2),boja);
hold on
% if (brojac_duzina==19+1)
%     plot(tmp(:,1),tmp(:,2),'k');
% end
% text(tmp(:,1),tmp(:,2),mat2str(brojac_duzina))
% pause
set(h,'LineWidth',1)
end

fi=strcat('globskocni',num2str(brojac))
rezultat=load(fi);
boja='r'
brojac_duzina=0;
for i = 1:2:length(rezultat)
    brojac_duzina=brojac_duzina+1;
tmp = [rezultat(i,:);rezultat(i+1,:)];
h = plot(tmp(:,1),tmp(:,2),boja);
% if (brojac_duzina==19+1)
%     plot(tmp(:,1),tmp(:,2),'k');
% end
% text(tmp(:,1),tmp(:,2),mat2str(brojac_duzina))
% pause
set(h,'LineWidth',1)
end

end

if(0)
fi=strcat('najboljiskocni',num2str(brojac))
rezultat=load(fi)*metric;
boja='r'
brojac_duzina=0;
for i = 1:2:length(rezultat)
    brojac_duzina=brojac_duzina+1;
tmp = [rezultat(i,:);rezultat(i+1,:)];
h = plot(tmp(:,1),tmp(:,2),boja);
% if (brojac_duzina==19+1)
%     plot(tmp(:,1),tmp(:,2),'k');
% end
% text(tmp(:,1),tmp(:,2),mat2str(brojac_duzina))
% pause
set(h,'LineWidth',2)
end
end

% filename=strcat('podaci',mat2str(brojac));
% fid=fopen(filename);
%         if (fid~=-1)
%             fclose(fid);
%             laserr=load(filename);
% %plot(laserr(:,1)*metric,laserr(:,2)*metric,'g.');
%         end
%         
%         filename=strcat('podacisick',mat2str(brojac));
% fid=fopen(filename);
%         if (fid~=-1)
%             fclose(fid);
%             laser=load(filename);
% plot(laser(:,1)*metric,laser(:,2)*metric,'k.');
    %    end

   filename=strcat('podacitraj',mat2str(brojac));
fid=fopen(filename);
        if (fid~=-1)
            fclose(fid);
            traj=load(filename);
            if (isempty(traj)~=1)
plot(traj(:,1)*metric,traj(:,2)*metric,'Color',[0 0 0]);
            end
        end

        filename=strcat('pozicija',mat2str(brojac));
fid=fopen(filename);
        if (fid~=-1)
            fclose(fid);
            poz=load(filename)
%plot(poz(1)*metric,poz(2)*metric,'b*');
    %nacrtajmo robota polumjer robota (mm)
% 	rr=260*metric;
    rr=290*metric;
	fi=linspace(0,2*pi,30);
	x_temp=poz(1)*metric;y_temp=poz(2)*metric;th_temp=poz(3);
	if (0)
    for i=1:30
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
    end
	plot(x_circle,y_circle,'k--');
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');
    end
    plot((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2)),(y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2)),'Color',[1 0 0]);
     pomak=-298*metric;
    x_temp=x_temp+pomak*cos(th_temp);
    y_temp=y_temp+pomak*sin(th_temp);
        rr=udaljenosti(3)+pomak;%230*metric;
        if (0)
    for i=1:30
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
    end
	plot(x_circle,y_circle,'k--');
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');
        end
        end
        
        %provjera translacije lasera
        %plot((laser(:,1)-255*cos(th_temp))*metric,(laser(:,2)-255*sin(th_temp))*metric,'b.');

    if (0)    
         filename=strcat('globskocnigoal',mat2str(brojac));
fid=fopen(filename);
        if (fid~=-1)
            fclose(fid);
            pozg=load(filename)
            if length(pozg)>0
plot(pozg(:,1)*metric,pozg(:,2)*metric,'b*');
            end
        end
 
        
            filename=strcat('goal',mat2str(brojac));
fid=fopen(filename);
        if (fid~=-1)
            fclose(fid);
            poz=load(filename)
plot(poz(1)*metric,poz(2)*metric,'ro');
        end
        
    end
        xlabel('x [m]')
        ylabel('y [m]')
	axis equal tight;

    if (0)
    if (brojac==0)
    xlim1=xlim;
    ylim1=ylim;
    else
       xlim2=xlim;
    ylim2=ylim;
    if norm(xlim2-xlim1)>1 || norm(ylim2-ylim1)>1
        xlim1=xlim2;
        ylim1=ylim2;
    end
    end
    axis([xlim1 ylim1])
% axis([-10 15 -15 20]);
    end

        pause
        
%         print(gcf,'-dpng',mat2str(brojac+100));
        
brojac=brojac+1
% break
% hold off
end