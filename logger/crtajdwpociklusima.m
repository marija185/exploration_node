metric=0.001;
%  footprint=[0.17, 0.27;-0.15, 0.27;-0.48, 0.17;-0.48,-0.17;-0.15, -0.27;0.17, -0.27;0.17, 0.27];
  footprint=[0.26/2, 0.52/2;-0.15, 0.52/2;-0.48, 0.17;-0.48,-0.17;-0.15, -0.52/2;0.26/2, -0.52/2; 0.26/2 -0.102/2; (0.12+0.105/2) -0.102/2; (0.12+0.105/2) 0.102/2; 0.26/2 0.102/2; 0.26/2, 0.52/2];

 x_circle=[];
 y_circle=[];
    	figure;
zastavica=0;
 brojac=0;
 log_sizes=load('logger_sizes.dat')+1000
while(brojac<log_sizes)
% figure

boja='b'
fi=strcat('dwlog_TB_flag',num2str(brojac))
fid=fopen(fi)

while fid<0 && (brojac<log_sizes)
    brojac=brojac+1
fi=strcat('dwlog_TB_flag',num2str(brojac))
    fid=fopen(fi)
end
if (fid>0)
    fclose(fid);    
end
if (brojac==log_sizes)
    continue;
end

fi=strcat('dwlog_TB_flag',num2str(brojac));
	TB_flag=load(fi);
fi=strcat('dwlog',num2str(brojac));
	LOG=load(fi); %[indeks v, indeks w, dimenzija v, dimenzija w, trenutna brzina v, trenutna brzina w]
fi=strcat('dwlog_putanje_x',num2str(brojac));
	M_putanje_x=load(fi); %cijele krivulje (dimenzija v)x(dimenzija w)x30
fi=strcat('dwlog_putanje_y',num2str(brojac));
	M_putanje_y=load(fi);
fi=strcat('dwlog_obstacle_point_x',num2str(brojac));
    M_obstacle_point_x=load(fi);
fi=strcat('dwlog_obstacle_point_y',num2str(brojac));
    M_obstacle_point_y=load(fi);
fi=strcat('dwlog_obstacle_point2_x',num2str(brojac));
    M_obstacle_point_x2=load(fi);
fi=strcat('dwlog_obstacle_point2_y',num2str(brojac));
    M_obstacle_point_y2=load(fi);
fi=strcat('dwlog_back_collision',num2str(brojac));
	back_collision=load(fi);
fi=strcat('dwlog_robot_position',num2str(brojac));
	M_robot_position=load(fi); %pozicija robota [x y th]
fi=strcat('dwlog_laser_obstacles',num2str(brojac));
	M_laser_obstacles=load(fi); %prepreke [x1 y1;x2 y2;x3 y3;...]
fi=strcat('dwlog_pokretne_prepreke_point_x',num2str(brojac));
	pokretne_prepreke_x=load(fi);
fi=strcat('dwlog_pokretne_prepreke_point_y',num2str(brojac));
  	pokretne_prepreke_y=load(fi);
fi=strcat('dwlog_dstar_path_putanje_x',num2str(brojac));
	dstar_path_putanje_x=load(fi);
fi=strcat('dwlog_dstar_path_putanje_y',num2str(brojac));
	dstar_path_putanje_y=load(fi);
fi=strcat('dwlog_lin_brzine',num2str(brojac));
    TB_v=load(fi);
fi=strcat('dwlog_rot_brzine',num2str(brojac));
    TB_w=load(fi);

	%efektivni path (10 tocaka)
fi=strcat('dwlog_path_putanje_x',num2str(brojac));
	M_path_putanje_x=load(fi);
fi=strcat('dwlog_path_putanje_y',num2str(brojac));
	M_path_putanje_y=load(fi);
fi=strcat('dwlog_tocka_infleksije',num2str(brojac));
    M_tocka_infleksije=load(fi); % [x y]
%gridmap punjenja
fi=strcat('dwlog_punjenje_x',num2str(brojac));
	punjenje_x=load(fi);
fi=strcat('dwlog_punjenje_y',num2str(brojac));
	punjenje_y=load(fi);
fi=strcat('dwlog_praznjenje_x',num2str(brojac));
	praznjenje_x=load(fi);
fi=strcat('dwlog_praznjenje_y',num2str(brojac));
	praznjenje_y=load(fi);
fi=strcat('dwlog_zanepunjenje_x',num2str(brojac));
	zanepunjenje_x=load(fi);
fi=strcat('dwlog_zanepunjenje_y',num2str(brojac));
	zanepunjenje_y=load(fi);


    
    
    	%polumjer robota (mm)
% 	rr=260;
    RR=290;
    rr=RR*metric;
	fi=linspace(0,2*pi,30);
	x_temp=M_robot_position(1)*metric;y_temp=M_robot_position(2)*metric;th_temp=M_robot_position(3);
	if (1)
    for i=1:30
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
    end
	plot(x_circle,y_circle,'k--');
    hold on;
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');
    end
    plot((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2)),(y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2)),'k'); 
    DELTARR2=-298;%-300;
    pomak=DELTARR2*metric;
    x_temp=x_temp+pomak*cos(th_temp);
    y_temp=y_temp+pomak*sin(th_temp);
        RR2=211.2;%220;
        rr=RR2*metric;
    for i=1:30
		x_circle(i)=x_temp+rr*cos(fi(i));
		y_circle(i)=y_temp+rr*sin(fi(i));
    end
	plot(x_circle,y_circle,'k--');
	plot([x_temp x_temp+rr*cos(th_temp)], [y_temp y_temp+rr*sin(th_temp)],'k--');

    
    
    %nacrtat cu mjesto riegla
    pomak=-135;
    xrig=M_robot_position(1)+pomak*cos(M_robot_position(3));
    yrig=M_robot_position(2)+pomak*sin(M_robot_position(3));
%     plot(xrig*metric,yrig*metric,'bo')
    
    

    hstat=plot(M_laser_obstacles(:,1)*metric,M_laser_obstacles(:,2)*metric,'k.');
%     hstat=plot(M_laser_obstacles(1,1)*metric,M_laser_obstacles(1,2)*metric,'k.');
    
    hunkn=plot(pokretne_prepreke_x*metric,pokretne_prepreke_y*metric,'bo');
% 	plot(x_circle*metric, y_circle*metric,'r','Linewidth',2);
	plot([M_robot_position(1) M_robot_position(1)+rr*cos(M_robot_position(3))]*metric, [M_robot_position(2) M_robot_position(2)+rr*sin(M_robot_position(3))]*metric,'r','Linewidth',2);
    %crtanje efektivne putanje
	heff=plot(M_path_putanje_x*metric,M_path_putanje_y*metric,'m.-');
	plot(M_tocka_infleksije(1)*metric,M_tocka_infleksije(2)*metric,'bd');
    plot(punjenje_x*metric,punjenje_y*metric,'rs');
    plot(praznjenje_x*metric,praznjenje_y*metric,'yv');
    plot(zanepunjenje_x*metric,zanepunjenje_y*metric,'ch');
 
    %parametri iz Params.h i DynamicWindow.h
    SC1=10;
    SC2=500;
    SC_W=0;
    V_MAX=500;
    W_MAX=100;
    
    %inicijalni flagovi za labele
    flagclear=0;
    flagobst=0;
    flagnonad=0;
    hclear=-1; hobst=-1; hnonad=-1; hopt=-1;
    %crtanje globalnog puta
    hpath=plot(dstar_path_putanje_x(1:end)*metric,dstar_path_putanje_y(1:end)*metric,'Color',[0.2 0.5 1]);
    TB_v
    TB_w
    disp('stupci su w, a redci su v')
    TB_flag
	for i=1:LOG(3)
	     for j=1:LOG(4)
               limit_distance=((RR+SC1+(SC2-SC1)*TB_v(i)/V_MAX)+SC_W*TB_w(j)/W_MAX)*metric; 
                limit_distancec2=((RR2+SC1+(SC2-SC1)*TB_v(i)/V_MAX)+SC_W*TB_w(j)/W_MAX)*metric; %security distance za drugu kruznicu

		index=(i-1)*LOG(4)+j;%po redu par brzina, v fiksiran, w sece pa do 25. para

if i==2 && j==5 && 0 || 0
KORAK=0.1;
v=TB_v(i);
w=TB_w(j);
klx=ones(1,61);
kly=klx;
klth=klx;
klx(1)=M_robot_position(1);
kly(1)=M_robot_position(2);
klth(1)=M_robot_position(3);
for ii=2:61
			klth(ii)=klth(ii-1)+w*KORAK;
			klx(ii)=klx(ii-1)+v/w*(sin(klth(ii))-sin(klth(ii-1)));
			kly(ii)=kly(ii-1)-v/w*(cos(klth(ii))-cos(klth(ii-1)));
end
plot(klx*metric,kly*metric,'bo')
centre2_x=klx+(DELTARR2-limit_distancec2/metric)*cos(klth);
centre2_y=kly+(DELTARR2-limit_distancec2/metric)*sin(klth);
plot(centre2_x*metric,centre2_y*metric,'rd');
if (0)
klx2=[-5976.8244846288835, -5974.8277005816635, -5972.762149484005, -5970.628814426359, -5968.4287107606697, -5966.1628856171219,-5963.8324174057652, -5961.4384153032479, -5958.9820187249088, -5956.4643967824795, -5953.8867477276472, -5951.250298381754,-5948.5563035518971, -5945.8060454337055, -5943.0008330010869, -5940.1420013832267, -5937.2309112291387, -5934.2689480600675, -5931.2575216100604, -5928.1980651550048, -5925.0920348304717, -5921.9409089386691, -5918.7461872448539, -5915.5093902635235,  -5912.2320585347334, -5908.9157518908823, -5905.5620487143196, -5902.1725451861175, -5898.7488545263768, -5895.2926062264205,  -5891.8054452732467, -5888.2890313666012, -5884.7450381290519, -5881.1751523094326, -5877.5810729800469, -5873.9645107279957,-5870.3271868410347, -5866.6708324883339, -5862.9971878965334, -5859.3080015214919, -5855.6050292161135, -5851.8900333946585, -5848.1647821939314, -5844.4310486317409, -5840.690609763039, -5836.945245834143, -5833.1967394354297, -5829.4468746529228, -5825.6974362191631, -5821.9502086637722, -5818.2069754641125, -5814.4695181964498, -5810.7396156880204, -5807.0190431704032, -5803.3095714346064, -5799.612965988269, -5795.9309862153714, -5792.2653845388659, -5788.6179055866178, -5784.9902853610583, -5781.3842504129398];
kly2=[-3089.7360527885867, -3092.9101346568541, -3096.0399015652129, -3099.1238639141261, -3102.1605539045458, -3105.1485262365068, -3108.0863587970098, -3110.9726533368716, -3113.8060361362159, -3116.5851586582889, -3119.3086981912897, -3121.97535847791, -3124.583870332282, -3127.1329922440423, -3129.6215109692239, -3132.048242107694, -3134.4120306668647, -3136.7117516114063, -3138.9463103987046, -3141.1146434998027, -3143.2157189055829, -3145.2485366179485, -3147.2121291257677, -3149.1055618653568, -3150.9279336652817, -3152.6783771752666, -3154.3560592790068, -3155.9601814906855, -3157.4899803350108, -3158.9447277105878, -3160.3237312364563, -3161.626334581626, -3162.8519177774551, -3163.999897512721, -3165.0697274112449, -3166.0608982919384, -3166.9729384111452, -3167.8054136871651, -3168.5579279068543, -3169.2301229142013, -3169.8216787807892, -3170.3323139580648, -3170.76178541134, -3171.1098887354638, -3171.3764582521076, -3171.5613670886187, -3171.6645272384062, -3171.6858896028257, -3171.625444014549, -3171.4832192424024, -3171.259282977674, -3170.9537418018976, -3170.5667411361242, -3170.0984651717108, -3169.5491367826539, -3168.9190174195146, -3168.208406984982, -3167.4176436911357, -3166.5471038984747, -3165.5972019367914, -3164.5683899079722];
klth2=[5.2630159063157409, 5.284832521940741, 5.3066491375657412, 5.3284657531907413, 5.3502823688157415, 5.3720989844407416, 5.3939156000657418, 5.4157322156907419, 5.4375488313157421, 5.4593654469407422, 5.4811820625657424, 5.5029986781907425, 5.5248152938157427, 5.5466319094407428, 5.5684485250657429, 5.5902651406907431, 5.6120817563157432, 5.6338983719407434, 5.6557149875657435, 5.6775316031907437, 5.6993482188157438, 5.721164834440744, 5.7429814500657441, 5.7647980656907443, 5.7866146813157444, 5.8084312969407446, 5.8302479125657447, 5.8520645281907449, 5.873881143815745, 5.8956977594407451, 5.9175143750657453, 5.9393309906907454, 5.9611476063157456, 5.9829642219407457, 6.0047808375657459, 6.026597453190746, 6.0484140688157462, 6.0702306844407463, 6.0920473000657465, 6.1138639156907466, 6.1356805313157468, 6.1574971469407469, 6.179313762565747, 6.2011303781907472, 6.2229469938157473, 6.2447636094407475, 6.2665802250657476, 6.2883968406907478, 6.3102134563157479, 6.3320300719407481, 6.3538466875657482, 6.3756633031907484, 6.3974799188157485, 6.4192965344407487, 6.4411131500657488, 6.462929765690749, 6.4847463813157491, 6.5065629969407492, 6.5283796125657494, 6.5501962281907495, 6.5720128438157497];
plot(klx2*metric,kly2*metric,'mh')
centre2_x=klx+DELTARR2*cos(klth);
centre2_y=kly+DELTARR2*sin(klth);
plot(centre2_x*metric,centre2_y*metric,'go');
centre2_x=-6074.3594466804043;
centre2_y=-3244.7520405591936;
plot(centre2_x*metric,centre2_y*metric,'ks');
end
end
		if (TB_flag(i,j)==2) %clear
		hclear=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'g');
        if (flagclear==0)
            hclear1=hclear;
            flagclear=1;
        end
		elseif (TB_flag(i,j)==1) %has obstacle
		hobst=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'b');
        if (flagobst==0)
            hobst1=hobst;
            flagobst=1;
        end
%         disp('v(i); w(j)')
%         TB_v(i)
%         TB_w(j)
        if (back_collision(index)==1)
            if (1)
                x_temp=M_obstacle_point_x2(index)*metric;y_temp=M_obstacle_point_y2(index)*metric;
                for ii=1:30
                    x_circle(ii)=x_temp+limit_distancec2*cos(fi(ii));
                    y_circle(ii)=y_temp+limit_distancec2*sin(fi(ii));
                end
                plot(x_circle,y_circle,'b-');
            end
if 0
KORAK=0.1;
v=TB_v(i);
w=TB_w(j);
klx=ones(1,61);
kly=klx;
klth=klx;
klx(1)=M_robot_position(1);
kly(1)=M_robot_position(2);
klth(1)=M_robot_position(3);
for ii=2:61
			klth(ii)=klth(ii-1)+w*KORAK;
			klx(ii)=klx(ii-1)+v/w*(sin(klth(ii))-sin(klth(ii-1)));
			kly(ii)=kly(ii-1)-v/w*(cos(klth(ii))-cos(klth(ii-1)));
end
plot(klx*metric,kly*metric,'bo')
centre2_x=klx+DELTARR2*cos(klth);
centre2_y=kly+DELTARR2*sin(klth);
plot(centre2_x*metric,centre2_y*metric,'rd');
end

            plot([M_obstacle_point_x(index) M_obstacle_point_x2(index)]*metric,[M_obstacle_point_y(index) M_obstacle_point_y2(index)]*metric,'b--.');
        else
            if (0)
                x_temp=M_obstacle_point_x(index)*metric;y_temp=M_obstacle_point_y(index)*metric;
                for ii=1:30
                    x_circle(ii)=x_temp+limit_distance*cos(fi(ii));
                    y_circle(ii)=y_temp+limit_distance*sin(fi(ii));
                end
                plot(x_circle,y_circle,'b-');
            end
            plot(M_obstacle_point_x(index)*metric,M_obstacle_point_y(index)*metric,'xb');
        end
        elseif (TB_flag(i,j)==-2) %non admissible
		hnonad=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'k');
        if (flagnonad==0)
            hnonad1=hnonad;
            flagnonad=1;
        end
        if (back_collision(index)==1)
            if (1)
                x_temp=M_obstacle_point_x2(index)*metric;y_temp=M_obstacle_point_y2(index)*metric;
                for ii=1:30
                    x_circle(ii)=x_temp+limit_distancec2*cos(fi(ii));
                    y_circle(ii)=y_temp+limit_distancec2*sin(fi(ii));
                end
                plot(x_circle,y_circle,'k-');
            end
            plot([M_obstacle_point_x(index) M_obstacle_point_x2(index)]*metric,[M_obstacle_point_y(index) M_obstacle_point_y2(index)]*metric,'k--.');
        else
            if (0)
                x_temp=M_obstacle_point_x(index)*metric;y_temp=M_obstacle_point_y(index)*metric;
                for ii=1:30
                    x_circle(ii)=x_temp+limit_distance*cos(fi(ii));
                    y_circle(ii)=y_temp+limit_distance*sin(fi(ii));
                end
                plot(x_circle,y_circle,'k-');
            end
            plot(M_obstacle_point_x(index)*metric,M_obstacle_point_y(index)*metric,'sk');
        end
		elseif (TB_flag(i,j)==-3) %kinematic constraint
		plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m');
		elseif (TB_flag(i,j)==-4) %circular
		plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'m');
		end
		
	     end
	end
	%optimalnu putanju crtamo posebno
	if((LOG(1)>=1) && (LOG(2)>=1))
	index=(LOG(1)-1)*LOG(4)+LOG(2);
 	hopt=plot(M_putanje_x(index,:)*metric,M_putanje_y(index,:)*metric,'r-');
    end
    
    disp('optimalni par indeksi (v,w)')
    LOG(1)
    LOG(2)
    disp('trenutna brzina (v,w)')
    LOG(5)
    LOG(6)
    disp('referenca (v,w)')
    LOG(7)
    LOG(8)

	axis equal tight;

    if (zastavica==0)
    xlim1=xlim;
    ylim1=ylim;
    zastavica=1;
    else
       xlim2=xlim;
    ylim2=ylim;
    if norm(xlim2-xlim1)>1 || norm(ylim2-ylim1)>1
        xlim1=xlim2;
        ylim1=ylim2;
    end
    end
    axis([xlim1 ylim1])
    
 if (hopt~=-1) %nece crtati legendu ako nema optimalnog para
    
    if 1
    if (hclear~=-1 && hobst~=-1 && hnonad~=-1)
        if isempty(hunkn)
    hleg=legend([hstat hpath heff hclear hobst hnonad hopt],'static known obstacles','D* path','effective path','clear trajectory','obstacle trajectory','nonadmissible trajectory','optimal trajectory');
        else
    hleg=legend([hstat hunkn hpath heff hclear hobst hnonad hopt],'static known obstacles','unknown obstacles', 'D* path','effective path','clear trajectory','obstacle trajectory','nonadmissible trajectory','optimal trajectory');
        end
    else
        if hclear~=-1 && hobst~=-1
            if isempty(hunkn)
            hleg=legend([hstat hpath heff hclear hobst hopt],'static known obstacles', 'D* path','effective path','clear trajectory','obstacle trajectory','optimal trajectory');    
            else
            hleg=legend([hstat hunkn hpath heff hclear hobst hopt],'static known obstacles','unknown obstacles', 'D* path','effective path','clear trajectory','obstacle trajectory','optimal trajectory');    
            end
        end
        if hobst~=-1 && hnonad~=-1
            if isempty(hunkn)
            hleg=legend([hstat hpath heff hobst hnonad hopt],'static known obstacles', 'D* path','effective path','obstacle trajectory','nonadmissible trajectory','optimal trajectory');
            else
            hleg=legend([hstat hunkn hpath heff hobst hnonad hopt],'static known obstacles','unknown obstacles', 'D* path','effective path','obstacle trajectory','nonadmissible trajectory','optimal trajectory');
            end
        end    
        if hclear~=-1 && hnonad~=-1
            if isempty(hunkn)
            hleg=legend([hstat hpath heff hclear hnonad hopt],'static known obstacles', 'D* path','effective path','clear trajectory','nonadmissible trajectory','optimal trajectory');
            else
            hleg=legend([hstat hunkn hpath heff hclear hnonad hopt],'static known obstacles','unknown obstacles', 'D* path','effective path','clear trajectory','nonadmissible trajectory','optimal trajectory');
            end
        end
        if hclear~=-1 && hobst==-1 && hnonad==-1 %samo clear
            if isempty(hunkn)
            hleg=legend([hstat hpath heff hclear hopt],'static known obstacles', 'D* path','effective path','clear trajectory','optimal trajectory');    
            else
            hleg=legend([hstat hunkn hpath heff hclear hopt],'static known obstacles','unknown obstacles', 'D* path','effective path','clear trajectory','optimal trajectory');    
            end
        end
        if hobst~=-1 && hnonad==-1 && hclear==-1 %samo obstacle
            if isempty(hunkn)
            hleg=legend([hstat hpath heff hobst hopt],'static known obstacles', 'D* path','effective path','obstacle trajectory','optimal trajectory');
            else
            hleg=legend([hstat hunkn hpath heff hobst hopt],'static known obstacles','unknown obstacles', 'D* path','effective path','obstacle trajectory','optimal trajectory');
            end
        end
        if hclear==-1 && hnonad~=-1 && hobst==-1 %samo nonadmissible
            if isempty(hunkn)
            hleg=legend([hstat hpath heff hnonad hopt],'static known obstacles', 'D* path','effective path','nonadmissible trajectory','optimal trajectory');
            else
            hleg=legend([hstat hunkn hpath heff hnonad hopt],'static known obstacles','unknown obstacles', 'D* path','effective path','nonadmissible trajectory','optimal trajectory');
            end
        end
    end
    end
    
    set(hleg,'FontAngle','italic','TextColor',[.3 .2 .1], 'Location','NorthEastOutside')
 end
 
    xlabel('x [m]', 'fontsize',16,'fontname', 'times');
ylabel('y [m]', 'fontsize',16,'fontname', 'times');

    


% break;
%         pause
        
        print(gcf,'-dpng',mat2str(brojac+100));
        
brojac=brojac+1
hold off
end
 
        


