num=0;
filename=strcat('podaci',mat2str(num));
fid=fopen(filename);
        if (fid~=-1)
            fclose(fid);
            laser=load(filename);
plot(laser(:,1),laser(:,2),'m*');
        end