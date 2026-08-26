T_REFRESH = 15;
Y_MIN = -4;
Y_MAX = 4;
NUMPOINTS = 2000;
pkg load instrument-control
clear s1;
done = 1;
i = 0;
clf
plothandle = plot([0 0],[0 0]);
axis([0 T_REFRESH Y_MIN Y_MAX]);
axis manual;

clear recordtime;
clear recordvalue;
clear recordvalue2;
clear x;
clear t;
x = cell(4,1);
t = cell(4,1);
hline(1) = line([0 0],[0 0]);
hline(2) = line([0 0],[0 0]);
%hline(3) = line([0 0],[0 0]);
%hline(4) = line([0 0],[0 0]);
set(hline(1),'Color','blue');
set(hline(2),'Color','red');
%set(hline(3),'Color','yellow');
%set(hline(4),'Color','green');


s1 = serial("\\\\.\\COM8",115200,10)
sendvalues = [255 127 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0];

for isave = NUMPOINTS
	recordtime(isave) = 0;
	recordvalue(isave) = 0;
	recordvalue2(isave) = 0;
endfor

charsend = uint8(sendvalues);
srl_flush(s1);
srl_write(s1,charsend);
previoustime = 0;
i = 1;
isave = 1;
while(done == 1)
	 [recdata, reccount] = srl_read(s1,12);
	 srl_write(s1,charsend);
	 time = int32(recdata(1)) + int32(recdata(2))*256 + int32(recdata(3))*256*256 + int32(recdata(4))*256*256*256;
	 rtime = double(time)/10000.0;
	 yvalue = int64(recdata(5)) + int64(recdata(6))*256 + int64(recdata(7))*256*256 + int64(recdata(8))*256*256*256;
	 if (yvalue >= int64(2147483648)) 	
		yvalue = yvalue - int64(4294967296);
	 endif
	 
	 ryvalue = double(yvalue)/10000.0;

	 y2value = int64(recdata(9)) + int64(recdata(10))*256 + int64(recdata(11))*256*256 + int64(recdata(12))*256*256*256;
	 if (y2value >= int64(2147483648)) 	
		y2value = y2value - int64(4294967296);
	 endif
	 
	 ry2value = double(y2value)/10000.0;

	 
	 if (isave == 1)
		starttime = rtime;
		firststarttime = rtime;
	 endif
	 
%	 [rtime ryvalue]
	 recordtime(isave) = rtime-firststarttime;
	 recordvalue(isave) = ryvalue;
	 recordvalue2(isave) = ry2value;
	 
	 t{1}(i) = mod(rtime-starttime,T_REFRESH);
	 x{1}(i) = ryvalue;
	 x{2}(i) = ry2value;
	 
	 
	 if (previoustime > t{1}(i))
		clf
		axis([0 T_REFRESH Y_MIN Y_MAX]);
		axis manual;
		clear x;
		clear t;
		x = cell(4,1);
		t = cell(4,1);
		hline(1) = line([0 0],[0 0]);
		hline(2) = line([0 0],[0 0]);
		%hline(3) = line([0 0],[0 0]);
		%hline(4) = line([0 0],[0 0]);
		set(hline(1),'Color','blue');
		set(hline(2),'Color','red');
		starttime = starttime+T_REFRESH;
		i = 0;  % because incremented to 1 below
	 endif
	 
	 if (i == 0)
		previoustime = 0;
	 else
		previoustime = t{1}(i);
	 endif
	 
	 if (mod(i,10) == 0)
		
		set(hline(1),'XData',t{1},'Ydata',x{1});
		set(hline(2),'XData',t{1},'Ydata',x{2});
		pause(.001)
	 endif
	 
%	 if (mod(i,100) == 0 )
%		i
	
%	 endif
	 if (isave == NUMPOINTS) 
		done = 0
	 endif
	 i = i + 1;
	 isave = isave + 1;
endwhile
clear s1