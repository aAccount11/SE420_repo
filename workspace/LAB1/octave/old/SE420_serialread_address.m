function [out3] = SE420_serialread_address(address,varsize)
%SE420_SERIALREAD_ADDRESS
%   N = SE420_serialread_address(ADDRESS,VARSIZE) will return data from the
%   DSP stored at the location ADDRESS in decimal, not hex, and varsize 
%   refers to the size (number of 32-bit words) in the variable or array.
%
%	For example, to read the float variable at the memory address 
%   "0001f5b0" on the DSP and store it into a Matlab single variable called
%   "mydata" you would type:
%
%		mydata = SE420_serialread_address(128432,1);
%
%	To read the first 1000 entries in the float array starting at address 
%   "0001f5b0" and store it into a Matlab single variable array called 
%   "myarray" you would type:	
%
%		myarray = SE420_serialread_address(128432,1000);
%

pkg load instrument-control;

memloc = dec2hex(address,8);

hexsize = dec2hex(varsize,4);
hex_str = '2A0932'; % header
hex_str = strcat(hex_str,memloc,hexsize);
char_str = char(sscanf(hex_str,'%2x').');


s1 = serial("\\\\.\\COM8",115200,10)

srl_flush(s1);
srl_write(s1,char_str);



count = 0;
while 1
	[inchar, reccount] = srl_read(s1,1);
    if inchar == 42
		[inchar, reccount] = srl_read(s1,1);
		[inchar, reccount] = srl_read(s1,1);
        if inchar == 51
			[data,reccount] = srl_read(s1,4*varsize);
			myfid = fopen('dantemp.dat','w');
			writecount = fwrite(myfid,data);
			fclose(myfid);
			myfid = fopen('dantemp.dat','r');
            out3 = fread(myfid,varsize,'float32');
            break;
        end
    end
    count = count + 1;
    if count > 100
        break;
    end
end


fclose(myfid)
clear s1

