function [out3] = SE420_serialread(name,varsize,comport)
%SE420_SERIALREAD
%   N = SE420_serialread(NAME,VARSIZE,COMPORT) will return data from the DSP declared
%	in either the ".my_arrs" or ".my_vars" data sections where NAME refers to
%	the name of the variable or array on the DSP, and varsize refers to the
%	size (number of 32-bit words) in the variable or array.
%   comport is a string with the COM port connected to the "XDS100 Serial Port".
%	It is usually the largest number of COM port in your system.  If you do not
%	know your comport, run the command "seraillist" to see all the
%	available comports.
%
%	For example, to read the float variable "myfloat" from the DSP and store
%	it into a Matlab single variable called "mydata" you would type:
%
%		mydata = SE420_serialread('myfloat',1,"\\\\.\\COM8");
%
%	To read the first 1000 entries in the float array "myfloatarray" 
%	and store it into a Matlab single variable array called "myarray" 
%	you would type:	
%
%		myarray = SE420_serialread('myfloatarray',1000,"\\\\.\\COM8");
%

pkg load instrument-control;

if (varsize > 1000)
    display('Varsize must be 1000 or less');
    exception = MException('MATLAB:VarsizeMAX','Varsize must be 1000 or smaller');
    throw(exception);
end

filename = dir('../CPU1_RAM/*.map');

map = parseMap(strcat('../CPU1_RAM/',filename.name))

memloc = 0;
arrsize = size(map);
found = 1;

for i=1:arrsize(1)
    if strfind(char(map(i,1)),name)
        memloc = char(map(i,2));
        found = 0;
    end
end

if (found == 0)
  hexsize = dec2hex(varsize,4);
  hex_str = '2A0932'; % header
  hex_str = strcat(hex_str,memloc,hexsize);
  char_str = char(sscanf(hex_str,'%2x').');


  s1 = serial(comport,115200,10);

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
        myfid = fopen('tempscratch.dat','w');
        writecount = fwrite(myfid,data);
        fclose(myfid);
        myfid = fopen('tempscratch.dat','r');
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
else 
  display('Variable name not found.');  
endif