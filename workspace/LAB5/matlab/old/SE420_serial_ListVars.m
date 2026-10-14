function SE420_serial_ListVars()
%SE420_serial_ListVars
%   SE420_serial_ListVars() simple lists the possible variables Matlab has
%	access to inside your DSP application.  Helpful in finding the name of
%	the variable you would like to write to or read from.
%
%		SE420_serial_ListVars();
%
%

%s = instrfind;
%if length(s) > 0
%    fclose(s);
%end
%clear s;
filename = dir('../CPU1_RAM/*.map');

map = parseMap(strcat('../CPU1_RAM/',filename.name))

memloc = 0;
arrsize = size(map);
found = 1;
