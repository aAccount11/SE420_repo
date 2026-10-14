function restartCollect(comport)
% to use:
% to start another collection of SPIRAM data
% restartCollect('COM6') or whatever COM port 'list_serialport'

existingPorts = serialportfind();
% Check for existing serial port objects

% If any serial ports are found, clean them up
if ~isempty(existingPorts)
    fprintf('length %f\n',length(existingPorts));
    for i = 1:length(existingPorts)
        try
            % Close and delete the serial port object
            delete(existingPorts(i));
        catch
            % Handle errors if the object cannot be deleted
            fprintf('Could not delete serial port: %s\n', existingPorts(i).Port);
        end
    end
end

% Clear the variables
clear existingPorts;

if (testifcom(comport) == 0)
    display('Choose from the following COM ports');
        list_serialports()
    exception = MException('MATLAB:ComportNotFound','COM Port not found.');
    throw(exception);
end


hex_str = 'CC55'; % header
char_str = char(sscanf(hex_str,'%2X').');

s = serialport(comport,115200);
s.InputBufferSize = 150000;
fopen(s);
fwrite(s,char_str);

pause(2); 
clear s
