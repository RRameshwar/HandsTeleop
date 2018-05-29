%clear
% close all
clc
delete(instrfindall)

% Timer settings for data capture
end_time = 60;
controlFrequency = 1000; % Approximately 1 kHz. It's probaby changing between 1006-1008 Hz.
t=0;

% Com parameters
incomingLength = 16;
s = serial('COM8','BaudRate', 500000);
s.InputBufferSize = controlFrequency*end_time*incomingLength;
fopen(s);

% Arrays
T_Array = [];

thumbB_AngleArray = [];
index_AngleArray = [];
thumbB_AngleRefArray = [];
index_AngleRefArray = [];
Bx_Array = [];
By_Array = [];
Bz_Array = [];

% Wait until buffer is filled
tic
BufferSize = s.InputBufferSize
while(s.BytesAvailable < s.InputBufferSize)
%     s.BytesAvailable
end
toc
data = fread(s, s.InputBufferSize);

% Decode data collected from buffer
for i = 1:length(data)-incomingLength+1
    if(data(i) == 24 && data(i+incomingLength-1) == 129)
        data;

        
        thumbB_Angle = 0;
        index_Angle = 0;
        thumbB_RefAngle = 0;
        index_RefAngle = 0;
        Bx_Value = 0;
        By_Value = 0;
        Bz_Value = 0;
        
        thumbB_Angle = bitor(thumbB_Angle, bitand(bitshift(data(i+1),8), 65535));
        thumbB_Angle = bitor(thumbB_Angle, data(i+2));    
        thumbB_Angle = double(typecast(uint16(thumbB_Angle),'int16'));
        thumbB_Angle = thumbB_Angle/100;
        thumbB_Angle;           
        
        index_Angle = bitor(index_Angle, bitand(bitshift(data(i+3),8), 65535));
        index_Angle = bitor(index_Angle, data(i+4));    
        index_Angle = double(typecast(uint16(index_Angle),'int16'));
        index_Angle = index_Angle/100;
        index_Angle;         
        
        index_RefAngle = bitor(index_RefAngle, bitand(bitshift(data(i+5),8), 65535));
        index_RefAngle = bitor(index_RefAngle, data(i+6));    
        index_RefAngle = double(typecast(uint16(index_RefAngle),'int16'));
        index_RefAngle = index_RefAngle/100;
        index_RefAngle;      
        
        thumbB_RefAngle = bitor(thumbB_RefAngle, bitand(bitshift(data(i+7),8), 65535));
        thumbB_RefAngle = bitor(thumbB_RefAngle, data(i+8));    
        thumbB_RefAngle = double(typecast(uint16(thumbB_RefAngle),'int16'));
        thumbB_RefAngle = thumbB_RefAngle/100;
        thumbB_RefAngle;     

        Bx_Value = bitor(Bx_Value, bitand(bitshift(data(i+9),8), 65535));
        Bx_Value = bitor(Bx_Value, data(i+10));    
        Bx_Value = double(typecast(uint16(Bx_Value),'int16'));
        Bx_Value;        
        
        By_Value = bitor(By_Value, bitand(bitshift(data(i+11),8), 65535));
        By_Value = bitor(By_Value, data(i+12));    
        By_Value = double(typecast(uint16(By_Value),'int16'));
        By_Value;         
        
        Bz_Value = bitor(Bz_Value, bitand(bitshift(data(i+13),8), 65535));
        Bz_Value = bitor(Bz_Value, data(i+14));    
        Bz_Value = double(typecast(uint16(Bz_Value),'int16'));
        Bz_Value; 
        
        thumbB_AngleArray = [thumbB_AngleArray, thumbB_Angle];
        index_AngleArray = [index_AngleArray, index_Angle];
        thumbB_AngleRefArray = [thumbB_AngleRefArray, thumbB_RefAngle];
        index_AngleRefArray = [index_AngleRefArray, index_RefAngle];
        Bx_Array = [Bx_Array, Bx_Value];
        By_Array = [By_Array, By_Value];
        Bz_Array = [Bz_Array, Bz_Value];
        
        T_Array = [T_Array, i/1000];
        
    end
end
%timeArray = [1:length(encArrayRight)]'/1000;
T_Array = [0:length(T_Array)-1];

% Kill COM channel
fclose(s);
delete(instrfindall)
delete(s)
clear s

save([datestr(now,'mm-dd-yyyy HH-MM'),'_EggPickPlace'])

