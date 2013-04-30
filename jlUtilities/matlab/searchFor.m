function [ ret ] = searchFor( fid, note )
%SEARCHFOR Summary of this function goes here
%   Detailed explanation goes here

format = [];
numFormat = 0;
typeFormat = 0;

switch note
  case '-----'
    format = '%s';
    numFormat = 1;
    typeFormat = 1;
  case 'diskImageID'
    format = '%s %u';
    numFormat = 2;
    typeFormat = 2;
  case 'tagFound'
    format = '%s %u';
    numFormat = 2;
    typeFormat = 2;
  case 'totalTime'
    format = '%s %f';
    numFormat = 2;
    typeFormat = 3;
  otherwise
    format = '%s %f %f %f %f %f %f';
    numFormat = 7;
    typeFormat = 4;
end




tline = fgetl(fid);
while ischar(tline)
  tline = textscan(tline, format);
  if strcmp(tline{1}, note)
    switch typeFormat
      case 1
        ret = 1;
      case 2
        ret = tline{numFormat};
      case 3
        ret = tline{numFormat};
      case 4
        for n=2:numFormat
          ret(n-1) = tline{n};
        end
    end
    return
    
    
    
    
    
  else
    tline = fgetl(fid);
  end

end


ret = -1;

end