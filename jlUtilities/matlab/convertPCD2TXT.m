function convertPCD2TXT( filePath )
%CONVERTPCD2TXT Summary of this function goes here
%   Detailed explanation goes here

txt = importdata(filePath, ' ', 11);
data = txt.data(:, 1:3);
save( [filePath '.txt'], 'data', '-ascii');
end

