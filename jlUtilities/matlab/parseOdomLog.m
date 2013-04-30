function parseOdomLog( filePath )
%PARSEODOMLOG Summary of this function goes here
%   Detailed explanation goes here


% generate mat file 6*c:
%
% diskImageID 
% diskMapperInWorld
% diskLocalizerInWorld 
% ffMapperInWorld
% ffLocalizerInWorld 
% totalTime



% variables
imgID = 0;
validImgCounter = 1;



diskImageIDMat = zeros(1, 0);
diskMapperInWorldMat = zeros(6, 0);
diskLocalizerInWorldMat = zeros(6, 0);
ffMapperInWorldMat = zeros(6, 0);
ffLocalizerInWorldMat = zeros(6, 0);
totalTimeMat = zeros(1, 0);

fid = fopen(filePath);

while(1)
  ret = searchFor(fid, '-----');
  if ret == 1
    imgID = searchFor(fid, 'diskImageID');
    ret = searchFor(fid, 'tagFound');
    if ret == 0 % tag not found
      continue
    else
      diskImageIDMat(validImgCounter) = imgID;
      
      diskMapperInWorldMat(:, validImgCounter) = searchFor(fid, 'diskMapperInWorld');
      diskLocalizerInWorldMat(:, validImgCounter) = searchFor(fid, 'diskLocalizerInWorld');
      ffMapperInWorldMat(:, validImgCounter) = searchFor(fid, 'ffMapperInWorld');
      ffLocalizerInWorldMat(:, validImgCounter) = searchFor(fid, 'ffLocalizerInWorld');
      
      totalTimeMat(validImgCounter) = searchFor(fid, 'totalTime');
      
      validImgCounter = validImgCounter + 1;
    end
  else
    disp('End of file');
    break
  end

end

fclose(fid);

diskMapperInWorldMat = diskMapperInWorldMat';
diskLocalizerInWorldMat = diskLocalizerInWorldMat';
ffMapperInWorldMat = ffMapperInWorldMat';
ffLocalizerInWorldMat = ffLocalizerInWorldMat';

format('short');

dlmwrite( 'diskMapperInWorld.txt', diskMapperInWorldMat, ' ');
dlmwrite( 'diskLocalizerInWorld.txt', diskLocalizerInWorldMat, ' ');
dlmwrite( 'ffMapperInWorld.txt', ffMapperInWorldMat, ' ');
dlmwrite( 'ffLocalizerInWorld.txt', ffLocalizerInWorldMat, ' ');
end

