clear
clc
% data format for the following data groups
% 1. mocap data
% 2. april data
mDataXm = zeros(6, 0);
mDataPm = zeros(6, 0);
mDataWm = mDataPm;

mDataXa = zeros(6, 0);
mDataPa = zeros(6, 0);
mDataWa = mDataPa;

cnt = 1;
cntBig = cnt;

for numLayer = 1:5
  for numPosition = 0:2
    fileMocapName = ['mocap_1_' num2str(numLayer) '_' num2str(numPosition) '_0_100samples.log'];
    fileAprilName = ['april_1_' num2str(numLayer) '_' num2str(numPosition) '_0_100samples.log'];
    
    
    dataMocap = importdata(fileMocapName)';
    dataApril = importdata(fileAprilName)';
    
    % mocap data
    [ x P W ] = getStateFusionAvg(dataMocap);
    
    mDataXm(:, cnt) = x;
    mDataPm(:, cntBig:(cntBig+5)) = P;
    mDataWm(:, cntBig:(cntBig+5)) = W;

    % april data
    [ x P W ] = getStateFusionAvg(dataApril);
    
    mDataXa(:, cnt) = x;
    mDataPa(:, cntBig:(cntBig+5)) = P;
    mDataWa(:, cntBig:(cntBig+5)) = W;
    


    
    cnt = cnt + 1;
    cntBig = cntBig + 6;
  end
  
end

save('mdata.mat', 'mDataXm', 'mDataPm', 'mDataWm', 'mDataXa', 'mDataPa', 'mDataWa');


% numLayer =3;
% numPosition = 0;
% 
% fileMocapName = ['mocap_4171_' num2str(numLayer) '_' num2str(numPosition) '_0_100samples.log.centered'];
% fileAprilName = ['april_4171_' num2str(numLayer) '_' num2str(numPosition) '_0_100samples.log'];
% 
% 
% dataMocap = importdata(fileMocapName)';
% dataApril = importdata(fileAprilName)';
% 
% [ x P W ] = getStateFusionAvg(dataApril);