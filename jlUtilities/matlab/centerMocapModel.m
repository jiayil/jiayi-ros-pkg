% numLayer = 0;
% numPosition = 0;

for numLayer = 1:5
  for numPosition = 0:2
    fileName = ['mocap_4171_' num2str(numLayer) '_' num2str(numPosition) '_0_100samples.log'];
    
    data = importdata(fileName);
    data(:, 1) = data(:, 1) - 0.025;
    data(:, 2) = data(:, 2) - 0.01;
    data(:, 3) = data(:, 3) + 0.195;
    
    save( [fileName '.centered'], 'data', '-ascii');
    
    
  end
  
end