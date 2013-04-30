numPosition = 0;

for numLayer=1:5


  fileMocapName = ['april_1_' num2str(numLayer) '_' num2str(numPosition) '_0_100samples.log'];
  data = importdata(fileMocapName);
  figure, hist(data(1:100, 1));
  title(['Apriltag Pose Observation Histogram at Layer ' num2str(numLayer) ' Position ' num2str(numPosition)]);
  xlabel('x (meter)');
  ylabel('Frequency');
end