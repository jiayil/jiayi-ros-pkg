load mdata.mat


cntBig = 1;
fontSize = 15;
lineWidth = 2;

[sizeRow sizeCol] = size(mDataXm);



figure, plot(mDataXm(1, :), mDataXm(2, :), '+r', ...
  mDataXa(1, :), mDataXa(2, :), 'dr', ...
  'MarkerSize', fontSize, 'LineWidth', lineWidth);
% title('Apriltag Positions in the Field of View of the Low-resolution Camera - Using Undistorted Images', ...
%   'FontSize', fontSize);
xlabel('x (meter)', 'FontSize', fontSize);
ylabel('y (meter)', 'FontSize', fontSize);
set(gca,'FontSize',fontSize);


hleg1 = legend('Motion Capture','Apriltag','Yaw Difference (degree)');
set(hleg1, 'FontSize', fontSize);
hold on

rDiffYawRaw = abs(mDataXa(6, :) - mDataXm(6, :));
rDiffYaw = abs(mDataXa(6, :) - mDataXm(6, :))/20;



[rRow rCol] = size(rDiffYaw);
for n = 1:rCol
  
  [x y z] = cylinder(rDiffYaw(n), 200);
  x = x + mDataXa(1, n);
  y = y + mDataXa(2, n);
  plot(x(1,:), y(1,:), 'b', 'LineWidth', lineWidth);
  text(mDataXa(1, n)+0.07, mDataXa(2, n), sprintf('%.1f', rDiffYawRaw(n)),... %num2str(rDiffYawRaw(n)),...
       'FontSize', fontSize);

end
axis equal

% 
% for cnt = 1:sizeCol
% 
%     
%     xm = mDataXm(:, cnt);
%     Pm = mDataPm(:, cntBig:(cntBig+5));
%     Wm = mDataWm(:, cntBig:(cntBig+5));
%     
%     
%     xa = mDataXa(:, cnt);
%     Pa = mDataPa(:, cntBig:(cntBig+5));
%     Wa = mDataWa(:, cntBig:(cntBig+5));
%     
% 
%     
%     
%     cntBig = cntBig + 6;
% end