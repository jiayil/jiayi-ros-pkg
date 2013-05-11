close all
clear all
clc

% plot DataAndError6DOF
% load t_m data/metric/data_image
rawDataAngX = importdata('../../tracking_mapping/data/metric/data_image_0_0_770_0-55-5-12');
rawDataZ = importdata('../../tracking_mapping/data/metric/data_image_0_0_800-1500-50-15_0');


% data_image_0_0_770_0-55-5-12
% 12 x 6
rangeAngX = 0:5:55;
dataAngX = rawDataAngX;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Don't forget to change the range
dataAngX(:, 2) = dataAngX(:, 2) + 0.04;
dataAngX(:, 3) = dataAngX(:, 3) + 0.02;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataAngX(:, 4) = dataAngX(:, 4) + 180;
dataAngX(:, 6) = dataAngX(:, 6) + 90;
groundTruthAngX = zeros(12, 6);
groundTruthAngX(:, 3) = 0.770;
groundTruthAngX(:, 4) = rangeAngX;
featureNum = dataAngX(:, 7);
% [AX,H1,H2] = plotyy(rangeAngX, groundTruthAngX(:, 4)-dataAngX(:, 4), ...
%                     rangeAngX, featureNum);
% combine three
dist = sqrt( (groundTruthAngX(:, 1)-dataAngX(:, 1)).^2 + ...
             (groundTruthAngX(:, 2)-dataAngX(:, 2)).^2 + ...
             (groundTruthAngX(:, 3)-dataAngX(:, 3)).^2);
% m to mm
dist = dist*100;
dataAngX(:, 4:6) = degtorad(dataAngX(:, 4:6));      
groundTruthAngX(:, 4:6) = degtorad(groundTruthAngX(:, 4:6));  

dcmAngX = angle2dcm(dataAngX(:, 6), dataAngX(:, 5), dataAngX(:, 4));
dcmGroundTruth = angle2dcm(groundTruthAngX(:, 6), groundTruthAngX(:, 5), groundTruthAngX(:, 4));
angDist = dist;
for i = 1:size(dcmAngX,3)
    R = dcmGroundTruth(:,:,i)\dcmAngX(:,:,i);
    [r1 r2 r3] = dcm2angle(R);
    angDist(i) = rad2deg(sqrt(r1^2 + r2^2 + r3^2));
end

subplot(2, 1, 1)
plot(rangeAngX, featureNum, '*-r')
xlabel('Off-axis Angle (degrees)')
ylabel('Feature Number')
title({'\makebox[4in][c]{Target situated at (0, 0, 77, $\theta$, 0, 0)}',...
       '\makebox[4in][c]{Feature Number wrt. Increasing Angle}'},...
       'interpreter','latex');
grid on
set(gca,'YTick', 0:30:150)
subplot(2, 1, 2)
[AX,H1,H2] = plotyy(rangeAngX, dist, ...
                    rangeAngX, angDist);
                
set(get(AX(1),'Ylabel'), 'String','Translation Error (cm)') 
set(get(AX(2),'Ylabel'), 'String','Orientation Error (degrees)') 
set(AX(2),'ycolor','r')
set(AX(1),'Ytick')
set(AX(2),'Ytick')
set(H1,'LineStyle','-', 'marker', '*')
set(H2,'LineStyle','-.', 'marker', '*', 'color', 'r')

% set(gca,'XTick', groundTruth(:, 4))
% set(gca,'XTickLabel', cellstr(num2str(groundTruth(:, 4))))
title('Euclidean Distance and Single Rotation Angle');
xlabel('Off-axis Angle (degrees)')
% ylabel('Error (degrees)')
grid on



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure,
% data_image_0_0_800-1500-50-15_0
% 15 x 6
rangeZ = 0.8:0.05:1.5;
dataZ = rawDataZ;
groundTruthZ = zeros(15, 6);
% expand with angX[1] data
rangeZ = [0.77 rangeZ];
dataZ = [rawDataAngX(1, :); dataZ];
groundTruthZ = [groundTruthAngX(1, :); groundTruthZ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataZ(:, 2) = dataZ(:, 2) + 0.04;
dataZ(:, 3) = dataZ(:, 3) + 0.02;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataZ(:, 4) = dataZ(:, 4) + 180;
dataZ(:, 6) = dataZ(:, 6) + 90;

groundTruthZ(:, 3) = rangeZ;

featureNum = dataZ(:, 7);

% [AX,H1,H2] = plotyy(rangeZ, groundTruthZ(:, 3)-dataZ(:, 3) - (groundTruthZ(1, 3)-dataZ(1, 3)), ...
%                     rangeZ, featureNum);
                
                % combine three
dist = sqrt( (groundTruthZ(:, 1)-dataZ(:, 1)).^2 + ...
             (groundTruthZ(:, 2)-dataZ(:, 2)).^2 + ...
             (groundTruthZ(:, 3)-dataZ(:, 3)).^2);
% m to cm
dist = dist*100;
dataZ(:, 4:6) = degtorad(dataZ(:, 4:6));      
groundTruthZ(:, 4:6) = degtorad(groundTruthZ(:, 4:6));  

dcmZ = angle2dcm(dataZ(:, 6), dataZ(:, 5), dataZ(:, 4));
dcmGroundTruth = angle2dcm(groundTruthZ(:, 6), groundTruthZ(:, 5), groundTruthZ(:, 4));
angDist = dist;
for i = 1:size(dcmZ,3)
    R = dcmGroundTruth(:,:,i)\dcmZ(:,:,i);
    [r1 r2 r3] = dcm2angle(R);
    angDist(i) = rad2deg(sqrt(r1^2 + r2^2 + r3^2));
end

rangeZ = rangeZ*100;

subplot(2, 1, 1)
plot(rangeZ, featureNum, '*-r')
xlabel('Target Distance (cm)')
ylabel('Feature Number')
title({'\makebox[4in][c]{Target situated at (0, 0, z, 0, 0, 0)}',...
       '\makebox[4in][c]{Feature Number wrt. Increasing Distance}'},...
       'interpreter','latex');
grid on
set(gca,'YTick', 0:30:150)
subplot(2, 1, 2)
[AX,H1,H2] = plotyy(rangeZ, dist, ...
                    rangeZ, angDist);
                
set(get(AX(1),'Ylabel'), 'String','Translation Error (cm)') 
set(get(AX(2),'Ylabel'), 'String','Orientation Error (degrees)') 
set(AX(2),'ycolor','r')
set(AX(1),'Ytick', 0:2:10)
set(AX(2),'Ytick', 0:4:20)
set(H1,'LineStyle','-', 'marker', '*')
set(H2,'LineStyle','-.', 'marker', '*', 'color', 'r')

% set(gca,'XTick', groundTruth(:, 4))
% set(gca,'XTickLabel', cellstr(num2str(groundTruth(:, 4))))
title('Euclidean Distance and Single Rotation Angle');
xlabel('Target Distance (cm)')
% ylabel('Error (degrees)')
grid on

