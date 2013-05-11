close all
clear all
clc

% Figure saving size: w: max gedit paste button right white
%                     h: smplayer button bottom

% plot DataAndError6DOF
% load t_m data/UCSB/fi_bu_pd
rawDataMe = importdata('../../tracking_mapping/data/UCSB/fi_bu_pd');
rawDataGt = importdata('../../tracking_mapping/data/UCSB/fi_bu_pd_gt');


% data_image_0_0_770_0-55-5-12
% 50x7
% lenData = size(rawDataMe, 1);
lenData = 32;
rangeData = 1:lenData;
dataMe = rawDataMe;
dataGt = rawDataGt;

% dataAngX(:, 4) = dataAngX(:, 4) + 180;
% dataAngX(:, 6) = dataAngX(:, 6) + 90;

angRange = dataGt(1:lenData, 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataMe(1:lenData, 4) = roundAround180(dataMe(1:lenData, 4));
dataGt(1:lenData, 4) = roundAround180(dataGt(1:lenData, 4));
dataMe(1:lenData, 6) = roundAround180(dataMe(1:lenData, 6));
dataGt(1:lenData, 6) = roundAround180(dataGt(1:lenData, 6));







featureNum = dataMe(:, 7);

% [AX,H1,H2] = plotyy(rangeAngX, groundTruthAngX(:, 4)-dataAngX(:, 4), ...
%                     rangeAngX, featureNum);
% combine three
dist = sqrt( (dataGt(:, 1)-dataMe(:, 1)).^2 + ...
             (dataGt(:, 2)-dataMe(:, 2)).^2 + ...
             (dataGt(:, 3)-dataMe(:, 3)).^2);
% m to cm
dist = dist*100;
dataMe(:, 4:6) = degtorad(dataMe(:, 4:6));      
dataGt(:, 4:6) = degtorad(dataGt(:, 4:6));  

dcmMe = angle2dcm(dataMe(:, 6), dataMe(:, 5), dataMe(:, 4));
dcmGt = angle2dcm(dataGt(:, 6), dataGt(:, 5), dataGt(:, 4));
angDist = dist;
for i = 1:size(dcmMe,3)
    R = dcmGt(:,:,i)\dcmMe(:,:,i);
    [r1 r2 r3] = dcm2angle(R);
    angDist(i) = rad2deg(sqrt(r1^2 + r2^2 + r3^2));
end

subplot(2, 1, 1)
plot(angRange, featureNum(1:lenData), '*-r')
xlabel('Off-axis Angle (degrees)')
ylabel('Feature Number')
title({'\makebox[4in][c]{Y Axis Rotation (-45$^\circ$ to 0$^\circ$)}',...
       '\makebox[4in][c]{Feature Number}'},...
       'interpreter','latex');
grid on
set(gca,'YTick')
subplot(2, 1, 2)
[AX,H1,H2] = plotyy(angRange, dist(1:lenData), ...
                    angRange, angDist(1:lenData));
                
set(get(AX(1),'Ylabel'), 'String','Translation Error (cm)') 
set(get(AX(2),'Ylabel'), 'String','Orientation Error (degrees)') 
set(AX(2),'ycolor','r')
set(AX(1),'Ytick')
set(AX(2),'Ytick')
set(H1,'LineStyle','-', 'marker', '*')
set(H2,'LineStyle','-.', 'marker', 'o', 'color', 'r')

% set(gca,'XTick', groundTruth(:, 4))
% set(gca,'XTickLabel', cellstr(num2str(groundTruth(:, 4))))
title('Euclidean Distance and Single Rotation Angle');
xlabel('Off-axis Angle (degrees)')
% ylabel('Error (degrees)')
grid on


