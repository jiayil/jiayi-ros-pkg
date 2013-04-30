dL = 'diskLocalizerInWorld.txt';
dM = 'diskMapperInWorld.txt';
fL = 'ffLocalizerInWorld.txt';
fM = 'ffMapperInWorld.txt';

fontSize = 15;
markerSize = 10;
lineWidth = 3;


txt = importdata(dL, ' ');
dLData = txt(:, 1:2);
dLData(:, 3) = txt(:, 6);

txt = importdata(dM, ' ');
dMData = txt(:, 1:2);
dMData(:, 3) = txt(:, 6);

txt = importdata(fL, ' ');
fLData = txt(:, 1:2);
fLData(:, 3) = txt(:, 6);

txt = importdata(fM, ' ');
fMData = txt(:, 1:2);
fMData(:, 3) = txt(:, 6);

xErr = dMData(:, 1)-fMData(:, 1);
stdX = std(xErr);

numVect = 1:length(xErr);

yErr = dMData(:, 2)-fMData(:, 2);
stdY = std(yErr);

tErr = dMData(:, 3)-fMData(:, 3);
tErrMod = mod(tErr, -sign(tErr).*(tErr>200|tErr<-200).*360);
stdT = std(tErr);

figure,

subplot(3,2,1);
plot(numVect, dMData(:, 1), 'r+', ...
     numVect, fMData(:, 1), 'go')
title('X Direction Comparison', 'FontSize', fontSize)
xlabel('Samples', 'FontSize', fontSize)
ylabel('x (meter)', 'FontSize', fontSize)

subplot(3,2,2);
plot(xErr, '-*')
% plot(numVect, xErr, '-b*', ...
%      numVect, dMData(:, 1), 'r+', ...
%      numVect, fMData(:, 1), 'go')
title('X Direction Error', 'FontSize', fontSize)
xlabel('Samples', 'FontSize', fontSize)
ylabel('x (meter)', 'FontSize', fontSize)
%text(size(xErr, 1)+1, xErr(end), ['std = ', num2str(stdX)])

subplot(3,2,3);
% plot(yErr, '-*')
plot(numVect, dMData(:, 2), 'r+', ...
     numVect, fMData(:, 2), 'go')
title('Y Direction Comparison', 'FontSize', fontSize)
xlabel('Samples', 'FontSize', fontSize)
ylabel('y (meter)', 'FontSize', fontSize)


subplot(3,2,4);
plot(yErr, '-*')
title('Y Direction Error', 'FontSize', fontSize)
xlabel('Samples', 'FontSize', fontSize)
ylabel('y (meter)', 'FontSize', fontSize)
%text(size(yErr, 1)+1, yErr(end), ['std = ', num2str(stdY)])

subplot(3,2,5);
% plot(tErr, '-*')
plot(numVect, dMData(:, 3), 'r+', ...
     numVect, fMData(:, 3), 'go')
title('Yaw Angle Comparison', 'FontSize', fontSize)
xlabel('Samples', 'FontSize', fontSize)
ylabel('yaw (degree)', 'FontSize', fontSize)

subplot(3,2,6);
plot(tErrMod, '-*')
title('Yaw Angle Error', 'FontSize', fontSize)
xlabel('Samples', 'FontSize', fontSize)
ylabel('yaw (degree)', 'FontSize', fontSize)
%text(size(tErr, 1)+1, tErr(end), ['std = ', num2str(stdT)])


% figure, plot(dLData(:, 1), dLData(:, 2), '-go', ...
%              dMData(:, 1), dMData(:, 2), '-gx', ...
%              fLData(:, 1), fLData(:, 2), ':rs', ...
%              fMData(:, 1), fMData(:, 2), ':rd', ...
%   'MarkerSize', markerSize, 'LineWidth', lineWidth);
% %title('Localization Result');
% xlabel('x (meter)', 'FontSize', fontSize);
% ylabel('y (meter)', 'FontSize', fontSize);
% set(gca,'FontSize',fontSize);
% % hleg1 = legend('Localizer by Hi-res Camera', ...
% %                'Mapper by Hi-res Camera', ...
% %                'Localizer by Lo-res Camera', ...
% %                'Mapper by Lo-res Camera');
% % set(hleg1, 'FontSize', fontSize);             