dL = 'diskLocalizerInWorld.txt';
dM = 'diskMapperInWorld.txt';
fL = 'ffLocalizerInWorld.txt';
fM = 'ffMapperInWorld.txt';

fontSize = 15;
markerSize = 10;
lineWidth = 3;


txt = importdata(dL, ' ');
dLData = txt(:, 1:2);

txt = importdata(dM, ' ');
dMData = txt(:, 1:2);

txt = importdata(fL, ' ');
fLData = txt(:, 1:2);

txt = importdata(fM, ' ');
fMData = txt(:, 1:2);

figure, plot(dLData(:, 1), dLData(:, 2), '-go', ...
             dMData(:, 1), dMData(:, 2), '-gx', ...
             fLData(:, 1), fLData(:, 2), ':rs', ...
             fMData(:, 1), fMData(:, 2), ':rd', ...
  'MarkerSize', markerSize, 'LineWidth', lineWidth);
%title('Localization Result');
xlabel('x (meter)', 'FontSize', fontSize);
ylabel('y (meter)', 'FontSize', fontSize);
set(gca,'FontSize',fontSize);
% hleg1 = legend('Localizer by Hi-res Camera', ...
%                'Mapper by Hi-res Camera', ...
%                'Localizer by Lo-res Camera', ...
%                'Mapper by Lo-res Camera');
% set(hleg1, 'FontSize', fontSize);             