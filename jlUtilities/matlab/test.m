plot(1:10)
title({'This is a long title for a simple plot of y=x from 1 to 10, isn"t it?';'Yes, it most certainly is a long title.'})
disp('This is how it looks before moving the axis')
disp('Press a key to continue')
pause
set(gca, 'Units', 'Normalized')
P=get(gca,'Position');
% Shrink the bottom margin by half its height and enlarge the top margin by the same amount
% (since the axis is the same height before & after)
set(gca, 'Position', [P(1) P(2)/2 P(3) P(4)])
disp('This is how it looks after moving the axis')