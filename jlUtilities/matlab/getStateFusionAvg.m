function [ xk Pk W ] = getStateFusionAvg( data )
% function [ x1 P1 x2 P2 W ] = getStateFusionAvg( data )
%GETSTATEFUSIONAVG Summary of this function goes here
%   Detailed explanation goes here

% X
% Y
% Z
% r
% p
% y

[dataRow dataCol] = size(data);

W = cov(data');
[sizeRow sizeCol] = size(W);

Pk_1 = eye(sizeRow);

Pk = Pk_1;


xk_1 = zeros(dataRow, 1);
xk = xk_1;

for r = 1:dataCol
  Sk = Pk_1 + W;
  Kk = Pk_1 / Sk;
  xk = xk_1 + Kk * ( data(:, r) - xk_1 );
  Pk = ( eye(sizeRow) - Kk ) * Pk_1;
  
  Pk_1 = Pk;
  xk_1 = xk;
  
end

x1 = xk;
P1 = Pk;

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Pk_1 = W;
% Pk = Pk_1;
% 
% 
% xk_1 = zeros(dataRow, 1);
% xk = xk_1;
% 
% for r = 1:dataCol
%   Sk = Pk_1 + W;
%   Kk = Pk_1 / Sk;
%   xk = xk_1 + Kk * ( data(:, r) - xk_1 );
%   Pk = ( eye(sizeRow) - Kk ) * Pk_1;
%   
%   Pk_1 = Pk;
%   xk_1 = xk;
%   
% end
% 
% x2 = xk;
% P2 = Pk;

end

