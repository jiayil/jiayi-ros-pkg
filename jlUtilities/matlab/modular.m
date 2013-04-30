function [ y ] = modular( x, n )
%MODULAR Summary of this function goes here
%   Detailed explanation goes here
y=rem(x,n);
ix=find(y<0);
y(ix)=y(ix)+n*ones(size(ix));

end

