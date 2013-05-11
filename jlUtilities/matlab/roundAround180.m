function [ angs ] = roundAround180( angs )
%ROUNDAROUND180 Summary of this function goes here
%   Detailed explanation goes here

temp = (angs>0).*(angs -360);
angs = temp + (angs<0).*angs;
end

