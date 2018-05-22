function [ overImg ] = imoverlay( bgImage, fgImage, opacity )
%IMOVERLAY Overlay two images
%   Detailed explanation goes here
if ~exist('opacity','var')
  opacity = 0.5;
end
if size(bgImage,3) == 1
  bgImage = repmat(bgImage,1,1,3);
end

if size(fgImage,3) == 1
  fgImage = repmat(fgImage,1,1,3);
end

overImg = opacity*fgImage + (1-opacity)*bgImage;


end

