function [ imgSeg, segLabels ] = imsegfilt( imgRaw, imgValues, numSegments, minRatio )
%IMSEGFILT Segment image and filter values according to superpixels
%   imgRaw ... original image for segmentation
%   imgValues ... image of the same size with values
%   imgSeg ... filtered value image
%   segLabels ... segmentation map

if ~exist('numSegments','var')
  numSegments = 100;
end

if ~exist('minRatio','var')
  minRatio = 0.5;
end

method = 'slic';

%% segment

if strcmp(method,'slic')
  %% SLIC
  [segLabels,numLabels] = superpixels(imgRaw,numSegments);
elseif strcmp(method,'ugm')
  %% UGM
  tic;
  segUcm = ucmseg(imgRaw,0.1);
  segLabels2 = bwlabel(segUcm <= 0.01);
  segLabels = segLabels2(2:2:end, 2:2:end);
  numLabels = max(segLabels(:));
  toc;
elseif strcmp(method,'meanshift')
  %% quickshift
  [imgSegColor, segLabels] = vl_quickseg(imgRaw,0.5,2,10);
end

%figure; imagesc(segLabels); colorbar;

%% filter
numLabels = max(segLabels(:));
imgSeg = imgValues;
idx = label2idx(segLabels);
for labelVal = 1:numLabels
  redIdx = idx{labelVal};
  redVals = imgValues(redIdx);
  medVal = median(redVals);
  medRatio = sum(redVals==medVal)/length(redVals);
  if medRatio > minRatio
    %% only fill if label has majority
    imgSeg(redIdx) = median(imgValues(redIdx));
  else
    imgSeg(redIdx) = imgValues(redIdx);
  end
end

end

