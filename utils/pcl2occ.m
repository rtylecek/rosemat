function [bmap, pmap, hmap, gmapf, grx,gry] = pcl2occ(pcl,res,rng) 
%% generate occupancy grid from point cloud
%%  omap is generated in the xy limits of pcl
% res = 0.2;  % resolution of grid in pcl units
% rng = 1.5;  % point neighborhood size (1 = grid distance)

%% init grid

rx = pcl.XLimits(1):res:pcl.XLimits(2);
ry = pcl.YLimits(1):res:pcl.YLimits(2);

grx = rx(1:end-1) + res/2;
gry = ry(1:end-1) + res/2;

ocx = length(grx);
ocy = length(gry);

hmap = zeros(ocy,ocx);
gmap = zeros(ocy,ocx);

%% med height
pcx = pcl.Location;
pcx(:,3) = 0;
pcz = pointCloud(pcx);

for x=1:ocx
  for y=1:ocy
    % max = height
    [ki,kd] = pcz.findNeighborsInRadius([grx(x) gry(y) 0],res*rng);
    hm = max(pcl.Location(ki,3));
    if isempty(hm), hm = nan; end;
    hmap(y,x) = hm;
    % min = ground
    [ki,kd] = pcl.findNearestNeighbors([grx(x) gry(y) pcl.ZLimits(1)],5);
    gmap(y,x) = max(pcl.Location(ki,3));

  end
end
%% detect obstacles above ground
gmapf = medfilt2(gmap,[10 10],'symmetric');
%gmapf = imfilter(gmap,fspecial('gaussian',[10 10],2));
pmap = (hmap-gmapf);
pmap(pmap<0) = 0;
pmap(pmap<0.2) = 0.05;
pmap(isnan(pmap)) = 0;
pmap = pmap/max(pmap(:));
%% obstacle map
bmap = pmap<0.2;
bmap = flip(bmap,1);

%% results
hmap = flip(hmap,1);
gmapf = flip(gmapf,1);
pmap = flip(pmap,1);

