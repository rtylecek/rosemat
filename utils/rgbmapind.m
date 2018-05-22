function [ ind ] = rgbmapind( rgb, cmap )
%RGBMAPIND Convert color image exactly to given colormap 
%   rgb ... color image (uint8)
%   cmap ... colormap (uint8)
rgb = uint32(rgb);
cmap = uint32(cmap);
srgb = rgb(:,:,1) + 256*rgb(:,:,2) + 256^2 * rgb(:,:,3);
smap = cmap(:,1) + 256*cmap(:,2) + 256^2 * cmap(:,3);

ind = zeros(size(srgb,1),size(srgb,2),'uint16');
for i = 1:length(smap)
  ind(srgb==smap(i)) = i;  
end

