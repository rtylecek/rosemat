function varargout = AnotForm(varargin)
% ANOTFORM MATLAB code for AnotForm.fig
%      ANOTFORM, by itself, creates a new ANOTFORM or raises the existing
%      singleton*.
%
%      H = ANOTFORM returns the handle to a new ANOTFORM or the handle to
%      the existing singleton*.
%
%      ANOTFORM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ANOTFORM.M with the given input arguments.
%
%      ANOTFORM('Property','Value',...) creates a new ANOTFORM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AnotForm_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AnotForm_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AnotForm

% Last Modified by GUIDE v2.5 21-Jun-2017 21:34:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
  'gui_Singleton',  gui_Singleton, ...
  'gui_OpeningFcn', @AnotForm_OpeningFcn, ...
  'gui_OutputFcn',  @AnotForm_OutputFcn, ...
  'gui_LayoutFcn',  [] , ...
  'gui_Callback',   []);
if nargin && ischar(varargin{1})
  gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
  [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
  gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before AnotForm is made visible.
function AnotForm_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AnotForm (see VARARGIN)

addpath ./utils/yaml;
addpath ./utils/render;
addpath ./utils/ply;
addpath ./utils/epicflow;
addpath ./utils/epicflow/utils/flow-code-matlab;
addpath ./utils;

% Choose default command line output for AnotForm
handles.output = hObject;

handles.project.workPath = pwd;
handles.project.bagPath = [];
handles.project.modelPath = [];
handles.project.calibPath = [];

handles.project.optOverAlpha = 0.5;
handles.project.optPointSize = 3;

handles.bag = [];
handles.model = [];
handles.calib = [];

handles.imgGray = zeros(100,100);
handles.imgAnot = zeros(100,100);
handles.hPlot = imshow(handles.imgAnot);
handles.prevAnot = [];
handles.nextAnot = [];
handles.anotLoaded = 0;
handles.drawing = 0;
handles.camMode = 0;

%% read labels
handles = read_labels('data/def',handles);
%%
handles.lstClass.String = handles.labelNames;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AnotForm wait for user response (see UIRESUME)
% uiwait(handles.figAnot);

function handles = LoadBag(filePath, handles)
%% read stream from bag file
fprintf('Reading bag %s ... ',filePath); tic;
handles.bag = rosbag(filePath);
handles.project.bagPath = filePath;
handles.project.anotPath = fullfile(fileparts(filePath),'anot');

%handles.tfData = readMessages(select(handles.bag,'Topic','/tf'), 1:300);
handles.bagOdometry = select(handles.bag,'Topic','/ground_truth/odom');
handles.gtOdometry = readMessages(handles.bagOdometry);
toc;
%% image topics list
disp(handles.bag.AvailableTopics);
handles.camList = {};
for i = 1:size(handles.bag.AvailableTopics,1)
  top = string(handles.bag.AvailableTopics.Properties.RowNames(i));
  if strcmp(string(handles.bag.AvailableTopics{i,'MessageType'}),'sensor_msgs/Image')
    if endsWith(top,'_raw')
      handles.camList = [handles.camList; top ];
    end
  end
end
handles.lstCameras.String = handles.camList;
%% img
handles = LoadTopic(handles.camList{1},handles);

function handles = LoadTopic(camTopic, handles)
%% read topic from bag file
fprintf('Reading cam %s ... ',camTopic); tic;
handles.cam = select(handles.bag,'Topic',camTopic);
handles.camTopic = camTopic;
handles.idCam = str2double(camTopic(length('/uvc_camera/cam_%'))); % TODO: cam id from selection

%% cam calib
if ~isempty(handles.calib)
  cam = handles.calib.camChain.(sprintf('cam%d',handles.idCam));  % TODO: find by topic
  if ~strcmp(cam.rostopic, camTopic)
    warning('rostopic mismatch bag <> yaml: %s <> %s',cam.rostopic,camTopic);
  end
  K = eye(3);
  K(1,1) = cam.intrinsics(1);
  K(2,2) = cam.intrinsics(2);
  K(1,3) = cam.intrinsics(3);
  K(2,3) = cam.intrinsics(4);
  handles.calib.K = K;
  handles.calib.cam = cam;
  handles.calib.camParams = cameraParameters('IntrinsicMatrix',K', ...
    'RadialDistortion',cam.distortion_coeffs(1:2), ...
    'TangentialDistortion',cam.distortion_coeffs(3:4));
  
  %% get extrinsic recursively
  TFext = eye(4);
  if handles.idCam>0
    for pid = handles.idCam:-1:1
      camp = handles.calib.camChain.(sprintf('cam%d',pid));
      TFext = camp.T_cn_cnm1 * TFext;
    end
  end
  handles.calib.Text = TFext(1:3,4)';
  handles.calib.Rext = TFext(1:3,1:3);
  %% mesh cam format
  handles.calib.Rtx = [ 0 1 0; -1 0 0; 0 0 1]; % matlab image coords style
  handles.calib.camParsMesh{1} = struct('TcV', handles.calib.Rext*handles.calib.Rtx*[handles.calib.tfCamT' + handles.calib.Text'], ...
    'RcM', handles.calib.Rproj * handles.calib.tfCamR, ...
    'fcV', [K(1,1); K(2,2)], ...
    'ccV', [K(1,3); K(2,3)], ...
    'imSizeV', [cam.resolution(2); cam.resolution(1)] );
  
end

%% idx slider
handles.sldFrames.Max = handles.bag.AvailableTopics{camTopic,'NumMessages'};
if handles.sldFrames.Value > handles.sldFrames.Max
  handles.sldFrames.Value = 1;
end
handles.sldFrames.Min = 1;
handles.sldFrames.SliderStep = [(1/handles.sldFrames.Max) (1/handles.sldFrames.Max)];
%% load
handles = LoadFrame(handles.sldFrames.Value, handles);
handles = ShowFrame(handles, 1);
toc;

function handles = LoadFrame(idFrame, handles)
handles.idFrame = round(idFrame);
SaveCurrentFrame(handles)
%% img
handles.msgData = readMessages(handles.cam, handles.idFrame); handles.msgData = handles.msgData{1};
handles.imgRaw = readImage(handles.msgData);
if size(handles.imgRaw,3) == 3
  imgGray = im2double(rgb2gray(handles.imgRaw));
else
  imgGray = im2double(handles.imgRaw);
end
%% undist
if isempty(handles.calib)
  handles.imgGray = imgGray;
  warning('no calibration to perform undistortion!');
else
  handles.imgGray = undistortImage(imgGray,handles.calib.camParams);
end
%% anot
tp = fileparts(handles.camTopic);
top = replace(tp(2:end),'/','_');
handles.frameName = sprintf('%s_f%05d',top,handles.idFrame);
handles.framePath = [fullfile(handles.project.anotPath,top,handles.frameName) '.png'];
if exist(handles.framePath,'file')
  handles.imgAnot = imread(handles.framePath);
else
  handles.imgAnot = zeros(size(handles.imgGray),'uint8');
end
handles.anotLoaded = 1;
%% show
if 0
  handles.imgShow = handles.imgGray;
else
  handles.imgShow = histeq(handles.imgGray);
end
handles = ShowFrame(handles);
title(replace(handles.frameName,'_','-'));
handles.prevAnot = [];  % clear undo
handles.nextAnot = [];
handles.sldFrames.Value = idFrame;
drawnow;

function SaveCurrentFrame(handles)
%% save
if handles.anotLoaded
  if ~exist(fileparts(handles.framePath),'dir')
    mkdir(fileparts(handles.framePath));
  end
  if any(handles.imgAnot(:)>0)
    imwrite(uint8(handles.imgAnot), handles.labelColors, handles.framePath);
    fprintf('Annotation saved to %s.\n',handles.framePath);
  end
end

function handles = ShowFrame(handles, init)
%% compose overlay
if nargin<2
  init = 0;
end
%
imgAnotColor = ind2rgb(handles.imgAnot,handles.labelColors);
if handles.project.optOverAlpha == 0
  frm = undistortImage(handles.imgRaw,handles.calib.camParams);
else
  frm = imoverlayc(handles.imgShow, imgAnotColor,  handles.project.optOverAlpha);
end
if init
  handles.hPlot = imshow(frm); % slow
else
  try
    handles.hPlot.CData = frm;  % fast rendering
  catch
    handles.hPlot = imshow(frm);
  end
end

function calib = ReadCalibration(calibPath,handles)
%% camera params
calib.camChain = YAML.read(calibPath);

%% camera to robot pose 
calib.tfCamR = axang2rotm(calib.camChain.camfix.rotation); % manual correction
calib.tfCamEA = rotm2eul(calib.tfCamR);
calib.tfCamT = calib.camChain.camfix.translation; % + calib.tfCamT
calib.Rproj = [0 -1 0; 0 0 -1; 1 0 0]; % change  axes for projection pose (XY image plane, Z depth)

disp(calib.camChain);

function model = ReadModel(modelPath, handles)
%%
fprintf('Reading model %s ... ',modelPath); tic;
[~,~,fext] = fileparts(modelPath);
if strcmp(fext,'.pcd')
  %% point cloud
  model.data = pcread(modelPath);
  % figure; pcshow(model.data);
  model.vtx = double(model.data.Location(:,1:3));
  model.vtxColor = double(model.data.Color)/255;
  model.vtxLabels = [];
  model.tri = [];
  model.triColor = [];
elseif strcmp(fext,'.ply')
  %% mesh
  model.data = pcread(modelPath);
  model.vtx = double(model.data.Location(:,1:3));
  model.vtxColor = double(model.data.Color);
  model.vtxLabels = [];
  model.tri = [];
  model.triColor = [];  
%   [tri, vtx, model.data, model.name] = ply_read(modelPath,'tri');
%   model.vtx = vtx';
%   model.tri = tri';
%   model.vtxColor = [model.data.vertex.red model.data.vertex.green model.data.vertex.blue];
%   model.vtxLabels = []; %model.data.vertex.alpha;
%   model.triColor = [];
  % figure; trimesh(model.tri,model.vtx(:,1),model.vtx(:,2),model.vtx(:,3)); axis equal;
end
toc;
%% color point cloud matrix
model.ptXcol = [model.vtx model.vtxColor];
model.ptXh = [model.vtx, ones(size(model.vtx,1),1)];
model.ptBox = [min(model.vtx,[],1); max(model.vtx,[],1)];

%% top view
tRes = [480 640];
tK = [tRes(1) 0 tRes(1)/2; 0 tRes(2) tRes(2)/2; 0 0 1];
tPos = [mean(model.ptBox(:,1)); mean(model.ptBox(:,2)); model.ptBox(2,1)-model.ptBox(1,1)];
tRot = diag([1 -1 -1]); %eul2rotm([0,0,pi]);
tP = tK*tRot*[eye(3) -tPos];
% project the points to create an image
[model.imTop, model.dmTop] = points2Image(model.ptXcol, tRes, tP, [], 2, 1, true);
model.imTop = uint8(model.imTop);
%%
figure; set(gcf,'Name','Top view (points)');
subplot(1,2,1); imshow(model.imTop); title('render');
subplot(1,2,2); imagesc(model.dmTop); axis image; colormap jet; colorbar; title('dmap');
figure(handles.figAnot);

function [P, R, T, camShift, msgOdometry, To,Ro, txtCam, txtPath] = GetCameraPose(handles,fromTxt)
if ~exist('fromTxt','var')
  fromTxt = 1;
end
%% vehicle pose
camTime = handles.msgData.Header.Stamp.seconds + handles.calib.camChain.camfix.msg_time_shift;
tdif = handles.bagOdometry.MessageList{:,1} - camTime;
[camShift, oidx] = min(abs(tdif));
camShift = tdif(oidx);
%% start from bag stamp
if camShift>0 && oidx>=2
  oidx = oidx - 1;
end
%% check real timestamp
if oidx<length(handles.gtOdometry)-1
  while handles.gtOdometry{oidx+1}.Header.Stamp.seconds<camTime && oidx<length(handles.gtOdometry)-1
    oidx = oidx +1;
  end
end
while handles.gtOdometry{oidx}.Header.Stamp.seconds>camTime && oidx>=2
  oidx = oidx -1;
end
oidx = min(oidx,length(handles.gtOdometry)-1);
[To, Ro, g, camShift] = interpose(handles.gtOdometry{oidx}.Header.Stamp.seconds,   handles.gtOdometry{oidx}.Pose.Pose, ...
  handles.gtOdometry{oidx+1}.Header.Stamp.seconds, handles.gtOdometry{oidx+1}.Pose.Pose, camTime);
fprintf('Cam2odo: %.1f ms   \n',camShift*1000);
msgOdometry = handles.gtOdometry{oidx};

K = handles.calib.K;

txtPath = replace(replace(handles.framePath,'/anot/','/cam/'),'.png','_cam.txt');
if fromTxt && exist(txtPath,'file')
  %% read from sfm txt
  txtCam = load(txtPath);
  cam.f = txtCam(1:2); % fx fy
  cam.c = txtCam(3:4); % cx cy
  cam.q = txtCam(5:8); % qw qx qy qz
  cam.t = txtCam(9:11); % tx ty tz
  
  cam.R = quat2rotm(cam.q);
  
  Tcam = [cam.R cam.t'];
  Tcam(4,:) = [0 0 0 1];
%   Treg = [ 0.999981939793 -0.005909791682 -0.001093471074 -0.011107921600
%     0.005909302738 0.999982416630 -0.000450019201 -0.009130358696
%     0.001096111373 0.000443549419 0.999999284744 -0.014796212316
%     0.000000000000 0.000000000000 0.000000000000 1.000000000000];
%   Treg = inv(Treg);
  Treg = eye(4);
  Tout = Tcam * Treg;
  P = K * Tout(1:3,:);
  fprintf('Cam pose loaded from %s\n',txtPath);
  %
  R = Tout(1:3,1:3);
  T = -Tout(1:3,4)'*R;  
  %% update txt
  txtCam(5:8) = rotm2quat(R);
  txtCam(9:11) = Tout(1:3,4);
else
  %% vehicle to camera + ring extrinsics
  handles.calib.tfCamR = eul2rotm(handles.calib.tfCamEA);
  
  Rc = handles.calib.Rext * handles.calib.Rproj * handles.calib.tfCamR;
  Tc = handles.calib.tfCamT*handles.calib.tfCamR' - handles.calib.Text*handles.calib.Rproj'*handles.calib.tfCamR';
  
  R = Rc*Ro';
  T = To' - Ro*handles.calib.Rproj'*eul2rotm([0 0 pi])*handles.calib.Text' + Ro*handles.calib.tfCamR'*handles.calib.tfCamT';
  %Tp = Rp*To' + Rp*Tc';
  
  P = K * R * [eye(3) -T];
end
% figure; clf; hold on; drawcam(Tv,Rv,0.2); axis equal; grid on; drawcam(T,R,0.1);
% xlabel X; ylabel Y; zlabel Z;

function handles = ProjectModel(handles)
%% cam pose
[P, R, T, handles.camShift, handles.msgOdometry] = GetCameraPose(handles);
tic;
if ~isempty(handles.model.tri)
  %% mesh projection
  tform = eye(4);
  tform(1:3,1:3) = R';
  tform(1:3,4) = -T*R;
  ptXt = (tform*handles.model.ptXh')';
  projZrange =  [1e-3; 2000];
  handles.imgAnot = RenderColorMesh(handles.model.tri, ptXt(:,1:3), single(handles.model.vtxColor)/255, ...
    handles.calib.camParsMesh{1}, [handles.calib.cam.resolution(2); handles.calib.cam.resolution(1)], projZrange, 1);
  [handles.projDmap, cx] = RenderDepthMesh(handles.model.tri, ptXt(:,1:3), handles.calib.camParsMesh{1}, ...
    [handles.calib.cam.resolution(2); handles.calib.cam.resolution(1)], projZrange, 1, 0);
  %handles.projPts = sum(isfinite(projDmap(:)));
else
  %% points projection
  %handles.project.optPointSize = 2;
  [projColor, handles.projDmap, handles.projPts] = points2Image(handles.model.ptXcol, ...
    [handles.calib.cam.resolution(2) handles.calib.cam.resolution(1)], P, [], handles.project.optPointSize, 1, true);
end

%%
projColor = uint8(projColor);
if 0
  %% show projection
  act = gcf;
  figure(333); set(gcf,'Name','Current view');
  subplot(1,2,1); imshow(projColor); title('render');
  subplot(1,2,2); imagesc(handles.projDmap); axis image; colormap jet; colorbar; title('dmap');
  figure(act);
end
%% tranform color to labels
handles.imgAnot = rgbmapind(projColor,uint8(handles.labelColors*255)) - 1;
if isempty(handles.model.tri)
  %% fill holes between points using DT
  optMaxTriFill = 5;
  dm = handles.projDmap;
  lbl = handles.imgAnot;
  dm(isinf(dm(:))) = 0;
  dobj = lbl(:)>=10 & lbl(:)~=33;
  dm(~dobj) = 0;
  [px, py, dxy] = find(dm);
  if length(px)>=3
    
    tri = delaunay(px,py);
    tcol = zeros(size(tri,1),1);
    for i = 1:size(tri,1)
      tx = px(tri(i,:));
      ty = py(tri(i,:));
      mdx = max(abs(diff([tx; tx(1)])));
      mdy = max(abs(diff([ty; ty(1)])));
      vcol = [lbl(tx(1),ty(1)) lbl(tx(2),ty(2)) lbl(tx(3),ty(3))];
      dlim = max([2 optMaxTriFill / dxy(tri(i,1))]);
      if mdx>1 && mdy>1 && mdx<dlim && mdy<dlim
        if all(vcol==vcol(1))
          tcol(i) = vcol(1);
        elseif all(vcol>0) && sum(vcol==median(vcol))>=2
          tcol(i) = median(vcol);
        end
      end
    end
    %figure; triplot(tri(tcol>0,:),px,py);
    %% fill polygons
    lset = unique(tcol);
    lset = lset(lset>0);
    for l = lset'
      %%
      li = tcol==l;
      ti = tri(li,:);
      shx = [py(ti(:,1)) px(ti(:,1)) py(ti(:,2)) px(ti(:,2)) py(ti(:,3)) px(ti(:,3))];
      msk = uint8(lbl==l);
      msk = insertShape(msk,'FilledPolygon',shx,'Color',[1 1 1],'Opacity',1,'SmoothEdges',false);
      msk = mean(msk,3);
      msk = bwfill(msk,'holes',8);
      msk = imclose(msk,strel('disk',2));
      lbl(msk(:)>0) = l;
    end
    %% second pass - ground
    optMaxTriFill = 25;
    dm = handles.projDmap;
    dm(isinf(dm(:))) = 0;
    dm(dobj) = 0;
    [px, py, dxy] = find(dm);
    if length(px)>=3
      tri = delaunay(px,py);
      tcol = zeros(size(tri,1),1);
      for i = 1:size(tri,1)
        tx = px(tri(i,:));
        ty = py(tri(i,:));
        mdx = max(abs(diff([tx; tx(1)])));
        mdy = max(abs(diff([ty; ty(1)])));
        vcol = [lbl(tx(1),ty(1)) lbl(tx(2),ty(2)) lbl(tx(3),ty(3))];
        dlim = max([2 optMaxTriFill / dxy(tri(i,1))]);
        if mdx>1 && mdy>1 && mdx<dlim && mdy<dlim
          if all(vcol==vcol(1))
            tcol(i) = vcol(1);
          elseif all(vcol>0) && sum(vcol==median(vcol))>=2
            tcol(i) = median(vcol);
          end
        end
      end
      %figure; triplot(tri(tcol>0,:),px,py);
      %% fill polygons
      lset = unique(tcol);
      lset = lset(lset>0);
      for l = lset'
        %%
        li = tcol==l;
        ti = tri(li,:);
        shx = [py(ti(:,1)) px(ti(:,1)) py(ti(:,2)) px(ti(:,2)) py(ti(:,3)) px(ti(:,3))];
        msk = uint8(lbl==l);
        msk = insertShape(msk,'FilledPolygon',shx,'Color',[1 1 1],'Opacity',1,'SmoothEdges',false);
        msk = mean(msk,3);
        msk = bwfill(msk,'holes',8);
        msk = imclose(msk,strel('disk',2));
        lbl(msk(:)>0) = l;
      end
    end
  end
  
  %% fill borders
  for i = 10:-1:1
    % l
    rz = lbl(:,i)==0;
    lbl(rz,i) = lbl(rz,i+1);
    % r
    rz = lbl(:,end-i+1)==0;
    lbl(rz,end-i+1) = lbl(rz,end-i);
    % t
    rz = lbl(i,:)==0;
    lbl(i,rz) = lbl(i+1,rz);
    % b
    rz = lbl(end-i+1,:)==0;
    lbl(end-i+1,rz) = lbl(end-i,rz);
  end
  handles.imgAnot = imclose(lbl,strel('disk',2));
  handles.imgAnot = medfilt2(handles.imgAnot,[3 3],'symmetric');
end
handles.anotLoaded = 1;
toc;

function handles = TransferFrame(handles, useFlow)
%% move forward
prevAnot = handles.imgAnot;
prevGray = medfilt2(handles.imgGray,[3 3]);
handles = LoadFrame(handles.idFrame+1,handles);

%% transfer
handles.imgAnot = prevAnot;
thisGray = medfilt2(handles.imgGray,[3 3]);

if ~useFlow
  %% img correlation
  thisAnot = prevAnot;
  if handles.idCam == 0
    prevGray = prevGray(1:400,:);  % vehicle out in front cam
    thisGray = thisGray(1:400,:);
  end
  fprintf('Xcorr...'); tic;
  c = normxcorr2(prevGray,thisGray);
  % figure, surf(c), shading flat
  [ypeak, xpeak] = find(c==max(c(:)));
  dy = ypeak-size(prevGray,1);
  dx = xpeak-size(prevGray,2);
  thisAnot = imtranslate(prevAnot,[dx dy],'nearest');
  fprintf('Translation = [%d %d]\n',dx,dy);   toc;
  %% fill borders
  if dx>0
    thisAnot(:,1:dx) = repmat(thisAnot(:,dx+1),1,dx);
  elseif dx<0
    thisAnot(:,end+dx+1:end) = repmat(thisAnot(:,end+dx),1,abs(dx));
  end
  %
  if dy>0
    thisAnot(1:dy,:) = repmat(thisAnot(dy+1,:),dy,1);
  elseif dy<0
    thisAnot(end+dy+1:end,:) = repmat(thisAnot(end+dy,:),abs(dy),1);
  end
else
  %% optical flow - https://github.com/suhangpro/epicflow.git
  fprintf('Optical flow (ETA: 60 s) ...'); tic;
  %flowPath = get_epicflow(prevGray, thisGray);
  flowPath = get_epicflow(thisGray, prevGray);
  flow = readFlowFile(flowPath);
  %figure; imshow(flowToColor(flow));
  toc;
  %% transfer
  thisAnot = zeros(size(prevAnot),'uint8');
  flowX = flow(:,:,1);
  flowY = flow(:,:,2);
  [flowPhi, flowRho] = cart2pol(flowX,flowY);
  
  movpix = abs(flowRho(:))>1;
  thisAnot(~movpix) = prevAnot(~movpix);
  movpix = find(movpix==1)';
  for i = movpix
    %%
    [py, px] = ind2sub(size(thisAnot),i);
    px = round(px + flowX(i));
    py = round(py + flowY(i));
    if px>0 && py>0 && py<size(thisAnot,1) && px<size(thisAnot,2)
      thisAnot(i) = prevAnot(py,px);
    end
  end
end
%figure; imagesc(thisAnot)
handles.imgAnot = thisAnot;
handles = ShowFrame(handles);

%% mser matching
%   [regions,cc] = detectMSERFeatures(thisGray,'ThresholdDelta',4,'RegionAreaRange',[100 1e6]);
%
%   figure; imshow(thisGray); hold on;
%   plot(regions,'showPixelList',true,'showEllipses',false);
%   thisAnot = prevAnot;
%
%% segment matching transfer
% numSegments = 1000;
%
% [prevSeg,numLabelsP] = superpixels(prevRaw, numSegments); % SLIC
% [curSeg,numLabelsC] = superpixels(handles.imgRaw, numSegments); % SLIC
% %% features
% prevFtr = ExtractSegFeatures(prevSeg,prevRaw);
% curFtr = ExtractSegFeatures(curSeg,handles.imgRaw);
%
% %% distance
% dst = zeros(numLabelsP,numLabelsC);
% for i = 1:numLabelsP
%   for j = i+1:numLabelsC
%     delta = prevFtr(i,:) - curFtr(j,:);
%     dst(i,j) = norm(delta);
%   end
% end
%
% handles.imgAnot = imgSeg;
% if 0
%   %%
%   imgSegMapOver = imoverlay(prevRaw,255*uint8(boundarymask(prevSeg)));
%   figure(277); imshow(imgSegMapOver);
%   imgSegMapOver = imoverlay(handles.imgRaw,255*uint8(boundarymask(curSeg)));
%   figure(278); imshow(imgSegMapOver);
% end
%
% function ftr = ExtractSegFeatures(segMap,imgRaw)
% %%
% n = max(segMap(:));
% ftr = zeros(n,5);
% hsv = rgb2hsv(imgRaw);
% imgC1 = hsv(:,:,1);
% imgC2 = hsv(:,:,2);
% imgC3 = hsv(:,:,3);
% [h,w,c] = size(imgRaw);
% [posx, posy] = meshgrid(1:w,1:h);
% posx = posx/w;
% posy = posy/h;
% for i = 1:n
%   %%
%   idx = find(segMap(:)==i);
%   ftr(i,1) = median(posx(idx));
%   ftr(i,2) = median(posy(idx));
%   ftr(i,3) = median(imgC1(idx));
%   ftr(i,4) = median(imgC2(idx));
%   ftr(i,5) = median(imgC3(idx));
% end


% --- Outputs from this function are returned to the command line.
function varargout = AnotForm_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function sldFrames_Callback(hObject, eventdata, handles)
% hObject    handle to sldFrames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = LoadFrame(round(hObject.Value),handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function sldFrames_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sldFrames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in lstClass.
function lstClass_Callback(hObject, eventdata, handles)
% hObject    handle to lstClass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns lstClass contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lstClass


% --- Executes during object creation, after setting all properties.
function lstClass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lstClass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in lstCameras.
function lstCameras_Callback(hObject, eventdata, handles)
% hObject    handle to lstCameras (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = LoadTopic(handles.camList{hObject.Value}, handles);
% Hints: contents = cellstr(get(hObject,'String')) returns lstCameras contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lstCameras
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function lstCameras_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lstCameras (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function sldBrush_Callback(hObject, eventdata, handles)
% hObject    handle to sldBrush (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function sldBrush_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sldBrush (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --------------------------------------------------------------------
function tlbReadBag_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbReadBag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile({'*.bag','ROS bagfile (*.bag)'},'Open bagfile',handles.project.bagPath);
handles.bagPath = [PathName FileName];
handles = LoadBag(handles.bagPath, handles);
guidata(hObject,handles);
fprintf('Loaded %s.\n',handles.bagPath);




% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figAnot_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figAnot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%
handles = guidata(hObject);
hObject.UserData.imgAnot = handles.imgAnot;
hObject.UserData.brushSize = round(handles.sldBrush.Value)-1;
hObject.UserData.drawClass = handles.lstClass.Value-1;
hObject.UserData.imgShow = handles.imgShow;
hObject.UserData.project.optOverAlpha = handles.project.optOverAlpha;
hObject.UserData.labelColors = handles.labelColors;
hObject.UserData.hPlot = handles.hPlot;
hObject.UserData.pos = [];
hObject.UserData.mode = [];
% Set callbacks
thisfig = gcbf();
%set(thisfig,'WindowButtonMotionFcn',@figAnot_WindowButtonMotionFcn);
set(thisfig,'WindowButtonUpFcn',@figAnot_WindowButtonUpFcn);
set(thisfig,'Pointer','crosshair');
figAnot_WindowButtonMotionFcn(hObject, eventdata, handles)

function figAnot_WindowButtonMotionFcn(hObject, eventdata, handles)
%% move
%handles = guidata(hObject);
ax = gca;
npos = round(ax.CurrentPoint);
npos = npos(1,1:2);
%disp(npos);

if all([npos>0 npos(1)<=ax.XLim(2) npos(2)<=ax.YLim(2)])
  hObject.Pointer = 'crosshair';
  
  if ~isempty(hObject.UserData)
    %% drawing
    if isfield(hObject.UserData,'pos')
      ppos = hObject.UserData.pos;
    else
      ppos = [];
    end
    %%
    if strcmp(hObject.SelectionType,'extend')
      if isempty(ppos)
        [hObject.UserData.pos(1), hObject.UserData.pos(2)] = getAbsCoords(handles.axImg);
      else
        %% panning
        [x,y] = getAbsCoords(handles.axImg);
        [x_rel, y_rel] = abs2relCoords(handles.axImg, x, y);
        [cx_rel, cy_rel] = abs2relCoords(handles.axImg, hObject.UserData.pos(1), hObject.UserData.pos(2));
        delta_x_rel = x_rel - cx_rel;
        delta_y_rel = y_rel - cy_rel;
        % set new limits
        [new_xlim(1), new_ylim(1)] = rel2absCoords(handles.axImg, -delta_x_rel, -delta_y_rel);
        [new_xlim(2), new_ylim(2)] = rel2absCoords(handles.axImg, 1-delta_x_rel, 1-delta_y_rel);
        setNewLimits(handles.axImg, new_xlim, new_ylim);
      end
    else
      %% drawing
      hObject.UserData.pos = npos;
      
      if isempty(ppos)
        dpos = 1;
      else
        dpos = norm(npos-ppos);
      end
      r=hObject.UserData.brushSize;
      
      imgAnot = hObject.UserData.imgAnot;
      [h,w] = size(imgAnot);
      if dpos<1
        % no change
        return;
      elseif dpos>1
        %% interpolate
        drp = round([npos; (npos+ppos)/2]);
        npt = round(dpos)+1;
        drp = round(repmat(ppos,npt,1) + repmat(npos-ppos,npt,1).*repmat(linspace(0,1,npt)',1,2));
        %imgLine = insertShape(imgAnot,'Line',[ppos npos],'LineWidth',r);
      else
        % just one
        drp = npos;
      end
      %% draw
      for i = 1:size(drp,1)
        % draw mask into anot
        cm=drp(i,2);
        cn=drp(i,1);
        
        if strcmp(hObject.SelectionType,'normal')
          imgAnot(max(cm-r,1):min(cm+r,h), max(cn-r,1):min(cn+r,w)) = hObject.UserData.drawClass;
        elseif strcmp(hObject.SelectionType,'alt')
          imgAnot(max(cm-r,1):min(cm+r,h), max(cn-r,1):min(cn+r,w))=0;
        end
      end
      
      hObject.UserData.imgAnot = imgAnot;
      
      ShowFrame(hObject.UserData);
    end
  end
  %% show label
  try
    posl = handles.imgAnot(npos(2),npos(1)) + 1;
    handles.txtInfo.String = sprintf('%s [%d:%d]',handles.labelNames{posl}, npos(1),npos(2));
    %handles.txtInfo.ForegroundColor = handles.labelColors(posl,:);
  catch
  end
else
  %% outside
  hObject.Pointer = 'arrow';
end

function figAnot_WindowButtonUpFcn(hObject, eventdata, handles)
%% finish
handles = guidata(hObject);
thisfig = gcbf();
set(thisfig,'WindowButtonUpFcn','');
%set(thisfig,'WindowButtonMotionFcn','');
handles = BeforeEdit(handles);
handles.imgAnot = hObject.UserData.imgAnot;
guidata(hObject,handles);
hObject.UserData = [];

% --------------------------------------------------------------------
function tlbPointCloud_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbPointCloud (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile({'*.pcd','Point cloud file (*.pcd)'; '*.ply','Mesh file (*.ply)'},'Import model',handles.project.modelPath);
handles.project.modelPath = [PathName FileName];
fprintf('Loading %s... ',handles.project.modelPath); tic;
handles.model = ReadModel(handles.project.modelPath, handles);
guidata(hObject,handles);
toc;


% --------------------------------------------------------------------
function tlbProjectModel_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbProjectModel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.project.optPointSize = 1;
handles = ProjectModel(handles);
handles = ShowFrame(handles);
guidata(hObject,handles);


% --------------------------------------------------------------------
function tlbReadCalib_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbReadCalib (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile({'*.yaml','Kalibr cam file (*.yaml)'},'Read calibration',handles.project.calibPath);
handles.project.calibPath = [PathName FileName];
handles.calib = ReadCalibration(handles.project.calibPath,handles);
guidata(hObject,handles);
fprintf('Loaded %s.\n',handles.project.calibPath);



function txtInfo_Callback(hObject, eventdata, handles)
% hObject    handle to txtInfo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtInfo as text
%        str2double(get(hObject,'String')) returns contents of txtInfo as a double


% --- Executes during object creation, after setting all properties.
function txtInfo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtInfo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function tlbTransfer_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbTransfer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

modifiers = get(handles.figAnot,'currentModifier');
handles = TransferFrame(handles,ismember('shift',modifiers));
guidata(hObject,handles);


% --- Executes on slider movement.
function sldOpacity_Callback(hObject, eventdata, handles)
% hObject    handle to sldOpacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.project.optOverAlpha = hObject.Value;
ShowFrame(handles);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function sldOpacity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sldOpacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function handles = BeforeEdit(handles)
%% save for undo
handles.prevAnot = [ {handles.imgAnot}, handles.prevAnot];
if length(handles.prevAnot)>=100
  handles.prevAnot = handles.prevAnot(1:100);
end

function handles = Undo(handles)
%% undo last action
if ~isempty(handles.prevAnot)
  handles.nextAnot = [ {handles.imgAnot}, handles.nextAnot];
  handles.imgAnot = handles.prevAnot{1};
  if length(handles.prevAnot)<=1
    handles.prevAnot = [];
  else
    handles.prevAnot = handles.prevAnot(2:end);
  end
  ShowFrame(handles);
end

function handles = Redo(handles)
%% undo last action
if ~isempty(handles.nextAnot)
  handles.prevAnot = [ {handles.imgAnot}, handles.prevAnot];
  handles.imgAnot = handles.nextAnot{1};
  if length(handles.nextAnot)<=1
    handles.nextAnot = [];
  else
    handles.nextAnot = handles.nextAnot(2:end);
  end
  ShowFrame(handles);
end

% --------------------------------------------------------------------
function tlbUndo_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbUndo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = Undo(handles);
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbFill_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbFill (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgAnot)
  %%
  handles = BeforeEdit(handles);
  pos = round(ginput(1));
  poslbl = handles.imgAnot(pos(2),pos(1));
  if poslbl==0
    mask = handles.imgAnot > 0;
  else
    mask = handles.imgAnot ~= poslbl;
  end
  nmask = imfill(mask,pos([2 1]),4);
  lbl = handles.lstClass.Value-1;
  handles.imgAnot(find(nmask(:)-mask(:)==1)) = lbl;
  ShowFrame(handles);
  guidata(hObject,handles);
end

% --------------------------------------------------------------------
function tlbFillImage_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbFillImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%
handles = BeforeEdit(handles);
pos = round(ginput(1));
drawnow;
%thisRaw = imfilter(handles.imgGray,fspecial('gaussian',[5 5],1));
thisRaw = medfilt2(handles.imgGray,[5 5]);
fprintf('Floodfill...'); tic;
filpix = floodfill(repmat(uint8(thisRaw*255),[1 1 3]),pos,round(handles.sldBrush.Value));
toc;
filim = zeros(size(handles.imgAnot));
filim(filpix) = 1;
filim = imclose(filim,strel('disk',5));
filim = filim & (handles.imgAnot==0);
handles.imgAnot(filim(:)) = handles.lstClass.Value-1;
ShowFrame(handles);
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbSaveProject_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbSaveProject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.project.workPath = uigetdir(handles.project.workPath,'Save project workspace');
handles.project.wsPath = fullfile(handles.project.workPath,'workspace.yaml');
YAML.write(handles.project.wsPath,handles.project);
fprintf('Workspace saved to %s.\n',handles.project.wsPath);

% --------------------------------------------------------------------
function tlbOpenProject_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbOpenProject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
workPath = uigetdir(handles.project.workPath,'Open project workspace');
if length(workPath)==1, return; end
wsPath = fullfile(workPath,'workspace.yaml');
fprintf('*** Loading workspace %s ***\n',wsPath); tic;
%% load
handles.project = YAML.read(wsPath);
% remap paths to folder location
handles.project.calibPath = replace(handles.project.calibPath, fileparts(handles.project.workPath), fileparts(workPath));
handles.project.bagPath = replace(handles.project.bagPath, fileparts(handles.project.workPath), fileparts(workPath));
handles.project.modelPath = replace(handles.project.modelPath, fileparts(handles.project.workPath), fileparts(workPath));
handles.project.workPath = workPath;
%
if ~isempty(handles.project.calibPath)
  try
    handles.calib = ReadCalibration(handles.project.calibPath,  handles);
  catch exc
    warning(exc.message);
  end
end
guidata(hObject,handles);
%
if ~isempty(handles.project.bagPath)
  try
    handles = LoadBag(handles.project.bagPath,handles);
  catch exc
    warning(exc.message);
  end
end
guidata(hObject,handles);
%
if ~isempty(handles.project.modelPath)
  try
    handles.model = ReadModel(handles.project.modelPath,handles);
  catch exc
    warning(exc.identifier);
  end
end
guidata(hObject,handles);
fprintf('*** Workspace loaded %s *** ',handles.project.workPath); toc;


% --- Executes when user attempts to close figAnot.
function figAnot_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figAnot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles)
   SaveCurrentFrame(handles);
end
% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on scroll wheel click while the figure is in focus.
function figAnot_WindowScrollWheelFcn(hObject, eventdata, handles)
% hObject    handle to figAnot (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	VerticalScrollCount: signed integer indicating direction and number of clicks
%	VerticalScrollAmount: number of lines scrolled for each click
% handles    structure with handles and user data (see GUIDATA)
% zoom in to the current point with the mouse wheel
modifiers = get(handles.figAnot,'currentModifier');
if ismember(modifiers,'control')
  %% brush size
  newSize = round(handles.sldBrush.Value - eventdata.VerticalScrollCount);
  handles.sldBrush.Value = min([max([handles.sldBrush.Min newSize]) handles.sldBrush.Max]);
  handles.txtInfo.String = sprintf('BRUSH = %d pix',handles.sldBrush.Value);
else
  %% zoom
  wheel_zoomFactor = 20;
  
  h = hittest(gcf);
  if isempty(h), return, end
  try
    switch get(h,'Type')
      case 'axes'
        currAx = h;
      case 'image'
        currAx = get(h,'Parent');
      case 'line'
        currAx = get(h,'Parent');
      case 'patch'
        currAx = get(h,'Parent');
      otherwise
        return
    end
  catch
    % in Matlab2015 and newer an opaque object can be returned by the
    % hittest, which results in an error when trying to apply the 'get'
    % method on it
    return
  end
  if ~any(currAx == handles.axImg), return, end
  [x,y] = getAbsCoords(currAx);
  if ~coordsWithinLimits(currAx,x,y), return, end
  [x_rel, y_rel] = abs2relCoords(currAx, x, y);
  sc = eventdata.VerticalScrollCount;
  zoomFactor = (abs(sc)*(1+wheel_zoomFactor/100))^sign(sc);
  if zoomFactor ~= 0 % could happen when fast scrolling
    new_xlim_rel = ([0,1] - x_rel) * zoomFactor + x_rel;
    new_ylim_rel = ([0,1] - y_rel) * zoomFactor + y_rel;
    [new_xlim(1), new_ylim(1)] = rel2absCoords(handles.axImg, new_xlim_rel(1), new_ylim_rel(1));
    [new_xlim(2), new_ylim(2)] = rel2absCoords(handles.axImg, new_xlim_rel(2), new_ylim_rel(2));
    setNewLimits(handles.axImg, new_xlim, new_ylim);
  end
end
%
function [x_rel, y_rel] = abs2relCoords(h_ax, x, y)
XLim = get(h_ax, 'xlim');
if strcmp(get(h_ax, 'XScale'), 'log')
  x_rel = ( log(x) - log(XLim(1)) ) / ( log(XLim(2)) - log(XLim(1)) );
else
  x_rel = (x-XLim(1))/(XLim(2)-XLim(1));
end
YLim = get(h_ax, 'ylim');
if strcmp(get(h_ax, 'YScale'), 'log')
  y_rel = ( log(y) - log(YLim(1)) ) / ( log(YLim(2)) - log(YLim(1)) );
else
  y_rel = (y-YLim(1))/(YLim(2)-YLim(1));
end
function [x, y] = rel2absCoords(h_ax, x_rel, y_rel)
XLim = get(h_ax, 'xlim');
if strcmp(get(h_ax, 'XScale'), 'log')
  x = exp( x_rel * ( log(XLim(2)) - log(XLim(1)) ) + log(XLim(1)) );
else
  x = x_rel*diff(XLim)+XLim(1);
end
YLim = get(h_ax, 'ylim');
if strcmp(get(h_ax, 'YScale'), 'log')
  y = exp( y_rel * ( log(YLim(2)) - log(YLim(1)) ) + log(YLim(1)) );
else
  y = y_rel*diff(YLim)+YLim(1);
end
function [x, y, z] = getAbsCoords(h_ax)
crd = get(h_ax, 'CurrentPoint');
x = crd(2,1);
y = crd(2,2);
z = crd(2,3);
function tf = coordsWithinLimits(h_ax, x, y)
% check if the given point (x,y) is within the limits of the axis h_ax
XLim = get(h_ax, 'xlim');
YLim = get(h_ax, 'ylim');
tf = x>XLim(1) && x<XLim(2) && y>YLim(1) && y<YLim(2);
function setNewLimits(ax, xlim, ylim)
validX = ~any(isnan(xlim)) && ~any(isinf(xlim)) && diff(xlim)>0;
if strcmp(get(ax,'XScale'),'log')
  validX = validX && xlim(1) ~= 0;
end
if validX
  set(ax, 'Xlim', xlim);
else
  if strcmp(tX.Running, 'off')
    old_color = get(ax, 'YColor');
    set(ax,'YColor','r');
    tX.TimerFcn = @(x,y)set(ax,'YColor',old_color);
    start(tX);
  end
end
validY = ~any(isnan(ylim)) && ~any(isinf(ylim)) && diff(ylim)>0;
if strcmp(get(ax,'YScale'),'log')
  validY = validY && ylim(1) ~= 0;
end
if validY
  set(ax, 'Ylim', ylim);
else
  if strcmp(tY.Running, 'off')
    old_color = get(ax, 'XColor');
    set(ax,'XColor','r');
    tY.TimerFcn = @(x,y)set(ax,'XColor',old_color);
    start(tY);
  end
end

% --- Executes on key press with focus on figAnot or any of its controls.
function figAnot_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figAnot (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
%%
camMoveDelta = 0.001*handles.sldBrush.Value;
modifiers = get(handles.figAnot,'currentModifier');
switch eventdata.Key
  case 'backslash'
    %% opacity
    if isfield(handles,'prevOpacity') && handles.prevOpacity >0
      handles.sldOpacity.Value = handles.prevOpacity;
      handles.project.optOverAlpha = handles.prevOpacity;
      handles.prevOpacity = 0;
    else
      handles.prevOpacity = handles.project.optOverAlpha;
      handles.sldOpacity.Value = 0;
      handles.project.optOverAlpha = 0;
    end
    ShowFrame(handles);
    guidata(hObject,handles);
  case 'backquote'
    %% change mode
    handles.camMode = mod(handles.camMode+1,3);
    switch handles.camMode
      case 0
        fprintf('Camera control: 2D (image translation and rotation)\n');
        handles.txtInfo.String = 'CAM: 2D';
      case 1
        fprintf('Camera control: 3DT (world translation) + projection\n');
        handles.txtInfo.String = 'CAM: 3D-T';
      case 2
        fprintf('Camera control: 3DR (world rotation) + projection\n');
        handles.txtInfo.String = 'CAM: 3D-R';
    end
    guidata(hObject,handles);
  case '0'
    %% reset camera
    handles = BeforeEdit(handles);
    handles.calib.tfCamR = axang2rotm(handles.calib.camChain.camfix.rotation); % manual correction
    handles.calib.tfCamEA = rotm2eul(handles.calib.tfCamR);
    handles.calib.tfCamT = handles.calib.camChain.camfix.translation;
    handles = ProjectModel(handles);
    handles = ShowFrame(handles);
    guidata(hObject,handles);
  case 'p'
    %% pick label under cursor
    try
      ax = gca;
      npos = round(ax.CurrentPoint);
      npos = npos(1,1:2);
      handles.lstClass.Value = handles.imgAnot(npos(2),npos(1)) + 1;
    catch
    end
  case 'w'
    %% move anot up one line
    handles = BeforeEdit(handles);
    switch handles.camMode
      case 0
        handles.imgAnot(1:end-1,:) = handles.imgAnot(2:end,:);
      case 1
        handles.calib.tfCamT(1) = handles.calib.tfCamT(1) + camMoveDelta;
        handles = ProjectModel(handles);
      case 2
        handles.calib.tfCamEA(2) = handles.calib.tfCamEA(2) - camMoveDelta;
        handles = ProjectModel(handles);
    end
    handles = ShowFrame(handles);
    guidata(hObject,handles);
  case 's'
    %% move anot down one line
    handles = BeforeEdit(handles);
    switch handles.camMode
      case 0
        handles.imgAnot(2:end,:) = handles.imgAnot(1:end-1,:);
      case 1
        handles.calib.tfCamT(1) = handles.calib.tfCamT(1) - camMoveDelta;
        handles = ProjectModel(handles);
      case 2
        handles.calib.tfCamEA(2) = handles.calib.tfCamEA(2) + camMoveDelta;
        handles = ProjectModel(handles);
    end
    handles = ShowFrame(handles);
    guidata(hObject,handles);
  case 'a'
    %% move anot left one line
    handles = BeforeEdit(handles);
    switch handles.camMode
      case 0
        handles.imgAnot(:,1:end-1) = handles.imgAnot(:,2:end);
      case 1
        handles.calib.tfCamT(2) = handles.calib.tfCamT(2) - camMoveDelta;
        handles = ProjectModel(handles);
      case 2
        handles.calib.tfCamEA(1) = handles.calib.tfCamEA(1) + camMoveDelta;
        handles = ProjectModel(handles);
    end
    
    handles = ShowFrame(handles);
    guidata(hObject,handles);
  case 'd'
    %% move anot right one line
    handles = BeforeEdit(handles);
    switch handles.camMode
      case 0
        handles.imgAnot(:,2:end) = handles.imgAnot(:,1:end-1);
      case 1
        handles.calib.tfCamT(2) = handles.calib.tfCamT(2) + camMoveDelta;
        handles = ProjectModel(handles);
      case 2
        handles.calib.tfCamEA(1) = handles.calib.tfCamEA(1) - camMoveDelta;
        handles = ProjectModel(handles);
    end
    handles = ShowFrame(handles);
    guidata(hObject,handles);
    
  case 'e'
    %% rotate cw
    handles = BeforeEdit(handles);
    switch handles.camMode
      case 0
        handles.imgAnot = imrotate(handles.imgAnot,-handles.sldBrush.Value/10,'nearest','crop');
      case 1
        handles.calib.tfCamT(3) = handles.calib.tfCamT(3) - camMoveDelta;
        handles = ProjectModel(handles);
      case 2
        handles.calib.tfCamEA(3) = handles.calib.tfCamEA(3) + camMoveDelta;
        handles = ProjectModel(handles);
    end
    handles = ShowFrame(handles);
    guidata(hObject,handles);
  case 'q'
    %% rotate ccw
    handles = BeforeEdit(handles);
    switch handles.camMode
      case 0
        handles.imgAnot = imrotate(handles.imgAnot,handles.sldBrush.Value/10,'nearest','crop');
      case 1
        handles.calib.tfCamT(3) = handles.calib.tfCamT(3) + camMoveDelta;
        handles = ProjectModel(handles);
      case 2
        handles.calib.tfCamEA(3) = handles.calib.tfCamEA(3) - camMoveDelta;
        handles = ProjectModel(handles);
    end
    handles = ShowFrame(handles);
    guidata(hObject,handles);
    
  case 'h'
    %% time back
    handles.calib.camChain.camfix.msg_time_shift = handles.calib.camChain.camfix.msg_time_shift - camMoveDelta;
    fprintf('Time shift = %f\n',handles.calib.camChain.camfix.msg_time_shift);
    handles = ProjectModel(handles);
    handles = ShowFrame(handles);
    guidata(hObject,handles);
  case 'j'
    %% time fwd
    handles.calib.camChain.camfix.msg_time_shift = handles.calib.camChain.camfix.msg_time_shift + camMoveDelta;
    fprintf('Time shift = %f\n',handles.calib.camChain.camfix.msg_time_shift);
    handles = ProjectModel(handles);
    handles = ShowFrame(handles);
    guidata(hObject,handles);
    
    
  case 'z'
    %% prev frame
    if ismember('shift',modifiers)
      skip = 10;
    else
      skip = 1;
    end
    if handles.idFrame>=skip+1
      handles = LoadFrame(handles.idFrame-skip,handles);
      guidata(hObject,handles);
    end
  case 'x'
    %% next frame
    if ismember('shift',modifiers)
      skip = 10;
    else
      skip = 1;
    end
    if handles.idFrame<=handles.sldFrames.Max-skip
      handles = LoadFrame(handles.idFrame+skip,handles);
      guidata(hObject,handles);
    end
  case 'c'
    %% copy to next
    prevAnot = handles.imgAnot;
    handles = LoadFrame(handles.idFrame+1,handles);
    handles.imgAnot = prevAnot;
    handles = ShowFrame(handles);
    guidata(hObject,handles);
  case 't'
    %% transfer to next
    if handles.idFrame<=handles.sldFrames.Max
      handles = TransferFrame(handles,ismember('shift',modifiers));
      guidata(hObject,handles);
    end
  case 'f'
    %% fill
    tlbFill_ClickedCallback(hObject, eventdata, handles);
  case 'g'
    %% fill im
    tlbFillImage_ClickedCallback(hObject, eventdata, handles)
  case 'u'
    %% undo
    handles = Undo(handles);
    guidata(hObject,handles);
    
  case 'i'
    %% redo
    handles = Redo(handles);
    guidata(hObject,handles);
  case '1'
    %% reset zoom
    handles.axImg.XLim = 0.5 + [0 size(handles.imgGray,2)];
    handles.axImg.YLim = 0.5 + [0 size(handles.imgGray,1)];
    
  case 'l'
    %% draw line
    handles = BeforeEdit(handles);
    [px1, py1] = ginput(1);
    while ~isempty(px1)
      [px2, py2] = ginput(1);
      if ~isempty(px2)
        msk = zeros(size(handles.imgAnot));
        msk = insertShape(msk,'Line',[px1 py1 px2 py2],'Color',[1 1 1],'Opacity',1,'SmoothEdges',false);
        msk = mean(msk,3);
        msk = imdilate(msk,ones(1+round(handles.sldBrush.Value/10)));
        handles.imgAnot(msk(:)>0) = handles.lstClass.Value - 1;
        handles = ShowFrame(handles);
      end
      px1 = px2; py1 = py2;
    end
    guidata(hObject,handles);
    
end


% --------------------------------------------------------------------
function tlbSegFilter_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbSegFilter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%
handles = BeforeEdit(handles);
%%
fprintf('Segmenting...'); tic;
numSegments = 2000;
thisGray = medfilt2(handles.imgGray,[3 3]);
[handles.imgAnot,imgSegMap] = imsegfilt(thisGray,handles.imgAnot,numSegments,0.75);
handles.imgAnot = imclose(handles.imgAnot,strel('disk',2));
%handles.imgAnot(boundarymask(imgSegMap)) = 5;
toc;
handles = ShowFrame(handles);
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbExport_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbExport (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
startFrame = handles.idFrame;
export.project = 0;
export.gta = 0;
export.gtr = 0;
export.undist = 0;
export.over = 0;
export.cam = 0;
export.vid = 0;
export.dmap = 2;
export.sgm = 0;

%% all cameras from the same frame
for idCam = 0:2:9
  %% load camera
  handles = LoadTopic(sprintf('/uvc_camera/cam_%d/image_raw',idCam),handles);
  handles = LoadFrame(startFrame,handles);
  
  exPath = replace(handles.framePath,'/anot/','/export/');
  if ~exist(fileparts(exPath),'dir')
    mkdir(fileparts(exPath));
  end
  %% video
  if export.vid
    %vidPath = replace(exPath,'.png','.avi');
    if export.gtr
      vidPath = sprintf('%s/export/cam%d_gtr_semantic_%d.avi',handles.project.workPath,handles.idCam,handles.idFrame);
    else
      vidPath = sprintf('%s/export/cam%d_gta_semantic_%d.avi',handles.project.workPath,handles.idCam,handles.idFrame);
    end
    vid = VideoWriter(vidPath);
    vid.FrameRate = 3;
    vid.Quality = 100;
    open(vid);
  end
  %% read mask
  tp = fileparts(handles.camTopic);
  top = replace(tp(2:end),'/','_');
  maskPath = [fullfile(handles.project.anotPath,top,sprintf('%s_f%05d',top,0)) '.png'];
  if exist(maskPath,'file')
    imgMask = imread(maskPath);
  else
    imgMask = zeros(size(handles.imgAnot));
  end
  %% read reduced map
  reduced = ReadLabels('yaml/reduced/');
  camsT = zeros(100,3);
  camsR = zeros(100,3);
  fid = 1;
  
  %% enumerate
  while handles.idFrame<1020 %handles.sldFrames.Max-10
    %% go through frames
    if export.project
      handles.project.optPointSize = 1;
      handles = ProjectModel(handles);
    end
    handles = ShowFrame(handles);
    drawnow;
    [dname,fname,ext] = fileparts(replace(handles.framePath,'/anot/','/export/'));
    exPath = fullfile(dname,fname);
    %% cam pose
    if export.cam
      cam = struct();
      [cam.P, cam.R, cam.T, cam.Stamp.FrameOffset, msg, To,Ro, txtCam, txtPath] = GetCameraPose(handles);
      cam.Stamp.Sec = msg.Header.Stamp.Sec;
      cam.Stamp.Nsec = msg.Header.Stamp.Nsec;
      YAML.write([exPath '_cam.yml'],cam);
      % update txt
      ft = fopen(txtPath,'w');
      fprintf(ft,'%.16g ',txtCam);
      fclose(ft);
      save(txtPath,'txtCam','-ascii');
      %
      camsT(fid,:) = cam.T;
      camsR(fid,:) = cam.R(:,3);
    end
    %% undist color img
    if isempty(handles.calib)
      imgUndist = handles.imgRaw;
      warning('no calibration to perform undistortion!');
    else
      imgUndist = undistortImage(handles.imgRaw,handles.calib.camParams);
    end
    if export.undist
      imwrite(imgUndist, [exPath '_undist.png']);
    end
    if any(handles.imgAnot(:)>0)
      %% anot with IDs
      imgAnot = zeros(size(handles.imgAnot),'uint8');
      imgReduced = zeros(size(handles.imgAnot),'uint8');
      cmap = zeros(256,3);
      for i=1:length(handles.labelIDs)-1
        imgAnot(handles.imgAnot(:)==i) = handles.labelIDs(i+1);
        cmap(handles.labelIDs(i+1)+1,:) = handles.labelColors(i+1,:);
        imgReduced(handles.imgAnot(:)==i) = reduced.mapping(handles.labelIDs(i+1));
      end
      imgAnot(handles.imgAnot(:)==0) = 10; % background-generic
      imgAnot = medfilt2(imgAnot,[3 3],'symmetric');
      imgAnot(imgMask(:)>0) = imgMask(imgMask(:)>0);
      if export.gta
        imwrite(uint8(imgAnot), cmap, [exPath '_gta.png']);
      end
      %% reduce non-ground to generic
      % ngi = find(imgAnot(:)>=10);
      % imgAnot(ngi) = floor(imgAnot(ngi)/10)*10;
      imgReduced(handles.imgAnot(:)==0) = 11; % background
      imgReduced = medfilt2(imgReduced,[3 3],'symmetric');
      imgReduced(imgMask(:)>0) = 0;
      if export.gtr
        imwrite(uint8(imgReduced), reduced.labelColors, [exPath '_gtr.png']);
      end
      %% over
      if export.gtr
        imgAnotColor = ind2rgb(imgReduced,reduced.labelColors);
      else
        imgAnotColor = ind2rgb(handles.imgAnot,handles.labelColors);
      end
      if export.over
        imgExport = imoverlay(handles.imgShow, imgAnotColor,  handles.project.optOverAlpha);
        imwrite(imgExport, [exPath '_over.png']); 
      else
        imgExport = imgUndist;
      end
    else
      imgExport = imgUndist;
    end
    %% dmap
    if export.dmap>=1
      %%
      handles.project.optPointSize = 2;
      handles = ProjectModel(handles);
      dmap = handles.projDmap;
      exPathDmap = exPath;
      fx_base = handles.calib.K(1,1)*0.03;
      if export.dmap>=2
        %% simulate disparity quantisation
        dsp = fx_base./dmap;
        dsp = round(dsp);
        dsp(dsp(:)>31) = 31;
        % figure; imagesc(dsp); axis image
        dmap = fx_base./dsp;
        % figure; imagesc(dmap); axis image
        exPathDmap = [exPath '_q32'];
      end
      %%
      imgDepth = dmap/10;
      imgDepth(imgDepth(:)>1) = 1;
      imgDepth(imgDepth(:)<0) = 0;
      imgDepth = uint8(255*imgDepth);
      %imgDepthMap = ind2rgb(gray2ind(imgDepth,255),jet(255));
      imwrite(imgDepth, jet(255), [exPathDmap '_dmap.png']); 
      %% pcl
      xyzPoints = dmap2pcl(dmap,handles.calib.K);
      pcl = pointCloud(xyzPoints);
      dd = fx_base./xyzPoints(:,3);
      dsigma = (xyzPoints(:,3) - fx_base./(dd+1)) / 3;
      dsigma(dsigma>1) = 1;
      pcl.Color = zeros([length(dsigma) 3],'uint8');
      pcl.Color(:,1) = 255*dsigma;
      pcl.Color(:,2) = 255*(1-dsigma);
      pcl.Color(:,3) = 255*dsigma;
      
      [cam.P, cam.R, cam.T] = GetCameraPose(handles);
      cam.tx = eye(4);
      cam.tx(1:3,1:3) = cam.R;
      cam.tx(4,1:3) = cam.T;
      pclt = pctransform(pcl,affine3d(cam.tx));
      pcwrite(pclt,[exPathDmap '_pcl.ply'],'Encoding','binary');
    end
    %% video
    if export.vid
      imgExport = insertText(imgExport, [0 0], sprintf('C%d#%04d',handles.idCam,handles.idFrame));
      writeVideo(vid,imgExport);
    end
    %% go to next
    handles = LoadFrame(handles.idFrame+10, handles);
    handles.sldFrames.Value = handles.idFrame;
    guidata(hObject,handles);
    fid = fid+1;
  end
  %%
  if export.vid
    close(vid);
    fprintf('Written video %s.\n',vidPath);
  end
  %% write poses
  if export.cam
    camsT = camsT(1:fid-1,:);
    camsR = camsR(1:fid-1,:);
    pclCams = pointCloud(camsT,'Normal',camsR);
    pcwrite(pclCams,sprintf('%s/export/cam%d_poses_%d.ply',handles.project.workPath,handles.idCam,startFrame));
    %figure; pcshow(pclCamls);
  end
  %%
  if export.sgm
    imgLeft = imgUndist;
    %% load right camera
    handles = LoadTopic(sprintf('/uvc_camera/cam_%d/image_raw',idCam+1),handles);
    handles = LoadFrame(startFrame,handles);
    imgRight = undistortImage(handles.imgRaw,handles.calib.camParams);
    %% sgm
    dispSGM = disparity(imgLeft,imgRight);
  end  
  
end

% --------------------------------------------------------------------
function tlbExportCloud_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbExportCloud (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% export camera poses
figure(112); clf;
hold on; axis equal; grid on; zlabel Z;
figure(handles.figAnot);
for startFrame = (handles.idFrame:10:handles.idFrame+100)
  for idCam = 0:9
    %% load camera
    handles = LoadTopic(sprintf('/uvc_camera/cam_%d/image_raw',idCam),handles);
    handles = LoadFrame(startFrame,handles);
    drawnow;
    %% cam pose
    [cam.P, cam.R, cam.T, cam.Stamp.FrameOffset, msg, To, Ro] = GetCameraPose(handles,0);
    %
    figure(112);
    plot3(To(1),To(2),To(3),'b*'); hold on; 
    %cam.fig = plotCamera('Location',-cam.T*cam.R','Orientation',cam.R','Size',0.01,'Opacity',0.2);
    cam.fig = plotCamera('Location',cam.T,'Orientation',cam.R,'Size',0.02,'Opacity',0.2,'Label',sprintf('C%d',idCam));
    drawnow;
    figure(handles.figAnot);
  end
end
%%
%keyboard;



%% export cloud
% [FileName,PathName] = uiputfile({'*.ply','Mesh file (*.ply)'},'Export model',handles.project.modelPath);
% exportPath = [PathName FileName];
% %% tranform color to labels
% col = reshape(handles.model.vtxColor,[size(handles.model.vtxColor,1) 1 3]);
% lbl = rgbmapind(col,uint8(handles.labelColors*255)) - 1;
% redlist = ReadLabels('yaml/reduced/');
% %%
% redl = zeros(size(lbl),'uint8');
% red_col = zeros([size(redl,1) 3],'double');
% cmap = zeros(256,3);
% for i=1:length(handles.labelIDs)-1
%   %red(lbl(:)==i) = handles.labelIDs(i+1);
%   cmap(handles.labelIDs(i+1)+1,:) = handles.labelColors(i+1,:);
%   maplbl = redlist.mapping(handles.labelIDs(i+1));
%   redl(lbl(:)==i) = maplbl;
%   for c = 1:3
%     red_col(lbl(:)==i,c) = redlist.labelColors(maplbl+1,c);
%   end
% end
% ucol = uint8(red_col*255);
% %% write file
% data.vertex.x = handles.model.vtx(:,1);
% data.vertex.y = handles.model.vtx(:,2);
% data.vertex.z = handles.model.vtx(:,3);
% data.vertex.red = ucol(:,1);
% data.vertex.green = ucol(:,2);
% data.vertex.blue = ucol(:,3);
% data.vertex.semantic = redl;
% ply_write(data,exportPath);
% 
% 
% 
