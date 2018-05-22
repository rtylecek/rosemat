function varargout = Main(varargin)
% MAIN MATLAB code for Main.fig
%      MAIN, by itself, creates a new MAIN or raises the existing
%      singleton*.
%
%      H = MAIN returns the handle to a new MAIN or the handle to
%      the existing singleton*.
%
%      MAIN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAIN.M with the given input arguments.
%
%      MAIN('Property','Value',...) create  s a new MAIN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Main_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Main_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Main

% Last Modified by GUIDE v2.5 09-Jul-2017 19:18:40


%% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
  'gui_Singleton',  gui_Singleton, ...
  'gui_OpeningFcn', @Main_OpeningFcn, ...
  'gui_OutputFcn',  @Main_OutputFcn, ...
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


%% --- Executes just before Main is made visible.
function Main_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Main (see VARARGIN)

addpath './utils/geom3d';     % http://www.mathworks.com/matlabcentral/fileexchange/24484-geom3d
addpath './utils/meshes3d';   % subdir from geom3d package
addpath './utils/xml';     % http://www.mathworks.com/matlabcentral/fileexchange/12907-xml-io-tools
addpath './utils/ply';
addpath './utils/yaml';
addpath './utils/x3d';
addpath './utils/x3d/functions';

% Choose default command line output for Main
handles.rosNode = [];
handles.output = hObject;
handles.selectedNode = [];
handles.selectedFace = 0;
handles.selectedFaceMarker = [];
handles.selectionMode = 0;
handles.multiSelect = [];
handles.multiSelectLine = [];
handles.mesh = [];
handles.filePath = [];
handles.modelPath = [];
handles.occPath = [];

handles.modelTx = eye(4);

handles = read_labels('data/def',handles);

handles.popLabel.String = handles.labelNames;
assert(length(handles.labelIDs) == length(handles.labelNames)); 
assert(size(handles.labelColors,1) == length(handles.labelNames)); 

handles.delta = 0.01;
handles.transAlpha = 0.6;

handles.shapes.Unknown = 0;
handles.shapes.Cuboid = 1;
handles.shapes.Ellipsoid = 2;
handles.shapes.Cylinder = 3;
handles.shapes.Cone = 4;

handles.objects = {};

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Main wait for user response (see UIRESUME)
% uiwait(handles.figMain);


%% --- Outputs from this function are returned to the command line.
function varargout = Main_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function hline = DrawNodeMarker(handles, pos, id)
%% draw map node marker
hline = line(pos(1),pos(2),pos(3)+handles.delta, ...
             'Marker','s','MarkerFaceColor','b','MarkerEdgeColor','k', ...
             'UserData',id,'ButtonDownFcn',{@mapNode_ButtonDownFcn,[]});

function [handles, hline] = AddMapNode(handles, pos)
%% add new vertex
if isempty(handles.mesh)
  hline = [];
  return;
end
handles.mesh.Vertices = [handles.mesh.Vertices; pos(1) pos(2) pos(3)];
% edges update
id = size(handles.mesh.Vertices,1);
% if id>=3
%   handles.mesh.Faces = delaunay(handles.mesh.Vertices(:,1:2));
%   handles.mesh.FaceVertexCData = ones(size(handles.mesh.Faces,1),1);
% end
% add control point
hline = DrawNodeMarker(handles, pos, id);
handles.meshMarkers{id} = hline;
guidata(handles.mesh, handles);

function handles = ResetMap(handles, mapRange)
cla(handles.axMap);
handles.axMap.DataAspectRatioMode = 'manual';
handles.axMap.DataAspectRatio = [1 1 1];
grid(handles.axMap,'on');
%% reset selection
handles.selectedNode = [];
handles.selectedFace = 0;
handles.selectedFaceMarker = [];
handles.selectionMode = 0;
handles.selectedObject = [];
%% set map size
handles.axMap.XLim = mapRange(1,:);
handles.axMap.YLim = mapRange(2,:);
handles.axMap.ZLim = mapRange(3,:);
handles.sldElevation.Min = mapRange(3,1);
handles.sldElevation.Max = mapRange(3,2);

handles.occMap = [];
%% label colors
colormap(handles.labelColors);



function handles = InitMap(handles)
p = GetPropValues(handles);
diam = [0 30; 0 20; 0 5];
%diam = [-3 3.15; -2.0 1.95; 0 2];
%diam = [0 0 0; p.diam]'; 
handles = ResetMap(handles, diam);
%% init mesh to grass
handles.mesh = patch('Vertices',[],'Faces',[], 'FaceVertexCData',[], ...
  'FaceColor', 'flat', 'CDataMapping','direct',  'EdgeAlpha',0.1, ...
  'ButtonDownFcn',@mapPatch_ButtonClickedFcn, 'PickableParts','none'); %,'Marker','x','MarkerFaceColor','b');
%% add face to cover axes
handles = AddMapNode(handles, [diam(1,1) diam(2,1) 0]);
handles = AddMapNode(handles, [diam(1,1) diam(2,2) 0]);
handles = AddMapNode(handles, [diam(1,2) diam(2,2) 0]);
handles = AddMapNode(handles, [diam(1,2) diam(2,1) 0]);

handles.mesh.Faces = [1 2 3 4];
handles.mesh.FaceVertexCData = [3]; % grass
handles.mesh.FaceAlpha = handles.transAlpha;

handles.objects = {};
hold on;

guidata(handles.mesh, handles);

function handles = ImportModel(modelPath, handles, reset)

%% read
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
%   [tri, vtx, pdata, model.name] = ply_read(modelPath,'tri');
%   model.vtx = vtx';
%   model.tri = tri';
%   model.vtxColor = double([pdata.vertex.red pdata.vertex.green pdata.vertex.blue])/255;
%   model.vtxLabels = []; %model.data.vertex.alpha;
%   model.triColor = [];
%   model.data = pointCloud(vtx','Color',model.vtxColor);
%   % figure; trimesh(model.tri,model.vtx(:,1),model.vtx(:,2),model.vtx(:,3)); axis equal;
end
%% show map
disp(model);
handles.model = model;
mapRange = [min(model.vtx,[],1); max(model.vtx,[],1)]';
if reset
  handles = ResetMap(handles, mapRange);
else
  %% extend if needed
  handles.axMap.XLim(1) = min([handles.axMap.XLim(1) mapRange(1,1)]);
  handles.axMap.XLim(2) = max([handles.axMap.XLim(2) mapRange(1,2)]);
  handles.axMap.YLim(1) = min([handles.axMap.YLim(1) mapRange(2,1)]);
  handles.axMap.YLim(2) = max([handles.axMap.YLim(2) mapRange(2,2)]);
  handles.axMap.ZLim(1) = min([handles.axMap.ZLim(1) mapRange(3,1)]);
  handles.axMap.ZLim(2) = max([handles.axMap.ZLim(2) mapRange(3,2)]);
  handles.sldElevation.Min = handles.axMap.ZLim(1);
  handles.sldElevation.Max = handles.axMap.ZLim(2);
end
sel = unique(round(linspace(1,size(model.vtx,1),20000)));
handles.cloud = scatter3(model.vtx(sel,1),model.vtx(sel,2),model.vtx(sel,3), ...
  10, model.vtxColor(sel,:), 'filled','PickableParts','none');

% bring renderer
% handles.axMap.Parent = handles.figMain;  
% handles.axPoints = pcshow(model.data,'Parent',handles.axMap,'MarkerSize',10);
% handles.axMap.Parent = handles.pnlMap;
% if strcmp(fext,'.pcd')
%   ptCloud = pcread(modelPath);
%   diam = double([ptCloud.XLimits; ptCloud.YLimits; ptCloud.ZLimits]); 
%   handles = ResetMap(handles, diam);
%   %pcshow(ptCloud);
%   ptColor = double(ptCloud.Color)/255;
%   handles.cloud = scatter3(ptCloud.Location(:,1),ptCloud.Location(:,2),ptCloud.Location(:,3),20,ptColor,'filled','PickableParts','none');
% elseif strcmp(fext,'.ply')
%   [tri,vtx,dat] = ply_read(modelPath,'tri');
%   diam = [min(vtx,[],1); max(vtx,[],1)]';
%   handles = ResetMap(handles, diam);
%   if isempty(tri)
%     %% only points
%     ptColor = double([dat.vertex.red dat.vertex.green dat.vertex.blue])/255;
%     ptCloud.Location = vtx';
%     handles.cloud = scatter3(ptCloud.Location(:,1),ptCloud.Location(:,2),ptCloud.Location(:,3),20,ptColor,'filled','PickableParts','none');
%   else
%     handles.model = trimesh(tri,vtx(:,1),vtx(:,2),vtx(:,3),'PickableParts','none');
%   end
% end

if reset
  %% init mesh to grass
  handles.mesh = patch('Vertices',[],'Faces',[], 'FaceVertexCData',[], ...
    'FaceColor', 'flat', 'CDataMapping','direct',  ...
    'ButtonDownFcn',@mapPatch_ButtonClickedFcn, 'PickableParts','none'); %,'Marker','x','MarkerFaceColor','b');
  %% add face to cover axes
  handles = AddMapNode(handles, [mapRange(1,1) mapRange(2,1) mapRange(3,1)]);
  handles = AddMapNode(handles, [mapRange(1,1) mapRange(2,2) mapRange(3,1)]);
  handles = AddMapNode(handles, [mapRange(1,2) mapRange(2,2) mapRange(3,1)]);
  handles = AddMapNode(handles, [mapRange(1,2) mapRange(2,1) mapRange(3,1)]);
  
  handles.mesh.Faces = [1 2 3 4];
  handles.mesh.FaceVertexCData = [3]; % grass
  
  handles.objects = {};
  axis equal;
end
%%
guidata(handles.figMain, handles);

function handles = SegmentModel(handles)
%% segment model to init objects
vtc = uint32(handles.model.vtxColor*255);
clusterObj = vtc(:,1) + bitshift(vtc(:,2),8) + bitshift(vtc(:,3),16);
clusterColors = unique(clusterObj);
objCount = length(clusterColors);
%% bounding boxes
objPoints = zeros(objCount,1);
for oi = 1:objCount
  %% fit to clusters
  opts = (clusterObj==clusterColors(oi));
  objX = handles.model.vtx(opts,:);
  objPoints(oi) = sum(opts);
  props.diam = max(objX) - min(objX);
  props.pos = mean([max(objX); min(objX)]); % median(objX);
  props.pos(3) = props.pos(3) - props.diam(3)/2;
  props.orient = [0 0 0];
  props.label = handles.popLabel.Value;
  props.id = length(handles.objects)+1;
  handles.objects{props.id} = DrawObject(handles, handles.shapes.Cuboid, props);
end


function handles = ExportModel(modelPath, handles, ground)
%% color points by label
%ptCloud = pcread('/home/radim/TrimBot/data/WageningenGardenSept2016/160920_Wageningen_10mm_downsampled_binary.pcd_down0.025.pcd');
%ptXYZ = ptCloud.Location;
% ptXYZ = [handles.cloud.XData; handles.cloud.YData; handles.cloud.ZData]';
if hasfield(handles,'model')
  ptXYZ = handles.model.vtx;
  ptLabels = zeros(size(ptXYZ,1),1);
  ptIDs = 0;
  ptColor = zeros(size(ptXYZ));
  ptColor(:) = 0.01;
  if ~ground
    %% color by bbox
    for id = 1:length(handles.objects)
      if ~isempty(handles.objects{id})
        obj = handles.objects{id}.UserData;
        box = [obj.pos-obj.diam/2; obj.pos+obj.diam/2];
        box(:,3) = box(:,3) + obj.diam(3)/2;  % from Z=0
        [~, pind] = clipPoints3d(ptXYZ,box);
        ptLabels(pind) = id;
        ptColor(pind,:) = repmat(handles.labelColors(obj.label,:),length(pind),1);
        ptIDs = [ptIDs id];
      end
    end
  else
    %% color by ground mesh
    for i = 1:size(handles.mesh.Faces,1)
      vtx = handles.mesh.Faces(i,:);
      vtx = vtx(isfinite(vtx));
      pxyz = handles.mesh.Vertices(vtx,:);
      %[hit, edge] = inpolygon(eventdata.IntersectionPoint(1),eventdata.IntersectionPoint(2),pxy(:,1),pxy(:,2));
      if ~any(isnan(pxyz(:))) && ~isempty(vtx)
        pind = inpoly(ptXYZ(:,1:2), pxyz(:,1:2), [], handles.delta/100)>0;
        ptLabels(pind) = i;
        ptColor(pind,:) = repmat(handles.labelColors(handles.mesh.CData(i),:),sum(pind),1);
      end
    end
  end
  %% write
  fprintf('Exporting %s ...',modelPath); tic;
  pcExport = pointCloud(single(ptXYZ),'Color',uint8(ptColor*255));
  pcwrite(pcExport,modelPath);
  toc;
else
  %% export mesh
  fprintf('Exporting %s ...',modelPath); tic;
  figure2xhtml(modelPath,handles.axMap);
  toc;
  
end

function SaveMap(filePath, handles)
%% save map to xml file
root = struct();
root.Version = 2;
root.Range = [handles.axMap.XLim; handles.axMap.YLim; handles.axMap.ZLim];
root.Labels = handles.labelNames;
%% mesh
root.Terrain = struct('Vertices', handles.mesh.Vertices, ...
  'Faces', handles.mesh.Faces, ...
  'FaceLabels', handles.mesh.FaceVertexCData);
%% objects
root.Objects = {};
for id = 1:length(handles.objects)
  if ~isempty(handles.objects{id})
    root.Objects{length(root.Objects)+1} = handles.objects{id}.UserData;
  end
end
%% model
root.Model.Path = handles.modelPath;
root.Model.Transform = handles.modelTx;
%root.Model.Occupancy = handles.occPath;
%% write file
xml_write(filePath,root,'GardenMap');

function handles = LoadMap(filePath, handles)
%% read map from xml file
root = xml_read(filePath);
%% load map objects
handles = ResetMap(handles, root.Range);
% mesh
handles.mesh = patch('Vertices',root.Terrain.Vertices,'Faces',root.Terrain.Faces, 'FaceVertexCData',root.Terrain.FaceLabels, ...
  'FaceColor', 'flat', 'CDataMapping','direct',  'FaceAlpha', handles.transAlpha, 'EdgeAlpha',0.2, ...
  'ButtonDownFcn',@mapPatch_ButtonClickedFcn, 'PickableParts','none');
hold on;
% node markers
for id = 1:size(root.Terrain.Vertices,1)
  handles.nodeMarkers{id} = DrawNodeMarker(handles, root.Terrain.Vertices(id,:), id);
end
% objects
handles.objects = {};
for id = 1:length(root.Objects)
  props = root.Objects(id);
  handles.objects{props.id} = DrawObject(handles, props.shape, props);
end
% model
if hasfield(root,'Model')
  if exist(root.Model.Path,'file')
    handles.modelPath = root.Model.Path;
    handles = ImportModel(handles.modelPath, handles, false);
  end
end
if root.Version>=2
  handles.modelTx = root.Model.Transform;
end

function handles = MoveMapNode(handles,id,pos)
%% move vertex
pos(3) = pos(3) - handles.delta;
handles.mesh.Vertices(id,:) = pos;
guidata(handles.mesh, handles);

function handles = DeleteMapNode(handles,id)
%% remove node
handles.mesh.Vertices(id,:) = nan;
guidata(handles.mesh, handles);

function [selectedFace, selectedEdge] = GetMapSelection(handles,intersectionPoint)
selectedFace = 0;
selectedEdge = [];
if any(isnan(intersectionPoint(:)))
  return;
end
for i = 1:size(handles.mesh.Faces,1)
  vtx = handles.mesh.Faces(i,:);
  vtx = vtx(isfinite(vtx));
  pxyz = handles.mesh.Vertices(vtx,:);
  %[hit, edge] = inpolygon(eventdata.IntersectionPoint(1),eventdata.IntersectionPoint(2),pxy(:,1),pxy(:,2));
  if ~any(isnan(pxyz(:))) && ~isempty(vtx)
    [hit, edge] = inpoly(intersectionPoint(1:2), pxyz(:,1:2), [], handles.delta/100);
    if hit
      selectedFace = i;
      if edge>0
        selectedEdge = [vtx(edge) vtx(mod(edge,length(vtx))+1)] ;
      end
      break;
    end
  end
end

function [selectedFaces, selectedEdges] = TestMapSelection(handles,intersectionPoint)
selectedFaces = zeros(size(handles.mesh.Faces,1),1);
selectedEdges = zeros(size(handles.mesh.Faces,1),1);
if any(isnan(intersectionPoint(:)))
  return;
end
for i = 1:size(handles.mesh.Faces,1)
  vtx = handles.mesh.Faces(i,:);
  vtx = vtx(isfinite(vtx));
  pxyz = handles.mesh.Vertices(vtx,:);
  %[hit, edge] = inpolygon(eventdata.IntersectionPoint(1),eventdata.IntersectionPoint(2),pxy(:,1),pxy(:,2));
  if ~any(isnan(pxyz(:))) && ~isempty(vtx)
    [selectedFaces(i), selectedEdges(i)] = inpoly(intersectionPoint(1:2), pxyz(:,1:2), [], handles.delta/100);
  end
end

function mapPatch_ButtonClickedFcn(hObject, eventdata, handles)
fprintf('mapFace: click @ %s',sprintf('%.3f ',eventdata.IntersectionPoint));
handles = guidata(hObject);
[handles.selectedFace, handles.selectedEdge] = GetMapSelection(handles,eventdata.IntersectionPoint);

if handles.selectedFace>0
  vtx = hObject.Faces(handles.selectedFace,:);
  vtx = vtx(isfinite(vtx));
  pxyz = hObject.Vertices(vtx,:);
  if handles.selectedEdge>0
    %% edge selected
    fprintf(' (%d -- %d)',handles.selectedEdge(1),handles.selectedEdge(2));
    if eventdata.Button==3
      if handles.selectionMode==1
        %% add new node
        [handles,nn] = AddMapNode(handles,eventdata.IntersectionPoint);
        handles = SelectMapNode(nn,handles);
        %% split edge
        fce = handles.mesh.Faces == handles.selectedEdge(1) | handles.mesh.Faces == handles.selectedEdge(2);
        fcs = sum(fce,2);
        ff = find(fcs==2);
        for f = ff'
          edge = find(fce(f,:)==1);
          if edge(2)>edge(1)+1
            edge = edge(2);
          else
            edge = edge(1);
          end
          nfx = [hObject.Faces(f,1:edge) nn.UserData hObject.Faces(f,edge+1:end)];
          nfx = nfx(isfinite(nfx));
          hObject.Faces = [hObject.Faces nan(size(hObject.Faces,1), length(nfx)-size(hObject.Faces,2))];
          hObject.Faces(f,1:length(nfx)) = nfx;
        end
      elseif handles.selectionMode==2
        %% remove edge
        fce = handles.mesh.Faces == handles.selectedEdge(1) | handles.mesh.Faces == handles.selectedEdge(2);
        fcs = sum(fce,2);
        ff = find(fcs==2);
        f = ff(1);
        edge = find(fce(f,:)==1);
        if edge(2)>edge(1)+1
          edge = edge(2);
        else
          edge = edge(1);
        end
        dfx = hObject.Faces(ff(2),:);
        nfx = [hObject.Faces(f,1:edge) dfx(fce(ff(2),:)==0 & ~isnan(dfx)) hObject.Faces(f,edge+1:end)];
        nfx = nfx(isfinite(nfx));
        hObject.Faces = [hObject.Faces nan(size(hObject.Faces,1), length(nfx)-size(hObject.Faces,2))];
        hObject.Faces(f,1:length(nfx)) = nfx;
        hObject.Faces(ff(2),:) = nan;
      end
    end
  else
    %% face selected
    fprintf(' : %d',handles.selectedFace);
    if handles.selectionMode==1 && eventdata.Button==3
      %% add new node
      [handles,nn] = AddMapNode(handles,eventdata.IntersectionPoint);
      handles = SelectMapNode(nn,handles);
      %% subdivide face
      ni = nn.UserData;
      nfc = length(vtx);
      nf = zeros(nfc,3);
      nf(:,1) = vtx;
      nf(2:end,2) = vtx(1:end-1);
      nf(1,2) = vtx(end);
      nf(:,3) = ni;
      hObject.Faces(handles.selectedFace,1:3) = nf(1,:);
      hObject.Faces(handles.selectedFace,4:end) = nan;
      hObject.Faces(end+1:end+nfc-1,:) = nan;
      hObject.Faces((end-nfc+2):end,1:3) = nf(2:end,:);
      hObject.FaceVertexCData = [hObject.FaceVertexCData; ...
                                 hObject.FaceVertexCData(handles.selectedFace)*ones(nfc-1,1)];
    elseif handles.selectionMode ~= 1
      %% mark selected face
      mxyz = mean(pxyz,1);
      if ~isempty(handles.selectedFaceMarker)
        delete(handles.selectedFaceMarker);
        handles.selectedFaceMarker = [];
      end
      handles.selectedFaceMarker = line(mxyz(1),mxyz(2),mxyz(3)+0.01, ...
        'Marker','o','MarkerFaceColor','m','MarkerEdgeColor','k');
      handles = UpdatePropControls(handles, mxyz,[],[],handles.mesh.FaceVertexCData(handles.selectedFace));   
    end
  end
end
fprintf('\n');
guidata(hObject, handles);

% --- Executes on mouse press over axes background.
function axMap_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf('axMap: click @ %s\n',sprintf('%.3f ',eventdata.IntersectionPoint));
if eventdata.Button == 1 && handles.selectionMode == 1
  %% add point
  [handles, node] = AddMapNode(handles, ...
    [eventdata.IntersectionPoint(1) eventdata.IntersectionPoint(2) handles.sldElevation.Value]);
  if ~isempty(node)
    handles = SelectMapNode(node,handles);
  end
end

function handles = SelectMapNode(hObject,handles)
  handles.selectedNode.MarkerFaceColor = 'b';
  hObject.MarkerFaceColor = 'm';
  UpdateMapNode(hObject,handles);
  handles.selectedNode = hObject;


% --- Executes on mouse press over line object.
function mapNode_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isempty(handles)
  handles = guidata(hObject);
end
fprintf('mapNode: click @ %d\n',hObject.UserData);

if handles.selectionMode <=1
  %% select node
  handles = SelectMapNode(hObject,handles);
end

switch eventdata.Button
  case 1 % left
    %% move
    if handles.selectionMode == 1
      set(ancestor(hObject,'figure'),'WindowButtonMotionFcn',{@mapNode_ButtonMotionFcn,hObject});
      set(ancestor(hObject,'figure'),'WindowButtonUpFcn',@mapNode_ButtonUpFcn);
      handles.selectedNodeOrigPos = [hObject.XData hObject.YData hObject.ZData];
    end
   
  case 2 % mid
   
  case 3 % right
    %% delete
    if handles.selectionMode == 1
      DeleteMapNode(handles,hObject.UserData)
      delete(hObject);
      handles.selectedNode = [];
    end
end
guidata(handles.figMain, handles);

%
function handles = UpdatePropControls(handles, pos, diam, orient, label)
%% position
if ~isempty(pos)
  handles.edtPosX.String = sprintf('%.03g',pos(1));
  handles.edtPosY.String = sprintf('%.03g',pos(2));
  handles.edtPosZ.String = sprintf('%.03g',pos(3));
  handles.sldElevation.Value = pos(3);
end
%% dimension
if nargin>=3 && ~isempty(diam)
  handles.edtDimX.String = sprintf('%.03g',diam(1));
  handles.edtDimY.String = sprintf('%.03g',diam(2));
  handles.edtDimZ.String = sprintf('%.03g',diam(3));
end
%% rotation
if nargin>=4 && ~isempty(orient)
  handles.edtRotX.String = sprintf('%.03g',orient(1));
  handles.edtRotY.String = sprintf('%.03g',orient(2));
  handles.edtRotZ.String = sprintf('%.03g',orient(3));
end
%% label
if nargin>=4 && ~isempty(label)
  handles.popLabel.Value = label;
end

function props = GetPropValues(handles)
props.pos  = [str2double(handles.edtPosX.String) str2double(handles.edtPosY.String) str2double(handles.edtPosZ.String)];
props.diam = [str2double(handles.edtDimX.String) str2double(handles.edtDimY.String) str2double(handles.edtDimZ.String)];
props.orient = [str2double(handles.edtRotX.String) str2double(handles.edtRotY.String) str2double(handles.edtRotZ.String)];
props.label = handles.popLabel.Value;

%
function UpdateMapNode(hObject,handles)
handles = UpdatePropControls(handles, [hObject.XData hObject.YData hObject.ZData-handles.delta]);
MoveMapNode(handles, hObject.UserData, [hObject.XData hObject.YData hObject.ZData]);

%
function mapNode_ButtonMotionFcn(hObject,eventdata,src)
coords=get(gca,'CurrentPoint');
x=coords(1,1,1);
y=coords(1,2,1);
if x>-100 && y>-100
  set(src,'XData',x,'YData',y);
  UpdateMapNode(src,guidata(hObject))
end

function mapNode_ButtonUpFcn(hObject,eventdata)
set(hObject,'WindowButtonMotionFcn','');
set(hObject,'WindowButtonUpFcn','');
handles = guidata(hObject);

% for v = 1:size(handles.mesh.Vertices,1)
%   selFaces = TestMapSelection(handles, handles.mesh.Vertices(v,:));
%   % check if vtx position is just within faces around it
%   for f = find(selFaces==1)'
%     if all(handles.mesh.Faces(f,:) ~= v)
%       warning('Invalid position!');
%       set(handles.selectedNode,'XData',handles.selectedNodeOrigPos(1),'YData',handles.selectedNodeOrigPos(2),'ZData',handles.selectedNodeOrigPos(3));
%       UpdateMapNode(handles.selectedNode, handles);
%       guidata(hObject,handles);
%       return;
%     end
%   end
% end

handles.axMap.XLim(2) = max(handles.axMap.XLim(2),handles.selectedNode.XData);
handles.axMap.YLim(2) = max(handles.axMap.YLim(2),handles.selectedNode.YData);

% [selFaces] = TestMapSelection(handles, ...
%   [handles.selectedNode.XData handles.selectedNode.YData handles.selectedNode.ZData]);
% % check if new position is within faces around the node
% for f = find(selFaces==1)'
%   if all(handles.mesh.Faces(f,:)~=handles.selectedNode.UserData)
%     disp('Invalid position!');
%     set(handles.selectedNode,'XData',handles.selectedNodeOrigPos(1),'YData',handles.selectedNodeOrigPos(2),'ZData',handles.selectedNodeOrigPos(3));
%     UpdateMapNode(handles.selectedNode, handles);
%     guidata(hObject,handles);
%     return;
%   end
% end


% --- Executes when pnlMap is resized.
function pnlMap_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to pnlMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isempty(handles)
  handles.axMap.Position(4) = hObject.Position(4) - 2*handles.axMap.Position(2);
  handles.axMap.Position(3) = hObject.Position(3) - 2*handles.axMap.Position(1);
end



%% --- Executes during object creation, after setting all properties.
function axMap_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
hObject.DataAspectRatioMode = 'manual';
hObject.DataAspectRatio = [1 1 1];
% Hint: place code in OpeningFcn to populate axMap

%% --- Executes when figMain is resized.
function figMain_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figMain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pnlMap.Position(4) = hObject.Position(4) - handles.pnlMap.Position(1);
handles.pnlMap.Position(3) = hObject.Position(3) - handles.pnlMap.Position(1)*3 - handles.pnlProps.Position(3);

handles.pnlProps.Position(4) = hObject.Position(4) - handles.pnlMap.Position(1);
handles.pnlProps.Position(1) = hObject.Position(3) - handles.pnlProps.Position(3)  - handles.pnlMap.Position(1);

handles.popLabel.Position(4) = handles.pnlProps.Position(4) - 30;
handles.txtLabel.Position(2) = handles.popLabel.Position(2) + handles.popLabel.Position(4);

%% --- Executes when pnlProps is resized.
function pnlProps_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to pnlProps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% if ~isempty(handles)
%   handles.sldElevation.Position(4) = hObject.Position(4) - handles.pnlMap.Position(1)*5;
% end

%% --- Executes on slider movement.
function sldElevation_Callback(hObject, eventdata, handles)
% hObject    handle to sldElevation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.edtPosZ.String = sprintf('%.03g',hObject.Value);
switch(handles.selectionMode)
  case {0,1}
    if ~isempty(handles.selectedNode)
      handles.selectedNode.ZData = hObject.Value;
      MoveMapNode(handles,handles.selectedNode.UserData, ...
          [handles.selectedNode.XData handles.selectedNode.YData handles.selectedNode.ZData]);
    end
    
  case 2
    if ~isempty(handles.selectedFace)
      dz = hObject.Value - handles.selectedFaceMarker.ZData;
      handles.selectedFaceMarker.ZData = hObject.Value + handles.delta;
      for v = handles.mesh.Faces(handles.selectedFace,:)
        if ~isnan(v)
          vnode = findobj(handles.axMap,'UserData',v);
          vnode.ZData = hObject.Value;
          MoveMapNode(handles,v, handles.mesh.Vertices(v,:) + [0 0 dz+handles.delta]);
        end
      end
    end
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

function MoveSelectedNode(handles)
MoveMapNode(handles,handles.selectedNode.UserData,[handles.selectedNode.XData handles.selectedNode.YData handles.selectedNode.ZData]);


function edtPosX_Callback(hObject, eventdata, handles)
% hObject    handle to edtPosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.selectedNode)
  handles.selectedNode.XData = str2double(hObject.String);
  MoveSelectedNode(handles)
end
% Hints: get(hObject,'String') returns contents of edtPosX as text
%        str2double(get(hObject,'String')) returns contents of edtPosX as a double

function edtPosY_Callback(hObject, eventdata, handles)
% hObject    handle to edtPosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.selectedNode)
  handles.selectedNode.YData = str2double(hObject.String);
  MoveSelectedNode(handles)
end
% Hints: get(hObject,'String') returns contents of edtPosY as text
%        str2double(get(hObject,'String')) returns contents of edtPosY as a double

%%
function edtPosZ_Callback(hObject, eventdata, handles)
% hObject    handle to edtPosZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.selectedNode)
  handles.sldElevation.Value = str2double(hObject.String);
  handles.selectedNode.ZData = handles.sldElevation.Value+handles.delta;
  MoveSelectedNode(handles)
end
% Hints: get(hObject,'String') returns contents of edtPosZ as text
%        str2double(get(hObject,'String')) returns contents of edtPosZ as a double

% --------------------------------------------------------------------
function tlbNewMap_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbNewMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = InitMap(handles);
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbImport_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbImport (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
modifiers = get(handles.figMain,'currentModifier');
[FileName,PathName] = uigetfile({'*.ply','Mesh file (*.ply)'; '*.pcd','Point cloud file (*.pcd)'},'Import model',handles.modelPath);
if ~isempty(FileName)
  handles.modelPath = [PathName FileName];
  handles = ImportModel(handles.modelPath, handles, ismember('shift',modifiers));
  guidata(hObject,handles);
  fprintf('Loaded %s.\n',handles.modelPath);
end

% --------------------------------------------------------------------
function tlbExport_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbExport (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% objects
modifiers = get(handles.figMain,'currentModifier');
[FileName,PathName] = uiputfile({'*.ply','Mesh file (*.ply)';'*.pcd','Point cloud file (*.pcd)'},'Export model',handles.modelPath);
if ~isempty(FileName)
  handles.exportPath = [PathName FileName];
  ExportModel(handles.exportPath, handles, ismember('shift',modifiers));
  fprintf('Saved %s.\n',handles.exportPath);
  guidata(hObject,handles);
end

% --------------------------------------------------------------------
function tlbOpenMap_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbOpenMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile({'*.xml','XML map file (*.xml)'},'Open map',handles.filePath);
if FileName>0 %~isempty(FileName)
  handles.filePath = [PathName FileName];
  handles = LoadMap(handles.filePath, handles);
  guidata(hObject,handles);
  fprintf('Loaded %s.\n',handles.filePath);
end
% --------------------------------------------------------------------
function tlbSaveMap_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbSaveMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uiputfile({'*.xml','XML map file (*.xml)'},'Save map',handles.filePath);
if FileName>0 %~isempty(FileName)
  handles.filePath = [PathName FileName];
  SaveMap(handles.filePath, handles);
  fprintf('Saved %s.\n',handles.filePath);
  guidata(hObject,handles);
end

% --------------------------------------------------------------------
function tlbRotate_OffCallback(hObject, eventdata, handles)
% hObject    handle to tlbRotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rotate3d(handles.axMap,'off');

% --------------------------------------------------------------------
function tlbRotate_OnCallback(hObject, eventdata, handles)
% hObject    handle to tlbRotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[az,el] = view();
if az==0 && el==90
  view(3);
end
rotate3d(handles.axMap,'on'); %'Enable','on', 'RotateStyle','orbit');
handles.tlbFaces.State = 'off';
handles.tlbNodes.State = 'off';
handles.selectionMode = 0;
if ~isempty(handles.mesh), handles.mesh.PickableParts = 'none'; end;
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbNodes_OnCallback(hObject, eventdata, handles)
% hObject    handle to tlbNodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles = guidata(hObject);
view(2);
handles.tlbFaces.State = 'off';
handles.tlbRotate.State = 'off';
handles.selectionMode = 1;
if ~isempty(handles.mesh), handles.mesh.PickableParts = 'visible'; end;
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbNodes_OffCallback(hObject, eventdata, handles)
% hObject    handle to tlbNodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.selectionMode = 0;
if ~isempty(handles.selectedNode)
  handles.selectedNode.MarkerFaceColor = 'b';
end
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbFaces_OnCallback(hObject, eventdata, handles)
% hObject    handle to tlbFaces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.tlbNodes.State = 'off';
handles.tlbRotate.State = 'off';
handles.mesh.PickableParts = 'visible';
handles.selectionMode = 2;
modifiers = get(handles.figMain,'currentModifier');
if ismember('shift',modifiers) && size(handles.mesh.Vertices,1)>=3
  %% remesh using DT
  handles.mesh.Faces = delaunay(handles.mesh.Vertices(:,1:2));
  handles.mesh.FaceVertexCData = 3*ones(size(handles.mesh.Faces,1),1);
end
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbFaces_OffCallback(hObject, eventdata, handles)
% hObject    handle to tlbFaces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.mesh.PickableParts = 'none';
handles.selectionMode = 0;
if ~isempty(handles.selectedFaceMarker)
  delete(handles.selectedFaceMarker);
  handles.selectedFaceMarker = [];
end
guidata(hObject,handles);


% --------------------------------------------------------------------
function tlbGrass_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbGrass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.selectionMode == 2 && handles.selectedFace>0
  handles.mesh.FaceVertexCData(handles.selectedFace) = 3; %handles.labels.Grass;
  guidata(hObject,handles);
end

% --------------------------------------------------------------------
function tlbDirt_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbDirt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.selectionMode == 2 && handles.selectedFace>0
  handles.mesh.FaceVertexCData(handles.selectedFace) = 4; %handles.labels.Dirt;
  guidata(hObject,handles);
end

% --------------------------------------------------------------------
function tlnGravel_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlnGravel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.selectionMode == 2 && handles.selectedFace>0
  handles.mesh.FaceVertexCData(handles.selectedFace) = 5; %handles.labels.Gravel;
  guidata(hObject,handles);
end

% --------------------------------------------------------------------
function tlbMulch_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbMulch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.selectionMode == 2 && handles.selectedFace>0
  handles.mesh.FaceVertexCData(handles.selectedFace) = 6; %handles.labels.Mulch;
  guidata(hObject,handles);
end

% --------------------------------------------------------------------
function tlbPebbles_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbPebbles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.selectionMode == 2 && handles.selectedFace>0
  handles.mesh.FaceVertexCData(handles.selectedFace) = 7; %handles.labels.Pebbles;
  guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function figMain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figMain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --------------------------------------------------------------------
function tlbRotate_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbRotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function obj = DrawObject(handles, shape, props)
%% draw map object with given parameters
switch shape
  case handles.shapes.Cuboid
    obj = drawCuboid([props.pos(1) props.pos(2) props.pos(3)+props.diam(3)/2 props.diam props.orient], ...
      'Tag', 'Cuboid');
    
  case handles.shapes.Ellipsoid
    obj = drawEllipsoid([props.pos(1) props.pos(2) props.pos(3)+props.diam(3)/2 props.diam/2 props.orient], ...
      'Tag', 'Ellipsoid', 'LineStyle','-','EdgeColor','k', 'EdgeAlpha',0.5);
    
  case handles.shapes.Cylinder
    obj = drawEllipseCylinder([props.pos(1) props.pos(2) props.pos(3) props.pos(1) props.pos(2) props.pos(3)+props.diam(3) props.diam(1)/2 props.diam(2)/2 props.orient(1)],'closed', ...
      'Tag', 'Cylinder', 'LineStyle','-', 'EdgeColor','k', 'EdgeAlpha',0.5);
    
  case handles.shapes.Cone
    if props.diam(1)==props.diam(2)
      props.diam(2) = 0;
    end
    obj = drawCone([props.pos(1) props.pos(2) props.pos(3)], [props.pos(1) props.pos(2) props.pos(3)+props.diam(3)], [props.diam(1)/2 props.diam(2)/2], 32,'g',0,1);
    set(obj, 'Tag', 'Cone', 'LineStyle','-', 'EdgeColor','k', 'EdgeAlpha',0.5);
    
end
obj.ButtonDownFcn = @objBox_ButtonClickedFcn;
obj.FaceColor = handles.labelColors(props.label,:);
obj.FaceAlpha = handles.transAlpha;
props.shape = shape;
obj.UserData = props;

function handles = tlb_PlaceObject(hObject, shape, handles)
props = GetPropValues(handles);
modifiers = get(handles.figMain,'currentModifier');
if ismember('shift',modifiers)
  gi = ginput(2);
  props.pos(1:2) = mean(gi,1);
  props.diam(1:2) = abs(diff(gi,1));
else
  props.pos(1:2) = ginput(1);
end
selectedFace = GetMapSelection(handles,props.pos);
if ~isempty(selectedFace)
  ff = handles.mesh.Faces(selectedFace,:);
  props.pos(3) = max([props.pos(3); handles.mesh.Vertices(ff(isfinite(ff)),3)]);
end
props.id = length(handles.objects)+1;
handles.objects{props.id} = DrawObject(handles, shape, props);

% --------------------------------------------------------------------
function tlbBox_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = tlb_PlaceObject(hObject, handles.shapes.Cuboid, handles);
guidata(hObject,handles);

% --------------------------------------------------------------------
function tblEllipsoid_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tblEllipsoid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = tlb_PlaceObject(hObject, handles.shapes.Ellipsoid, handles);
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbCylinder_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbCylinder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles = tlb_PlaceObject(hObject, handles.shapes.Cylinder, handles);
guidata(hObject,handles);

% --------------------------------------------------------------------
function tlbCone_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbCone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = tlb_PlaceObject(hObject, handles.shapes.Cone, handles);
guidata(hObject,handles);

%%
function objBox_ButtonClickedFcn(hObject, eventdata, handles)
handles = guidata(hObject);
modifiers = get(handles.figMain,'currentModifier');

fprintf('object-%s [%d]: click @ %s\n', hObject.Tag, hObject.UserData.id, sprintf('%.3f ',eventdata.IntersectionPoint));
if ~isempty(handles.selectedObject)
  try
    handles.selectedObject.EdgeColor = 'k';
  catch
    handles.selectedObject = [];
  end
end
if strcmp(hObject.Tag,'Robot') && ~isempty(hObject.UserData.msg)
  disp(hObject.UserData.msg.Pose.Orientation);
  disp(hObject.UserData);
end
%% multiselect
if ismember('shift',modifiers)
  %% multiselect
  handles.multiSelect = [handles.multiSelect hObject.UserData.id];
  cpos = hObject.UserData.pos;
  if isempty(handles.multiSelectLine)
    ppos = handles.selectedObject.UserData.pos;
    handles.multiSelectLine = line('XData',[ppos(1) cpos(1)],'YData',[ppos(2) cpos(2)],'ZData',[ppos(3) cpos(3)], ...
      'LineWidth',10,'Color','m');
  else
    set(handles.multiSelectLine,'XData',[handles.multiSelectLine.XData cpos(1)], ...
      'YData', [handles.multiSelectLine.YData cpos(2)], ... 
      'ZData',[handles.multiSelectLine.ZData cpos(3)]);
  end
else
  handles.multiSelect = hObject.UserData.id;
  if ~isempty(handles.multiSelectLine)
    delete(handles.multiSelectLine);
    handles.multiSelectLine = [];
  end
end
%% select new
hObject.EdgeColor = 'm';
handles.selectedObject = hObject;

UpdatePropControls(handles,hObject.UserData.pos,hObject.UserData.diam,hObject.UserData.orient,hObject.UserData.label);

switch eventdata.Button
  case 1 % left
    %% move
    if handles.selectionMode == 1
      set(ancestor(hObject,'figure'),'WindowButtonMotionFcn',{@obj_ButtonMotionFcn,hObject});
      set(ancestor(hObject,'figure'),'WindowButtonUpFcn',@obj_ButtonUpFcn);
      handles.objectClickedPoint = eventdata.IntersectionPoint;
    end
    
  case 3 % right
    if handles.selectionMode == 1
      %% delete
      if(hObject.UserData.id>0)
        handles.objects{hObject.UserData.id} = [];
      end
      delete(hObject);
      handles.selectedObject = [];
    end
end
guidata(handles.figMain,handles);


function obj_ButtonMotionFcn(hObject,eventdata,src)
% coords=get(gca,'CurrentPoint');
% x=coords(1,1,1);
% y=coords(1,2,1);
% if x>0 && y>0
%   set(src,'XData',x,'YData',y);
% 
% end

function selectedObject = MoveObject(selectedObject,dx)
%% move object
if hasfield(selectedObject,'Vertices')
  selectedObject.Vertices(:,1) = selectedObject.Vertices(:,1) + dx(1);
  selectedObject.Vertices(:,2) = selectedObject.Vertices(:,2) + dx(2);
  selectedObject.Vertices(:,3) = selectedObject.Vertices(:,3) + dx(3);
else
  selectedObject.XData = selectedObject.XData + dx(1);
  selectedObject.YData = selectedObject.YData + dx(2);
  selectedObject.ZData = selectedObject.ZData + dx(3);
end
selectedObject.UserData.pos = selectedObject.UserData.pos + dx;


function obj_ButtonUpFcn(hObject,eventdata)
set(hObject,'WindowButtonMotionFcn','');
set(hObject,'WindowButtonUpFcn','');

handles = guidata(hObject);
%% calculate drift
coords=get(gca,'CurrentPoint');
dx = coords(1,1,1) - handles.objectClickedPoint(1);
dy = coords(1,2,1) - handles.objectClickedPoint(2);
if abs(dx)+abs(dy)>0.01
  %% move object
  handles.selectedObject = MoveObject(handles.selectedObject,[dx dy 0]);
  fprintf('move: %.3g, %.3g\n',dx,dy);
end
guidata(hObject,handles);

function edtRotZ_Callback(hObject, eventdata, handles)
% hObject    handle to edtRotZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edtRotZ as text
%        str2double(get(hObject,'String')) returns contents of edtRotZ as a double


% --- Executes during object creation, after setting all properties.
function edtRotZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edtRotZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edtRotX_Callback(hObject, eventdata, handles)
% hObject    handle to edtRotX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edtRotX as text
%        str2double(get(hObject,'String')) returns contents of edtRotX as a double

% --- Executes during object creation, after setting all properties.
function edtRotX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edtRotX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edtRotY_Callback(hObject, eventdata, handles)
% hObject    handle to edtRotY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edtRotY as text
%        str2double(get(hObject,'String')) returns contents of edtRotY as a double

% --- Executes during object creation, after setting all properties.
function edtRotY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edtRotY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edtDimZ_Callback(hObject, eventdata, handles)
% hObject    handle to edtDimZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edtDimZ as text
%        str2double(get(hObject,'String')) returns contents of edtDimZ as a double

% --- Executes during object creation, after setting all properties.
function edtDimZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edtDimZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edtDimX_Callback(hObject, eventdata, handles)
% hObject    handle to edtDimX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edtDimX as text
%        str2double(get(hObject,'String')) returns contents of edtDimX as a double


% --- Executes during object creation, after setting all properties.
function edtDimX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edtDimX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edtDimY_Callback(hObject, eventdata, handles)
% hObject    handle to edtDimY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edtDimY as text
%        str2double(get(hObject,'String')) returns contents of edtDimY as a double


% --- Executes during object creation, after setting all properties.
function edtDimY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edtDimY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popLabel.
function popLabel_Callback(hObject, eventdata, handles)
% hObject    handle to popLabel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popLabel contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popLabel
if handles.selectionMode==2 && handles.selectedFace>0
  handles.mesh.FaceVertexCData(handles.selectedFace) = handles.popLabel.Value;
elseif ~isempty(handles.selectedObject)
  handles.selectedObject.UserData.label = handles.popLabel.Value;
  handles.selectedObject.FaceColor = handles.labelColors(handles.popLabel.Value,:); 
end
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function popLabel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popLabel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function tlbFaces_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbFaces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% mapRange = [min(handles.model.vtx,[],1); max(handles.model.vtx,[],1)]';
% 
% handles.axMap.XLim(1) = min([handles.axMap.XLim(1) mapRange(1,1)]);
% handles.axMap.XLim(2) = max([handles.axMap.XLim(2) mapRange(1,2)]);
% handles.axMap.YLim(1) = min([handles.axMap.YLim(1) mapRange(2,1)]);
% handles.axMap.YLim(2) = max([handles.axMap.YLim(2) mapRange(2,2)]);
% handles.axMap.ZLim(1) = min([handles.axMap.ZLim(1) mapRange(3,1)]);
% handles.axMap.ZLim(2) = max([handles.axMap.ZLim(2) mapRange(3,2)]);
% handles.sldElevation.Min = handles.axMap.ZLim(1);
% handles.sldElevation.Max = handles.axMap.ZLim(2);

% --------------------------------------------------------------------
function tlbSegment_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbSegment (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = SegmentModel(handles);
guidata(hObject,handles);


% --- Executes on button press in btnUpdate.
function btnUpdate_Callback(hObject, eventdata, handles)
% hObject    handle to btnUpdate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

props = handles.selectedObject.UserData;
nprops = GetPropValues(handles);
props.diam = nprops.diam;
props.pos = nprops.pos;
props.orient = nprops.orient;
%
delete(handles.selectedObject);
handles.objects{props.id} = DrawObject(handles, props.shape, props);
handles.selectedObject = handles.objects{props.id};
guidata(hObject,handles);
 
function [response, occ] = RenderOccGrid(handles)
%% render obstacles from map to occ grid
  if ~isfield(handles,'occGrid') || isempty(handles.occGrid)
    if ~isfield(handles,'model') || isempty(handles.model.data)
      if 1
        %% import occ map
        [occ, bocc, mapResponse] = ReadStaticMap();
      else
        %% draw purely from sketch map
        mapH = handles.axMap.XLim(2) - handles.axMap.XLim(1);
        mapW = handles.axMap.YLim(2) - handles.axMap.YLim(1);
        occ = robotics.BinaryOccupancyGrid(mapW,mapH,10);
        occ.GridLocationInWorld = [handles.axMap.XLim(1) handles.axMap.YLim(1)];
      end
    else
      fdir = fileparts(handles.modelPath);
      ocfn = fullfile(fdir,'occ-maps.mat');
      if exist(ocfn,'file')
        %% load
        load(ocfn);
      else
        %% generate
        cpm = 5;
        fprintf('pcl2occ: generating occ map (%d cells per m)... ',cpm); tic;
        [bmap, pmap, hmap, gmap] = pcl2occ(handles.model.data, 1/cpm, 1.15);
        toc;
        save(ocfn,'bmap','pmap','hmap','gmap','cpm');
      end
      occ = robotics.BinaryOccupancyGrid(~bmap,cpm);
      occ.GridLocationInWorld = [handles.model.data.XLimits(1) handles.model.data.YLimits(1)];
      handles.occGrid = occ;
      guidata(handles.figMain,handles);
    end
  else
    occ = handles.occGrid;
  end
  mapResponse.Map = rosmessage('nav_msgs/OccupancyGrid');
  mapResponse.Map.writeBinaryOccupancyGrid(occ);
  %mapResponse.Map.writeOccupancyGrid(occ);
  mapResponse.Map.Header.FrameId = 'map';

%% surface mesh
tris = zeros(size(handles.mesh.Faces,1),6);
for ti = 1:size(handles.mesh.Faces,1)
  %% add triangle
  mtf = handles.mesh.Faces(ti,:);
  mtx = handles.mesh.Vertices(mtf,:);
  gtx = occ.world2grid(mtx(:,1:2))';
  gtx = gtx([2 1],:);
  tris(ti,:) = gtx(:);
end
%% draw terrain obstacles
driveable = handles.mesh.CData==3 | handles.mesh.CData==9;  % grass + pavement
mapSem = insertShape(zeros(occ.GridSize),'FilledPolygon',tris(~driveable,:),'Opacity',1,'SmoothEdges',false);
mapSem = flip(mean(mapSem,3)',2);
%mapSem = mean(mapSem,3)';
%mapSem = imdilate(mapSem,strel('disk',1,4));
mapResponse.Map.Data(mapResponse.Map.Data<=100 & mapSem(:)>0) = 100;
if 1
  %% clear free terain
  mapSem = insertShape(zeros(occ.GridSize),'FilledPolygon',tris(driveable,:),'Opacity',1,'SmoothEdges',false);
  mapSem = flip(mean(mapSem,3)',2);
  mapResponse.Map.Data(mapSem(:)>0) = 0;
end
%% objects to 2D
mapObj = zeros(occ.GridSize);
for i = 1:length(handles.objects)
  mobj = handles.objects{i};
  if ~isempty(mobj)
    if hasfield(mobj,'Faces')
      tris = cell(size(mobj.Faces,1),1);
      for ti = 1:size(mobj.Faces,1)
        %% add triangle
        mtf = mobj.Faces(ti,:);
        mtf = mtf(~isnan(mtf));
        mtx = mobj.Vertices(mtf,:);
        %mtx(:,2) = -mtx(:,2);
        gtx = occ.world2grid(mtx(:,1:2))';
        gtx = gtx([2 1],:);
        tris{ti} = gtx(:);
      end
      %% draw object
      mapObj = insertShape(mapObj,'FilledPolygon',tris','Opacity',1,'SmoothEdges',false);
    else
      tris = cell(size(mobj.XData,2),1);
      for ti = 1:size(mobj.XData,2)
        %% add triangle
        mtx = mobj.XData(:,ti);
        mtx(:,2) = mobj.YData(:,ti);
        gtx = occ.world2grid(mtx(:,[1 2]))';
        gtx = gtx([2 1],:);
        tris{ti} = gtx(:);
      end
      %% draw object
      mapObj = insertShape(mapObj,'FilledPolygon',tris','Opacity',1,'SmoothEdges',false);
    end
  end
end
mapObj = flip(mean(mapObj,3)',2);
%mapObj = imdilate(mapObj,strel('disk',1,4));
mapResponse.Map.Data(mapResponse.Map.Data<=100 & mapObj(:)>0) = 100;
%% show
occ = mapResponse.Map.readOccupancyGrid;
response = mapResponse.Map;
%% save
if ~isempty(handles.modelPath)
  fmap = (flip(reshape(response.Data,response.Info.Width,response.Info.Height)',1))==0;
  imwrite(fmap,fullfile(fileparts(handles.modelPath),'occ-map.png'));
end

function handles = ShowOccGrid(occmap, Xlims, Ylims, handles)
%% show map
docc = occmap+1;
docc(occmap>0) = 2;
% Xlims = occGrid.XWorldLimits;
% Ylims = occGrid.YWorldLimits;

handles.occMap = surface('XData',[Xlims(1) Xlims(2); Xlims(1) Xlims(2)], ...
  'YData',[Ylims(1) Ylims(1); Ylims(2) Ylims(2)],...
  'ZData',handles.axMap.ZLim(1)+handles.delta/2*[1 1; 1 1], ...
  'CData',imrotate(flip(docc,2),180),'FaceAlpha',0.5,...
  'FaceColor','texturemap','EdgeColor','none');

function [occGrid, occ] = GetOccMap(handles,cpm)
    
if ~isfield(handles,'occGrid') || isempty(handles.occGrid) || ~isfield(handles,'occMaps') || isempty(handles.occMaps)
  if ~hasfield(handles,'model') || isempty(handles.model.data)
    %% request map
    [occGrid, occ.BinMap] = ReadStaticMap();
    occ.ProbMap = occ.BinMap;
  else
    fdir = fileparts(handles.modelPath);
    ocfn = fullfile(fdir,'occ-maps.mat');
    if exist(ocfn,'file')
      %% load
      load(ocfn);
      occ.BinMap = bmap;
      occ.ProbMap = pmap;
      occ.HeightMap = pmap;
      occ.GroundMap = gmap;
    else
      %% generate
      cpm = 5;
      occ.BinMap = bmap;
      fprintf('pcl2occ: generating occ map (%d cells per m)... ',cpm); tic;
      [occ.BinMap, occ.ProbMap, occ.HeightMap, occ.GroundMap] = pcl2occ(handles.model.data, 1/cpm, 1.15);
      toc;
    end
    occGrid = robotics.BinaryOccupancyGrid(occ.BinMap,cpm);
    occGrid.GridLocationInWorld = [handles.model.data.XLimits(1) handles.model.data.YLimits(1)];
  end
else
  occGrid = handles.occGrid;
  occ = handles.occMaps;
end

% --------------------------------------------------------------------
function tlbRegister_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to tlbRegister (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cpm = 5;
    
if ~isfield(handles,'occGrid') || isempty(handles.occGrid)
  if ~hasfield(handles,'model') || isempty(handles.model.data)
    %% request map
    [occGrid, occ.BinMap] = ReadStaticMap();
    occ.ProbMap = occ.BinMap;
  else
    fdir = fileparts(handles.modelPath);
    ocfn = fullfile(fdir,'occ-maps.mat');
    if exist(ocfn,'file')
      %% load
      load(ocfn);
      occ.BinMap = bmap;
      occ.ProbMap = pmap;
      occ.HeightMap = pmap;
      occ.GroundMap = gmap;
    else
      %% generate
      cpm = 5;
      occ.BinMap = bmap;
      fprintf('pcl2occ: generating occ map (%d cells per m)... ',cpm); tic;
      [occ.BinMap, occ.ProbMap, occ.HeightMap, occ.GroundMap] = pcl2occ(handles.model.data, 1/cpm, 1.15);
      toc;
    end
    occGrid = robotics.BinaryOccupancyGrid(occ.BinMap,cpm);
    occGrid.GridLocationInWorld = [handles.model.data.XLimits(1) handles.model.data.YLimits(1)];
  end
else
  occGrid = handles.occGrid;
  occ = handles.occMaps;
end
%% show map
docc = occ.BinMap+1;
docc(occ.BinMap>0) = 2;
X = occGrid.XWorldLimits;
Y = occGrid.YWorldLimits;

handles.occMap = surface('XData',[X(1) X(2); X(1) X(2)], ...
  'YData',[Y(1) Y(1); Y(2) Y(2)],...
  'ZData',handles.axMap.ZLim(1)+handles.delta/2*[1 1; 1 1], ...
  'CData',imrotate(flip(docc,2),180),'FaceAlpha',0.5,...
  'FaceColor','texturemap','EdgeColor','none');
if 0
  %%
  figure; subplot(1,2,1); imagesc(occ,[-1 1]); axis image;
  subplot(1,2,2); occGrid.show();
end
params = inputdlg({'Initial angle [deg cw]:'},...
                   'Register sketch map',1, ...
                   {'0'});
ainit = str2double(params{1});
%%
if 1
  %% manual objects
  %occGrid = robotics.BinaryOccupancyGrid(occ.BinMap,cpm);
  docmap = double(occ.ProbMap)/double(max(occ.ProbMap(:)));
  docmap(docmap<0) = 0;
  poccg = robotics.OccupancyGrid(docmap,occGrid.Resolution);
  poccg.GridLocationInWorld = occGrid.GridLocationInWorld;
  figure(111); clf;poccg.show(); xlim(X); ylim(Y); hold on;
  title('Click map object centers in random order; right button to end');
  wobjpos = zeros(0,3);
  while true
    [gpx,gpy,gpb] = ginput(1);
    if gpb==1
      plot(gpx,gpy,'ro','MarkerSize',10);
      wobjpos = [wobjpos; gpx gpy 0]; 
    else
      break;
    end
  end
%   wobjpos = ginput();
%   plot(wobjpos(:,1),wobjpos(:,2),'ro'); 
%   wobjpos(:,3) = 0;
else
  %% detect objects
  bwdisk = strel('disk',2,4);
  occb = occ>0;
  occd = imopen(occb,bwdisk);
  occs = bwmorph(occd,'shrink',inf);
  %figure; imagesc(occd+10*occs); axis image;
  [gobjpos(:,2), gobjpos(:,1)] = find(occs>0);
  wobjpos = occGrid.grid2world(gobjpos);
  wobjpos(:,2) = -wobjpos(:,2);
  wobjpos(:,3) = 0;
end
%% map objects
mobjpos = zeros(length(handles.objects),3);
for i = 1:length(handles.objects)
  if isempty(handles.objects{i})
    mobjpos(i,:) = nan;
  else
    mobjpos(i,1:2) = handles.objects{i}.UserData.pos(1:2);
  end
end
%% icp
pcocc = pointCloud(wobjpos);
if isempty(handles.multiSelect) || length(handles.multiSelect)<3
  mobjidx = find(~isnan(mobjpos(:,1)));
else
  mobjidx = handles.multiSelect;
end
pcmap = pointCloud(mobjpos(mobjidx,:));
tinit = affine3d(eul2tform([deg2rad(ainit) 0 0]));  % manual angle
tinit.T(4,1:2) = wobjpos(1,1:2) - mobjpos(mobjidx(1),1:2); % init to first clicked
mobjidx = find(~isnan(mobjpos(:,1)));
if 0
  %% ICP
  [tform, tpcmap, trmse] = pcregrigid(pcmap,pcocc,'Verbose',true,'InlierRatio',0.9,'InitialTransform',tinit);
  fprintf('mapreg: icp transform %f rmse \n',trmse);
  figure(111);
  plot(tpcmap.Location(:,1),tpcmap.Location(:,2),'g.','MarkerSize',10);
  % apply to all objects
  tpcmap = pctransform(pointCloud(mobjpos(~isnan(mobjpos(:,1)),:)),tform);
  
  if 0
    %%
    figure; hold on;
    pcshow(tpcmap.Location,'r','MarkerSize',10);
    pcshow(pcocc.Location,'b','MarkerSize',10);
    figure(handles.figMain);
  end
  
else
  %% assuming 1-1 correspondences
  mapxyz = pcmap.Location(:,1:3)';
  occxyz = pcocc.Location(:,1:3)';
  [c, R, t] = ralign(mapxyz, occxyz);
  fprintf('mapreg: scaling %f \n',c);
  Tx = eye(4);
  Tx(1:3,1:3) = c*eye(3)*R';
  Tx(4,1:3) = t;
  
  tform = affine3d(Tx);
  tpcorig = mobjpos(~isnan(mobjpos(:,1)),:);
  tpcxyz = tform.transformPointsForward(tpcorig);
  tpcmap = pointCloud(tpcxyz);
  
  figure(111);
  tpcmapx = tform.transformPointsForward(mapxyz');
  plot(tpcmapx(:,1),tpcmapx(:,2),'g.','MarkerSize',10);

  
  %figure; plot3(tpcxyz(:,1),tpcxyz(:,2),tpcxyz(:,3),'r.'); hold on; plot3(tpcorig(:,1),tpcorig(:,2),tpcorig(:,3),'g.');
end
%%
disp(tform.T);
handles.modelTx = tform.T;
teul = tform2eul(tform.T);
disp(rad2deg(teul));
figure(handles.figMain);
%% transform map objects
for mi = 1:length(mobjidx)
  pid = mobjidx(mi);
  if ~isempty(handles.objects{pid})
    props = handles.objects{pid}.UserData;
    props.pos = tpcmap.Location(mi,:);
    props.orient(1) = props.orient(1) - rad2deg(teul(1));
    delete(handles.objects{pid});
    handles.objects{pid} = DrawObject(handles, props.shape, props);
    %cpos = pcmap.Location(mi,:);
    %npos = tpcmap.Location(mi,:);
    %dx = npos-cpos;
    %handles.objects{mobjidx(mi)} = MoveObject(handles.objects{mobjidx(mi)},dx);
  end
end
%% transform terrain
handles.mesh.Vertices = tform.transformPointsForward(handles.mesh.Vertices);
for i = 1:min(length(handles.nodeMarkers),size(handles.mesh.Vertices,1))
  handles.nodeMarkers{i}.XData = handles.mesh.Vertices(i,1); 
  handles.nodeMarkers{i}.YData = handles.mesh.Vertices(i,2);
  handles.nodeMarkers{i}.ZData = handles.mesh.Vertices(i,3)+handles.delta;
end
%%
handles.occGrid = occGrid;
handles.occMaps = occ;
guidata(hObject,handles);

function val = ToggleOnOff(val)
if strcmp(val,'off')
  val = 'on';
else
  val = 'off';
end
% --- Executes on key press with focus on figMain and none of its controls.
function figMain_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figMain (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
%modifiers = get(handles.figMain,'currentModifier');
switch eventdata.Key
  case 'm'
    %% publish map
    PublishOccGrid(hObject, eventdata, handles);
  case 'p'
    %% toggle point cloud vis
    if ~isempty(handles.cloud)
      if strcmp(handles.cloud.Visible,'off')
        handles.cloud.Visible = 'on';
      else
        handles.cloud.Visible = 'off';
      end
    end
  case 'e'
    %% toggle map edges
    if handles.mesh.EdgeAlpha < 0.25
      handles.mesh.EdgeAlpha = 1;
    else
      handles.mesh.EdgeAlpha = abs(handles.mesh.EdgeAlpha-0.25);
    end

    for i = 1:length(handles.nodeMarkers)
      m = handles.nodeMarkers{i};
      if handles.mesh.EdgeAlpha < 1 %strcmp(m.Visible,'off')
        m.Visible = 'off';
      else
        m.Visible = 'on';
      end
    end
    
  case 'o'
    %% load occ map
    if isfield(handles,'occMap') && ~isempty(handles.occMap)
      handles.occMap.Visible = ToggleOnOff(handles.occMap.Visible);
    else
      [handles.occGrid, handles.occMaps] = GetOccMap(handles,5);
      handles = ShowOccGrid(handles.occMaps.BinMap, handles.occGrid.XWorldLimits, handles.occGrid.YWorldLimits, handles);
      guidata(hObject,handles);
    end
    
  case '2'
    view(2);
  case '3'
    view(3);
end


% --- Executes when user attempts to close figMain.
function figMain_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figMain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.rosNode)
  handles.rosNode.delete;
  handles.rosNode = [];
end
if isfield(handles,'tmrOccGrid') && ~isempty(handles.tmrOccGrid)
  stop(handles.tmrOccGrid);
  delete(handles.tmrOccGrid);
end
% Hint: delete(hObject) closes the figure
delete(hObject);
