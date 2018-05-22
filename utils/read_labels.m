function handles = read_labels( dir, handles )
%READLABELS Read semantic shape labels and colors into handles struct
%           from yaml files in dir
%   labelNames
%   labelIDs
%   labelColors
%% read labels
handles.labels = YAML.read(fullfile(dir, 'labels.yaml'));
handles.colors = YAML.read(fullfile(dir, 'colors.yaml'));
groupNames = fieldnames(handles.labels.Semantic);
id = 1;
for g = 1:length(groupNames)
  gn = groupNames{g};
  gel = handles.labels.Semantic.(gn);
  if isstruct(gel)
    groupLabels = fieldnames(gel);
    for gi = 1:length(groupLabels)
      gl = groupLabels{gi};
      handles.labelNames{id} = [gn '-' gl];
      handles.labelIDs(id) = handles.labels.Semantic.(gn).(gl);
      handles.labelColors(id,1:3) = handles.colors.Semantic.(gn).(gl);
      id = id + 1;
    end
  else
    handles.labelNames{id} = gn;
    handles.labelIDs(id) = handles.labels.Semantic.(gn);
    handles.labelColors(id,1:3) = handles.colors.Semantic.(gn);
    id = id + 1;
  end
end
%% mapping
if isfield(handles.labels,'Mapping')
  handles.mapping = zeros(255,1);
  groupNames = fieldnames(handles.labels.Mapping);
  for g = 1:length(groupNames)
    gn = groupNames{g};
    gel = handles.labels.Mapping.(gn);
    if isstruct(gel)
      groupLabels = fieldnames(gel);
      for gi = 1:length(groupLabels)
        gl = groupLabels{gi};
        lmap = handles.labels.Mapping.(gn).(gl);
        handles.mapping(lmap(1)) = lmap(2);
      end
    end
  end
end