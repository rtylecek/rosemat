function has = hasfield(obj,fld)
has = 0;
if isfield(obj,fld)
  has = 1; return;
else
  ff = fieldnames(obj);
  for i = 1:length(ff)
    if strcmp(ff{i},fld)
      has = 1; return;
    end
  end
end