function addMLROtoPath

% Get the current course folder
rootDir = fileparts(mfilename('fullpath'));

% Populate list of folders to add to path
path2add = {};
path2add{end+1} = rootDir;
path2add{end+1} = fullfile(rootDir,'chapter2');
path2add{end+1} = fullfile(rootDir,'chapter3');
path2add{end+1} = fullfile(rootDir,'chapter4');
path2add{end+1} = fullfile(rootDir,'chapter2','exercise');
path2add{end+1} = fullfile(rootDir,'chapter3','exercise');
path2add{end+1} = fullfile(rootDir,'chapter4','exercise');
path2add{end+1} = fullfile(rootDir,'simulator');

% Add folders to the path
addpath(path2add{:},'-end');