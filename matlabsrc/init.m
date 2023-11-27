

src = fileparts(which('init'));
cd(src);

addpath(genpath(fullfile(src, 'external/')));
addpath(genpath(fullfile(src, 'fb_distance/')));
addpath(genpath(fullfile(src, 'interface/')));
addpath(genpath(fullfile(src, 'scripts/')));
addpath(genpath(fullfile(src, 'util/')));

clear src
install();
