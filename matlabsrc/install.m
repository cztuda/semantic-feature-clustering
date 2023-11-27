
%% install missing dependencies

if isempty(listdir('external/altmany-export_fig'))
    installDependency('https://github.com/altmany/export_fig/archive/refs/heads/master.zip', 'altmany-export_fig', 'export_fig-master');
end

if isempty(listdir('external/fig2svg'))
    installDependency('https://github.com/kupiqu/fig2svg/archive/refs/heads/master.zip', 'fig2svg', 'fig2svg-master');
end

if isempty(listdir('external/cmap'))
    installDependency('https://github.com/tsipkens/cmap/archive/refs/heads/master.zip', 'cmap', 'cmap-master');
end

if isempty(listdir('external/iosr'))
    installDependency('https://github.com/IoSR-Surrey/MatlabToolbox/archive/4bff1bb2da7c95de0ce2713e7c710a0afa70c705.zip', 'iosr', ...
    'MatlabToolbox-4bff1bb2da7c95de0ce2713e7c710a0afa70c705');
    cellfun(@(c)rmdir(c, 's'), ...
        fullfile('external', 'iosr', '+iosr', {'+acoustics', '+auditory', '+bss', '+dsp', '+figures', '+general', '+svn'}));
    delete(fullfile('external', 'iosr', '+iosr', 'Contents.m'));
    delete(fullfile('external', 'iosr', '+iosr', 'install.m'));
end


%% compile missing mex files

if exist('getFunctionArguments', 'file') ~= 3
    fprintf('Compiling the following mex file: %20s\n', 'getFunctionArguments');
    make(101);
end

if exist('problem_interface_mex', 'file') ~= 3
    fprintf('Compiling the following mex file: %20s\n', 'problem_interface_mex');
    make(1);
end

if exist('universalFunction_mex', 'file') ~= 3
    fprintf('Compiling the following mex file: %20s\n', 'universalFunction_mex');
    make(4);
end

if exist('stringmap_mex', 'file') ~= 3
    fprintf('Compiling the following mex file: %20s\n', 'stringmap_mex');
    make(16);
end

if exist('svrspell_mex', 'file') ~= 3
    fprintf('Compiling the following mex file: %20s\n', 'svrspell_mex');
    make(17);
end



%% utility functions

function installDependency(url, foldername, masterfoldername)
    fprintf('Installing the following dependency: %s\n', foldername);
    src = pwd();
    cdBack = onCleanup(@()cd(src));
    cd('external/');
    downloadDependency(url, foldername, masterfoldername);
    clear('cdBack')
    addpath(genpath(fullfile(src, 'external/')), foldername);
    fprintf('\b\b\b\b  -> done\n')
end

function downloadDependency(url, foldername, masterfoldername)
    websave('tmp_archive', url);
    unzip('tmp_archive.zip', foldername);
    if ~isempty(masterfoldername)
        movefile([foldername, filesep(), masterfoldername, filesep(), '*'], foldername);
        rmdir([foldername, filesep(), masterfoldername]);
    end
    delete('tmp_archive.zip');
end
