%% make(index)
% Create mex-files. Select, which to compile, using the index. Call make('all') or make() to compile all mex-files.
%
%  (1)   -> problem interface
%  (4)   -> universal function interface
% (16)   -> svrspell
% (17)   -> stringmap
% (101)  -> getFunctionArguments
%
function make(index, varargin)
    [directory, cout] = getArguments(varargin);
    global path_to_eigen;
    path_to_eigen = fullfile(pwd(), '..', 'cppsrc', 'external', 'eigen-3.4.0');

    all = false;
    if nargin < 1 || (ischar(index) && strcmp(index, 'all'))
        all = true;
        index = -1;
    end
    source = pwd();
    subsource = cstrcat(source, '/interface');

    COMPILER = 'mex';
        OPTIONS = [ ...
            ' LDFLAGS=''$LDFLAGS ' ...
                sprintf(' -Wl,-rpath,%s/../build/lib', directory) ...
                ' -Wl,-rpath,', matlabroot, '/bin/glnxa64 ', ...
                ' -Wl,--export-dynamic ' ...
                ... ' -Wl,--no-undefined ' ...
                ' -Wl,-Bsymbolic-functions ' ...
                ... ' -Wl,-z,relro ' ...
                ' -Wl,-Bsymbolic ' ...
            '''' ...
            ' CXXFLAGS=''$CXXFLAGS ' ...
                ... ' -rdynamic ' ...
                ' -Dnspace=mbslib ' ...
            ' '' ' ...
            ' -g -R2018a ' ... % enable debugging
            ... ' -n ' ... % output the command for debuggin purposes instead of executing the command
            ];
    END = '.mexa64';

    chdir(directory);
    cleanupObj = onCleanup(@()cleanupChdir(source));

    if all || any(index == 1)
        output = makeMex(get_problemInterface_command(COMPILER, OPTIONS), 'Problem interface');
        move(directory, subsource, 'problem_interface_mex', END);
    end
    if all || any(index == 4)
        output = makeMex(get_universalFunction_command(COMPILER, OPTIONS), 'Universal function');
        move(directory, subsource, 'universalFunction_mex', END);
    end
    if all || any(index == 16)
        output = makeMex(get_svrspell_command(COMPILER, OPTIONS), 'Sequence similarity measure');
        move(directory, subsource, 'svrspell_mex', END);
    end
    if all || any(index == 17)
        output = makeMex(get_stringmap_command(COMPILER, OPTIONS), 'std::map<std::string, double> object');
        move(directory, subsource, 'stringmap_mex', END);
    end
    
    if all || any(index == 101)
        command = sprintf('mex -I%s getFunctionArguments.cpp', path_to_eigen);
        output = makeMex(command);
        move(directory, subsource, 'getFunctionArguments', END);
    end

    cellfun(@(x)delete(x), arrayfun(@(x)x.name, dir('*.o'), 'UniformOutput', false));
    clear cleanupObj
    
    file = fopen('make_output.log', 'w');
    if file >= 0
        closeFile = onCleanup(@()fclose(file));
        try
            fwrite(file, output);
        catch
        end
    end
    if cout
        disp(output);
    end

end

%% move(fromDirectory, toDirectory, file)
% move(fromDirectory, toDirectory, file, END)
function move(fromDirectory, toDirectory, file, varargin)
    if nargin >= 4
        END = varargin{1};
    end
    if isOctave() && ~exist('END', 'var')
        END = '.mex';
    elseif ~exist('END', 'var')
        END = '.mexa64';
    end
    if toDirectory ~= 0
        name1 = strcat(fromDirectory, '/', file, END);
        name2 = strcat(toDirectory, '/', file, END);
        if isOctave()
            if ~is_absolute_filename(name1) || ~is_absolute_filename(name2)
                warning('Filename not valid. Aborting move.');
                return;
            end
        end
        status = system(cstrcat('mv ', ' ', name1, ' ', name2));
    end
    succ = (status == 0 && ~isOctave()) || (~status && isOctave);
    if (isscalar(toDirectory) && toDirectory == 0) || ~succ
        warning('mv mex-file: Unable to move the file to the current directory');
    end
end


%% get_problemInterface_command
function command = get_problemInterface_command(COMPILER, OPTIONS)
global path_to_eigen;

command = strcat( ...
    COMPILER, ...
    OPTIONS, ... 
    ... %%% includes: %%%
    sprintf(' -I%s ', path_to_eigen), ...
    ' -I../ ', ...
    ' -I../../../external/cpp/mujoco_from_git/include/ ', ...
    ... %%% linked libraries: %%%
    ' -L../build/lib ', ...
    ' -lfunctionLib ', ...
    ' -lmodelLib ', ...
    ' -lboost_serialization ', ...
    ' -ldl ', ...
    ... %%% code files: %%%
    ' problem_interface_mex.cpp ', ...
    ' MFun.cpp', ...
    ' SingleVariableMFun.cpp', ...
    ' ../external/lodepng/lodepng.cpp', ...
    '');
end


%% get_universalFunction_command
function command = get_universalFunction_command(COMPILER, OPTIONS)
global path_to_eigen;
command = strcat( ...
    COMPILER, ...
    OPTIONS, ...
    ...  %%% includes: %%%
    sprintf(' -I%s ', path_to_eigen), ...
    ' -I../ ', ...
    ...  %%% linked libraries: %%%
    ' -L../build/lib ', ...
    ' -lfunctionLib ', ...
    ' -lboost_serialization ', ...
    ...  %%% code files: %%%
    ' universalFunction_mex.cpp ', ...
    '');
end


%% get_svrspell_command
function command = get_svrspell_command(COMPILER, OPTIONS)
global path_to_eigen;
command = strcat( ...
    COMPILER, ...
    OPTIONS, ...
    ... %%% includes: %%%
    sprintf(' -I%s ', path_to_eigen), ...
    ' -I../ ', ...   
    ... %%% linked libraries: %%%
    ... %%% code files: %%%
    ' svrspell_mex.cpp ', ...
    ' ../util/elzinga_grid.cpp ', ...
    '');
end

%% get_stringmap_command
function command = get_stringmap_command(COMPILER, OPTIONS)
global path_to_eigen;
command = strcat( ...
    COMPILER, ...
    OPTIONS, ...
    ... %%% output name: %%%
    ' -output stringmap_mex ', ...
    ... %%% includes: %%%
    sprintf(' -I%s ', path_to_eigen), ...
    ' -I../ ', ...   
    ... %%% linked libraries: %%%
    ... %%% code files: %%%
    ' map_interface_mex.cpp ', ...
    '');
end


%% makeMex(command, name='');
function output = makeMex(command, varargin)
    if nargin >= 2
        name = varargin{1};
    else
        name = '';
    end
    output = evalc(command);
    res = ~endsWith(strtrim(output), 'MEX completed successfully.');

    if ~res
        res = 'Successfully';
    else
        res = 'Unsuccessfully';
    end
    fprintf('%s: %s executed system command:\n $ %s\n\n', name, res, command);
end


function cleanupChdir(path)
    chdir(path);
end


function isO = isOctave()
    isO = false;
end


function [directory, cout] = getArguments(args)
    if length(args) > 0 && ~isempty(args{0})
        directory = args{0};
    else
        directory = fullfile(pwd(), '..', 'cppsrc', 'matlab_interface');
    end

    cout = false;
    if length(args) > 1
        if length(args) > 2
            if strcmp(args{1}, 'stdout')
                cout = args{2};
            end
        end
    end
end
