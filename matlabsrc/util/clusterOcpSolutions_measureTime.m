%% [T, similarIndices, coph, D, Z] = clusterOcpSolutions_measureTime(opts, varargin)
% Same as clusterOcpSolutions.
% 
% Measure times and save them in the folder /tmp/trajClustering
%
function [T, similarIndices, coph, D, Z] = clusterOcpSolutions_measureTime(opts, varargin)

    [T,similarIndices,coph,D, Z] = clusterOcpSolutions(opts, varargin{:}, 'measureTime', true);

    global measured_times_1 measured_times_2
    persistent counter;
    if isempty(counter)
        counter = 1;
    end
    if ~isfolder('/tmp/trajClustering')
        mkdir('/tmp/trajClustering');
    end
    save(['/tmp/trajClustering/measuredTimes_', num2str(counter, '%02i')], 'measured_times_1', 'measured_times_2');
    counter = counter + 1;
end