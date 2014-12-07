% Parse and exploit the simulation logs
% Expected format:
%  time, state, 0 neighbors, 1 neighbor, 2 neighbors, ...
%  With, for each timestamp, one line per state.
%
% States numbering:
%   FORWARD = 0,
%   FORWARD_AVOIDANCE = 1,
%   COHERENCE = 2,
%   COHERENCE_AVOIDANCE = 3
clearvars;

logsDirectory = '../../webots/data';
nStates = 4;
nRobots = 10;
alpha = 4;

pattern = [logsDirectory, '/simulation-', int2str(nRobots), '-alpha', int2str(alpha), '-*.csv'];
filenames = dir(pattern);
nExperiments = size(filenames, 1);

% TODO: average over experiments (with error bars)
stats = zeros(nRobots, nStates, nExperiments);
for i = 1:nExperiments
    simulation = csvread([logsDirectory, '/', filenames(i).name], 1, 0);
    % TODO: use only data starting *after* the steady state has been reached
    nTimesteps = length(unique(simulation(:, 1)));
    stats(:, :, i) = experiment_stats(simulation);
end;

averaged = mean(stats, 3);
errors = std(stats, 0, 3);
averaged = averaged(:, :, 1);

figure;
hold on;
for k = 1:4
    errorbar(0:(nRobots-1), averaged(:, k), errors(:, k));
end;
title(['Average over ', int2str(nExperiments), ' experiments']);
xlabel('Connections (number of neighbors)');
ylabel('Number of robots');
axis([0 nRobots 0 max(averaged(:, 4))+1]);
legend('Forward', 'Coherence', 'Avoidance', 'Any state');



