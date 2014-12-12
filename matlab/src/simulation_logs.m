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

%clearvars;
clear all
%% Configuration

logsDirectory = '../data';
nStates = 4;
nRobots = 40;
alpha = 15;

% Number of initial timesteps to skip
% (making sure our system has reached its stable state)
skipped = 10;

%TODO: check if it isn't right but there you skip only 10 lines!? not 10 timesteps.
%could it be like :
% skipped = 10;
% skippedRow = skipped * (nStates + 1)


% Symbols for each curve of the plot
symbols = ['x', '*', 'v', '.'];

pattern = [logsDirectory, '/simulation-', int2str(nRobots), '-alpha', int2str(alpha), '-*.csv'];
filenames = dir(pattern);
nExperiments = size(filenames, 1);

%% Handle the stats

stats = zeros(nRobots, nStates, nExperiments);
for i = 1:nExperiments
    simulation = csvread([logsDirectory, '/', filenames(i).name], skipped + 1, 0);
    nTimesteps = length(unique(simulation(:, 1)));
    stats(:, :, i) = experiment_stats(simulation);
end;

averaged = mean(stats, 3);
errors = std(stats, 0, 3);
averaged = averaged(:, :, 1);

%% Plot

if(nExperiments > 0)
    figure;
    hold on;

    errorbar(0:(nRobots-1), averaged(:, 1), errors(:, 1), [symbols(1), '-']);
    errorbar(0:(alpha-1), averaged(1:alpha, 2), errors(1:alpha, 2), [symbols(2), '-']);
    errorbar(0:(nRobots-1), averaged(:, 3), errors(:, 3), [symbols(3), '-']);
    errorbar(0:(nRobots-1), averaged(:, 4), errors(:, 4), [symbols(4), '-']);

    title(['Average over ', int2str(nExperiments), ' experiments']);
    xlabel('Connections (number of neighbors)');
    ylabel('Number of robots');
    axis([0 nRobots 0 max(averaged(:, 4))+1]);
    legend('Forward', 'Coherence', 'Avoidance', 'Any state');
else
    fprintf('No simulation results found for alpha = %d with %d robots.\n', alpha, nRobots);
end;
