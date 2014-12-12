function [] = probability_generation(alpha)
% Parse and exploit the simulation logs with LOG_DETAILS on
% Expected format:
%  Time, Robot ID, Robot state, Number of neighbors
%  With, for each timestamp, one line per Robot ID.
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
nProbability = 6;
nbSkip=20;
% Number of initial timesteps to skip
% (making sure our system has reached its stable state)
skipped = nbSkip *(nRobots+1);

%TODO: check if it isn't right but there you skip only 10 lines!? not 10 timesteps.
%could it be like :
% skipped = 10;
% skippedRow = skipped * (nStates + 1)

% Symbols for each curve of the plot
%symbols = ['x', '*', 'v', '.'];

pattern = [logsDirectory, '/detail_simulation-', int2str(nRobots), '-alpha', int2str(alpha), '-*.csv'];
filenames = dir(pattern);
nExperiments = size(filenames, 1);

P = zeros(nRobots, nProbability, nExperiments);

for i = 1:nExperiments
  simulation = csvread([logsDirectory, '/', filenames(i).name], skipped + 1, 0);
  nTimesteps = length(unique(simulation(:, 1)));
  P(:, :, i) = experimentProbabilities(simulation);
end;
save([logsDirectory,'/probability-alpha-',num2str(alpha),'.mat'], 'P')
end
