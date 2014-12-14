function [] = probability_generation(alpha,nbSkip)
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

  logsDirectory = '../data';
  nStates = 4;
  nRobots = 40;
  nProbability = 6;

  % Number of initial timesteps to skip
  % (making sure our system has reached its stable state)
  skipped = nbSkip *(nRobots+1);

  pattern = [logsDirectory, '/detail_simulation-', int2str(nRobots), '-alpha', int2str(alpha), '-*.csv'];
  filenames = dir(pattern);
  nExperiments = size(filenames, 1);

  P = zeros(nRobots, nProbability, nExperiments);

  for i = 1:nExperiments
    simulation = csvread([logsDirectory, '/', filenames(i).name], skipped + 1, 0);
    nTimesteps = length(unique(simulation(:, 1)));
    P(:, :, i) = experimentProbabilities(simulation);
  end;
  P=mean(P,3);
  save([logsDirectory,'/probability-alpha-',num2str(alpha),'.mat'], 'P')
end
