% Parse and exploit the simulation logs

%% CSV read
% Expected columns: time, robot id, state, number of neighbors
% States numbering:
%   FORWARD = 0,
%   FORWARD_AVOIDANCE = 1,
%   COHERENCE = 2,
%   COHERENCE_AVOIDANCE = 3

logsDirectory = '../../webots/data';
nRobots = 10;
alpha = 4;
% TODO: automatically parse all available CSV files form the directory
timestamp = 1417595583;

filename = [logsDirectory, '/simulation-', int2str(nRobots), '-alpha', int2str(alpha), '-', int2str(timestamp), '.csv'];

simulation = csvread(filename, 1, 0);

% TODO: make sure to use only data starting *after* the steady state has been reached
nTimesteps = length(unique(simulation(:, 1)));

%% Plots
% Overall repartition of time spent in the different states
hist(simulation(:, 4));

% Main plot: average number of robots having each number of
% neighbors (one curve for each state)
robotsPerNeighbors = zeros(nRobots, 3);
for i = 1:nRobots
    idx = (simulation(:, 4) == (i - 1));
    % Forward, coherence, avoidance
    robotsPerNeighbors(i, 1) = sum(simulation(idx, 3) == 0);
    % Coherence
    robotsPerNeighbors(i, 2) = sum(simulation(idx, 3) == 2);
    % Avoidance
    robotsPerNeighbors(i, 3) = sum((simulation(idx, 3) == 1) | (simulation(idx, 3) == 3));
end;
% Take the average over the number of timesteps
robotsPerNeighbors = robotsPerNeighbors / nTimesteps;
% Last column: sum over states
robotsPerNeighbors(:, 4) = sum(robotsPerNeighbors, 2);

plot(0:(nRobots-1), robotsPerNeighbors);
xlabel('Connections (number of neighbors)');
ylabel('Number of robots');
axis([0 nRobots 0 max(robotsPerNeighbors(:, 4))+3]);
legend('Forward state', 'Coherence state', 'Avoidance state', 'Sum');