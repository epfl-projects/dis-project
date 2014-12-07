% Parse and exploit the simulation logs

%% CSV read
% Expected columns: time, robot id, state, number of neighbors
% States numbering:
%   FORWARD = 0,
%   FORWARD_AVOIDANCE = 1,
%   COHERENCE = 2,
%   COHERENCE_AVOIDANCE = 3
NUM_STATES = 4;

logsDirectory = '../../webots/data';
nRobots = 10;
alpha = 4;
% TODO: automatically parse all available CSV files form the directory
timestamp = 1417962356;

filename = [logsDirectory, '/simulation-', int2str(nRobots), '-alpha', int2str(alpha), '-', int2str(timestamp), '.csv'];

simulation = csvread(filename, 1, 0);

% TODO: make sure to use only data starting *after* the steady state has been reached
nTimesteps = length(unique(simulation(:, 1)));

%% Plots
% Overall repartition of time spent in the different states
averageRobotsPerState = sum(simulation(:, 2:5)) ./ nTimesteps;
plot(0:3, averageRobotsPerState, '.', 'MarkerSize', 20);

% Main plot: average number of robots having each number of
% neighbors (one curve for each state)
robotsPerNeighbors = zeros(nRobots, NUM_STATES);
for i = 0:(NUM_STATES-1)
    % Select lines about this state
    thisState = simulation(simulation(:, 2) == i, 3:end);
    robotsPerNeighbors(:, i+1) = sum(thisState, 1)' ./ nTimesteps;
end;
% Merge the two (subsumed) avoidance states
robotsPerNeighbors(:, 2:3) = [robotsPerNeighbors(:, 3), robotsPerNeighbors(:, 2) + robotsPerNeighbors(:, 4)];
% Total
robotsPerNeighbors(:, 4) = sum(robotsPerNeighbors(:, 1:3), 2);


plot(0:(nRobots-1), robotsPerNeighbors);
xlabel('Connections (number of neighbors)');
ylabel('Number of robots');
axis([0 nRobots 0 max(robotsPerNeighbors(:, 4))+1]);
legend('Forward', 'Coherence', 'Avoidance', 'Any state');

