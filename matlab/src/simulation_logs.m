% Parse and exploit the simulation logs

%% CSV read
% Expected columns: time, robot id, state, number of neighbors
logsDirectory = '../../webots/data';
nRobots = 3;
% TODO: automatically parse all available CSV files form the directory
timestamp = 1417526895;

filename = [logsDirectory, '/simulation-', int2str(nRobots), '-', int2str(timestamp), '.csv'];

simulation = csvread(filename, 1, 0);

%% Plots
% Overall repartition of time spent in the different states
hist(simulation(:, 3));

% TODO: plots from the paper