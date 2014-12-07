function robotsPerNeighbors = experiment_stats(simulation)
%
% INPUT:
%   simulation: contents of a CSV file from a single Webots simulation run
% OUTPUT:
%   robotsPerNeighbors: Average number of robots having a given number of
%     neighbors for each state (neighbors in line, state in column).
%     The two avoidance subsumed states are merged into one. The last
%     column counts regardless of the state.

    NUM_STATES = 4;
    nRobots = size(simulation, 2) - 2;
    nTimesteps = length(unique(simulation(:, 1)));

    % Overall repartition of time spent in the different states
    %averageRobotsPerState = sum(simulation(:, 2:5)) ./ nTimesteps;
    %plot(0:3, averageRobotsPerState, '.', 'MarkerSize', 20);

    % Average number of robots having each number of
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
end