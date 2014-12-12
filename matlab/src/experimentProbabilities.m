function P = experimentProbabilities(simulation)

  %INPUT:
  %   simulation: contents of a CSV file from a single Webots simulation run
  %   Expected format:
  %     Time, Robot ID, Robot state, Number of neighbors
  %OUTPUT:
  %   P: estimation of the Probabilities of changing state for each number of
  %     connections. The estimated Probabilities are :
  %      BOTH | FORWARD | COHERENCE
  %     [  Pa    Pg Pl    Pr Pf Pla ]
nP=7;
nC=9;
nRobots = length(unique(simulation(:,2)));
nTimesteps = length(unique(simulation(:, 1)));
%keep the change between two timesteps for each robots
%resp. : numbConnection, numbChange, timestep
nChange=zeros(nRobots,nC,nTimesteps-1);

%this variable regroups :
%IDRobots, StateRobots, numbNeighborRobots
infoRobots=zeros(nRobots,3,nTimesteps);



%first iteration to avoid a comparaison with nothing
i=1;
%interogation is the ID of Robots always in the same Order
for j=1:3
  infoRobots(:,j,i) = simulation( rowTimestep+1:rowTimestep + 1 + nRobots , 1+j );
end


%in case there aren't ordered : order with DIM 1 and MOD 'ascend'
%[idRobots(:,i) stateRobots(:,i) neighborRobots(:,i)] = sort([idRobots(:,i) stateRobots(:,i) neightorRobots(:,i)],1 , 'ascend');
for i=1:nTimesteps
  %first row of
  rowTimestep = (i - 1 ) * nRobots + 1;
  %iteration over each timestep

  %Q: Is the IDRobots always in the same Order ?
  for j=1:3
    infoRobots(:,j,i) = simulation( rowTimestep:rowTimestep + nRobots , 1+j );
  end

  %in case there aren't ordered : order with DIM 1 and MOD 'ascend'
  infoRobots(:,:,i) = sortrows( infoRobots(:,:,i),1,'ascend' );

  nChange(:,:,i-1) = experimentChange(infoRobots(:,:,i),infoRobots(:,:,i-1));
end
