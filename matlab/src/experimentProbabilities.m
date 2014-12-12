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
  Pa=zeros(nRobots,1);
  Pg=zeros(nRobots,1);
  Pl=zeros(nRobots,1);
  Pr=zeros(nRobots,1);
  Pf=zeros(nRobots,1);
  Pla=zeros(nRobots,1);


  %first iteration to avoid a comparaison with nothing
  i=1;
  %interogation is the ID of Robots always in the same Order
  for j=1:3
    infoRobots(:,j,i) = simulation( 1 : nRobots , 1+j );
  end

  for i=2:nTimesteps
    %first row of
    rowTimestep = (i - 1 ) * nRobots + 1;
    %iteration over each timestep

    %Q: Is the IDRobots always in the same Order ?
    for j=1:3
      infoRobots(:,j,i) = simulation( rowTimestep:rowTimestep + nRobots-1 , 1+j );
    end


    nChange(:,:,i-1) = experimentChange(infoRobots(:,:,i),infoRobots(:,:,i-1));
  end
  nChange;
  %Compute the probabilities from the nChange 3 dimention matrix
  totChange=sum(nChange,3);

  totChangeA=sum(totChange(:,2:end),2);
  totChangeF=sum(totChange(:,2:5),2);
  totChangeC=sum([totChange(:,2),totChange(:,6:9)],2);
  totChangeA(totChangeA==0)=1;
  totChangeF(totChangeF==0)=1;
  totChangeC(totChangeC==0)=1;
  %TODO:check probabilities : is it
  Pa=totChange(:,1)./totChangeA;
  Pg=totChange(:,3)./totChangeF;
  Pl=totChange(:,4)./totChangeF;
  Pr=totChange(:,6)./totChangeC;
  Pf=totChange(:,7)./totChangeC;
  Pla=totChange(:,8)./totChangeC;

  figure
  plot([0:39]',Pa)
  hold on
  plot([0:39]',Pg)
  plot([0:39]',Pl)
  plot([0:39]',Pr)
  plot([0:39]',Pf)
  plot([0:39]',Pla)
  P=[Pa Pg Pl Pr Pf Pla];

  end
