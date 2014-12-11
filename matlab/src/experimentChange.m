function nChange = experimentChange(cInfos, pInfos)
  %INPUT:
  %   cInfos, pInfos : current and previous informations for each robots
  %    regarding (COLUMN) :
  %         IDRobots, StateRobots, numbNeighborRobots
  %OUTPUT:
  %   nChange : vector

  nC=8;
  %Na Nrest| Ng Nl Nrest | Nr Nf Nla Nrest
  nChange = zeros(nRobots,nP);
  nRobots=size(currentInfos,1);
  for i=1:nRobots
    cState=cInfos(i,2);
    pState=pInfos(i,2);
    cConnect=cInfos(i,3);
    pConnect=pInfos(i,3);
    %Forward$

    switch pState

    case: 0 %robot in FORWARD

      switch cState

      case:0 %the robot is still in FORWARD
        %so 3 possibilities
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pConnect,3) = nChange(pConnect,3) + (cConnect-pConnect);
          %2. Loss a/several connections
        else cConnect < pConnect
          nChange(pConnect,4) = nChange(pConnect,4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pConnect,5) = nChange(pConnect,5) + 1;
        end
        break;
        %go into avoidance

      case:1
        nChange(pConnect,1)= nChange(pConnect,1) + 1;
        %Could also gain/loose connections
        if cConnect > pConnect
          nChange(pConnect,3) = nChange(pConnect,3) + (cConnect-pConnect);
          %2. Loss a/several connections
        else cConnect < pConnect
          nChange(pConnect,4) = nChange(pConnect,4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pConnect,2) = nChange(pConnect,2) + 1;
        end

        break;
      case:2 %From Forward to Coherence state (means loss of a connection)
        nChange(pConnect,4) = nChange(pConnect,4) + (pConnection-cConnection);
      case:3 %From Forward to Coherence avoidance state
        nChange(pConnect,4) = nChange(pConnect,4) + (pConnection-cConnection);
        nChange(pConnect,1) = nChange(pConnect,1) + 1;
      otherwise
        error('no such State');
        %Shouldn't happen
        break;

      break;

    case: 1 %was in Forward Avoidance

      break;
    case:2 %was in Coherence state

      break;
    case:3 %was in Coherence Avoidance State

      break;
    otherwise
      error('no such State');
      %shouldn't happen
      break;


  end


end
