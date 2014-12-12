function nChange = experimentChange(cInfos, pInfos)
  %INPUT:
  %   cInfos, pInfos : current and previous informations for each robots
  %    regarding (COLUMN) :
  %         IDRobots, StateRobots, numbNeighborRobots
  %OUTPUT:
  %   nChange : vector

  nC=9;
  nRobots=size(cInfos,1);
  %Na Nrest| Ng Nl Nrest | Nr Nf Nla Nrest
  nChange = zeros(nRobots,nC);

  for i=1:nRobots
    cState=cInfos(i,2);
    pState=pInfos(i,2);
    cConnect=cInfos(i,3);
    pConnect=pInfos(i,3);
    pCindex =pConnect+1;
    %Forward

    switch pState

    case {0} %robot in FORWARD

      switch cState

      case {0} %the robot is still in FORWARD
        %so 3 possibilities
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pCindex, 5) = nChange(pCindex, 5) + 1;
        end


      case {1} %go into avoidance
        nChange(pCindex, 1)= nChange(pCindex, 1) + 1;
        %Could also gain/loose connections
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, 2) = nChange(pCindex, 2) + 1;
        end


      case {2} %From Forward to Coherence state (means loss of a connection)
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, 5) = nChange(pCindex, 5) + 1;
        end

      case {3} %From Forward to Coherence avoidance state
        nChage(pCindex, 1) = nChange(pCindex, 1) + 1;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, 2) = nChange(pCindex, 2) + 1;
        end
      otherwise
        error('no such State');
        %Shouldn't happen
      endswitch



    case {1} %was in Forward Avoidance
      %I don't know exactly where they could go from AF state
      switch cState
      case {0} %Go back into Forward, since TA is fixed no probabilities exept if it gain/loss connecitons
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
        %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
        %3. no change in number of connections (keep track)
        else
          nChange(pCindex, 5) = nChange(pCindex, 5) + 1;
        end

      case {1} %stay in AF state
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pCindex, 2) = nChange(pCindex, 2) + 1;
        end

      case {2} %Go into Coherence state
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pCindex, 5) = nChange(pCindex, 5) + 1;
        end

      case {3} %Go into Coherence Avoidance
        nChange(pCindex, 1) = nChange(pCindex, 1) + 1 ;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 3) = nChange(pCindex, 3) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 4) = nChange(pCindex, 4) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, 2) = nChange(pCindex, 2) + 1;
        end

      otherwise
        error('No such state');
      endswitch

    case {2} %was in Coherence state
      switch cState
      case {0}
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 6) = nChange(pCindex, 6) + (cConnect-pConnect);
          %2. Loss a/several connections => Nla
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections => fail to recover
        else
          nChange(pCindex, 7) = nChange(pCindex, 7) + 1;
        end

      case {1} %go in Avoidance Forward
        nChange(pCindex, 1)=nChange(pCindex, 1) + 1;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 6) = nChange(pCindex, 6) + (cConnect-pConnect);
          %2. Loss a/several connections => Nla
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections => fail to recover
        else
          nChange(pCindex, 7) = nChange(pCindex, 7) + 1;
        end

      case {2} %Stay in Coherence
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 7) = nChange(pCindex, 7) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, 9) = nChange(pCindex, 9) + 1;
        end

      case {3} %Go in avoidance
        nChange(pCindex, 1) = nChange(pCindex, 1) + 1;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, 7) = nChange(pCindex, 7) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, 9) = nChange(pCindex, 9) + 1;
        end

      otherwise
        error('No such state');
      endswitch

    case {3} %was in Coherence Avoidance State
      switch cState
      case {0} % Go in forward (don't know if possible)
        %it would be like from Coherence
        %1. Gain a/several connections (recover connections)
        if cConnect > pConnect
          nChange(pCindex, 7) = nChange(pCindex, 7) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, 9) = nChange(pCindex, 9) + 1;
        end

      case {1}
        nChange(pCindex, 1) = nChange(pCindex, 1) + 1;
        %1. Gain a/several connections (recover connections)
        if cConnect > pConnect
          nChange(pCindex, 7) = nChange(pCindex, 7) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, 9) = nChange(pCindex, 9) + 1;
        end

      case {2} %finish avoidance and go back in Coherence
        %1. Gain a/several connections (recover connections)
        if cConnect > pConnect
          nChange(pCindex, 7) = nChange(pCindex, 7) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, 9) = nChange(pCindex, 9) + 1;
        end

      case {3} %stay in avoidance
        %1. Gain a/several connections (recover connections)
        if cConnect > pConnect
          nChange(pCindex, 7) = nChange(pCindex, 7) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, 8) = nChange(pCindex, 8) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, 9) = nChange(pCindex, 9) + 1;
        end

      otherwise
        error('No such State');

    endswitch
    otherwise
      error('no such State');
      %shouldn't happen
    endswitch



  end


end
