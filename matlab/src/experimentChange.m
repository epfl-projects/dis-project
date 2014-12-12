function nChange = experimentChange(cInfos, pInfos)
  %INPUT:
  %   cInfos, pInfos : current and previous informations for each robots
  %    regarding (COLUMN) :
  %         IDRobots, StateRobots, numbNeighborRobots
  %OUTPUT:
  %   nChange : vector

  nC=10;
  nRobots=size(cInfos,1);
  %Na Naf Nac| Ng Nl Nfrest | Nr Nf Nla Ncrest
  nChange = zeros(nRobots,nC);

  Naindex = 1;
  Nafindex = 2;
  Nacindex = 3;
  Ngindex = 4;
  Nlindex = 5;
  Nfrestindex = 6;
  Nrindex = 7;
  Nfindex = 8;
  Nlaindex = 9;
  Ncrestindex = 10;

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
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pCindex, Nfrestindex) = nChange(pCindex, Nfrestindex) + 1;
        end


      case {1} %go into avoidance
        nChange(pCindex, Naindex)= nChange(pCindex, Naindex) + 1;
        %Could also gain/loose connections
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, Nafindex) = nChange(pCindex, Nafindex) + 1;
        end


      case {2} %From Forward to Coherence state (means loss of a connection)
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, Nfrestindex) = nChange(pCindex, Nfrestindex) + 1;
        end

      case {3} %From Forward to Coherence avoidance state
        nChage(pCindex, 1) = nChange(pCindex, Naindex) + 1;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, Nacindex) = nChange(pCindex, Nacindex) + 1;
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
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
        %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
        %3. no change in number of connections (keep track)
        else
          nChange(pCindex, Nfrestindex) = nChange(pCindex, Nfrestindex) + 1;
        end

      case {1} %stay in AF state
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pCindex, Nafindex) = nChange(pCindex, Nafindex) + 1;
        end

      case {2} %Go into Coherence state
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track)
        else
          nChange(pCindex, Nfrestindex) = nChange(pCindex, Nfrestindex) + 1;
        end

      case {3} %Go into Coherence Avoidance
        nChange(pCindex, Naindex) = nChange(pCindex, Naindex) + 1 ;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Ngindex) = nChange(pCindex, Ngindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlindex) = nChange(pCindex, Nlindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in avoidance)
        else
          nChange(pCindex, Nafindex) = nChange(pCindex, Nafindex) + 1;
        end

      otherwise
        error('No such state');
      endswitch

    case {2} %was in Coherence state
      switch cState
      case {0}
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Nrindex) = nChange(pCindex, Nrindex) + (cConnect-pConnect);
          %2. Loss a/several connections => Nla
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections => fail to recover
        else
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + 1;
        end

      case {1} %go in Avoidance Forward
        nChange(pCindex, Naindex)=nChange(pCindex, Naindex) + 1;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Nrindex) = nChange(pCindex, Nrindex) + (cConnect-pConnect);
          %2. Loss a/several connections => Nla
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections => fail to recover
        else
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + 1;
        end

      case {2} %Stay in Coherence
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, Ncrestindex) = nChange(pCindex, Ncrestindex) + 1;
        end

      case {3} %Go in avoidance
        nChange(pCindex, Naindex) = nChange(pCindex, Naindex) + 1;
        %1. Gain a/several connections
        if cConnect > pConnect
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, Ncrestindex) = nChange(pCindex, Ncrestindex) + 1;
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
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, Nacindex) = nChange(pCindex, Nacindex) + 1;
        end

      case {1}
        nChange(pCindex, Naindex) = nChange(pCindex, Naindex) + 1;
        %1. Gain a/several connections (recover connections)
        if cConnect > pConnect
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, Nacindex) = nChange(pCindex, Nacindex) + 1;
        end

      case {2} %finish avoidance and go back in Coherence
        %1. Gain a/several connections (recover connections)
        if cConnect > pConnect
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence)
        else
          nChange(pCindex, Nacindex) = nChange(pCindex, Nacindex) + 1;
        end

      case {3} %stay in avoidance
        %1. Gain a/several connections (recover connections)
        if cConnect > pConnect
          nChange(pCindex, Nfindex) = nChange(pCindex, Nfindex) + (cConnect-pConnect);
          %2. Loss a/several connections
        elseif cConnect < pConnect
          nChange(pCindex, Nlaindex) = nChange(pCindex, Nlaindex) + (pConnect-cConnect);
          %3. no change in number of connections (keep track in coherence avoidance)
        else
          nChange(pCindex, Nacindex) = nChange(pCindex, Nacindex) + 1;
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
