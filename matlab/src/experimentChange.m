function nChange = experimentChange(cInfos, pInfos)
  %INPUT:
  %   cInfos, pInfos : current and previous informations for each robots
  %    regarding (COLUMN) :
  %         IDRobots, StateRobots, numbNeighborRobots
  %OUTPUT:
  %   nChange : vector

  nC=8;
  nChange = zeros(nRobots,nP);
  nRobots=size(currentInfos,1);
  for i=1:nRobots
    %first check the if the states have changed
    if cInfos(i,2)~=pInfos(i,2)


    end
    %if not that means it is in Forward 
  end


end
