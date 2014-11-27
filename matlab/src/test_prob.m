
%function testing the number of robots which are changing state
function varargout = test_prob ( varargin )

N=varargin{end};

%Tranfigure each probability vector into a matrix
for k=1:nargin-1
  P(:,k)=varargin{k}(:,1);
end
P=[zeros(size(P)(1),1) P]; %addition of a first column fill with 0

for i=1:size(N)(1) %for each number of connections

  if N(i) !=0 %Only for non zero number of robots

    N_rand=rand( N(i), 1 ); %Generate one random number per robot at i connection(s)
    tot=0;
    for k=2:nargin

      varargout{k-1}(i,1) = sum( ( ( sum( P(i,1:k-1)) <= N_rand ) + ( N_rand < sum( P(i,1:k) ) ) == 2 ) );
      tot = tot + varargout{k-1}(i,1);

    end
    varargout{nargin}(i,1) = N(i) - tot; % the rest of the robots (mostly for test and debugging)

  else %meaning that N(i)=0 <=> no robot with i connection(s) so all outputs are equal to zero

    for k=1:nargin, varargout{k}(i,1) = 0; end

  end
end

endfunction
