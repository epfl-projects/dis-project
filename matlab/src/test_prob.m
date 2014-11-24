
%function testing the number of robots which are changing state
function varargout = test_prob (varargin)

if nargin < 2
  error("not enougth parameters");

elseif nargin == 2
  P=varargin{1};
  N=varargin{2};
    for i=1:size(N)(1)
      N_passed(i,1)=sum(rand(N(i),1)<P(i));
    end
  varargout{1}=N_passed;

  return
elseif nargin ==4
  N=varargin{end};
  for i=1:nargin-1,varargout{i}=0; end

  for i=1:size(N)(1)
    if N(i) !=0
      P1=varargin{1}( i, 1 );
      P2=varargin{2}( i, 1 );
      P3=varargin{3}( i, 1 );
      N_rand=rand( N(i), 1 );

      varargout{1}(i,1) = sum( N_rand <= varargin{1}(i) );
      varargout{2}(i,1) = sum( P1 < N_rand && N_rand <= ( P1 + P2 ) );
      varargout{3}(i,1) = sum( P1 + P2 < N_rand && N_rand <= ( P1 + P2 + P3 ) );
      varargout{4}(i,1) = N(i) - ( varargout{1}(i,1)+varargout{2}(i,1)+varargout{3}(i,1) );
      % for j=2:nargin-1
      %   a = ( N_rand <= sum( [ varargin{j-1:j} ]' )(i) )
      %   b = ( sum( [varargin{1:j-1}]' )(i) > N_rand )
      %   varargout{j}(i,1) = sum( sum( [varargin{1:j-1}]' )(i) < N_rand && N_rand <= sum( [varargin{j-1:j}]' )(i) );
      % end
    else
      for k=1:nargin-1, varargout{k}(i,1) = 0; end
    end
  end

  return
end
endfunction
