
%function testing the number of robots which are changing state
function N_passed = test_prob (P,N)

  for i=1:size(P)(1)
    N_passed(i,1)=sum(rand(N(i),1)<P(i));
  end

endfunction
