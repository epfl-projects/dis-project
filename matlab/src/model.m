%DIS Project : Macroscopic model of a wireless connected swarm

%*************************************%
%Initial conditions
%*************************************%

N_rob=10;
dmax=N_rob-1;

alpha=5; %should be greater than 2

N_AF(:,1)=zeros(dmax,1);	% # of robots
N_F(:,1)=zeros(dmax,1);
N_AC(:,1)=zeros(dmax,1);
N_C(:,1)=zeros(dmax,1);
N_Fbar(:,1)=zeros(dmax,1);
N_Cbar(:,1)=zeros(dmax,1);


%*************************************%
%Probabilities to calibrate
%*************************************%

Pa=rand(dmax,1);	%collision with another robot
Pl=rand(dmax,1); 	%loss of a connection in forward state
Pg=rand(dmax,1);		%gain of a connection
Pr=rand(dmax,1);		%recovery of a connection
Pf=rand(dmax,1);	%failure to recover a connection
Pla=rand(dmax,1);	%loss of a connection in coherence state

%NOTE : dimension = dmax rows for 1 column

%*************************************%
%Other simulation parameters
%*************************************%

TA = 10; %number of timesteps to spend in the avoidance state
TC = 10; %number of timesteps to spend in the coherence state


%*************************************%
%Simulation
%*************************************%

k=0; %initialisation of the simulation timestep
while(k<10)
	k++;
	
	%~ for i=1:dmax+1 % # of connections for a robot (from 0 to dmax)
		
		% # of robots in Forwar and in Coherence state
		%~ N_Fbar(i,k)=N_AF(i,k)+N_F(i,k);
		%~ N_Cbar(i,k)=N_AC(i,k)+N_C(i,k);
		
		N_F(:,k)=N_Fbar(:,k)-N_AF(:,k);
		N_C(:,k)=N_Cbar(:,k)-N_AC(:,k);
		
		%the corr factor remove the k-TA factor if k-TA is before the beginning of the simulation
		if k>TA, PaNF=Pa(:).*N_F(:,k-TA);
		else, PaNF=0;
		end
		
		N_AF(:,k+1)=N_AF(:,k)+Pa(:).*N_F(:,k)-PaNF;
		N_AC(:,k+1)=N_AC(:,k)+Pa(:).*N_C(:,k)-PaNF;
	%~ end
	%part taking into account the interval for the connectivity sampling (TC)
	if mod(k-1,TC)==0
		%~ N_Fbar(1,(k+1)*TC) = N_Fbar(1,k*TC) + Pf(1) * N_Cbar(1,k*TC) - Pg(1) * N_Fbar(k*TC);
		%~ N_Cbar(1,(k+1)*TC) = N_Cbar(1,k*TC) + Pla(2) * N_Cbar(2,k*TC) + Pl(2) * N_Fbar(2,k*TC) - ( Pr(1) + Pf(1) ) * N_Cbar(1,k*TC);
		
		N_Fbar(1,k+1:k+TC) = repmat( N_Fbar(1,k) + Pf(1) * N_Cbar(1,k) ...
			- Pg(1) * N_Fbar(k), 1, TC );
		
		
		for i=2:alpha-1
		
			N_Fbar(i,k+1:k+TC) = repmat( N_Fbar(i,k) + Pg(i-1) * N_Fbar(i-1,k) ...
				+ Pf(i) * N_Cbar(k) + Pr(i-1) * N_Cbar(i-1,k) ...
				- ( Pg(i) + Pl(i) ) * N_Fbar(i,k), 1, TC );
			
		end
		N_Fbar(alpha,k+1:k+TC) = repmat( N_Fbar(alpha,k) + Pg(alpha-1) * N_Fbar(alpha-1,k) ...
			+ Pl(alpha+1) * N_Fbar(alpha+1,k) + Pr(alpha-1) * N_Cbar(alpha-1,k) ...
			- ( Pg(alpha) + Pl(alpha) ) * N_Fbar(alpha,k), 1, TC );
			
		for i=alpha+1:dmax-1
			N_Fbar(i,k+1:k+TC) = repmat( N_Fbar(i,k) + Pg(i-1) * N_Fbar(i-1,k) ...
				+ Pl(i+1) * N_Fbar(i+1,k) - ( Pg(i) + Pl(i) ) * N_Fbar(i,k), 1, TC );
		end
		N_Fbar(dmax,k+1:k+TC) = repmat ( N_Fbar(dmax,k) + Pg(dmax-1) * N_Fbar(dmax-1,k) ...
			- Pl(dmax) * N_Fbar(dmax,k), 1, TC );
		
		N_Cbar(1,k+1:k+TC) = repmat( N_Cbar(1,k) + Pla(2) * N_Cbar(2,k) ...
			+ Pl(2) * N_Fbar(2,k) - ( Pr(1) + Pf(1) ) * N_Cbar(1,k), 1, TC );
		
		for i=2:alpha-2
			N_Cbar(i,k+1:k+TC) = repmat( N_Cbar(i,k) + Pla(i+1) * N_Cbar(i+1,k) ...
				+ Pl(i+1) * N_Fbar(i+1,k) - ( Pr(i) + Pf(i) + Pla(i) ) * N_Cbar(i,k), 1, TC );
		end
		i=alpha-1;
		N_Cbar(i,k+1:k+TC) = repmat( N_Cbar(i,k) + Pl(alpha) * N_Fbar(alpha,k) ...
			- ( Pr(i) + Pf(i) + Pla(i) ) * N_Cbar(i,k), 1, TC );
	end
end
