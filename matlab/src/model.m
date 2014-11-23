close all
clear all

%DIS Project : Macroscopic model of a wireless connected swarm

%*************************************%
%Initial conditions
%*************************************%

N_rob=10;
dmax=N_rob-1;

alpha=5; %should be greater than 2

N_AF(1:N_rob,1)=0;	% # of robots
N_F(1:N_rob,1)=0;
N_F(alpha+2,1)=N_rob;
N_AC(1:N_rob,1)=0;
N_C(1:N_rob,1)=0;
N_Fbar(:,1)=N_AF + N_F;
N_Cbar(:,1)=N_AC + N_C;


%*************************************%
%Probabilities to calibrate
%*************************************%
%those probabilities were extract from the second article page 256 the top figure on the left

Pa=[0.00 0.00 0.01 0.02 0.02 0.03 0.03 0.03 0.05 0.05]';	%collision with another robot
Pl=[0.00 0.12 0.15 0.17 0.18 0.26 0.28 0.31 0.33 0.33]'; 	%loss of a connection in forward state
Pg=[0.19 0.22 0.24 0.23 0.22 0.17 0.15 0.13 0.11 0.90]';	%gain of a connection
Pr=[0.28 0.30 0.31 0.29 0.28]';		%recovery of a connection [1:alpha]
Pf=[0.70 0.58 0.56 0.54 0.52]';	%failure to recover a connection [1:alpha]
Pla=[0.00 0.05 0.09 0.12 0.15];	%loss of a connection in coherence state [1:alpha]

%NOTE : dimension = dmax rows for 1 column

%*************************************%
%Other simulation parameters
%*************************************%

TA = 10; %number of timesteps to spend in the avoidance state
TC = 5; %number of timesteps to spend in the coherence state
k_end=1000;


%*************************************%
%Simulation
%*************************************%

k=0; %initialisation of the simulation timestep
while(k<k_end)
	k++;


		% # of robots in Forwar and in Coherence state
		%~ N_Fbar(i,k)=N_AF(i,k)+N_F(i,k);
		%~ N_Cbar(i,k)=N_AC(i,k)+N_C(i,k);

		N_F(:,k)=N_Fbar(:,k)-N_AF(:,k);
		N_C(:,k)=N_Cbar(:,k)-N_AC(:,k);

		%********************************************************
		%Calculation of the probability for the step simulation k
		%********************************************************

		% # of robots which enter in avoidance from forward state
		PaN_F(:,k) = test_prob( Pa, N_F(:,k) );
		% # of robots which enter in avoidance from coherence state
		PaN_C(:,k) = test_prob( Pa, N_C(:,k) );
		% # of robots which fail to recover a connection in coherence state
		PfN_Cbar(:,k) = test_prob( Pf, N_Cbar(:,k) );
		PrN_Cbar(:,k) = test_prob( Pr, N_Cbar(:,k) );
		PgN_Fbar(:,k) = test_prob( Pg, N_Fbar(:,k) );

		PlN_Fbar(:,k) = test_prob(Pl(i+1), N_Fbar(i+1,k));
		PrN_Cbar(:,k) = test_prob(Pr(i-1), N_Cbar(i-1,k)) ...
		PgPl_Fbar(:,k) =  test_prob( Pg, Pl , N_Fbar(:,k) )

		%Check the existence of PN_F(:,k-TA)
		if k>TA
			PaNF=PN_F(:,k-TA);
			PaNC=PN_C(:,k-TA);
		else, PaNF=0; PaNC=0;
		end

		N_AF(:,k+1)=N_AF(:,k)+PaN_F(:,k)-PaNF;
		N_AC(:,k+1)=N_AC(:,k)+PaN_C(:,k)-PaNC;

	%part taking into account the interval for the connectivity sampling (TC)
	%(sampling every TC timestep)

	%************* simulation for the forward state ************
	if mod(k-1,TC)==0 && k<k_end-1
		%~ N_Fbar(1,(k+1)*TC) = N_Fbar(1,k*TC) + Pf(1) * N_Cbar(1,k*TC) - Pg(1) * N_Fbar(k*TC);
		%~ N_Cbar(1,(k+1)*TC) = N_Cbar(1,k*TC) + Pla(2) * N_Cbar(2,k*TC) + Pl(2) * N_Fbar(2,k*TC) - ( Pr(1) + Pf(1) ) * N_Cbar(1,k*TC);

		% # robot which doesn't recover a connection


		%for 0 connection
		N_Fbar(1,k+1:k+TC) = repmat( N_Fbar(1,k) + PfN_Cbar(1) ...
			- test_prob(Pg(1), N_Fbar(1,k)), 1, TC );

		%for 1 to alpha-1 connection
		for i=2:alpha

			N_Fbar(i,k+1:k+TC) = repmat( N_Fbar(i,k) + test_prob(Pg(i-1), N_Fbar(i-1,k)) ...
				+ PfN_Cbar(i) + test_prob(Pr(i-1), N_Cbar(i-1,k)) ...
				- test_prob( Pg(i) + Pl(i), N_Fbar(i,k)), 1, TC );

		end
		%for alpha connections
		i=alpha+1;
		N_Fbar(i,k+1:k+TC) = repmat( ...
			N_Fbar(i,k) ...
			+ test_prob(Pg(i-1), N_Fbar(i-1,k)) ...
			+ test_prob(Pl(i+1), N_Fbar(i+1,k)) ...
			+ test_prob(Pr(i-1), N_Cbar(i-1,k)) ...
			- test_prob( Pg(i) + Pl(i) , N_Fbar(i,k)), 1, TC );
		%for alpha+1 to max connections -1 connections
		for i=alpha+2:dmax-1
			N_Fbar(i,k+1:k+TC) = repmat( N_Fbar(i,k) + test_prob( Pg(i-1), N_Fbar(i-1,k) ) ...
				+ test_prob( Pl(i+1), N_Fbar(i+1,k) ) - test_prob( Pg(i) + Pl(i), N_Fbar(i,k)), 1, TC );
		end
		%for the full connectivity (N_rob-1 connections)
		N_Fbar(dmax,k+1:k+TC) = repmat ( N_Fbar(dmax,k) + test_prob(Pg(dmax-1), N_Fbar(dmax-1,k)) ...
			- test_prob( Pl(dmax), N_Fbar(dmax,k) ), 1, TC );

		%************* simulation for the coherence state ***********
		%for 0 connection
		N_Cbar(1,k+1:k+TC) = repmat( ...
			N_Cbar(1,k) ...
			+ test_prob( Pla(2), N_Cbar(2,k) ) ...
			+ test_prob( Pl(2), N_Fbar(2,k) ) ...
			- test_prob( Pr(1), N_Cbar(1,k) ) ...
			- PfN_Cbar(1) ...
			, 1, TC );
		%for 1 to alpha-2 connections
		for i=2:alpha-1
			N_Cbar(i,k+1:k+TC) = repmat( ...
				N_Cbar(i,k) ...
				+ Pla(i+1) * N_Cbar(i+1,k) ...
				+ Pl(i+1) * N_Fbar(i+1,k) ...
				- ( Pr(i) + Pla(i) ) * N_Cbar(i,k) ...
				- PfN_Cbar(i) ...
				, 1, TC );
		end
		%for alpha-1 connections
		i=alpha;
		N_Cbar(i,k+1:k+TC) = repmat( ...
			N_Cbar(i,k) ...
			+ test_prob( Pl(alpha), N_Fbar(alpha,k)) ...
			- test_prob( Pr(i) + Pla(i), N_Cbar(i,k)) ...
			- PfN_Cbar(i) ...
			, 1, TC );
	end
end

%*************************************%
%Plot of the figures
%*************************************%

figure();
plot(N_Fbar');
legend();
figure();
plot(N_Cbar');
figure();
plot((N_Fbar+N_Cbar)');
