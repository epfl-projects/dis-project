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
N_AC(1:alpha,1)=0;
N_C(1:alpha,1)=0;
N_AC(alpha+1:N_rob,1)=0;
N_C(alpha+1:N_rob,1)=0;

N_Fbar(:,1) = N_AF + N_F;
N_Cbar(:,1) = N_AC + N_C;


%*************************************%
%Probabilities to calibrate
%*************************************%
%those probabilities were extract from the second article page 256 the top figure on the left

Pa=[0.00 0.00 0.01 0.02 0.02 0.03 0.03 0.03 0.05 0.05]';	%collision with another robot
Pl=[0.00 0.12 0.15 0.17 0.18 0.26 0.28 0.31 0.33 0.33]'; 	%loss of a connection in forward state
Pg=[0.19 0.22 0.24 0.23 0.22 0.17 0.15 0.13 0.11 0.09]';	%gain of a connection
Pr=[0.28 0.30 0.31 0.29 0.28]';		%recovery of a connection [1:alpha]
Pf=[0.70 0.58 0.56 0.54 0.52]';	%failure to recover a connection [1:alpha]
Pla=[0.00 0.05 0.09 0.12 0.15]';	%loss of a connection in coherence state [1:alpha]
%test
Pr(alpha+1:N_rob)=0;
Pf(alpha+1:N_rob)=0;
Pla(alpha+1:N_rob)=0;
Pla=ones(N_rob,1)-Pr-Pf;

%*************************************%
%Other simulation parameters
%*************************************%

TA = 10; %number of timesteps to spend in the avoidance state
TC = 1; %number of timesteps to spend in the coherence state
k_end=20;


%*************************************%
%Simulation
%*************************************%

k=0; %initialisation of the simulation timestep
while(k<k_end)
	k++;


	% # of robots in Forwar and in Coherence state
	N_F(:,k)=N_Fbar(:,k)-N_AF(:,k);
	N_C(:,k)=N_Cbar(:,k)-N_AC(:,k);

	%**************************************************************************
	%Calculation of the number of robots which changed states during timestep k
	%**************************************************************************

	%lecture of the following variables
	% P*N_* means the number of robots of N_* who successfully pass the P* test
	% # of robots which enter in avoidance from forward state

	% # of robots which entered in avoidance from forward state
	PaN_F(:,k) = test_prob( Pa, N_F(:,k) );
	% # of robots which entered in avoidance from coherence state
	PaN_C(:,k) = test_prob( Pa, N_C(:,k) );

	%this part need only to be calculate every TC timestep
%
	% # of robots which fail to recover / recover / lost a connection from coherence state
	[PfN_Cbar(:,k) PrN_Cbar(:,k) PlaN_Cbar(:,k) ] = test_prob( Pf, Pr, Pla, N_Cbar(:,k) );
	% # of robots which gain / lost (/ "the rest") a connection from forward state
	[PgN_Fbar(:,k) PlN_Fbar(:,k) restN_Fbar(:,k)] =  test_prob( Pg, Pl , N_Fbar(:,k) );
%

	%Check the existence of PN_F(:,k-TA)
	if k>TA
		PaNF=PaN_F(:,k-TA);
		PaNC=PaN_C(:,k-TA);
	else
		PaNF=0;
		PaNC=0;
	end

	if k<k_end

		N_AF(:,k+1)=N_AF(:,k)+PaN_F(:,k)-PaNF;
		N_AC(:,k+1)=N_AC(:,k)+PaN_C(:,k)-PaNC;

		%***********************************************************************%
		%Simulation for every TC timestep (interval of the connectivity sampling)
		%***********************************************************************%

		if mod(k-1,TC)==0

			%************* simulation for the forward state ************

			%for 0 connection
			N_Fbar(1,k+1:k+TC) = repmat( ...
				N_Fbar(1,k) ...
				+ PfN_Cbar(1,k) ...
				- PgN_Fbar(1,k) ...
			, 1, TC );

			%for 1 to alpha-1 connection
			for i=2:alpha

				N_Fbar(i,k+1:k+TC) = repmat( N_Fbar(i,k) ...
					+ PgN_Fbar(i-1,k) ...
					+ PfN_Cbar(i,k) ...
					+ PrN_Cbar(i-1,k) ...
					- PgN_Fbar(i,k) ...
					- PlN_Fbar(i,k) ...
				, 1, TC );

			end

			%for alpha connections
			i=alpha+1;

			N_Fbar(i,k+1:k+TC) = repmat( N_Fbar(i,k) ...
				+ PgN_Fbar(i-1,k) ...
				+ PlN_Fbar(i+1,k) ...
				+ PrN_Cbar(i-1,k) ...
				- PgN_Fbar(i,k) ...
				- PlN_Fbar(i,k) ...
			, 1, TC );

			%for alpha+1 to max connections -1 connections
			for i=alpha+2:dmax-1

				N_Fbar(i,k+1:k+TC) = repmat( N_Fbar(i,k) ...
					+ PgN_Fbar(i-1,k) ...
					+ PlN_Fbar(i+1,k) ...
					- PgN_Fbar(i,k) ...
					- PlN_Fbar(i,k) ...
				, 1, TC );

			end
			%for the full connectivity (N_rob-1 connections)
			i=dmax;

			N_Fbar(i,k+1:k+TC) = repmat ( N_Fbar(i,k) ...
				+ PgN_Fbar(i-1,k) ...
				- PlN_Fbar(i,k)  ...
			, 1, TC );

			%******************* simulation for the coherence state ******************

			%for 0 connection
			i=1;
			N_Cbar(i,k+1:k+TC) = repmat( N_Cbar(i,k) ...
				+ PlaN_Cbar(i+1,k) ...
				+ PlN_Fbar(i+1,k) ...
				- PrN_Cbar(i,k) ...
				- PfN_Cbar(i,k) ...
			, 1, TC );

			%for 1 to alpha-2 connections
			for i=2:alpha-1
				N_Cbar(i,k+1:k+TC) = repmat( N_Cbar(i,k) ...
					+ PlaN_Cbar(i+1,k) ...
					+ PlN_Fbar(i+1,k) ...
					- PrN_Cbar(i,k) ...
					- PlaN_Cbar(i,k) ...
					- PfN_Cbar(i,k) ...
				, 1, TC );
			end

			%for alpha-1 connections
			i=alpha;
			N_Cbar(i,k+1:k+TC) = repmat( N_Cbar(i,k) ...
				+ PlN_Fbar(i+1,k) ...
				- PrN_Cbar(i,k) ...
				- PlaN_Cbar(i,k) ...
				- PfN_Cbar(i) ...
			, 1, TC );

		end
	end
end
%*************************************%
%Plot of the figures
%*************************************%

figure();
plot(sum(N_Fbar+N_Cbar)','.');
figure();
plot(sum(N_F+N_AF+N_C+N_AC)','.');
% figure();
% plot(N_Fbar');
% legend();
% figure();
% plot(N_Cbar');
% figure();
% plot(PlN_Fbar');
% figure();
% plot(PlaN_Cbar');


%************* debuging part *************
% deb = fopen("debugging.txt","w");
%
% for k=1:k_end
% 	a=[N_Fbar(:,k) N_F(:,k) N_AF(:,k) N_Cbar(:,k) N_C(:,k) N_AC(:,k)];
% 	for i=1:N_rob
% 		for j=1:size(a)(2)
% 			fprintf(deb,'%d\t', a(i,j));
% 		end
% 		fprintf(deb,'\n');
% 	end
% end
