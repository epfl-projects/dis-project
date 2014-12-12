close all
clear all

%DIS Project : Macroscopic model of a wireless connected swarm


%*************************************%
%Initial conditions
%*************************************%

N_rob=40;
dmax=N_rob-1;

alpha=15; %should be greater than 2

TA = 5; %number of timesteps to spend in the avoidance state
TC = 15; %number of timesteps to spend in the coherence state
k_end=10000; %length of the simulation in timestep

%*************************************%
%Load Probabilities
%*************************************%
load(['../data/Probability-alpha-',num2str(alpha),'.mat'])

Pa=P(:,1);
Pg=P(:,2);
Pl=P(:,3);
Pr=P(:,4);
Pf=P(:,5);
Pla=P(:,6);

%*************************************%
%Probabilities to calibrate
%*************************************%
%those probabilities were extract from the second article page 256 the top figure on the left
%
% Pa=[0.00 0.00 0.01 0.02 0.02 0.03 0.03 0.03 0.05 0.06]';	%collision with another robot
% Pl=[0.00 0.12 0.15 0.17 0.18 0.26 0.28 0.31 0.33 0.33]'; 	%loss of a connection in forward state
% Pg=[0.19 0.22 0.24 0.23 0.22 0.17 0.15 0.13 0.11 0.09]';	%gain of a connection
% Pr=[0.28 0.30 0.31 0.29 0.28]';		%recovery of a connection [1:alpha]
% Pf=[0.70 0.58 0.56 0.54 0.52]';	%failure to recover a connection [1:alpha]
% Pla=[0.00 0.05 0.09 0.12 0.15]';	%loss of a connection in coherence state [1:alpha]
%
% %test
% Pr(alpha+1:N_rob)=0;
% Pf(alpha+1:N_rob)=0;
% Pla(alpha+1:N_rob)=0;
% Pla=ones(N_rob,1)-Pr-Pf;
%

%*************************************%
%Simulation
%*************************************%
%for n=1:n_end



	N_AF=zeros(N_rob,k_end);	% # of robots
	N_F=zeros(N_rob,k_end);
	N_F(alpha+1,1)=N_rob;
	N_AC=zeros(N_rob,k_end);
	N_C=zeros(N_rob,k_end);

	N_Fbar = N_AF + N_F;
	N_Cbar = N_AC + N_C;


	%initialisation of the simulation timestep
	for(k=1:k_end)



		%*************************************************************************%
		%Calculation of the number of robots which changed states at timestep k
		%*************************************************************************%

		%lecture of the following variables
		% P*N_* means the number of robots of N_* who successfully pass the P* test
		% # of robots which enter in avoidance from forward state

		% # of robots which entered in avoidance from forward state
		PaN_F(:,k) = Pa .* N_F(:,k);

		% # of robots which entered in avoidance from coherence state
		PaN_C(:,k) = Pa .* N_C(:,k);

		%this part need only to be calculate every TC timestep
	%
		% # of robots which fail to recover / recover / lost a connection from coherence state
		%[PfN_Cbar(:,k) PrN_Cbar(:,k) PlaN_Cbar(:,k) ] = test_prob( Pf, Pr, Pla, N_Cbar(:,k) );
		PfN_Cbar(:,k)=Pf .* N_Cbar(:,k);
		PrN_Cbar(:,k)=Pr .* N_Cbar(:,k);
		PlaN_Cbar(:,k)=Pla .* N_Cbar(:,k);

		% # of robots which gain / lost (/ "the rest") a connection from forward state
		%[PgN_Fbar(:,k) PlN_Fbar(:,k) restN_Fbar(:,k)] =  test_prob( Pg, Pl , N_Fbar(:,k) );
		PgN_Fbar(:,k)=Pg .* N_Fbar(:,k);
		PlN_Fbar(:,k)=Pl .* N_Fbar(:,k);
		restN_Fbar(:,k)=(1-Pg-Pl) .* N_Fbar(:,k) ;
	%


	% # of robots in Forwar and in Coherence state
	%N_F(:,k)=N_Fbar(:,k)-N_AF(:,k);
	%N_C(:,k)=N_Cbar(:,k)-N_AC(:,k);

		%Check the existence of PN_F(:,k-TA)
		if k>TA
			PaNF=PaN_F(:,k-TA);
			PaNC=PaN_C(:,k-TA);
		else
			PaNF=0;
			PaNC=0;
		end


		if k<k_end

			%PaNC could be the source of the negative terms (observed on N_C)
			N_AF(:,k+1)=N_AF(:,k)+PaN_F(:,k)-PaNF;
			N_AC(:,k+1)=N_AC(:,k)+PaN_C(:,k)-PaNC;

			%***********************************************************************%
			%Simulation for every TC timestep (interval of the connectivity sampling)
			%***********************************************************************%

			if mod(k-1,TC)==0

				%**************** simulation for the forward state *****************%

				%for 0 connection
				N_Fbar(1,k+1:k+TC) = repmat( N_Fbar(1,k) ...
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

				%******************* simulation for the coherence state ****************%

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
					- PfN_Cbar(i,k) ...
				, 1, TC );



				%*********************** debuging part ***********************%

				%reset the files

				%~ if k==1
					%~ a=fopen("debugging_N_rob.txt","w");
					%~ fclose(a);
					%~ a=fopen("debugging_N_change.txt","w");
					%~ fclose(a);
				%~ end
				%~
				%~ deb = fopen("debugging_N_rob.txt","a+");
				%~
				%~ fprintf(deb,"step : %d\n",k);
				%~ a1=[N_Fbar(:,k) N_F(:,k) N_AF(:,k) N_Cbar(:,k) N_C(:,k) N_AC(:,k)];
				%~ a11=[a1 a1(:,1)+a1(:,4)];
				%~ for i=1:N_rob
					%~ for j=1:size(a1)(2)
						%~ fprintf(deb,'%d\t', a1(i,j));
					%~ end
					%~ fprintf(deb,'\n');
				%~ end
				%~ fprintf(deb,'%d\n\n',sum(a1(:,size(a1)(2))));
				%~ fclose(deb);
				%~
				%~ deb=fopen("debugging_N_change.txt","a+");
				%~ fprintf(deb,"step : %d\n",k);
				%~ a2=[ N_Fbar(:,k) PgN_Fbar(:,k) PlN_Fbar(:,k) restN_Fbar(:,k) N_Cbar(:,k) PfN_Cbar(:,k) PrN_Cbar(:,k) PlaN_Cbar(:,k)];
				%~ a2=[a2 a2(:,1)+a2(:,5)];
				%~ for i=1:N_rob
					%~ for j=1:size(a2)(2)
						%~ fprintf(deb,'%d\t', a2(i,j));
					%~ end
					%~ fprintf(deb,'\n');
				%~ end
				%~ fprintf(deb,'%d\n\n',sum(a2(:,size(a2)(2))));
				%~ fclose(deb);
				%~
				%~ %pause();

				%************************** end debugging part **************************%
			end

			% # of robots in Forwar and in Coherence state
			N_F(:,k+1)=N_Fbar(:,k+1)-N_AF(:,k+1);
			N_C(:,k+1)=N_Cbar(:,k+1)-N_AC(:,k+1);

		end

	end



%**********************************************************%
%mean over the number of steps
%**********************************************************%
%
m_N_F = mean( N_F ,2 ) ;
m_N_Fbar = mean( N_Fbar, 2 ) ;
m_N_C = mean( N_C, 2 ) ;
m_N_Cbar = mean( N_Cbar, 2 ) ;
m_N_A = mean ( N_AC + N_AF, 2 );
m_N_T = mean ( N_Fbar + N_Cbar, 2 );

%***********************************************************%
% Plot of the figure(s)
%***********************************************************%


y=[ m_N_F, m_N_C, m_N_A, m_N_T ];


lab=['rgbk'];
symb=['+ox*'];

figure();
hold('on');
ht=title('Macroscopic model : Number of robots per state and number of connections');
for i=1:size(y, 2)
	%h(i)=errorbar([0:9], y(:,i), e(:,i), [ '-' symb(i) lab(i) ] );
	h(i)=plot([0:dmax], y(:,i), [ '-' symb(i) lab(i) ] );

end
legend('Forward', 'Coherence', 'Avoidance', 'Total');
xlabel('N of connections');
ylabel('N of robots per state');
axis([0 dmax, 0 dmax/5]);
set(h,'linewidth',1.5);
set(ht,'fontsize',16);
grid('on');
hold('off');
