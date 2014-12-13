close all
clear all

%DIS Project : Macroscopic model of a wireless connected swarm

alphaInit=1;
alphaEnd=3;

for nalpha=alphaInit:alphaEnd

	showFigure = 1; %show the figure
	saveFigure = 0; %save it in the figurePath folder
	generateProbabilities = 0; %generate the probabilities from files see the function probability_generation.m

	%different paths
	dataPath= '../data';
	figurePath= '../../report/figures';

	%*************************************%
	%Initial conditions
	%*************************************%
	alpha = [5,10,15]; %different alpha for simulation
	alpha = alpha( nalpha );

	k_end=1000; %length of the simulation in timesteps

	%CONSTANTS
	nRobots=40; %number of robots
	TA = 5; %number of timesteps to spend in the avoidance state
	TC = 15; %number of timesteps to spend in the coherence state
	dmax=nRobots-1;	%max number of connection possible

	%*************************************%
	%Load Probabilities
	%*************************************%

	if generateProbabilities
		%function who generate probabilities from experiment file with LOG_DETAILS=1
		initialisationSteps = 10 ; %number of steps to avoid for the probability estimation
		probability_generation(alpha, initialisationSteps);
	end
	%load the P variable regrouping probabilities generate before (if didn't exist)
	load( [dataPath, '/probability-alpha-',num2str(alpha),'.mat'] )

	Pa=P(:,1);
	Pg=P(:,2);
	Pl=P(:,3);
	Pr=P(:,4);
	Pf=P(:,5);
	Pla=P(:,6);


	%*************************************%
	%Simulation : initial conditions
	%*************************************%


	N_AF=zeros(nRobots,k_end);	% # of robots
	N_F=zeros(nRobots,k_end);

	N_F(alpha+1,1)=nRobots; % intial condition everyone at alpha connections

	N_AC=zeros(nRobots,k_end);
	N_C=zeros(nRobots,k_end);
	N_Fbar = N_AF + N_F;
	N_Cbar = N_AC + N_C;


	%initialisation of the simulation timestep
	for(k=1:k_end-1)



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
		PfN_Cbar(:,k)=Pf .* N_Cbar(:,k);
		PrN_Cbar(:,k)=Pr .* N_Cbar(:,k);
		PlaN_Cbar(:,k)=Pla .* N_Cbar(:,k);

		% # of robots which gain / lost (/ "the rest") a connection from forward state
		PgN_Fbar(:,k)=Pg .* N_Fbar(:,k);
		PlN_Fbar(:,k)=Pl .* N_Fbar(:,k);
		restN_Fbar(:,k)=(1-Pg-Pl) .* N_Fbar(:,k) ;
	%

		%Check the existence of PN_F(:,k-TA)
		if k>TA
			PaNF=PaN_F(:,k-TA);
			PaNC=PaN_C(:,k-TA);
		else
			PaNF=0;
			PaNC=0;
		end



		%PaNC could be the source of the negative terms (observed on N_C)
		N_AF(:,k+1)=N_AF(:,k)+PaN_F(:,k)-PaNF;
		N_AC(:,k+1)=N_AC(:,k)+PaN_C(:,k)-PaNC;

		%***********************************************************************%
		%Simulation for every TC timestep (interval of the connectivity sampling)
		%***********************************************************************%

		if mod(k-1,TC)==0

			%**************** simulation for the FORWARD state *****************%

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
			%for the full connectivity (nRobots-1 connections)
			i=dmax;

			N_Fbar(i,k+1:k+TC) = repmat ( N_Fbar(i,k) ...
				+ PgN_Fbar(i-1,k) ...
				- PlN_Fbar(i,k)  ...
			, 1, TC );

			%******************* simulation for the COHERENCE state ****************%

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

		end

		% # of robots in Forwar and in Coherence state
		N_F(:,k+1)=N_Fbar(:,k+1)-N_AF(:,k+1);
		N_C(:,k+1)=N_Cbar(:,k+1)-N_AC(:,k+1);

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


	lab=['rcbk'];
	symb=['dox*'];

	figure();
	hold('on');
	ht=title( ['Macroscopic model : Number of robots per state and number of connections with alpha = ' num2str(alpha)] );
	for i=1:size(y, 2)
		h(i)=plot([0:dmax], y(:,i), [ '-' symb(i) lab(i) ], 'markersize', 9 );
	end
	hl=legend('Forward', 'Coherence', 'Avoidance', 'Total');
	xlabel('N of connections');
	ylabel('N of robots per state');
	axis([0 dmax, 0 floor(max(y(:,4))+1)]);
	set(h,'linewidth',1.5);
	set(ht,'fontsize',16);
	set(hl,'fontsize',14);
	grid('on');
	hold('off');

	if saveFigure
		print([figurePath, '/',num2str(nRobots),'-macroscopic-alpha-', num2str(alpha),'.png'] );
	end
	if ~showFigure
		close
	end

end
