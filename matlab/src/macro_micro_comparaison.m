clear all

logsDirectory= '../data';
NRobots=40;
Allalpha=[5 10 15];
nameModel={'microscopic','macroscopic'};
alphaInit = 1;
alphaEnd = 3;

symbols = {'-x', '.-', '-v', '-.'};

for i=alphaInit:alphaEnd
  alpha=Allalpha(i);

  for Nmodel=1:2
    load( [logsDirectory, '/', nameModel{Nmodel}, '-alpha-', num2str(alpha), '.mat' ] );
  end
  y=[ m_N_F, m_N_C, m_N_A, m_N_T ];

  modelDifference=abs(averaged-y);
  figure()
  hold on
  for j=1:4
    plot([0:NRobots-1]',modelDifference(:,j),symbols{j});
  end
  hold off
  title(['alpha = ', int2str(alpha)]);
  xlabel('Connections (number of neighbors)');
  ylabel('Number of robots');
  axis([0 NRobots 0 max(modelDifference(:, 4))+1]);
  legend('Forward', 'Coherence', 'Avoidance', 'Any state');

end
