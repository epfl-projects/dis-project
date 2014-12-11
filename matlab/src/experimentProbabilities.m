function P = experimentProbabilities(simulation)

  INPUT:
  %   simulation: contents of a CSV file from a single Webots simulation run
  %   Expected format:
  %     Time, Robot ID, Robot state, Number of neighbors
  OUTPUT:
  %   P: estimation of the Probabilities of changing state for each number of
  %     connections. The estimated Probabilities are :
  %      BOTH | FORWARD | COHERENCE
  %     [  Pa    Pg Pl    Pr Pf Pla ]
