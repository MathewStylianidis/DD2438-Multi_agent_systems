using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/*
 *Implementation of Genetic Algorithms largely based on their description in:
 * Eiben, A.E. and Smith, J.E., 2008. Introduction to evolutionary computing 
 * (natural computing series). Publisher: Springer-Verlag, New York, LLC.
 */
public class GeneticAlgorithm {

	private int M; //population size
	private int lambda; // parents selected in each iteration
	private int customers; //number of customers
	private int vehicles;
	private List<int>[] population; //population array with lists
	private float[,] distanceMatrix;
	private bool overselection;
	private float fitGroupPerc;
	private int solutionSize;
	private int mutationProbability;
	private int maxIterations;
	private float eps;
	private bool alternativeFitness;

	public GeneticAlgorithm(int M, int lambda, int customers, int vehicles, float[,] distanceMatrix, float mutationProbability, int maxIterations = 1000,
							bool overselection = false, float fitGroupPerc = 0.04f, float eps = 0.01f, bool alternativeFitness = false) {
		this.M = M;
		this.lambda = lambda;
		this.customers = customers;
		this.vehicles = vehicles;
		population = new List<int>[M];
		this.distanceMatrix = distanceMatrix;
		this.overselection = overselection;
		this.fitGroupPerc = fitGroupPerc;
		this.solutionSize = this.customers + this.vehicles;
		this.mutationProbability = (int)(mutationProbability * 100);
		this.maxIterations = maxIterations;
		this.eps = eps;
		this.alternativeFitness = alternativeFitness;
	}


	public List<int> getFittestIndividual() 
	{
		float[] populationFitness = calculateFitness (this.population);
		System.Array.Sort (populationFitness, this.population);
		Debug.Log("SOLUTION FITNESS: " + calculateIndividualFitness(this.population[0]));
		return this.population [0];
	}



	public void generationalGeneticAlgorithm() {
		// Initialize population
		initializePopulation ();

		// Get parent selection probabilities - They do not depend on fitness values but only on population size and hence can be precomputed
		double[] selectionProbabilities = calculateRankingProbabilities(population.Length, "Linear");
		System.Array.Reverse (selectionProbabilities);

		// If overselection is true, split selection probabilities in two
		double[] fitSelectionProbabilities = null; //user if overselection is true
		double[] unfitSelectionProbabilities = null; //user if overselection is true
		int seperatingIdx = -1; //user if overselection is true
		if(overselection) {
			seperatingIdx = M - (int)(this.M * this.fitGroupPerc);
			fitSelectionProbabilities = calculateRankingProbabilities((int)(this.M * this.fitGroupPerc), "Linear");
			unfitSelectionProbabilities = calculateRankingProbabilities(seperatingIdx, "Linear");
		}

		float bestFitness = float.MaxValue;
		float prevBestFitness = float.MaxValue;
		int count = 0;
		for(int i = 0; i < this.maxIterations; i++) {
			// Calculate fitness for each individual
			float[] populationFitness = calculateFitness (this.population);
			int[] populationIndices = populationFitness.getIndexList ();
			// Sort fitness of each individual along with their indices
			System.Array.Sort (populationFitness, populationIndices);

			prevBestFitness = bestFitness;
			bestFitness = populationFitness [0];
			// check for convergence
			if (GeneticAlgorithmHelper.abs (populationFitness [0] - prevBestFitness) < this.eps) 
				count++;
			//if(count >= this.maxIterations * 0.05)
				//break;
		

			// Sample parents from population
			int[] matingPool;
			if (!overselection) 
				//Run stochastic universal sampling algorithm to get mating pool (selected parents)
				matingPool = stochasticUniversalSampling (selectionProbabilities, populationIndices, this.lambda);
			else {
				// If overselection is activated
				int[] fitPopulationIndices = new int[(int)(this.M * this.fitGroupPerc)];
				int[] unfitPopulationIndices = new int[this.M - (int)(this.M * this.fitGroupPerc)];
				// Split population in fit and unfit where fit is <fitGroupPerc>% of the initial population
				System.Array.Copy (populationIndices, 0, unfitPopulationIndices, 0, seperatingIdx);
				System.Array.Copy (populationIndices, seperatingIdx, fitPopulationIndices, 0, (int)(this.M * this.fitGroupPerc));
				// Select 80% parents from fit group and 20% from unfit group
				int[] fitMatingPool = stochasticUniversalSampling (fitSelectionProbabilities, fitPopulationIndices, (int)(0.8f * this.M));
				int[] unfitMatingPool = stochasticUniversalSampling (unfitSelectionProbabilities, unfitPopulationIndices, (int)(0.2f * this.M));
				// Merge fit and unfit parents selected in the same mating pool
				matingPool = GeneticAlgorithmHelper.mergeArrays(fitMatingPool, unfitMatingPool);
			}

			// Mate the parents selected to get offsprings
			List<int>[] offsprings = mateParents (matingPool);
			float[] offspringsFitness = calculateFitness (offsprings);

			// Pick the best from the population and the new offsprings as the new population
			List<int>[] sortedPopulation = getOrderedPopulation(this.population, populationIndices);
			List<int>[] intermediatePool = GeneticAlgorithmHelper.mergeArrays (sortedPopulation, offsprings);
			float[] intermediateFitness = GeneticAlgorithmHelper.mergeArrays (populationFitness, offspringsFitness);
			int[] intermediatePoolIndices = intermediatePool.getIndexList ();
			System.Array.Sort (intermediateFitness, intermediatePoolIndices);
			System.Array.Copy(getOrderedPopulation (intermediatePool, intermediatePoolIndices), 0, this.population, 0, this.population.Length);
		}
	}


	List<int>[] mateParents(int[] matingPool) {
		List<int>[] offsprings = new  List<int>[matingPool.Length];
		System.Random rng = new System.Random (System.Guid.NewGuid().GetHashCode());

		// Shuffle parents to make mating random
		matingPool.Shuffle ();
		for (int i = 0; i + 1 < matingPool.Length; i = i + 2) {
			IndividualTuple x = crossover (population[matingPool[i]], population[matingPool[i + 1]]);
			offsprings [i] = x.individual_x;
			offsprings [i + 1] = x.individual_y;
			if (rng.Next (0, 100) <= this.mutationProbability)
				mutate (offsprings [i]);
			if (rng.Next (0, 100) <= this.mutationProbability)
				mutate (offsprings [i + 1]);
		}

		// If the number is even and one parent has no mate to create offsprings
		if (matingPool.Length % 2 != 0) {
			// Then make one parent mate twice and create the two extra offsprings
			IndividualTuple x = crossover (population[matingPool[matingPool.Length - 2]], population[matingPool[matingPool.Length - 1]]);
			offsprings [matingPool.Length - 2] = x.individual_x;
			offsprings [matingPool.Length - 1] = x.individual_y;
			if (rng.Next (0, 100) <= this.mutationProbability)
				mutate (offsprings [matingPool.Length - 2]);
			if (rng.Next (0, 100) <= this.mutationProbability)
				mutate (offsprings [matingPool.Length - 1]);
		}

		return offsprings;
	}




	// Performs a two-point ordered crossover and gives two offsprings
	private IndividualTuple crossover(List<int> parent_1, List<int> parent_2) 
	{
		System.Random rng = new System.Random (System.Guid.NewGuid().GetHashCode());
		int randomCutIdx1 = rng.Next (0, this.solutionSize - 1);
		int randomCutIdx2 = rng.Next (0, this.solutionSize - 1);
		// if the two cuts fall in the same position, the children remain the same as the parents
		if (randomCutIdx1 == randomCutIdx2) {
			return new IndividualTuple (new List<int> (parent_1), new List<int> (parent_2));
		} else if (randomCutIdx1 > randomCutIdx2) {
			int tmp = randomCutIdx2;
			randomCutIdx2 = randomCutIdx1;
			randomCutIdx1 = tmp;
		}
		// Get one subtour from each parent based on the two cuts
		List<int> parent1Subset = parent_1.GetRange (randomCutIdx1, randomCutIdx2 - randomCutIdx1 + 1);
		List<int> parent2Subset = parent_2.GetRange (0, randomCutIdx1);
		parent2Subset.AddRange (parent_2.GetRange (randomCutIdx2 + 1, this.solutionSize - randomCutIdx2 - 1));

		// Do the two crossovers 
		List<int> offspring_1 = new List<int>(this.solutionSize);
		List<int> offspring_2 = new List<int>(this.solutionSize);
		int curIdx1 = 0, curIdx2 = 0;
		for (int i = 0; i < this.solutionSize; i++) {
			if (i >= randomCutIdx1 && i <= randomCutIdx2) {
				offspring_1.Add (parent_1 [i]);
				while (parent2Subset.Contains (parent_1 [curIdx1])) 
					curIdx1++;
				offspring_2.Add (parent_1 [curIdx1++]);
			} else {
				offspring_2.Add (parent_2 [i]);
				while (parent1Subset.Contains (parent_2 [curIdx2])) 
					curIdx2++;
				offspring_1.Add (parent_2 [curIdx2++]);
			}
		}

		return new IndividualTuple (offspring_1, offspring_2);
	}


	// Performs a swap mutation to an individual
	private void mutate(List<int> individual) 
	{
		System.Random rng = new System.Random (System.Guid.NewGuid().GetHashCode());
		int randomIdx1 = rng.Next (0, this.solutionSize - 1);
		int randomIdx2 = rng.Next (0, this.solutionSize - 1);
		int tmp = individual [randomIdx1];
		individual [randomIdx1] = individual [randomIdx2];
		individual [randomIdx2] = tmp;
	}


	private int[] stochasticUniversalSampling(double[] selectionProbabilities, int[] populationIndices, int lambda) {
		int[] matingPool = new int[lambda];
		double[] a = getCumulativeProbs (selectionProbabilities);
		// Selects lambda parents given the probability distribution <selectionProbabilities>
		System.Random rng = new System.Random (System.Guid.NewGuid().GetHashCode());
		double r = (double)rng.NextDouble () %  (1f / lambda);

		int currentMember = 0, i = 0;
		while (currentMember < lambda) {
			while (r < a [i]) {
				matingPool [currentMember] = populationIndices [i];
				r += 1f / lambda;
				currentMember++;
			}
			i++;
		}
			
		return matingPool;
	}


	private double[] calculateRankingProbabilities(int populationSize, string option = "Linear", float constant = 2f) {
		double[] selectProbs = new double[populationSize];

		if (option.Equals ("Linear")) {

			if (constant <= 1 || constant > 2)
				throw new System.Exception("Set parameter <constant> so that 1 < constant <= 2 when choosing the Linear option");
			for (int rank = 1; rank <= populationSize; rank++)
				selectProbs [rank - 1] = (2 - constant) / populationSize + 2 * rank * (constant - 1) / (populationSize * (populationSize - 1));

		} else if (option.Equals ("Exp1")) {

			double normalizationConstant = 0;
			for (int rank = 1; rank <= populationSize; rank++) {
				selectProbs [rank - 1] = 1 - Mathf.Exp (-rank);
				normalizationConstant += selectProbs [rank - 1];
			}
			// Normalize probabilities
			for (int rank = 0; rank < populationSize; rank++)
				selectProbs [rank] /= normalizationConstant;	

		} else if (option.Equals ("Exp2")) {

			if (constant >= 1f)
				throw new System.Exception("Set parameter <constant> to a value between 0 and 1 when choosing the Exp2 option");
			double normalizationConstant = 0;
			for (int rank = 1; rank <= populationSize; rank++) {
				selectProbs [rank - 1] = Mathf.Pow (constant, populationSize - rank); 
				normalizationConstant += selectProbs [rank - 1];
			}
			// Normalize probabilities
			for (int rank = 0; rank < populationSize; rank++)
				selectProbs [rank] /= normalizationConstant;	
		}

		return selectProbs;
	}

	private float[] calculateFitness(List<int>[] population) {
		float[] populationFitness = new float[population.Length];
		for (int i = 0; i < population.Length; i++)
			if(!alternativeFitness)
				populationFitness [i] = calculateIndividualFitness (population[i]); 
			else
				populationFitness [i] = alternativeIndividualFitness (population[i]);
			
		return populationFitness;
	}


	// Used to minimize total sum of costs of all paths
	private float calculateIndividualFitness(List<int> individual) {
		float pathCost = 0;
		int solutionLength = individual.Count;
		int startingIdx = getFirstVehicleIndex (individual);
		int currentVehicleStartIdx = individual [startingIdx];
		int currentVehicleGoalIdx = currentVehicleStartIdx + this.vehicles;
		int prevIdx = currentVehicleStartIdx; //contains the index of the previous node visited in the solution


		for (int i = 1; i < solutionLength; i++) {
			int currentIdx = individual [(startingIdx + i) % solutionLength];
			// If the current node in the solution is not a point of interest
			if (currentIdx >= this.customers) {
				prevIdx = individual [(startingIdx + i - 1) % solutionLength];
				// Add distance from previous node to goal node of this vehicle
				pathCost += this.distanceMatrix[prevIdx, currentVehicleGoalIdx];
				// Update vehicle indices
				currentVehicleStartIdx = currentIdx;
				currentVehicleGoalIdx = currentVehicleStartIdx + this.vehicles;
			} else {
				//Add distance from previous node to current one
				prevIdx = individual [(startingIdx + i - 1) % solutionLength];
				pathCost += this.distanceMatrix[prevIdx, currentIdx];
			}				
		}

		pathCost += this.distanceMatrix[prevIdx, currentVehicleGoalIdx];
	
		return pathCost;
	}

	// Used to minimize the cost of the longest path in the solution
	private float alternativeIndividualFitness(List<int> individual) {
		float pathCost = 0;
		float pathMax = float.MinValue;
		int solutionLength = individual.Count;
		int startingIdx = getFirstVehicleIndex (individual);
		int currentVehicleStartIdx = individual [startingIdx];
		int currentVehicleGoalIdx = currentVehicleStartIdx + this.vehicles;
		int prevIdx = currentVehicleStartIdx; //contains the index of the previous node visited in the solution


		for (int i = 1; i < solutionLength; i++) {
			int currentIdx = individual [(startingIdx + i) % solutionLength];
			// If the current node in the solution is not a point of interest
			if (currentIdx >= this.customers) {
				prevIdx = individual [(startingIdx + i - 1) % solutionLength];
				// Add distance from previous node to goal node of this vehicle
				pathCost += this.distanceMatrix[prevIdx, currentVehicleGoalIdx];
				if (pathCost > pathMax)
					pathMax = pathCost;
				pathCost = 0;
				// Update vehicle indices
				currentVehicleStartIdx = currentIdx;
				currentVehicleGoalIdx = currentVehicleStartIdx + this.vehicles;
			} else {
				//Add distance from previous node to current one
				prevIdx = individual [(startingIdx + i - 1) % solutionLength];
				pathCost += this.distanceMatrix[prevIdx, currentIdx];
			}				
		}

		pathCost += this.distanceMatrix[prevIdx, currentVehicleGoalIdx];
		if (pathCost > pathMax)
			pathMax = pathCost;
		
		return pathMax;
	}

	private void initializePopulation() {
		// Create a list with the indices of each customer
		List<int> custIdxList = new List<int> ();
		// Create a list with indices of each vehicle
		List<int> vehicleIdxList = new List<int> ();

		for (int i = 0; i < customers; i++) 
			custIdxList.Add (i);
		for (int i = 0; i < this.vehicles; i++)
			vehicleIdxList.Add (custIdxList.Count + i);

		// For each population individual
		for (int i = 0; i < M; i++) {
			custIdxList.Shuffle ();		
			vehicleIdxList.Shuffle (); 
			initializeIndividual(population, i, custIdxList, vehicleIdxList);
		}
	}


	private void initializeIndividual(List<int>[] population, int idx, List<int> custIdxList, List<int> vehicleIdxList) {
		List<int> individual;
		individual = new List<int> (custIdxList);
		// insert vehicle indices in list to split the path among the vehicles
		System.Random rng = new System.Random (System.Guid.NewGuid().GetHashCode());
		for (int i = 0; i < vehicleIdxList.Count; i++) {
			int index = rng.Next (0, individual.Count);
			individual.Insert (index, vehicleIdxList [i]);
		}
		population [idx] = individual;
	}

	private double[] getCumulativeProbs(double[] selectionProbabilities) {
		double[] a = new double[selectionProbabilities.Length];
		a [0] = selectionProbabilities [0];
		for (int i = 1; i < a.Length; i++)
			a [i] = a [i - 1] + selectionProbabilities [i];
		return a;
	}

	private int getFirstVehicleIndex(List<int> individual) {
		// Get first vehicle index in the individual
		for (int i = 0; i < individual.Count; i++)
			// If the index corresponds to a vehicle's starting node
			if (individual [i] >= this.customers)
				return i;
		return -1;
	}

	private List<int>[] getOrderedPopulation(List<int>[] p, int[] populationOrder) {
		if (p.Length != populationOrder.Length)
			throw new System.Exception (" Population and population indices defining their order must be of the same dimensions");
		List<int>[] orderedPopulation = new List<int>[p.Length];
		for (int i = 0; i < p.Length; i++)
			orderedPopulation [i] = p [populationOrder [i]];
		return orderedPopulation;
	}

	private class IndividualTuple
	{
		public List<int> individual_x;
		public List<int> individual_y;

		public IndividualTuple(List<int> x, List<int> y) {
			this.individual_x = x;
			this.individual_y = y;
		}
	}
}





public static class GeneticAlgorithmHelper {

	/// <summary>
	/// Adds the goal nodes to the solution parameter given and returns the result
	/// </summary>
	public static List<int>	 includeSolutionGoals(List<int> solution, int pointsOfInterest, int vehicles) {
		int solutionLength = solution.Count;
		int startingIdx = getFirstVehicleIndex (solution, pointsOfInterest);
		int currentVehicleStartIdx = solution [startingIdx];
		int currentVehicleGoalIdx = currentVehicleStartIdx + vehicles;
		int prevIdx = currentVehicleStartIdx; //contains the index of the previous node visited in the solution

		// List for solution including goal nodes
		List<int> indices = new List<int>();
		indices.Add (currentVehicleStartIdx);
		for (int i = 1; i < solutionLength; i++) {
			int currentIdx = solution [(startingIdx + i) % solutionLength];
			// If the current node in the solution is not a point of interest
			if (currentIdx >= pointsOfInterest) {
				indices.Add (currentVehicleGoalIdx);
				prevIdx = solution [(startingIdx + i - 1) % solutionLength];
				// Update vehicle indices
				currentVehicleStartIdx = currentIdx;
				currentVehicleGoalIdx = currentVehicleStartIdx + vehicles;
				indices.Add (currentVehicleStartIdx);
			} else {
				//Add distance from previous node to current one
				prevIdx = solution [(startingIdx + i - 1) % solutionLength];
				indices.Add (currentIdx);
			}				
		}

		indices.Add (currentVehicleGoalIdx);
		return indices;
	}


	/// <summary>
	/// Given a VRP solution, distribute the paths in the solution to different lists and return a list with those lists.
	/// </summary>
	public static List<List<int>> splitSolution(List<int> solution, int totalNodes, int vehicles) {
		List<List<int>> solutionList = new List<List<int>>();

		if (solution.Count == 0)
			throw new System.Exception ("Empty solution");

		int pathIdx = 0;
		solutionList.Add (new List<int> ());
		for (int i = 1; i < solution.Count; i++) {
			if (solution [i] >= totalNodes - vehicles * 2 && solution [i] < totalNodes - vehicles) {
				pathIdx++;
				solutionList.Add (new List<int> ());
				continue;
			}
			solutionList [pathIdx].Add (solution [i]);
		}

		return solutionList;
	}


	/// <summary>
	/// Get the first vehicle index in the solution list
	/// </summary>
	private static int getFirstVehicleIndex(List<int> solution, int pointsOfInterest) {
		// Get first vehicle index in the individual
		for (int i = 0; i < solution.Count; i++)
			// If the index corresponds to a vehicle's starting node
			if (solution [i] >= pointsOfInterest)
				return i;
		return -1;
	}


	private static System.Random rng = new System.Random(System.Guid.NewGuid().GetHashCode());
	public static void Shuffle<T>(this IList<T> list) {
		int n = list.Count;

		while (n > 1) {
			n--;
			int k = rng.Next (n + 1);
			T value = list [k];
			list [k] = list [n];
			list [n] = value;
		}
	}

	public static int[] getIndexList<T>(this IList<T> coll) {
		int length = coll.Count;
		int[] array = new int[length];
		for (int i = 0; i < length; i++)
			array [i] = i;
		return array;
	}

	public static T[] mergeArrays<T>(T[] x, T[] y) {
		T[] z = new T[x.Length + y.Length];
		x.CopyTo (z, 0);
		y.CopyTo (z, x.Length);
		return z;
	}

	public static float abs(float x) {
		if (x <= 0)
			return -x;
		return x;
	}
}

