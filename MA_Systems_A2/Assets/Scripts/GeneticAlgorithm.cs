﻿using System.Collections;
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

	public GeneticAlgorithm(int M, int lambda, int customers, int vehicles, float[,] distanceMatrix) {
		this.M = M;
		this.lambda = lambda;
		this.customers = customers;
		this.vehicles = vehicles;
		population = new List<int>[M];
		this.distanceMatrix = distanceMatrix;
	}


	public void generationalGeneticAlgorithm() {
		// Initialize chromosome population
		initializePopulation ();

		// Get parent selection probabilities - They do not depend on fitness values but only on population size and hence can be precomputed
		float[] selectionProbabilities = calculateRankingProbabilities(population.Length, "Linear");
 
		// Calculate fitness for each chromosome
		float[] populationFitness = calculateFitness ();
		int[] populationIndices = populationFitness.getIndexList ();
		// Sort fitness of each chromosome along with their indices
		System.Array.Sort(populationFitness, populationIndices);
		// Reverse since our specific problem (VRP) is a minimization problem
		System.Array.Reverse (populationFitness);
		System.Array.Reverse (populationIndices);

		//Run stochastic universal sampling algorithm to get mating pool (selected parents)
		int[] matingPool = stochasticUniversalSampling(selectionProbabilities, populationIndices); 

	}


	private int[] stochasticUniversalSampling(float[] selectionProbabilities, int[] populationIndices) {
		int[] matingPool = new int[this.lambda];
		// Selects lambda parents given the probability distribution <selectionProbabilities>
		System.Random rng = new System.Random (System.Guid.NewGuid().GetHashCode());

		int currentMember = 0, i = populationIndices - 1;

		while (currentMember < this.lambda) {
			float r = (float)rng.NextDouble () %  (1f / this.lambda); // this line is out from the loop in the book's pseudocode which might be incorrect
			while (r <= selectionProbabilities [i]) {
				matingPool [currentMember] = populationIndices [i];
				r += 1f / this.lambda;
				currentMember++;
			}
			i--;
		}
			
		return matingPool;
	}


	private float[] calculateRankingProbabilities(int populationSize, string option = "Linear", float constant = 2f) {
		float[] selectProbs = new float[populationSize];

		if (option.Equals ("Linear")) {
			
			if (constant <= 1 || constant > 2)
				throw new System.Exception("Set parameter <constant> so that 1 < constant <= 2 when choosing the Linear option");
			for (int rank = 0; rank < populationSize; rank++)
				selectProbs [rank] = (2 - constant) / populationSize + 2 * rank * (constant - 1) / (populationSize * (populationSize - 1));

		} else if (option.Equals ("Exp1")) {
			
			float normalizationConstant = 0;
			for (int rank = 0; rank < populationSize; rank++) {
				selectProbs [rank] = 1 - Mathf.Exp (-rank);
				normalizationConstant += selectProbs [rank];
			}
			// Normalize probabilities
			for (int rank = 0; rank < populationSize; rank++)
				selectProbs [rank] /= normalizationConstant;	
			
		} else if (option.Equals ("Exp2")) {
			
			if (constant >= 1f)
				throw new System.Exception("Set parameter <constant> to a value between 0 and 1 when choosing the Exp2 option");
			float normalizationConstant = 0;
			for (int rank = 0; rank < populationSize; rank++) {
				selectProbs [rank] = Mathf.Pow (constant, populationSize - rank); 
				normalizationConstant += selectProbs [rank];
			}
			// Normalize probabilities
			for (int rank = 0; rank < populationSize; rank++)
				selectProbs [rank] /= normalizationConstant;	
		}
			
		return selectProbs;
	}

	private float[] calculateFitness() {
		float[] populationFitness = new float[M];
		for (int i = 0; i < M; i++)
			populationFitness [i] = calculateIndividualFitness (i);
		return populationFitness;
	}


	private float calculateIndividualFitness(int chromosomeIdx) {
		//int nodeCount = this.customers + 2 * this.vehicles;
		float pathCost = 0;
		int currentVehicleStartIdx = population [chromosomeIdx] [0];
		int currentVehicleGoalIdx = currentVehicleStartIdx + this.vehicles;

		//List<int> indices = new List<int>();
		//indices.Add (currentVehicleStartIdx);

		for (int i = 1; i < population [chromosomeIdx].Count; i++) {
			int currentIdx = population [chromosomeIdx] [i];
			// If the current node in the solution is not a point of interest
			if (currentIdx >= this.customers) {
				//indices.Add (currentVehicleGoalIdx);
				int prevIdx = population [chromosomeIdx] [i - 1];
				// Add distance from previous node to goal node of this vehicle
				pathCost += this.distanceMatrix[prevIdx, currentVehicleGoalIdx];
				// Update vehicle indices
				currentVehicleStartIdx = currentIdx;
				currentVehicleGoalIdx = currentVehicleStartIdx + this.vehicles;
				//indices.Add (currentVehicleStartIdx);
			} else {
				//indices.Add (currentIdx);
				//Add distance from previous node to current one
				int prevIdx = population [chromosomeIdx] [i - 1];
				pathCost += this.distanceMatrix[prevIdx, currentIdx];
			}				
		}
		//indices.Add (currentVehicleGoalIdx);
		pathCost += this.distanceMatrix[population [chromosomeIdx].Count - 1, currentVehicleGoalIdx];

		/*
		Debug.Log (indices.Count);
		Debug.Log (this.customers);
		Debug.Log (this.vehicles);
		Debug.Log ("HERE IT COMES");
		for (int i = 0; i < indices.Count; i++)
			Debug.Log (indices [i]);
		*/
		return pathCost;
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
			initializeIndividual(population, i, custIdxList, vehicleIdxList);
		}
	}


	private void initializeIndividual(List<int>[] population, int idx, List<int> custIdxList, List<int> vehicleIdxList) {
		List<int> chromosome;
		custIdxList.Shuffle ();		
		vehicleIdxList.Shuffle (); 
		chromosome = new List<int> (custIdxList);

		// insert vehicle indices in list to split the path
		chromosome.Insert (0, vehicleIdxList[0]);
		System.Random rng = new System.Random (System.Guid.NewGuid().GetHashCode());
		for (int i = 1; i < vehicleIdxList.Count; i++) {
			int index = rng.Next (0, chromosome.Count);
			chromosome.Insert (index, vehicleIdxList [i]);
		}
		population [idx] = chromosome;
	}
		
}





public static class GeneticAlgorithmHelper {
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
}