using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GeneticAlgorithm {

	private int M; //population size
	private int customers; //number of customers
	private int vehicles;
	private List<int>[] population; //population array with lists
	private float[,] distanceMatrix;

	public GeneticAlgorithm(int M, int customers, int vehicles, float[,] distanceMatrix) {
		this.M = M;
		this.customers = customers;
		this.vehicles = vehicles;
		population = new List<int>[M];
		this.distanceMatrix = distanceMatrix;
	}


	public void steadyState() {
		initializePopulation ();
		float[] populationFitness = calculateFitness ();

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
}
