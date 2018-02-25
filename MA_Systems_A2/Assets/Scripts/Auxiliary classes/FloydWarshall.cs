using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FloydWarshall {

	private int nodeCount; // Number of nodes on the graph
	private float[][] weightMatrix; // Matrix with weights of edges between nodes
	private float[][,] shortestPathDist; // Matrix with the shortest path distances
	private int[][,] predecessorMatrix;

	public FloydWarshall(float[][] weightMatrix) {
		nodeCount = weightMatrix.Length;
		this.weightMatrix = weightMatrix;
		shortestPathDist = new float[nodeCount + 1][,];
		predecessorMatrix = new int[nodeCount + 1][,];
		for (int i = 0; i < nodeCount + 1; i++) {
			shortestPathDist [i] = new float[nodeCount, nodeCount];
			predecessorMatrix [i] = new int[nodeCount, nodeCount];
		}
	}


	/// <summary>
	/// Computes and returns the shortest path distance matrix.
	/// </summary>
	public float[,] findShortestPaths() {
		initializePathMatrix ();
		initializePredecessorMatrix ();
		for (int k = 1; k < nodeCount + 1; k++)
			for (int i = 0; i < nodeCount; i++)
				for (int j = 0; j < nodeCount; j++) {
					float dist = shortestPathDist [k - 1] [i, k - 1] + shortestPathDist [k - 1] [k - 1, j];
					float prevDist = shortestPathDist [k - 1] [i, j];
					if (dist < prevDist) {
						shortestPathDist [k] [i, j] = dist;
						predecessorMatrix [k] [i, j] = predecessorMatrix [k - 1] [k - 1, j];
					} else {
						shortestPathDist [k] [i, j] = prevDist;
						predecessorMatrix [k] [i, j] = predecessorMatrix [k - 1] [i, j];
					}
				}
		
		return shortestPathDist[nodeCount - 1];
	}

	 

	private void initializePathMatrix() {
		for (int i = 0; i < nodeCount; i++)
			for (int j = 0; j < nodeCount; j++)
				this.shortestPathDist [0] [i,j] = this.weightMatrix [i] [j];
	}



	private void initializePredecessorMatrix() {
		for (int i = 0; i < nodeCount; i++)
			for (int j = 0; j < nodeCount; j++)
				if(i == j || weightMatrix[i][j] == float.MaxValue) 
					this.predecessorMatrix [0] [i,j] = -1;
				else 
					this.predecessorMatrix [0] [i,j] = i;
	}


}

