﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WorldController : MonoBehaviour {

	public TextAsset data;
	//public TextAsset trajectoryData;
	public GameObject agentPrefab;
	public World world;
	public GameObject obstacleParent;
	public GameObject agentParent;
	public GameObject boundingPolygon;
	public float agentRadius = 0.5f;
	public float objectHeight = 4f;
	public float objectThickness = 0.2f;
	public Material fieldMaterial;
	private GameObject[] agents;
	private List<List<int>> solutionList; // List of list of indices to be visited by each agent in the VRP
	//private List<Stack<PointInfo>> solutionStacks; // List of stacks with all the path coordinates for each vehicle's simulation
 
	void Start () {
		world = World.FromJson (data.text); //, trajectoryData.text);
		world.currentPositions = (Vector2[]) world.startPositions.Clone ();
		world.currentAngularVel = new float[world.currentPositions.Length];
		initializeVelocities ();
		spawnObstacle (world.boundingPolygon, "Bounding polygon", boundingPolygon);
		spawnObstacles ();
		spawnActors ();

		if (data.name == "P22") {
			// Initialize visibility graph
			VisibilityGraph.initVisibilityGraph (world);
			// Execute FloydWarshall algorithm on visibility graph to get shortest paths
			FloydWarshall fw = new FloydWarshall(world.visibilityGraph);
			float[,] floydWarshallDistMatrix = fw.findShortestPaths ();
			// Get number of obstacle vertices
			int obstacleVertCount = world.graphVertices.Count - world.pointsOfInterest.Length - agents.Length * 2;
			// Remove obstacle vertex rows and columns from the distance matrix as GA does not work with them to find a solution
			float[,] distanceMatrix = getSubArray(floydWarshallDistMatrix, obstacleVertCount);
			// Use the genetic algorithm for the Vehicle Routing Problem
			int M = 10000;
			int lambda = 10000;
			GeneticAlgorithm ga = new GeneticAlgorithm (M, lambda, world.pointsOfInterest.Length, agents.Length, distanceMatrix, 0.02f, 200, false, 0.04f, 0.01f, true);
			ga.generationalGeneticAlgorithm ();
			List<int> solution = ga.getFittestIndividual ();
			solution = GeneticAlgorithmHelper.includeSolutionGoals (solution, world.pointsOfInterest.Length, agents.Length);
			// Reconstruct path including intermediate obstacle vertex nodes
			for (int i = 0; i < solution.Count; i++)
				solution [i] += obstacleVertCount;
			solution = reconstructShortestPath (solution, fw, agents.Length);
			// Visualize the reconstructed path (solution including intermediate nodes)
			Visualizer.visualizeVRPsolution(world.graphVertices, solution, agents.Length, obstacleVertCount);
			solutionList = GeneticAlgorithmHelper.splitSolution(solution, world.graphVertices.Count, agents.Length);
			//solutionStacks = getSolutionStacks (solutionList);
		} 
		else if (data.name == "P25") {
			// Read trajectory
			Vector3[] trajectory = new Vector3[world.trajectory.x.Length];
			for (int i = 0; i < trajectory.Length; i++)
				trajectory[i] = new Vector3 (world.trajectory.x[i], objectHeight, world.trajectory.y[i]);
		}
		else if (data.name == "P26") {
			// Formation problems
			GameObject plane = GameObject.Find ("Plane");
			Renderer ren = plane.GetComponent<Renderer> ();
			ren.material = fieldMaterial;
			// Read trajectory
			Vector3[] trajectory = new Vector3[world.trajectory.x.Length];
			for (int i = 0; i < trajectory.Length; i++)
				trajectory[i] = new Vector3 (world.trajectory.x[i], objectHeight, world.trajectory.y[i]);

		}
	}
	

	void Update () {
		if (data.name == "P22") {
			//Simulate VRP solution
			for (int i = 0; i < agents.Length; i++) {
				//agents[i].transform.
			}
		}
	}

	// Gets distance matrix between all critical nodes
	private float[,] calcDistanceMatrix () {
		int nodeCount = world.pointsOfInterest.Length + agents.Length * 2;
		float[,] distanceMatrix = new float[nodeCount,nodeCount];
		Vector2[] nodeArray = getNodeArray ();

		for (int i = 0; i < nodeCount; i++)
			for (int j = 0; j < nodeCount; j++) 
				distanceMatrix [i, j] = Vector2.Distance (nodeArray[i],nodeArray[j]); //need to replace euclidean distance with obstacle free path distance found by some algorithm
		
		return distanceMatrix;
	}

	// Create List with all the nodes (points of interests, starting points, goal points)
	private Vector2[] getNodeArray() {
		int nodeCount = world.pointsOfInterest.Length + agents.Length * 2;
		Vector2[] nodeArray = new Vector2[nodeCount];
		for (int i = 0; i < world.pointsOfInterest.Length; i++)
			nodeArray [i] = world.pointsOfInterest [i];
		for (int i = 0; i < world.startPositions.Length; i++)
			nodeArray [world.pointsOfInterest.Length + i] = world.startPositions [i];
		for (int i = 0; i < world.goalPositions.Length; i++)
			nodeArray [world.pointsOfInterest.Length + world.startPositions.Length + i] = world.goalPositions [i];
		return nodeArray;
	}

	void initializeVelocities() {
		world.currentVelocities = new Vector2[world.startPositions.Length];
		for (int i = 0; i < world.currentVelocities.Length - 1; i++) {
			Vector2 x;
			if (data.name != "P25" && data.name != "P26" && data.name != "P27")
				x = world.goalPositions [i] - world.startPositions [i];
			else
				x = Vector2.up;
			world.currentVelocities [i] = x.normalized * world.vehicle.maxVelocity;
		}
	}


	void spawnActors() {

		if (world.startPositions.Length != 0 && world.enemyPositions.Length != 0) {
			//Spawn actors in a way suitable for the shooting problem
		} else if (world.startPositions.Length != 0 && world.goalPositions.Length != 0) {
			//Spawn actors in a way suitable for all the other problems
			agents = new GameObject[world.startPositions.Length];
			for (int i = 0; i < world.startPositions.Length; i++)
				spawnActor (world.startPositions [i], world.goalPositions [i], i); 
		} else if (world.startPositions.Length != 0 && world.formationPositions.Length != 0) {
			//P25 and P26 problems
			agents = new GameObject[world.formationPositions.Length];
			for (int i = 0; i < world.formationPositions.Length; i++)
				spawnActor (world.formationPositions [i], i); 
		}
	}

	void spawnActor(Vector2 position, Vector2 goal, int agentIdx) {
		agents [agentIdx] = (GameObject)Instantiate (agentPrefab);
		scaleAgent (agents[agentIdx]);
		agents [agentIdx].transform.position = new Vector3 (position.x, agents [agentIdx].transform.localScale.y / 2, position.y);
		agents [agentIdx].transform.LookAt(new Vector3(goal.x, objectHeight, goal.y));
		agents [agentIdx].transform.parent = agentParent.transform;
		agents [agentIdx].name = "AgentNo_" + agentIdx;
		if (data.name == "P21")
			agents [agentIdx].AddComponent<AgentControllerP21> ();
	}

	// Overloaded method for the problems that include formation
	void spawnActor(Vector2 position, int agentIdx) {
		agents [agentIdx] = (GameObject)Instantiate (agentPrefab);
		scaleAgent (agents[agentIdx]);
		agents [agentIdx].transform.position = new Vector3 (position.x, agents [agentIdx].transform.localScale.y / 2, position.y);
		agents [agentIdx].transform.LookAt(agents[agentIdx].transform.position + (Vector3.right));
		agents [agentIdx].transform.parent = agentParent.transform;
		agents [agentIdx].name = "AgentNo_" + agentIdx;
		if (data.name == "P21")
			agents [agentIdx].AddComponent<AgentControllerP21> ();
	}

	void scaleAgent( GameObject agent) {
		//MeshRenderer renderer = agent.GetComponent<MeshRenderer> ();
		Vector3[] vertices = agent.GetComponent<MeshFilter> ().mesh.vertices;
		float maxX, minX, maxY, minY, maxZ, minZ;
		maxX = maxY = maxZ = float.MinValue;
		minX = minY = minZ = float.MaxValue;
		for (int i = 0; i < vertices.Length; i++) {
			if (maxX < vertices [i].x)
				maxX = vertices [i].x;
			else if (vertices [i].x < minX)
				minX = vertices [i].x;
			if (maxY < vertices [i].y)
				maxY = vertices [i].y;
			else if (vertices [i].y < minY)
				minY = vertices [i].y;
			if (maxZ < vertices [i].z)
				maxZ = vertices [i].z;
			else if (vertices [i].z < minZ)
				minZ = vertices [i].z;
		}

		// Get size in each direction
		float x_size = Mathf.Abs (maxX - minX);
		float y_size = Mathf.Abs (maxY - minY);
		float z_size = Mathf.Abs (maxZ - minZ);
		// Find proportion of change needed to reach desired size radius
		float proportion_x = agentRadius / x_size;
		float proportion_y = agentRadius / y_size;
		float proportion_z = agentRadius / z_size;
		// Update object's vertices
		for (int i = 0; i < vertices.Length; i++) {
			vertices [i].x *= proportion_x;
			vertices [i].y *= proportion_y;
			vertices [i].z *= proportion_z;
		}
		// Update object
		agent.GetComponent<MeshFilter>().mesh.vertices = vertices;
	}

	void spawnObstacles() {
		//Spawn all the obstacles
		for (int i = 0; i < world.obstacles.Count; i++)
			spawnObstacle (world.obstacles [i], "obstacle_" + i, obstacleParent);
	}

	void spawnObstacle(Vector2[] obstacle, string name, GameObject parent) {
		//For each vertex spawn a wall between itself and the next vertex
		for (int i = 0; i < obstacle.Length - 1; i++) {
			Vector3 v1 = new Vector3(obstacle[i][0], objectHeight/2, obstacle[i][1]);
			Vector3 v2 = new Vector3(obstacle[i + 1][0], objectHeight/2, obstacle[i + 1][1]);
			spawnWall (v1, v2, name + "_Side" + i, parent);
		}
		// spawn wall between last and first vertices

		spawnWall (new Vector3(obstacle[0][0], objectHeight/2, obstacle[0][1]), new Vector3(obstacle[obstacle.Length - 1][0],  objectHeight/2, obstacle[obstacle.Length - 1][1]), name + "_Side" + (obstacle.Length - 1), parent);
	}

	void spawnWall(Vector3 v1, Vector3 v2, string name, GameObject parent) {
		//Spawn wall given two vertices and its name
		float width = Vector3.Distance(v1, v2);
		Vector3 polygonSide = v2 - v1;
		Vector3 middlePoint = v1 + polygonSide * 0.5f;
		GameObject wall = GameObject.CreatePrimitive (PrimitiveType.Cube);
		wall.name = name;
		wall.transform.position = middlePoint;
		wall.transform.rotation = Quaternion.LookRotation (polygonSide);
		wall.transform.localScale = new Vector3 (objectThickness, objectHeight, width);
		wall.transform.parent = parent.transform;
		//set obstacleParent as parent obstacle
	}
		
		

	/// <summary>
	/// Removes the first <count> rows and columns from the matrix and returns the result
	/// </summary>
	private float[,] getSubArray(float[,] matrix, int count) {
		int newCount = (int)Mathf.Sqrt(matrix.Length) - count;
		float[,] distanceMatrix = new float[newCount, newCount];
		for (int i = 0; i < newCount; i++)
			for (int j = 0; j < newCount; j++) 
				distanceMatrix [i, j] = matrix [count + i, count + j];
		return distanceMatrix;
	}


	/// <summary>
	/// Return the reconstructed shortest path indices given the indices of a path
	/// as well as the FloydWarshall object used to find the shortest paths.
	/// </summary>
	public List<int> reconstructShortestPath(List<int> path, FloydWarshall fw, int vehicles) {
		List<int> reconstructedPath = new List<int> ();
		for(int i = 0; i < path.Count - 1; i++) {
			reconstructedPath.Add (path[i]);
			// If the current node is a goal do not draw a path between it and the next node
			if (path [i] >= fw.getNodeCount () - vehicles) 
				continue;
			// For each pair of nodes i and i + 1 in path add the intermediate nodes		
			List<int> tmpPath = fw.reconstructShortestPath(path[i], path[i + 1]);
			for (int j = 1; j < tmpPath.Count - 1; j++) 
				reconstructedPath.Add (tmpPath [j]);
		}
		reconstructedPath.Add (path[path.Count - 1]);

		return reconstructedPath;
	}


	/// <summary>
	/// Based on the motion model chosen, returns 
	/// as well as the FloydWarshall object used to find the shortest paths.
	/// </summary>
	//private List<Stack<PointInfo>> getSolutionStacks(List<List<int>> solutionList) {
		
	//}
}
