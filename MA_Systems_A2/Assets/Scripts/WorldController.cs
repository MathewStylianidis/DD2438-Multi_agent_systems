using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public enum model {
	KinematicPoint, DynamicPoint //, DifferentialDrive, KinematicCar
}

public enum WeaponTypeEnum {
	Shotgun,
	Rifle
}


public class WorldController : MonoBehaviour {

	// General use variables
	public Button startButton;
	public Text timeText;
	public TextAsset data;
	public GameObject agentPrefab;
	public World world;
	public GameObject obstacleParent;
	public GameObject agentParent;
	public GameObject boundingPolygon;
	public float agentRadius = 0.5f;
	public float objectHeight = 4f;
	public float objectThickness = 0.2f;
	public Material fieldMaterial;
	public model modelName;
	public float simulationSpeedFactor = 1.0f;
	private GameObject[] agents;
	private BaseModel motionModel; // Motion model to be used


	// VRP variables
	public GameObject pointOfInterestModel;
	public int populationSize = 10000;
	public int geneticAlgorithmIterations = 500;
	private List<List<PointInfo>> solutionCoordinates; // Coordinates of the path of each vehicle for the VRP
	private List<List<int>> solutionList; // List of indices with the visiting order for each agent
	private int longestPathIndex;
	private int obstacleVertCount; 

	// Formation problem variables
	public TextAsset trajectoryData;
	public float deltaX = 5.0f; // Determines the expansion of the virtual formation rectangle on the x axis (sports formation)
	public float deltaY = 5.0f; // Determines the expansion of the virtual formation rectangle on the y axis (sports formation)
	// Is true if the agents converge to a zero velocity on the goal formation point without paying respect to any goal velocity direction and magnitude
	public bool formationDecreasingGoalVelocity;

	//Shooting coordination variables
	public static System.Random rand = new System.Random();

	public WeaponTypeEnum weaponType = WeaponTypeEnum.Shotgun;


	void Start () {
		Button btn = startButton.GetComponent<Button>();
		btn.onClick.AddListener(TaskOnClick);

		if (data.name == "P25" || data.name == "P26")
			world = World.FromJson (data.text, trajectoryData.text);
		else
			world = World.FromJson (data.text);

		initializeVelocities ();
		spawnObstacle (world.boundingPolygon, "Bounding polygon", boundingPolygon);
		spawnObstacles ();
		spawnActors ();
		initializeMotionModel ();

		if (data.name == "P22") {
			solveVRP (world);
		} else if (data.name == "P25") {
			Visualizer.visualizeTrajectory (world.trajectory.x, world.trajectory.y);
			GameObject tmp = new GameObject ("LeaderFormationController");
			tmp.AddComponent<LeaderFormationController> ();
			tmp.GetComponent<LeaderFormationController> ().initializeController (agents, world.trajectory, world.formationPositions, agents [0].transform.localScale.y / 2, formationDecreasingGoalVelocity);
		} else if (data.name == "P26") {
			// Formation problems
			GameObject plane = GameObject.Find ("Plane");
			Renderer ren = plane.GetComponent<Renderer> ();
			ren.material = fieldMaterial;
			//for (int i = 0; i < world.trajectory.x.Length; i++) {
			//world.trajectory.x [i] += -10.0f;
			//world.trajectory.y [i] += 35.0f;
			//}
			Visualizer.visualizeTrajectory (world.trajectory.x, world.trajectory.y);
			agentParent.AddComponent<VirtualStructure> ();
			// Add virtual center to formation positions
			Vector2[] formationPositions = new Vector2[world.formationPositions.Length + 1];
			for (int i = 0; i < formationPositions.Length - 1; i++)
				formationPositions [i] = world.formationPositions [i];
			formationPositions [formationPositions.Length - 1] = new Vector2 (agents [agents.Length - 1].transform.position.x, agents [agents.Length - 1].transform.position.z); //position of virtual center.
			Debug.Log (formationPositions [formationPositions.Length - 1]);
			agentParent.GetComponent<VirtualStructure> ().initializeController (agents, world.boundingPolygon, world.trajectory, formationPositions, agents [0].transform.localScale.y / 2, deltaX, deltaY,
				simulationSpeedFactor, world.vehicle.dt, formationDecreasingGoalVelocity);
		} else if (data.name == "P27") {
			float[] minMaxes = VirtualStructure.getMinMaxes (world.boundingPolygon);
			int n = 100; // number of samples
			Vector2[] points = new Vector2[n];

			for (int i = 0; i < n; i++) {
				do {
					points[i].x = (float)(rand.NextDouble () * (minMaxes [2] - minMaxes [0]) + minMaxes [0]);
					points[i].y = (float)(rand.NextDouble () * (minMaxes [3] - minMaxes [1]) + minMaxes [1]);
				} while ((!Raycasting.insidePolygon(points[i].x, points[i].y, world.boundingPolygon)) || Raycasting.insideObstacle (points[i].x, points[i].y, world.obstacles));
			}
			List<Vector2> addPoints = new List<Vector2> (points);
			foreach (var point in world.startPositions) {
				addPoints.Add (point);
			}
			foreach (var point in world.enemyPositions) {
				addPoints.Add (point);
			}
			VisibilityGraph.initVisibilityGraph (world, addPoints);

			agentParent.AddComponent<ShooterController> ();
            List<ShootingPlanner.OneStepPlan> gamePlan = (new ShootingPlanner (world, weaponType)).getPlan ();
			agentParent.GetComponent<ShooterController> ().initializeController (gamePlan);

		}
	}
	

	void Update () {
	}


	void TaskOnClick()
	{
		if (data.name == "P22") {
			for (int agentIdx = 0; agentIdx < agents.Length; agentIdx++)
				agents [agentIdx].AddComponent<AgentControllerVRP> ();
		}
	}

	public BaseModel getMotionModel() {
		return this.motionModel;
	}

	/// <summary>
	/// Given the sibling index of the agent get its respective route to follow
	/// </summary>
	public List<PointInfo> getRoute(int siblingIdx) {
		for (int i = 0; i < solutionList.Count; i++)
			// If the first node on the i_th path is this agent's starting position
			if (solutionList [i] [0] == obstacleVertCount + world.pointsOfInterest.Length + siblingIdx) 
				return solutionCoordinates [i];
		return null;
	}
		
	/// <summary>
	/// Returns true when the index of the agent with the longest path is provided.
	/// </summary>
	public bool keepTime(int siblingIdx) {
		if (solutionList [longestPathIndex] [0] == obstacleVertCount + world.pointsOfInterest.Length + siblingIdx)
			return true;
		return false;
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

	private void initializeVelocities() {
		if(data.name != "P26")
			world.currentVelocities = new Vector2[world.startPositions.Length];
		else // Sports formation need one extra velocity for the virtual structure center
			world.currentVelocities = new Vector2[world.startPositions.Length + 1];
		
		for (int i = 0; i < world.currentVelocities.Length; i++) {
			Vector2 x;
			if (data.name != "P25" && data.name != "P26" && data.name != "P27")
				x = world.goalPositions [i] - world.startPositions [i];
			else
				x = Vector3.zero ; // Should look towards formation position
			world.currentVelocities [i] = x.normalized * world.vehicle.maxVelocity;
		}
	}


	private void spawnActors() {

		if (world.startPositions.Length != 0 && world.enemyPositions.Length != 0) {
			//Spawn actors in a way suitable for the shooting problem
			agents = new GameObject[world.startPositions.Length + world.enemyPositions.Length];
			for (int i = 0; i < world.startPositions.Length; i++) {
				Vector3 orientation = Vector3.zero;
				spawnActor (world.startPositions [i], orientation, i); 
			}
			for (int i = 0; i < world.enemyPositions.Length; i++) {
				Vector3 orientation = Vector3.zero;
				GameObject actor = spawnActor (world.enemyPositions [i], orientation, world.startPositions.Length + i); 
				Renderer ren = actor.GetComponent<Renderer> ();
				ren.material = fieldMaterial;
			}
		} else if (world.startPositions.Length != 0 && world.goalPositions.Length != 0) {
			//Spawn actors in a way suitable for all the other problems
			world.currentAngularVel = new float[world.startPositions.Length];
			world.currentPositions = (Vector2[]) world.startPositions.Clone ();
			agents = new GameObject[world.startPositions.Length];
			for (int i = 0; i < world.startPositions.Length; i++) {
				Vector3 orientation = new Vector3 (world.goalPositions [i].x, objectHeight, world.goalPositions [i].y).normalized;
				spawnActor (world.startPositions [i], orientation, i); 
			}
		} else if (world.startPositions.Length != 0 && world.formationPositions.Length != 0) {
			// Actor spawning for P25 and P26 problems
			if (data.name == "P25") {
				world.currentAngularVel = new float[world.startPositions.Length + 1];
				world.currentPositions = new Vector2[world.startPositions.Length + 1];
				agents = new GameObject[world.startPositions.Length + 1];
				// Initialize leader
				Vector3 orientation = UtilityClass.rads2Vec (world.trajectory.theta [0]);
				Vector2 pos = new Vector2 (world.trajectory.x [0], world.trajectory.y [0]);
				spawnActor (pos, orientation, 0);
				world.currentPositions [0] = pos;
				// Initialize followers
				for (int i = 0; i < world.startPositions.Length; i++) {
					orientation = Vector3.left; //change so that they look towards their formation position
					spawnActor (world.startPositions [i], orientation, i + 1);
					world.currentPositions [i + 1] = world.startPositions [i];
				}
			} else if (data.name == "P26") {
				world.currentAngularVel = new float[world.startPositions.Length + 2];
				world.currentPositions = new Vector2[world.startPositions.Length + 2];
				agents = new GameObject[world.startPositions.Length + 2];
				// Initialize opponent football player
				Vector3 orientation = UtilityClass.rads2Vec (world.trajectory.theta [0]);
				Vector2 pos = new Vector2 (world.trajectory.x [0], world.trajectory.y [0]);
				spawnActor (pos, orientation, 0);
				world.currentPositions [0] = pos;
				// Initialize players
				for (int i = 0; i < world.startPositions.Length; i++) {
					orientation = Vector3.left; //change so that they look towards their formation position
					spawnActor (world.startPositions [i], orientation, i + 1);
					world.currentPositions [i + 1] = world.startPositions [i];
				}
				// Initialize virtual structure center
				world.currentPositions[world.currentPositions.Length - 1] = VirtualStructure.getCenter(world.formationPositions);
				spawnActor (world.currentPositions [world.currentPositions.Length - 1], Vector3.forward, agents.Length - 1, false);
			}
		}
	}

	private GameObject spawnActor(Vector2 position, Vector3 orientation, int agentIdx, bool addPrefab = true) {
		if (addPrefab) {
			agents [agentIdx] = (GameObject)Instantiate (agentPrefab);
			scaleAgent (agents [agentIdx]); 
		} else
			agents [agentIdx] = new GameObject ();
		agents [agentIdx].transform.position = new Vector3 (position.x, agents [agentIdx].transform.localScale.y / 2, position.y);
		agents [agentIdx].transform.LookAt(agents [agentIdx].transform.position  + orientation);
		agents [agentIdx].transform.parent = agentParent.transform;
		agents [agentIdx].transform.SetSiblingIndex (agentIdx);
		agents [agentIdx].name = "AgentNo_" + agentIdx;
		if (data.name == "P21")
			agents [agentIdx].AddComponent<AgentControllerCollAvoidance> ();
		return agents [agentIdx];
	}


	private void scaleAgent( GameObject agent) {
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

	private void spawnObstacles() {
		//Spawn all the obstacles
		for (int i = 0; i < world.obstacles.Count; i++)
			spawnObstacle (world.obstacles [i], "obstacle_" + i, obstacleParent);
	}

	private void spawnObstacle(Vector2[] obstacle, string name, GameObject parent) {
		//For each vertex spawn a wall between itself and the next vertex
		for (int i = 0; i < obstacle.Length - 1; i++) {
			Vector3 v1 = new Vector3(obstacle[i][0], objectHeight/2, obstacle[i][1]);
			Vector3 v2 = new Vector3(obstacle[i + 1][0], objectHeight/2, obstacle[i + 1][1]);
			spawnWall (v1, v2, name + "_Side" + i, parent);
		}
		// spawn wall between last and first vertices

		spawnWall (new Vector3(obstacle[0][0], objectHeight/2, obstacle[0][1]), new Vector3(obstacle[obstacle.Length - 1][0],  objectHeight/2, obstacle[obstacle.Length - 1][1]), name + "_Side" + (obstacle.Length - 1), parent);
	}

	private void spawnWall(Vector3 v1, Vector3 v2, string name, GameObject parent) {
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
		
	private void spawnPointOfInterestObjects() {
		for (int i = 0; i < world.pointsOfInterest.Length; i++) {
			GameObject gameObject = Instantiate(pointOfInterestModel);
			gameObject.transform.position = new Vector3 (world.pointsOfInterest [i].x, 0, world.pointsOfInterest [i].y);
		}
	}	


	private void initializeMotionModel()
	{
		if (modelName.Equals (model.KinematicPoint))
			motionModel = new KinematicPoint (world.vehicle.maxVelocity, world.vehicle.dt);
		else if(modelName.Equals(model.DynamicPoint))
			motionModel = new DynamicPoint (world.vehicle.maxVelocity, world.vehicle.dt, world.vehicle.maxAcceleration);
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
	/// Based on the motion model chosen, returns the list of coordinates each 
	/// agent has to visit.
	/// </summary>
	private List<List<PointInfo>> getSolutionCoordinates(List<List<int>> solutionList) {
		List<List<PointInfo>> solutionCoords = new List<List<PointInfo>> ();
		int maxPathLength = -1;

		// For each agent
		for (int i = 0; i < solutionList.Count; i++) {
			solutionCoords.Add(new List<PointInfo> ());
			float time = 0.0f;
			Vector3 curVelocity = new Vector3 (world.currentVelocities [i].x, 0, world.currentVelocities [i].y);
			Vector3 curOrientation = curVelocity.normalized; //should probably be initialized with the orientation to solution j + 1 from j
			// For each node in his path
			for (int j = 0; j < solutionList [i].Count - 1; j++) {
				// Get the coordinates between node j and j + 1
				Vector3 curPos = new Vector3(world.graphVertices[solutionList[i][j]].vertex.x, 0, world.graphVertices[solutionList[i][j]].vertex.y);
				Vector3 goalPos = new Vector3 (world.graphVertices[solutionList[i][j + 1]].vertex.x, 0, world.graphVertices[solutionList[i][j + 1]].vertex.y);
				PointInfo curPointInfo = new PointInfo(curPos, curVelocity, curOrientation, time);
				PointInfo goalPointInfo = new PointInfo (goalPos, Vector3.zero, Vector3.zero, time + world.vehicle.dt);
				List<PointInfo> subpathCoords = motionModel.completePath (curPointInfo, goalPointInfo, world, false);
				if (subpathCoords == null)
					throw new System.Exception ("A path has a point in an obstacle");
				curVelocity = subpathCoords [subpathCoords.Count - 1].vel;
				curOrientation = subpathCoords [subpathCoords.Count - 1].orientation;
				time = subpathCoords [subpathCoords.Count - 1].currentTime;
				// Add subpath coordinates to this vehicle's list
				if(j == 0)
					solutionCoords[i].Add(curPointInfo);
				solutionCoords[i].AddRange(subpathCoords);
			}

			if (solutionCoords [i].Count > maxPathLength) {
				longestPathIndex = i;
				maxPathLength = solutionCoords [i].Count;
			}
		}

	    
		return solutionCoords;
	}



	private void solveVRP(World world) {
		// Spawns objects that visualize the points of interest
		spawnPointOfInterestObjects ();
		// Initialize visibility graph

		List<Vector2> addPoints = new List<Vector2> (world.pointsOfInterest);
		foreach (var point in world.startPositions) {
			addPoints.Add (point);
		}
		foreach (var point in world.goalPositions) {
			addPoints.Add (point);
		}
		VisibilityGraph.initVisibilityGraph (world, addPoints);
		// Execute FloydWarshall algorithm on visibility graph to get shortest paths
		FloydWarshall fw = new FloydWarshall(world.visibilityGraph);
		float[,] floydWarshallDistMatrix = fw.findShortestPaths ();
		// Get number of obstacle vertices
		obstacleVertCount = world.graphVertices.Count - world.pointsOfInterest.Length - agents.Length * 2;
		// Remove obstacle vertex rows and columns from the distance matrix as GA does not work with them to find a solution
		float[,] distanceMatrix = getSubArray(floydWarshallDistMatrix, obstacleVertCount);
		// Use the genetic algorithm for the Vehicle Routing Problem
		GeneticAlgorithm ga = new GeneticAlgorithm (populationSize, populationSize, world.pointsOfInterest.Length, agents.Length, distanceMatrix, 0.02f, geneticAlgorithmIterations , false, 0.04f, 0.01f, true);
		ga.generationalGeneticAlgorithm ();
		List<int> solution = ga.getFittestIndividual ();
		solution = GeneticAlgorithmHelper.includeSolutionGoals (solution, world.pointsOfInterest.Length, agents.Length);
		// Reconstruct path including intermediate obstacle vertex nodes
		for (int i = 0; i < solution.Count; i++)
			solution [i] += obstacleVertCount;
		List<int> reconstructedSolution = reconstructShortestPath (solution, fw, agents.Length);
		// Visualize the reconstructed path (solution including intermediate nodes)
		Visualizer.visualizeVRPsolution(world.graphVertices, reconstructedSolution, agents.Length, obstacleVertCount); 
		solutionList = GeneticAlgorithmHelper.splitSolution(solution, world.graphVertices.Count, agents.Length);
		for (int i = 0; i < solutionList.Count; i++) 
			solutionList[i] = reconstructShortestPath (solutionList [i], fw, agents.Length);
		solutionCoordinates = getSolutionCoordinates (solutionList);
	}



}
