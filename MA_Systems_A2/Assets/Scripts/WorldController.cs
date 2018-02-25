using System.Collections;
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
 
	// Use this for initialization
	void Start () {
		world = World.FromJson (data.text); //, trajectoryData.text);
		world.currentPositions = (Vector2[]) world.startPositions.Clone ();
		world.currentAngularVel = new float[world.currentPositions.Length];
		initializeVelocities ();
		spawnObstacle (world.boundingPolygon, "Bounding polygon", boundingPolygon);
		spawnObstacles ();
		spawnActors ();

		if (data.name == "P22") {
			initVisibilityGraph ();

			// Use a genetic algorithm for the Vehicle Routing Problem
			int M = 10000;
			int lambda = 10000;
			float[,] distanceMatrix = calcDistanceMatrix ();
			GeneticAlgorithm ga = new GeneticAlgorithm (M, lambda, world.pointsOfInterest.Length, agents.Length, distanceMatrix, 0.02f, 50, false, 0.04f);
			ga.generationalGeneticAlgorithm ();
			List<int> solution = ga.getFittestIndividual ();
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

	void initVisibilityGraph () {

		var vertices = new List<Vector2> ();
		var obstEdges = new List<Segment> ();

		foreach (var obstacle in world.obstacles) {
			var prevVertex = obstacle [obstacle.Length - 1];
			foreach (var obstVertex in obstacle) {
				obstEdges.Add (new Segment (prevVertex, obstVertex));
				vertices.Add (obstVertex);
				prevVertex = obstVertex;
			}
		}

		foreach (var vertex in world.startPositions) {
			vertices.Add (vertex);
		}

		foreach (var vertex in world.goalPositions) {
			vertices.Add (vertex);
		}

		var visibilityGraph = new float[vertices.Count][];
		for (var i = 0; i < vertices.Count; i++) {
			var v1 = vertices [i];
			visibilityGraph [i] = new float[vertices.Count];
			for (var j = 0; j < vertices.Count; j++) {
				if (i == j) {
					visibilityGraph [i] [j] = 0;
					continue;
				}

				var v2 = vertices [j];

				for (var k = 0; k < obstEdges.Count; k++) {
					if (segmentsIntersect (v1, v2, obstEdges [k].vertex1, obstEdges [k].vertex2)) {
						visibilityGraph [i] [j] = float.MaxValue;
						break;
					}
					visibilityGraph [i] [j] = dist (v1, v2);
				}
			}
		}

		world.visibilityGraph = visibilityGraph;
		world.graphVertices = vertices;

	}

	// v11 -> v12 & v21 -> v22
	private bool segmentsIntersect(Vector2 v11, Vector2 v12, Vector2 v21, Vector2 v22) {
		
		var det = (v12.x - v11.x) * (v22.y - v21.y) - (v22.x - v21.x) * (v12.y - v11.y);
		if (det == 0) {
			return false;
		}

		var lambda = ((v22.y - v21.y) * (v22.x - v11.x) + (v21.x - v22.x) * (v22.y - v11.y)) / det;
		var gamma = ((v11.y - v12.y) * (v22.x - v11.x) + (v12.x - v11.x) * (v22.y - v11.y)) / det;
		return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
	}

	private float dist(Vector2 v1, Vector2 v2) {
		return Mathf.Sqrt ((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y));
	}

	class Segment {
		public readonly Vector2 vertex1;
		public readonly Vector2 vertex2;

		public Segment(Vector2 vertex1, Vector2 vertex2) {
			this.vertex1 = vertex1;
			this.vertex2 = vertex2;
		}
	}
}
