using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WorldController : MonoBehaviour {

	public TextAsset data;
	public GameObject agentPrefab;
	public World world;
	public GameObject obstacleParent;
	public GameObject agentParent;
	public GameObject boundingPolygon;
	public float agentRadius = 0.5f;
	public float objectHeight = 4f;
	public float objectThickness = 0.2f;
	private GameObject[] agents;

		
	// Use this for initialization
	void Start () {
		world = World.FromJson(data.text);
		spawnObstacle (world.boundingPolygon, "Bounding polygon", boundingPolygon);
		spawnObstacles ();
		spawnActors ();
	}
	

	void Update () {
		
	}











	void spawnActors() {

		if (world.startPositions.Length != 0 && world.enemyPositions.Length != 0) {
				//Spawn actors in a way suitable for the shooting problem
		} else if (world.startPositions.Length != 0 && world.goalPositions.Length != 0) {
			//Spawn actors in a way suitable for all the other problems
			agents = new GameObject[world.startPositions.Length];
			for (int i = 0; i < world.startPositions.Length; i++) 
				spawnActor (world.startPositions[i], world.goalPositions[i], i); 
		}
	}

	void spawnActor(Vector2 position, Vector2 goal, int agentIdx) {
		agents [agentIdx] = (GameObject)Instantiate (agentPrefab);
		scaleAgent (agents[agentIdx]);
		agents [agentIdx].transform.rotation = Quaternion.LookRotation (new Vector3(goal.x, objectHeight, goal.y));
		agents [agentIdx].transform.position = new Vector3 (position.x, agents [agentIdx].transform.localScale.y / 2, position.y);
		agents [agentIdx].transform.parent = agentParent.transform;
		agents [agentIdx].name = "AgentNo_" + agentIdx;
	}

	void scaleAgent( GameObject agent) {
		MeshRenderer renderer = agent.GetComponent<MeshRenderer> ();
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



}
