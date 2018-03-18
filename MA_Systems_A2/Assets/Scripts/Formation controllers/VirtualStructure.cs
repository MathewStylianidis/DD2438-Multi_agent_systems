using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualStructure : BaseFormationController {

	class VirtualStructureRectangle {
		public PointInfo[] vertices;

		public PointInfo[] getEdges() {return this.vertices;}

		public Vector2[] getEdgeVectors() {
			Vector2[] vectors = new Vector2[vertices.Length];
			for (int i = 0; i < vertices.Length; i++)
				vectors [i] = new Vector2(vertices [i].pos.x, vertices[i].pos.z);
			return vectors;
		}

		public VirtualStructureRectangle(Vector2[] formationPositions, Vector2[] boundingPoly, float deltaX, float deltaY, float agentHeight) {
			Vector2 center = getCenter(formationPositions);
			float[] formationMinMaxes = getMinMaxes(formationPositions);
			float[] fieldMinMaxes = getMinMaxes(boundingPoly);
			float fieldSideX = Mathf.Abs(fieldMinMaxes[2] - fieldMinMaxes[0]);
			float fieldSideY = Mathf.Abs(fieldMinMaxes[3] - fieldMinMaxes[1]);
			float formationSideX = Mathf.Abs(formationMinMaxes[2] - formationMinMaxes[0]);
			float formationSideY = Mathf.Abs(formationMinMaxes[3] - formationMinMaxes[1]);
			if(fieldSideX < formationSideX || fieldSideY < formationSideY)
				throw new System.Exception("The formation size is too large to fit in the field");
			float newFormationSideX = formationSideX + deltaX;
			float newFormationSideY = formationSideY + deltaY;
			if(fieldSideX < newFormationSideX || fieldSideY < newFormationSideY)
				throw new System.Exception("The virtual formation rectangle is too large to fit in the field");
			vertices = new PointInfo[4];
			// Upper left 
			vertices[0] = new PointInfo(new Vector3(center.x - newFormationSideX/2, agentHeight, center.y + newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			// Upper right 
			vertices[1] = new PointInfo(new Vector3(center.x + newFormationSideX/2, agentHeight, center.y + newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			// Bottom right 
			vertices[2] = new PointInfo(new Vector3(center.x + newFormationSideX/2, agentHeight, center.y - newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			// Bottom left
			vertices[3] = new PointInfo(new Vector3(center.x - newFormationSideX/2, agentHeight, center.y - newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			Visualizer.visualizePoints(getEdgeVectors());
		}
	}
			
	VirtualStructureRectangle formationRectangle;





	void Update () {
		// Get positions of virtual structure edges
		PointInfo[] edges = formationRectangle.getEdges ();

		Vector3 targetPosition = agents [0].transform.position;
		// If the opponent is not inside the virtual structure rectangle
		if (!Raycasting.insidePolygon (targetPosition.x, targetPosition.z, formationRectangle.getEdgeVectors ())) {
			// Get min x,y and max x,y of the virtual rectangle
			float[] rectMinMaxes = getMinMaxes (formationRectangle.getEdgeVectors ());
			if (isInFrontOfFormation (targetPosition, rectMinMaxes)) {
				
			} else if (isNextToFormation (targetPosition, rectMinMaxes)) {
				
			} else {
				
				//Otherwise get closest edge and move the whole structure
				// Move virtual center based on position of the closest edge to the opponent
				/*float minDistance = float.MaxValue;
			int minIdx = -1;
			for (int i = 0; i < edges.Length; i++) {
				float distance = Vector3.Distance (edges [i].pos, agents [0].transform.position);
				if (distance < minDistance) {
					minDistance = distance;
					minIdx = i;
				}
			}*/
			}


		}

		this.desiredRelativePositions = getDesiredPositions (agents.Length - 1, false);
		this.desiredAbsolutePositions = getDesiredPositions (agents.Length - 1);
		//Visualizer.visualizePoints(this.desiredAbsolutePositions);
	}

	public void initializeController(GameObject[] agents, Vector2[] boundingPoly, World.TrajectoryMap trajectory, Vector2[] formationPositions, float agentHeight, float deltaX, float deltaY) {
		this.agentHeight = agentHeight;
		this.agents = agents;
		// Get trajectory coordinates
		this.trajectory = new Vector3[trajectory.x.Length];
		for (int i = 0; i < trajectory.x.Length; i++) 
			this.trajectory [i] = new Vector3 (trajectory.x [i], agentHeight, trajectory.y [i]);
		// Get trajectory orientations
		this.trajectoryOrientation = new float[trajectory.theta.Length];
		for (int i = 0; i < trajectory.theta.Length; i++) 
			this.trajectoryOrientation [i] = trajectory.theta [i];
		// Get timestamps for each step of the trajectory
		this.trajectoryTimestamps = new float[trajectory.t.Length];
		for (int i = 0; i < trajectory.t.Length; i++)
			this.trajectoryTimestamps [i] = trajectory.t [i];
		// set formationPositions in parent class to relative positions from the virtual center
		setRelativeFormationPositions(formationPositions, formationPositions.Length - 1);	
		// Get starting absolute positions and relative to the leader positions.
		this.desiredRelativePositions = getDesiredPositions (agents.Length - 1, false);
		this.desiredAbsolutePositions = getDesiredPositions (agents.Length - 1);
		// Visualize starting desired positions
		//Visualizer.visualizePoints(this.desiredRelativePositions);
		//Visualizer.visualizePoints(this.desiredAbsolutePositions);
		// Set a controller within each agent
		agents [0].AddComponent<LeaderController> ();
		for (int i = 1; i < agents.Length; i++)
			agents [i].AddComponent<FootballPlayerController> ();
		formationRectangle = new  VirtualStructureRectangle (formationPositions, boundingPoly, deltaX, deltaY, agentHeight);
	}


	public static Vector2 getCenter(Vector2[] formationPositions) {
		float[] minMaxes = getMinMaxes (formationPositions);
		return new Vector2 ((minMaxes[0] +  minMaxes[2]) / 2, (minMaxes[1] + minMaxes[3]) / 2);
	}

	/// <summary>
	/// Returns an float array with the minimum x and y as well as the maximum x and y
	/// in the formationPositions, in this order.
	/// </summary>
	public static float[] getMinMaxes(Vector2[] formationPositions) {
		// Return null if there are less than 2 agents in the structure
		if (formationPositions.Length < 2)
			throw new System.Exception("agentsPos parameter should have more than 1 element.");
		float[] minMaxes = {float.MaxValue, float.MaxValue, float.MinValue, float.MinValue};
		for (int i = 0; i < formationPositions.Length; i++) {
			if (formationPositions [i].x < minMaxes[0])
				minMaxes[0] = formationPositions [i].x;
			else if (formationPositions [i].x > minMaxes[2])
				minMaxes[2] = formationPositions [i].x;
			if (formationPositions [i].y < minMaxes[1])
				minMaxes[1] = formationPositions [i].y;
			else if (formationPositions [i].y > minMaxes[3])
				minMaxes[3] = formationPositions [i].y;
		}
		return minMaxes;
	}


	private bool isInFrontOfFormation(Vector3 targetPos, float[] areaMinMaxes) {
		if (targetPos.x >= areaMinMaxes [0] && targetPos.x <= areaMinMaxes [2])
			return true;
		return false;
	}

	private bool isNextToFormation(Vector3 targetPos, float[] areaMinMaxes) {
		if (targetPos.z >= areaMinMaxes [1] && targetPos.z <= areaMinMaxes [3])
			return true;
		return false;
	}
}
