using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualStructure : BaseFormationController {

	void Update () {
		this.desiredRelativePositions = getDesiredPositions (agents.Length - 1, false);
		this.desiredAbsolutePositions = getDesiredPositions (agents.Length - 1);
		//Visualizer.visualizePoints(this.desiredAbsolutePositions);
	}
	public void initializeController(GameObject[] agents, World.TrajectoryMap trajectory, Vector2[] formationPositions, float agentHeight) {
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
	}


	public static Vector2 getCenter(Vector2[] agentsPos) {
		// Return null if there are less than 2 agents in the structure
		if (agentsPos.Length < 2)
			throw new System.Exception("agentsPos parameter should have more than 1 element.");
		
		float xMax, xMin, yMax, yMin;
		xMax = yMax = float.MinValue;
		xMin = yMin = float.MaxValue;
		for (int i = 0; i < agentsPos.Length; i++) {
			if (agentsPos [i].x < xMin)
				xMin = agentsPos [i].x;
			else if (agentsPos [i].x > xMax)
				xMax = agentsPos [i].x;
			if (agentsPos [i].y < yMin)
				yMin = agentsPos [i].y;
			else if (agentsPos [i].y > yMax)
				yMax = agentsPos [i].y;
		}
		return new Vector2 ((xMax + xMin) / 2, (yMax + yMin) / 2);
	}
}
