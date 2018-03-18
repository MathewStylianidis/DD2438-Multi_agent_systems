using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LeaderFormationController : BaseFormationController {
	
	void Start () {
	}
	

	void Update () {
		this.desiredRelativePositions = getDesiredPositions (0, false);
		this.desiredAbsolutePositions = getDesiredPositions (0);
		Visualizer.visualizePoints(this.desiredAbsolutePositions);
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
		// set formationPositions in parent class to relative positions from the leader
		setRelativeFormationPositions(formationPositions, 0);	
		// Get starting absolute positions and relative to the leader positions.
		this.desiredRelativePositions = getDesiredPositions (0, false);
		this.desiredAbsolutePositions = getDesiredPositions (0);
		// Visualize starting desired positions
		Visualizer.visualizePoints(this.desiredRelativePositions);
		//Visualizer.visualizePoints(this.desiredAbsolutePositions);
		// Set a controller within each agent
		agents [0].AddComponent<LeaderController> ();
		for (int i = 1; i < agents.Length; i++)
			agents [i].AddComponent<FollowerController> ();
	}
}
