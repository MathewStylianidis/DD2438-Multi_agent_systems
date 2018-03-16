using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FormationController : MonoBehaviour {

	public float agentHeight;
	private GameObject[] agents; // Agents in the formation
	private Vector3[] trajectory; // Trajectory coordinates
	private float[] trajectoryOrientation; // Orientation of virtual structure in each step of the trajectory
	private float[] trajectoryTimestamps; // Timestamp <t> at each step of the trajectory
	private Vector2[] formationPositions; // The formation positions of each agent assuming the first agent is the leader at 0.0
	private Vector2[] desiredRelativePositions; // True desired positions relative to the leader's position and orientation
	private Vector2[] desiredAbsolutePositions; // True desired world positions for the whole formation


	void Start () {
	}
	

	void Update () {
		this.desiredRelativePositions = getDesiredPositions (false);
		this.desiredAbsolutePositions = getDesiredPositions ();
		//Visualizer.visualizePoints(this.desiredAbsolutePositions);
	}

	public Vector2 getDesiredPosition(int agentIdx, bool absolute = true) {
		if(absolute)
			return desiredAbsolutePositions[agentIdx];
		else
			return desiredRelativePositions[agentIdx];
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
		// Read positioning of the formations
		this.formationPositions = getRelativeFormationPositions(formationPositions);	
		// Get starting absolute positions and relative to the leader positions.
		this.desiredRelativePositions = getDesiredPositions (false);
		this.desiredAbsolutePositions = getDesiredPositions ();
		// Visualize starting desired positions
		Visualizer.visualizePoints(this.desiredRelativePositions);
		//Visualizer.visualizePoints(this.desiredAbsolutePositions);
		// Set a controller within each agent
		agents[0].AddComponent<LeaderController>();
		for (int i = 1; i < agents.Length; i++)
			agents [i].AddComponent<FollowerController> ();
	}

	/// <summary>
	/// Returns the relative to the leader coordinates assuming that the leader has a forward orientation.
	/// The first position in the formation is regarded as the leader.
	/// </summary>
	private Vector2[] getRelativeFormationPositions(Vector2[] formationPositions) {
		Vector2 leaderPosition = formationPositions [0];
		Vector2[] translatedFormationPositions = new Vector2[formationPositions.Length];
		for (int i = 0; i < formationPositions.Length; i++)
			translatedFormationPositions [i] = formationPositions [i] - leaderPosition;			
		return translatedFormationPositions;
	}

	/// <summary>
	/// Get the positions of the formation, either in absolute coordinates or relative to the
	/// leader's coordinates and starting orientation.
	/// </summary>
	private Vector2[] getDesiredPositions(bool absolute = true) {
		Vector2[] desiredPositions = new Vector2[agents.Length];
		Vector3 leaderPosition = agents [0].transform.position;
		Vector2 leaderPosition2D = new Vector2(leaderPosition.x, leaderPosition.z);
		float[][] rotMatrix = UtilityClass.getRotationYMatrix(-agents [0].transform.rotation.eulerAngles.y * Mathf.Deg2Rad);
		for (int i = 0; i < agents.Length; i++) {
			desiredPositions [i] = UtilityClass.rotateVector(rotMatrix, this.formationPositions [i]);
			if (absolute) 
				desiredPositions [i] += leaderPosition2D;
		}
		return desiredPositions;
	}


	public Vector3[] getTrajectory() { return this.trajectory;	}
	public float[] getTrajectoryOrientation() { return this.trajectoryOrientation; }
	public float[] getTrajectoryTimestamps() { return this.trajectoryTimestamps; }
	public Vector3 getLeaderOrientation() { return UtilityClass.rads2Vec (getLeaderRotation());}
	public float getLeaderRotation() { return (90.0f - agents [0].transform.rotation.eulerAngles.y) * Mathf.Deg2Rad; }

}
