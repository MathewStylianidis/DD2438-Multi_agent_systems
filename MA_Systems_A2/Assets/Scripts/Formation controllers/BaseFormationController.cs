using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BaseFormationController : MonoBehaviour {

	public float agentHeight;
	public bool formationDecreasingGoalVelocity; // Is true if the agents converge to a zero velocity on the goal formation point without paying respect to any goal velocity direction and magnitude
	protected GameObject[] agents; // Agents in the formation
	protected Vector3[] trajectory; // Trajectory coordinates
	protected float[] trajectoryOrientation; // Orientation of virtual structure in each step of the trajectory
	protected float[] trajectoryTimestamps; // Timestamp <t> at each step of the trajectory
	protected Vector2[] relativeFormationPositions; // The formation positions of each agent assuming the first agent is the leader at 0.0
	protected Vector2[] desiredRelativePositions; // True desired positions relative to the leader's position and orientation
	protected Vector2[] desiredAbsolutePositions; // True desired world positions for the whole formation
	protected Vector3[] currentVelocities;

	public GameObject[] getAgents() {return agents;}

	public Vector2 getDesiredPosition(int agentIdx, bool absolute = true) {
		if(absolute)
			return desiredAbsolutePositions[agentIdx];
		else
			return desiredRelativePositions[agentIdx];
	}

	/// <summary>
	/// Returns the relative to an agent coordinates assuming that the agent has a forward orientation.
	/// The agent which is considered to be the center of the formation at (0,0) is denoted by agentReferenceIndex.
	/// </summary>
	protected void setRelativeFormationPositions(Vector2[] formationPositions, int agentReferenceIndex) {
		Vector2 referencePosition = formationPositions [agentReferenceIndex];
		Vector2[] translatedFormationPositions = new Vector2[formationPositions.Length];
		for (int i = 0; i < formationPositions.Length; i++) 
			translatedFormationPositions [i] = formationPositions [i] - referencePosition;			
		this.relativeFormationPositions = translatedFormationPositions;
	}

	/// <summary>
	/// Get the positions of the formation, either in absolute coordinates or relative to the
	/// a given agent's coordinates and starting orientation.
	/// </summary>
	protected Vector2[] getDesiredPositions(int agentReferenceIndex, bool rotate = true, bool absolute = true) {
		Vector2[] desiredPositions = new Vector2[this.relativeFormationPositions.Length];
		Vector3 leaderPosition = agents [agentReferenceIndex].transform.position;
		Vector2 leaderPosition2D = new Vector2(leaderPosition.x, leaderPosition.z);
		float[][] rotMatrix = null;
		if(rotate)
			rotMatrix = UtilityClass.getRotationYMatrix(-agents [agentReferenceIndex].transform.rotation.eulerAngles.y * Mathf.Deg2Rad);
		for (int i = 0; i < this.relativeFormationPositions.Length; i++) {
			if (rotate)
				desiredPositions [i] = UtilityClass.rotateVector (rotMatrix, this.relativeFormationPositions [i]);
			else
				desiredPositions [i] = this.relativeFormationPositions [i];
			if (absolute) 
				desiredPositions [i] += leaderPosition2D;
		}
		return desiredPositions;
	}


	public void setCurrentVelocity(int agentIndex, Vector3 value) {currentVelocities [agentIndex] = value;}
	public Vector3 getAgentVelocity(int agentIndex) {return currentVelocities [agentIndex];}
	public Vector3[] getTrajectory() { return this.trajectory;	}
	public float[] getTrajectoryOrientation() { return this.trajectoryOrientation; }
	public float[] getTrajectoryTimestamps() { return this.trajectoryTimestamps; }
	public Vector3 getAgentOrientation(int agentIndex) { return UtilityClass.rads2Vec (getAgentRotation(agentIndex));}
	public float getAgentRotation(int agentIndex) { return (90.0f - agents [agentIndex].transform.rotation.eulerAngles.y) * Mathf.Deg2Rad; }

}
