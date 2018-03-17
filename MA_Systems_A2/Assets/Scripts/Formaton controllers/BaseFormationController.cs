using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BaseFormationController : MonoBehaviour {

	public float agentHeight;
	protected GameObject[] agents; // Agents in the formation
	protected Vector3[] trajectory; // Trajectory coordinates
	protected float[] trajectoryOrientation; // Orientation of virtual structure in each step of the trajectory
	protected float[] trajectoryTimestamps; // Timestamp <t> at each step of the trajectory
	protected Vector2[] formationPositions; // The formation positions of each agent assuming the first agent is the leader at 0.0
	protected Vector2[] desiredRelativePositions; // True desired positions relative to the leader's position and orientation
	protected Vector2[] desiredAbsolutePositions; // True desired world positions for the whole formation

	public Vector2 getDesiredPosition(int agentIdx, bool absolute = true) {
		if(absolute)
			return desiredAbsolutePositions[agentIdx];
		else
			return desiredRelativePositions[agentIdx];
	}

	/// <summary>
	/// Returns the relative to the leader coordinates assuming that the leader has a forward orientation.
	/// The first position in the formation is regarded as the leader.
	/// </summary>
	protected Vector2[] getRelativeFormationPositions(Vector2[] formationPositions) {
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
	protected Vector2[] getDesiredPositions(bool absolute = true) {
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
