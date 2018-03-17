using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LeaderFormationController : BaseFormationController {
	
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



	public Vector3[] getTrajectory() { return this.trajectory;	}
	public float[] getTrajectoryOrientation() { return this.trajectoryOrientation; }
	public float[] getTrajectoryTimestamps() { return this.trajectoryTimestamps; }
	public Vector3 getLeaderOrientation() { return UtilityClass.rads2Vec (getLeaderRotation());}
	public float getLeaderRotation() { return (90.0f - agents [0].transform.rotation.eulerAngles.y) * Mathf.Deg2Rad; }

}
