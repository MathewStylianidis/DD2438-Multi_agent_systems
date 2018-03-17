using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LeaderController : MonoBehaviour {

	private Vector3[] trajectory; // Trajectory coordinates
	private float[] trajectoryOrientation; // Orientation of virtual structure in each step of the trajectory
	private float[] trajectoryTimestamps; // Timestamp <t> at each step of the trajectory
	private float accumulatedDeltaTime = 0.0f;
	private float simulationSpeedFactor = 1.0f;
	private int routeIdx = 1;
	private float vehicle_dt;
	private Vector3 lastPos;

	// Use this for initialization
	void Start () {
		GameObject gameController = GameObject.Find ("GameController");
		if (gameController != null) {
			WorldController worldController = gameController.GetComponent<WorldController> ();
			vehicle_dt = worldController.world.vehicle.dt;
			if (worldController != null) {
				// Check if this agent has the longest path and should update the timer
				simulationSpeedFactor = worldController.simulationSpeedFactor;
			}
			GameObject formationController = GameObject.Find ("LeaderFormationController");
			LeaderFormationController formationScript = formationController.GetComponent<LeaderFormationController> ();
			if(gameController != null) {
				trajectory = formationScript.getTrajectory ();
				trajectoryOrientation = formationScript.getTrajectoryOrientation ();
				trajectoryTimestamps = formationScript.getTrajectoryTimestamps ();
				lastPos = trajectory [0];
				transform.LookAt (lastPos + UtilityClass.rads2Vec (trajectoryOrientation[0]));
			}
		}
	}
	
	// Update is called once per frame
	void Update () {

		if (routeIdx < trajectory.Length) {
			accumulatedDeltaTime += Time.deltaTime * simulationSpeedFactor;
			if (accumulatedDeltaTime >= vehicle_dt) {
				accumulatedDeltaTime = 0.0f;
				transform.position = trajectory [routeIdx];
				lastPos = trajectory [routeIdx];
				routeIdx++;
				if(routeIdx < trajectory.Length)
					transform.LookAt (lastPos + UtilityClass.rads2Vec (trajectoryOrientation[routeIdx]));
			} else 
				transform.position = lastPos + (trajectory [routeIdx] - lastPos) * Time.deltaTime / vehicle_dt;	
		}
	}
}
