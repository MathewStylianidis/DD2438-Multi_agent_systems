using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowerController : MonoBehaviour {

	private Vector3[] trajectory; // Trajectory coordinates
	private float[] trajectoryOrientation; // Orientation of virtual structure in each step of the trajectory
	private float[] trajectoryTimestamps; // Timestamp <t> at each step of the trajectory
	private World world;
	private float accumulatedDeltaTime = 0.0f;
	private float simulationSpeedFactor = 1.0f;
	private int routeIdx = 1;
	private float vehicle_dt;
	private float agentHeight;
	private BaseModel motionModel;
	private PointInfo lastPosInfo;
	private PointInfo nextPosInfo;
	private FormationController formationControl;
	private int agentIdx;


	// Use this for initialization
	void Start () {
		agentIdx = this.transform.GetSiblingIndex ();
		GameObject gameController = GameObject.Find ("GameController");
		if (gameController != null) {
			WorldController worldController = gameController.GetComponent<WorldController> ();
			if (worldController != null) {
				world = worldController.world;
				vehicle_dt = world.vehicle.dt;
				// Check if this agent has the longest path and should update the timer
				simulationSpeedFactor = worldController.simulationSpeedFactor;
				motionModel = worldController.getMotionModel ();
			}
			GameObject formationController = GameObject.Find ("FormationController");
			formationControl = formationController.GetComponent<FormationController> ();
			if(formationControl != null) {
				agentHeight = formationControl.agentHeight;
				trajectory = formationControl.getTrajectory ();
				trajectoryOrientation = formationControl.getTrajectoryOrientation ();
				trajectoryTimestamps = formationControl.getTrajectoryTimestamps ();
				Vector3 currentVelocity = new Vector3 (world.currentVelocities [agentIdx - 1].x, 0f, world.currentVelocities [agentIdx - 1].y);
				lastPosInfo = new PointInfo (this.transform.position, currentVelocity, currentVelocity.normalized, 0f);
				nextPosInfo = getNextPosition (lastPosInfo, world);

			}
		}
	}

	// Update is called once per frame
	void Update () {
		if (formationControl != null) {
			accumulatedDeltaTime += Time.deltaTime * simulationSpeedFactor;
			if (accumulatedDeltaTime >= vehicle_dt) {
				accumulatedDeltaTime = 0.0f;
				// Do completePath and get the first node only or do move forwards (maybe do it depending on how close you are)
				// Get current desired position
				if (agentIdx == 1) {
					
				}
				transform.position = nextPosInfo.pos;
				lastPosInfo = nextPosInfo;
				PointInfo tmp = getNextPosition (lastPosInfo, world);
				if (tmp != null) {
					world.currentVelocities [agentIdx - 1] = tmp.vel;
					nextPosInfo = tmp;
					if (agentIdx == 1) {
			
					}
				}
				if (routeIdx < trajectory.Length) {
					transform.LookAt (lastPosInfo.pos + nextPosInfo.orientation);
					world.currentVelocities [agentIdx - 1] = Vector3.zero;
				}
			} else {
				transform.position = lastPosInfo.pos + (nextPosInfo.pos - lastPosInfo.pos) * Time.deltaTime / vehicle_dt;	
			}
		}
	}


	private PointInfo getNextPosition(PointInfo lastPos, World world) {		
		Vector3 goalPoint = new Vector3(formationControl.getDesiredPosition (agentIdx).x, agentHeight, formationControl.getDesiredPosition (agentIdx).y);
		PointInfo goalPointInfo = new PointInfo (goalPoint, Vector3.right, formationControl.getLeaderOrientation(), lastPos.currentTime + vehicle_dt);
		List<PointInfo> path = motionModel.completePath (lastPos, goalPointInfo, world, false);
		if (path.Count > 0) {
			return path [0];
		}
		else 
			return null;
		
	}
}
