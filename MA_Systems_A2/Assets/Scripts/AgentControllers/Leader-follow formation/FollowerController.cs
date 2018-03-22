using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowerController : MonoBehaviour {

	private int leaderIndex = 0;
	private World world;
	private float accumulatedDeltaTime = 0.0f;
	private float simulationSpeedFactor = 1.0f;
	private int routeIdx = 1;
	private float vehicle_dt;
	private float agentHeight;
	private BaseModel motionModel;
	private PointInfo lastPosInfo;
	private PointInfo nextPosInfo;
	private LeaderFormationController formationControl;
	private int agentIdx;
	bool formationDecreasingGoalVelocity;

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
			GameObject formationController = GameObject.Find ("LeaderFormationController");
			formationControl = formationController.GetComponent<LeaderFormationController> ();
			if(formationControl != null) {
				agentHeight = formationControl.agentHeight;
				formationDecreasingGoalVelocity = formationControl.formationDecreasingGoalVelocity;
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
				transform.position = nextPosInfo.pos;
				lastPosInfo = nextPosInfo;
				PointInfo tmp = getNextPosition (lastPosInfo, world);
				if (tmp != null) {
					world.currentVelocities [agentIdx - 1] = tmp.vel;
					nextPosInfo = tmp;
				}
				transform.LookAt (lastPosInfo.pos + nextPosInfo.orientation);
				world.currentVelocities [agentIdx - 1] = lastPosInfo.vel;
				formationControl.setCurrentVelocity (0, world.currentVelocities [agentIdx - 1]);

			} else {
				transform.position = lastPosInfo.pos + (nextPosInfo.pos - lastPosInfo.pos) * Time.deltaTime / vehicle_dt;	
			}
		}
	}

	public void setLastPosInfo(PointInfo lastPosInfo) {this.lastPosInfo = lastPosInfo;}
	public BaseModel getMotionModel() { return motionModel;}
	public PointInfo getLastPosInfo() {return lastPosInfo;}
	public World getWorld() {return world;}

	private PointInfo getNextPosition(PointInfo lastPos, World world) {		
		Vector3 goalPoint = new Vector3(formationControl.getDesiredPosition (agentIdx).x, agentHeight, formationControl.getDesiredPosition (agentIdx).y);
		if (!formationDecreasingGoalVelocity) {
			PointInfo goalPointInfo = new PointInfo (goalPoint, formationControl.getAgentVelocity(leaderIndex), formationControl.getAgentOrientation(leaderIndex), lastPos.currentTime + vehicle_dt);
			List<PointInfo> path = motionModel.completePath (lastPos, goalPointInfo, world, false);
			if (path != null && path.Count > 0) {
				return path [0];
			} else {
				return null;
			}
		} else {
			PointInfo goalPointInfo = new PointInfo (goalPoint, Vector3.zero, formationControl.getAgentOrientation(leaderIndex), lastPos.currentTime + vehicle_dt);
			return motionModel.moveTowardsWithDecreasingVelocity (lastPos, goalPointInfo, world, false);
		}
		
	}
}
