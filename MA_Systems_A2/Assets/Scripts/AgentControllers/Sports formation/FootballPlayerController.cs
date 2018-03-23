using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FootballPlayerController : MonoBehaviour {

	private World world;
	private float accumulatedDeltaTime = 0.0f;
	private float simulationSpeedFactor = 1.0f;
	private int routeIdx = 1;
	private float vehicle_dt;
	private float agentHeight;
	private BaseModel motionModel;
	private PointInfo lastPosInfo;
	private PointInfo nextPosInfo;
	private VirtualStructure virtualStructure;
	private int agentIdx;
	private bool play = true;
	private float[] boundingMinMaxes;
	private bool formationDecreasingGoalVelocity;

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
			GameObject agentsObj = GameObject.Find ("Agents");
			virtualStructure = agentsObj.GetComponent<VirtualStructure> ();
			if(virtualStructure != null) {
				agentHeight = virtualStructure.agentHeight;
				formationDecreasingGoalVelocity = virtualStructure.formationDecreasingGoalVelocity;
				Vector3 currentVelocity = new Vector3 (world.currentVelocities [agentIdx - 1].x, 0f, world.currentVelocities [agentIdx - 1].y);
				lastPosInfo = new PointInfo (this.transform.position, currentVelocity, currentVelocity.normalized, 0f);
				nextPosInfo = getNextPosition (lastPosInfo, world, 1);
				boundingMinMaxes = VirtualStructure.getMinMaxes (world.boundingPolygon);
			}
		}
	}
	
	// LateUpdate  is called once per frame after update
	void LateUpdate  () {
		if (play && virtualStructure != null) {
			accumulatedDeltaTime += Time.deltaTime * simulationSpeedFactor;
			if (accumulatedDeltaTime >= vehicle_dt) {
				accumulatedDeltaTime = 0.0f;
				transform.position = nextPosInfo.pos;
				lastPosInfo = nextPosInfo;

				PointInfo tmp = getNextPosition (lastPosInfo, world, 1.0f);

				if (tmp != null) {
					world.currentVelocities [agentIdx - 1] = tmp.vel;
					nextPosInfo = tmp;
				}
				transform.LookAt (lastPosInfo.pos + nextPosInfo.orientation);
				world.currentVelocities [agentIdx - 1] = lastPosInfo.vel;
				virtualStructure.setCurrentVelocity (agentIdx, world.currentVelocities [agentIdx - 1]);

			} else {
				transform.position = lastPosInfo.pos + (nextPosInfo.pos - lastPosInfo.pos) * Time.deltaTime / vehicle_dt;	
			}
		}
		play = true;
		//set next point info to correct value in virtual formation shit
	}


	public void setLastPosInfo(PointInfo lastPosInfo) {this.lastPosInfo = lastPosInfo;}
	public void setNextPosInfo(PointInfo nextPosInfo) {this.nextPosInfo = nextPosInfo;}
	public BaseModel getMotionModel() { return motionModel;}
	public PointInfo getLastPosInfo() {return lastPosInfo;}
	public World getWorld() {return world;}
	public void setPlay(bool play) {this.play = play;}

	private PointInfo getNextPosition(PointInfo lastPos, World world, float constant = 1) {		
		// Get next desired position (agentIdx - 1 is used because the opponent player is part of the framework but is not included in the formation)
		Vector3 goalPoint = new Vector3(virtualStructure.getDesiredPosition (agentIdx - 1).x, agentHeight, virtualStructure.getDesiredPosition (agentIdx - 1).y);
		if (!formationDecreasingGoalVelocity) {
			PointInfo goalPointInfo = new PointInfo (goalPoint, virtualStructure.getAgentVelocity(virtualStructure.getWinnerIdx()), virtualStructure.getWinnerOrientation(), lastPos.currentTime + vehicle_dt);
			List<PointInfo> path = motionModel.completePath (lastPos, goalPointInfo, world, false);
			if (path != null && path.Count > 0) {
				return path [0];
			} else {
				return null;
			}
		} else {
			PointInfo goalPointInfo = new PointInfo (goalPoint, Vector3.zero, virtualStructure.getWinnerOrientation(), lastPos.currentTime + vehicle_dt);
			return motionModel.moveTowardsWithDecreasingVelocity (lastPos, goalPointInfo, world, false, constant);
		}
	}
}
