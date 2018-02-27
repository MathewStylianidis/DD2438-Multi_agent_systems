using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentControllerVRP : MonoBehaviour {

	private List<PointInfo> route;
	private int routeIdx = 1;
	private PointInfo lastPos;
	private float vehicle_dt;
	private float accumulatedDeltaTime = 0.0f;

	// Use this for initialization
	void Start () {
		GameObject gameController = GameObject.Find ("GameController");
		if (gameController != null) {
			WorldController worldController = gameController.GetComponent<WorldController> ();
			vehicle_dt = worldController.world.vehicle.dt;
			if (worldController != null) {
				route = worldController.getRoute (this.transform.GetSiblingIndex());
				lastPos = route [0];
			}
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (route != null && routeIdx < route.Count) {
			accumulatedDeltaTime += Time.deltaTime;
			if(accumulatedDeltaTime >= vehicle_dt) {
				accumulatedDeltaTime = 0.0f;
				transform.position = route [routeIdx].pos;
				lastPos = route [routeIdx];
				routeIdx++;
				transform.LookAt (lastPos.pos + route [routeIdx].orientation);
			}
			else
				transform.position  = lastPos.pos + (route[routeIdx].pos - lastPos.pos) * Time.deltaTime;	
		}
	}
}
