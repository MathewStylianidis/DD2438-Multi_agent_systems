using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class AgentControllerVRP : MonoBehaviour {


	private List<PointInfo> route;
	private int routeIdx = 1;
	private PointInfo lastPos;
	private float vehicle_dt;
	private float accumulatedDeltaTime = 0.0f;
	private bool keepTime = false;
	private Text timeText;
	private float simulationSpeedFactor = 1.0f;
	private Vector2[] pointsOfInterest;

	// Use this for initialization
	void Start () {

		// Get car model from world controller
		GameObject gameController = GameObject.Find ("GameController");
		if (gameController != null) {
			WorldController worldController = gameController.GetComponent<WorldController> ();
			if (worldController != null) {
				vehicle_dt = worldController.world.vehicle.dt;
				pointsOfInterest = worldController.world.pointsOfInterest;
				route = worldController.getRoute (this.transform.GetSiblingIndex());
				lastPos = route [0];
				// Check if this agent has the longest path and should update the timer
				if (worldController.keepTime (this.transform.GetSiblingIndex ())) {
					keepTime = true;
					timeText = worldController.timeText;
				}
				simulationSpeedFactor = worldController.simulationSpeedFactor;
			}
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (route != null && routeIdx < route.Count) {
			accumulatedDeltaTime += Time.deltaTime * simulationSpeedFactor;
			if (accumulatedDeltaTime >= vehicle_dt) {
				accumulatedDeltaTime = 0.0f;
				transform.position = route [routeIdx].pos;
				lastPos = route [routeIdx];
				routeIdx++;
				if(routeIdx < route.Count)
					transform.LookAt (lastPos.pos + route [routeIdx].orientation);
			} else 
				transform.position = lastPos.pos + (route [routeIdx].pos - lastPos.pos) * Time.deltaTime / vehicle_dt;	

			if(keepTime)
				timeText.text = "Time: " + System.Math.Round(lastPos.currentTime + accumulatedDeltaTime, 4);
		}
	}
		
}
