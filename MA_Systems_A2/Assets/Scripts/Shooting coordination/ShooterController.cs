using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShooterController : MonoBehaviour {

	private float accumulatedTimePlan;
	private float accumelatedTimeDt;
	private int stepIndex;
	private int positionIndex;
	List<ShootingPlanner.ShooterOneStepPlan[]> gamePlan;
	ShootingPlanner.ShooterOneStepPlan[] currentStepPlan;


	// Use this for initialization
	void Start () {
		accumulatedTimePlan = accumelatedTimeDt = 0.0f;
		stepIndex = positionIndex = 0;
	}
		

	// Update is called once per frame
	void Update () {
		accumulatedTimePlan += Time.deltaTime;
		accumelatedTimeDt += Time.deltaTime;
		if (accumulatedTimePlan >= 1.0f) {
			accumulatedTimePlan = 0.0f;
			accumelatedTimeDt = 0.0f;
			stepIndex++;
			positionIndex = 0;

			currentStepPlan = gamePlan [stepIndex];

			// Visualize shooting
			// Visualize positions
		}

		if (accumelatedTimeDt >= 0.1f) {
			accumelatedTimeDt = 0.0f;
			positionIndex++;
			Vector2 agent0Position = currentStepPlan [0].positions [positionIndex];
			// Visualize positions
		}
		
	}


	public void initializeController(List<ShootingPlanner.ShooterOneStepPlan[]> gamePlan) {
		this.gamePlan = gamePlan;
		currentStepPlan = gamePlan [0];
	}
}
