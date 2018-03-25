using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShooterController : MonoBehaviour {

	private float accumulatedTimePlan;
	private float accumelatedTimeDt;
	private int stepIndex;
	private int positionIndex;
    private GameObject[] agents;
    List<ShootingPlanner.OneStepPlan> gamePlan;
    ShootingPlanner.OneStepPlan currentStepPlan;

    private bool finished = false;


	// Use this for initialization
	void Start () {
		accumulatedTimePlan = accumelatedTimeDt = 0.0f;
		stepIndex = positionIndex = 0;
	}
		

	// Update is called once per frame
	void Update () {
        
        if (finished)
        {
            return;
        }

        float timePassed = Time.deltaTime;  
        accumulatedTimePlan += timePassed;
		accumelatedTimeDt += timePassed;

        /*

		if (accumulatedTimePlan >= 1.0f) {
            Debug.Log("Step index: " + stepIndex);

			accumulatedTimePlan = 0.0f;
			accumelatedTimeDt = 0.0f;
			positionIndex = 0;



			currentStepPlan = gamePlan [stepIndex];
            
            stepIndex++;

            // Visualize shooting
            // Visualize positions
        }

        */

		if (accumelatedTimeDt >= 0.1f) {

            Debug.Log("Position index " + positionIndex);

            accumelatedTimeDt = 0.0f;
            // Visualize positions
            for (int agentIndex = 0; agentIndex < currentStepPlan.shooterPlans.Length; agentIndex++)
            {
                var agentPosition = currentStepPlan.shooterPlans[agentIndex].positions[positionIndex];
                agents[agentIndex].transform.position = new Vector3(agentPosition.x, agents[agentIndex].transform.position.y, agentPosition.y);
            }

            positionIndex++;

            if (positionIndex == 10)
            {
                positionIndex = 0;
                stepIndex++;
                //Debug.Log("Step index " + stepIndex);
                Debug.Log("Starting step pos: " + currentStepPlan.shooterPlans[1].positions[0]);

                if (stepIndex >= gamePlan.Count)
                {
                    finished = true;
                    return;
                }
                currentStepPlan = gamePlan[stepIndex];
            }

        }
		
	}


	public void initializeController(List<ShootingPlanner.OneStepPlan> gamePlan, GameObject[] agents) {
		this.gamePlan = gamePlan;
		currentStepPlan = gamePlan [0];
        this.agents = agents;
	}
}
