using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShooterController : MonoBehaviour {

	private float accumulatedTimePlan;
	private float accumelatedTimeDt;
    private float animationTimer;
	private int stepIndex;
	private int positionIndex;
    private GameObject[] agents;
    List<ShootingPlanner.OneStepPlan> gamePlan;
    ShootingPlanner.OneStepPlan currentStepPlan;

    private int comradeCount;
    private int enemyCount;

    private bool finished = false;




	// Use this for initialization
	void Start () {
		accumulatedTimePlan = accumelatedTimeDt = 0.0f;
		stepIndex = positionIndex = 0;
        animationTimer = 0.0f;
	}
		

	// Update is called once per frame
	void Update () {
        if(animationTimer > 0.0f)
        {
            animationTimer -= Time.deltaTime;
            return;
        }
        animationTimer = 0.0f;

        if (finished)
        {
            return;
        }

        float timePassed = Time.deltaTime * 3.0f;  
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
            
            accumelatedTimeDt = 0.0f;
            // Visualize positions
            for (int agentIndex = 0; agentIndex < comradeCount; agentIndex++)
            {
                if (!(currentStepPlan.shooterPlans[agentIndex].health > 0.0f))
                {
                    continue;
                }

                var agentPosition = currentStepPlan.shooterPlans[agentIndex].positions[positionIndex];
                agents[agentIndex].transform.position = new Vector3(agentPosition.x, 0.0f, agentPosition.y);

                if (positionIndex != 9)
                {
                    Vector2 orientation = currentStepPlan.shooterPlans[agentIndex].positions[positionIndex + 1]; // - currentStepPlan.shooterPlans[agentIndex].positions[positionIndex];
                    agents[agentIndex].transform.LookAt(new Vector3(orientation.x, 0.0f, orientation.y));
                }
            }

            for (int enemyIndex = 0; enemyIndex < enemyCount; enemyIndex++)
            {
                if (currentStepPlan.enemyTargetIndices[enemyIndex] != -1 && positionIndex != 9)
                {
                    Vector2 orientation = currentStepPlan.shooterPlans[currentStepPlan.enemyTargetIndices[enemyIndex]].positions[positionIndex];
                    agents[comradeCount + enemyIndex].transform.LookAt(new Vector3(orientation.x, agents[comradeCount + enemyIndex].transform.position.y, orientation.y));
                    
                }

                //Vector2 orientation = currentStepPlan.shooterPlans[agentIndex].positions[positionIndex + 1]; // - currentStepPlan.shooterPlans[agentIndex].positions[positionIndex];
            }

            positionIndex++;

            if (positionIndex == 10)
            {
                animationTimer = 1.0f;


                for (int agentIndex = 0; agentIndex < comradeCount; agentIndex++)
                {
                    if (!(currentStepPlan.shooterPlans[agentIndex].health > 0.0f))
                    {
                        continue;
                    }

                    if (currentStepPlan.shooterPlans[agentIndex].targetAgentIndex != -1)
                    {
                        agents[agentIndex].transform.LookAt(agents[enemyCount + currentStepPlan.shooterPlans[agentIndex].targetAgentIndex].transform.position);
                        agents[agentIndex].GetComponent<Animator>().Play("Shoot");
                    }

                    
                }
                
                // Turn our agents towards their targets

                // shoot


                positionIndex = 0;
                stepIndex++;
                if (stepIndex >= gamePlan.Count)
                {
                    // Show final frame

                    for (int enemyIndex = 0; enemyIndex < enemyCount; enemyIndex++)
                    {
                        if (currentStepPlan.enemyHealths[enemyIndex] > 0.0f)
                        {
                            agents[comradeCount + enemyIndex].GetComponent<Animator>().Play("Death");
                        }
                    }

                    finished = true;
                    return;
                }
                currentStepPlan = gamePlan[stepIndex];



                for (int enemyIndex = 0; enemyIndex < enemyCount; enemyIndex++)
                {
                    if (!(currentStepPlan.enemyHealths[enemyIndex] > 0.0f))
                    {
                        if (stepIndex == 1 || gamePlan[stepIndex - 1].enemyHealths[enemyIndex] > 0.0f)
                        {
                            // show dying animation

                            agents[comradeCount + enemyIndex].GetComponent<Animator>().Play("Death");

                        }
                    }
                }


            }

        }
		
	}


	public void initializeController(List<ShootingPlanner.OneStepPlan> gamePlan, GameObject[] agents) {
		this.gamePlan = gamePlan;
		currentStepPlan = gamePlan [0];
        this.agents = agents;

        comradeCount = currentStepPlan.shooterPlans.Length;
        enemyCount = currentStepPlan.enemyHealths.Length;

    }
}
