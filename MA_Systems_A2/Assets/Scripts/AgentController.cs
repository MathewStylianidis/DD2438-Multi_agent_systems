using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentController : MonoBehaviour {

	public World world;
	private float maxTrackingError;
	private float lowerBounds; // Must have a value at least equal to controller's dt.
	private float prefVelocity;
	private float agentRadius;
	private int agentIdx;
	private static int idxCounter = 0;


	public AgentController(World world, float agentRadius, float lowerBounds) {
		this.world = world;
		this.agentRadius = agentRadius;
		this.maxTrackingError = agentRadius; //maxTrackingError equal to agent radius guaranteed collision free trajectories
		this.lowerBounds = lowerBounds;
		this.prefVelocity = world.vehicle.maxVelocity;
	}

	// Use this for initialization
	void Start () 
	{
		GameObject gameController = GameObject.Find ("GameController");
		if (gameController != null) {
			WorldController worldController = gameController.GetComponent<WorldController> ();
			if (worldController != null) {
				this.world = worldController.world;
				this.agentRadius = worldController.agentRadius;
				this.maxTrackingError = worldController.agentRadius; //maxTrackingError equal to agent radius guaranteed collision free trajectories
				this.lowerBounds = worldController.world.vehicle.dt;
				this.prefVelocity = worldController.world.vehicle.maxVelocity;		
				this.agentIdx = idxCounter++;
				//get current velocity of each agent
			}
		}
	}

	// Update is called once per frame
	void Update () {
		
	}
}
