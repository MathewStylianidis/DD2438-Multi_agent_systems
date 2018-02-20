using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Non holonomic avoidance collision implementation based on:
 *  Alonso-Mora, J., Breitenmoser, A., Rufli, M., Beardsley, P. and Siegwart, R., 2013.
 * Optimal reciprocal collision avoidance for multiple non-holonomic robots. In Distributed Autonomous
 * Robotic Systems (pp. 203-216). Springer, Berlin, Heidelberg.	
 */
public class AgentControllerP21 : MonoBehaviour {

	public World world;
	public static float epsilon = 1e-20f;
	public float wheelDist = epsilon; // Set to epsilon by default to discard car mechanics
	public float omegaMax = 0.5f; 
	private float maxTrackingError;
	private float lowerBounds; // Must have a value at least equal to controller's dt. Variable name in paper: T
	private float prefVelocity;
	private float agentRadius;
	private float orientation;
	private int agentIdx;
	private static int idxCounter = 0;



	public AgentControllerP21(World world, float agentRadius, float lowerBounds) {
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
				this.agentIdx = idxCounter++; //current positions and velocities of each agent can be accessed with their index
				this.orientation = getOrientation(agentIdx); // get current orientation based on current velocity
				 
			}
		}
	}

	// Update is called once per frame
	void Update () {
		
	}

	// Calculate set of allowed holonomic velocities S_ahv (Polygonal approximation)
	List<Vector2> calcSahv(float angle, int agentIdx) {
		float maxHolSpeed = getMaxHolSpeed(angle, agentIdx); //get maximum holonomic speed

		return null;
	}

	// Gets orientation theta given in rads, based on the direction of velocity
	float getOrientation(int agentIdx) {
		Vector2 velNorm = world.currentVelocities [agentIdx].normalized;
		float angle = Vector2.Angle (Vector2.right , velNorm);
		// if angle is in the third or fourth quadrant change sign
		if (velNorm.y < 0) 
			angle = -angle;
		return angle * Mathf.Deg2Rad;
	}


	// Equation 8 in paper (not used for the moment)
	float getNonHolOptVel(float angle, float curHolonomicVel) {
		return curHolonomicVel * angle * Mathf.Sin(angle) / (2 * (1 - Mathf.Cos(angle)));
	}

	// Equation 13 in paper
	float getMaxHolSpeed(float angle, int agentIdx) {
		float requiredOmega = Mathf.Abs(angle) / this.lowerBounds; //required omega to turn <angle> rads in <T> (lowerBounds) time
		float vStarEps = getVStarEps(angle);
		float vMaxOmega = world.vehicle.maxVelocity - world.currentAngularVel[agentIdx] * wheelDist / 2;

		if (requiredOmega >= omegaMax) {
			return Mathf.Min (this.maxTrackingError * omegaMax / angle,
				this.world.vehicle.maxVelocity);
		} else if (vStarEps >= vMaxOmega) {
			float alpha = this.lowerBounds * this.lowerBounds;
			float beta = -2 * alpha * Mathf.Sin (angle) * vMaxOmega / angle;
			float gamma = 2 * alpha * (1 - Mathf.Cos(angle)) * vMaxOmega * vMaxOmega / (angle * angle)
				- this.maxTrackingError * this.maxTrackingError;
			return Mathf.Min((-beta + Mathf.Sqrt(beta * beta - 4 * alpha * gamma)) / (2 * gamma),
				this.world.vehicle.maxVelocity);
		} else /*if (nonHolOptVel <= vMaxOmega)*/ {
			float tmp = angle * Mathf.Sin(angle) / (2 * (1 - Mathf.Cos(angle))); // 2nd term in formula in Eq. 14
			return Mathf.Min (vStarEps / tmp, this.world.vehicle.maxVelocity);
		}
	}

	// Equation 14 in paper
	float getVStarEps(float angle) {
		float tmp1 = this.maxTrackingError / this.lowerBounds; // 1st in formula
		float tmp2 = angle * Mathf.Sin(angle) / (2 * (1 - Mathf.Cos(angle))); // 2nd term in formula
		float numerator = 2 * (1 - Mathf.Cos(angle));
		float denominator = numerator - Mathf.Sin(angle) * Mathf.Sin(angle);
		float tmp3 = Mathf.Sqrt (numerator * denominator); //3rd term in formula
		return tmp1 * tmp2 * tmp3;
	}


}
