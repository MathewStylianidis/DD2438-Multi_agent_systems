using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualStructure : BaseFormationController {

	class VirtualStructureRectangle {
		public PointInfo[] vertices;
		public Vector3 center;

		public VirtualStructureRectangle(VirtualStructureRectangle structure) {
			center = new Vector3(structure.center.x, structure.center.y, structure.center.z);
			vertices = new PointInfo[structure.vertices.Length];
			for(int i = 0; i < vertices.Length; i++)
				vertices[i] = new PointInfo(structure.vertices[i].pos, structure.vertices[i].vel, structure.vertices[i].orientation, structure.vertices[i].currentTime);
		}

		public VirtualStructureRectangle(Vector2[] formationPositions, Vector2[] boundingPoly, float deltaX, float deltaY, float agentHeight) {
			Vector2 center2D = getCenter(formationPositions);
			float[] formationMinMaxes = getMinMaxes(formationPositions);
			float[] fieldMinMaxes = getMinMaxes(boundingPoly);
			float fieldSideX = Mathf.Abs(fieldMinMaxes[2] - fieldMinMaxes[0]);
			float fieldSideY = Mathf.Abs(fieldMinMaxes[3] - fieldMinMaxes[1]);
			float formationSideX = Mathf.Abs(formationMinMaxes[2] - formationMinMaxes[0]);
			float formationSideY = Mathf.Abs(formationMinMaxes[3] - formationMinMaxes[1]);
			if(fieldSideX < formationSideX || fieldSideY < formationSideY)
				throw new System.Exception("The formation size is too large to fit in the field");
			float newFormationSideX = formationSideX + deltaX;
			float newFormationSideY = formationSideY + deltaY;
			if(fieldSideX < newFormationSideX || fieldSideY < newFormationSideY)
				throw new System.Exception("The virtual formation rectangle is too large to fit in the field");
			vertices = new PointInfo[4];
			// Upper left 
			vertices[0] = new PointInfo(new Vector3(center2D.x - newFormationSideX/2, agentHeight, center2D.y + newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			// Upper right 
			vertices[1] = new PointInfo(new Vector3(center2D.x + newFormationSideX/2, agentHeight, center2D.y + newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			// Bottom right 
			vertices[2] = new PointInfo(new Vector3(center2D.x + newFormationSideX/2, agentHeight, center2D.y - newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			// Bottom left
			vertices[3] = new PointInfo(new Vector3(center2D.x - newFormationSideX/2, agentHeight, center2D.y - newFormationSideY/2), Vector3.zero, Vector3.forward, 0.0f);
			center = new Vector3(center2D.x, agentHeight, center2D.y);
			//Visualizer.visualizePoints(getEdgeVectors());
		}


		public PointInfo[] getEdges() {return this.vertices;}

		public Vector2[] getEdgeVectors() {
			Vector2[] vectors = new Vector2[vertices.Length];
			for (int i = 0; i < vertices.Length; i++)
				vectors [i] = new Vector2(vertices [i].pos.x, vertices[i].pos.z);
			return vectors;
		}

		public void updateRectangle(Vector3 newCenter) {
			for (int i = 0; i < vertices.Length; i++)
				vertices [i].pos += (newCenter - center);
			center = newCenter;
		}

		// Checks if all edges of the rectangle are in the bounding polygon
		public bool isInPolygon(Vector2[] boundingPolygon) {
			for (int i = 0; i < vertices.Length; i++)
				if (!Raycasting.insidePolygon (vertices [i].pos.x, vertices [i].pos.z, boundingPolygon))
					return false;
			return true;
		}
	}
			
	VirtualStructureRectangle formationRectangle;
	int winnerIdx;
	private float accumulatedDeltaTime = 0.0f;
	private float simulationSpeedFactor = 1.0f;
	private float vehicleDt;

	void Update () {
		accumulatedDeltaTime += Time.deltaTime * simulationSpeedFactor;
		if (accumulatedDeltaTime >= vehicleDt) {
			accumulatedDeltaTime = 0.0f;
			// Get positions of virtual structure edges
			PointInfo[] edges = formationRectangle.getEdges ();

			Vector3 targetPosition = agents [0].transform.position;
			List<int> failedAgents = new List<int> ();
			if (winnerIdx == agents.Length - 1 || !moveNearestAgent (targetPosition, winnerIdx)) {
				failedAgents.Add (winnerIdx);
				for (int t = 1; t < agents.Length - 1; t++) {
					// Get closest player to opponent
					float minDistance = float.MaxValue;

					for (int i = 1; i < agents.Length - 1; i++) {
						// If this agent has failed moving before, due to moving the formation out of the polygon
						if (failedAgents.Contains (i))
							continue;
						float distance = Vector2.Distance (new Vector2 (targetPosition.x, targetPosition.z), getDesiredPosition (i - 1));
						if (distance < minDistance) {
							minDistance = distance;
							winnerIdx = i;
						}
					}

					// If movement of agent succeeds without collision of the formation with the boundary polygon
					if (moveNearestAgent (targetPosition, winnerIdx)) {
						break;
					}
					failedAgents.Add (winnerIdx);
					winnerIdx = agents.Length - 1;
				}
			}
			//If noone can move
			if (winnerIdx == agents.Length - 1) {
				setRelativeFormationPositions(this.relativeFormationPositions, winnerIdx - 1);
				this.desiredRelativePositions = getDesiredPositions (winnerIdx, false, false);
				this.desiredAbsolutePositions = getDesiredPositions (winnerIdx, false, true);
				FootballPlayerController virtualCenterController = agents[agents.Length - 1].GetComponent<FootballPlayerController>();
				virtualCenterController.setPlay (false);
			}
		}
	}

	public void initializeController(GameObject[] agents, Vector2[] boundingPoly, World.TrajectoryMap trajectory, Vector2[] formationPositions, float agentHeight,
										float deltaX, float deltaY, float simulationFactor, float vehicleDt, bool formationDecreasingGoalVelocity) {
		this.simulationSpeedFactor = simulationFactor;
		this.vehicleDt = vehicleDt;
		this.agentHeight = agentHeight;
		this.agents = agents;
		this.formationDecreasingGoalVelocity = formationDecreasingGoalVelocity;
		// Get trajectory coordinates
		this.trajectory = new Vector3[trajectory.x.Length];
		for (int i = 0; i < trajectory.x.Length; i++) 
			this.trajectory [i] = new Vector3 (trajectory.x [i], agentHeight, trajectory.y [i]);
		// Get trajectory orientations
		this.trajectoryOrientation = new float[trajectory.theta.Length];
		for (int i = 0; i < trajectory.theta.Length; i++) 
			this.trajectoryOrientation [i] = trajectory.theta [i];
		// Get timestamps for each step of the trajectory
		this.trajectoryTimestamps = new float[trajectory.t.Length];
		for (int i = 0; i < trajectory.t.Length; i++)
			this.trajectoryTimestamps [i] = trajectory.t [i];
		// set formationPositions in parent class to relative positions from the virtual center
		setRelativeFormationPositions(formationPositions, formationPositions.Length - 1);	
		// Get starting absolute positions and relative to the leader positions.
		this.desiredRelativePositions = getDesiredPositions (agents.Length - 1, false);
		this.desiredAbsolutePositions = getDesiredPositions (agents.Length - 1);
		// Visualize starting desired positions
		//Visualizer.visualizePoints(this.desiredRelativePositions);
		//Visualizer.visualizePoints(this.desiredAbsolutePositions);
		// Set a controller within each agent
		agents [0].AddComponent<LeaderController> ();
		for (int i = 1; i < agents.Length; i++)
			agents [i].AddComponent<FootballPlayerController> ();
		formationRectangle = new  VirtualStructureRectangle (formationPositions, boundingPoly, deltaX, deltaY, agentHeight);
		// Get nearest agent to opponent player
		winnerIdx = agents.Length - 1;
		for (int t = 1; t < agents.Length - 1; t++) {
			// Get closest player to opponent
			float minDistance = float.MaxValue;
			for (int i = 1; i < agents.Length - 1; i++) {
				// If this agent has failed moving before, due to moving the formation out of the polygon
				float distance = Vector2.Distance (new Vector2 (agents[0].transform.position.x, agents[0].transform.position.z), getDesiredPosition (i - 1));
				if (distance < minDistance) {
					minDistance = distance;
					winnerIdx = i;
				}
			}
		}
	}

	public Vector3 getWinnerOrientation() {
		return getAgentOrientation (winnerIdx);
	}

	public int getWinnerIdx() {
		return winnerIdx;
	}

	public static Vector2 getCenter(Vector2[] formationPositions) {
		float[] minMaxes = getMinMaxes (formationPositions);
		return new Vector2 ((minMaxes[0] +  minMaxes[2]) / 2, (minMaxes[1] + minMaxes[3]) / 2);
	}

	/// <summary>
	/// Returns an float array with the minimum x and y as well as the maximum x and y
	/// in the formationPositions, in this order.
	/// </summary>
	public static float[] getMinMaxes(Vector2[] formationPositions) {
		// Return null if there are less than 2 agents in the structure
		if (formationPositions.Length < 2)
			throw new System.Exception("agentsPos parameter should have more than 1 element.");
		float[] minMaxes = {float.MaxValue, float.MaxValue, float.MinValue, float.MinValue};
		for (int i = 0; i < formationPositions.Length; i++) {
			if (formationPositions [i].x < minMaxes[0])
				minMaxes[0] = formationPositions [i].x;
			else if (formationPositions [i].x > minMaxes[2])
				minMaxes[2] = formationPositions [i].x;
			if (formationPositions [i].y < minMaxes[1])
				minMaxes[1] = formationPositions [i].y;
			else if (formationPositions [i].y > minMaxes[3])
				minMaxes[3] = formationPositions [i].y;
		}
		return minMaxes;
	}



	private bool moveNearestAgent(Vector3 target, int nearestAgentIdx) {
		// Get next position information of the nearestAgentIdx
		FootballPlayerController nearestAgentController = agents[nearestAgentIdx].GetComponent<FootballPlayerController>();
		PointInfo lastPos = nearestAgentController.getLastPosInfo ();
		BaseModel model = nearestAgentController.getMotionModel ();
		PointInfo goalPointInfo = new PointInfo (target, Vector3.zero, Vector3.forward, lastPos.currentTime + vehicleDt);
		PointInfo nextPoint = model.moveTowardsWithDecreasingVelocity (lastPos, goalPointInfo, nearestAgentController.getWorld(), false);
		// Get previous position of agent
		Vector3 prevPos = new Vector3(agents[nearestAgentIdx].transform.position.x , agents[nearestAgentIdx].transform.position.y, agents[nearestAgentIdx].transform.position.z);
		agents[nearestAgentIdx].transform.position = nextPoint.pos;
		Vector2[] tmpDesiredRelativePositions = getDesiredPositions (winnerIdx, false, false);
		// Store previous positional information
		Vector2[] prevRelativeFormationPositions = new Vector2[this.relativeFormationPositions.Length];
		for (int i = 0; i < prevRelativeFormationPositions.Length; i++)
			prevRelativeFormationPositions [i] = new Vector2 (this.relativeFormationPositions[i].x, this.relativeFormationPositions[i].y);
		Vector2[] prevDesiredRelative = getDesiredPositions (winnerIdx, false, false);
		Vector2[] prevDesiredAbsolute = getDesiredPositions (winnerIdx, false, true);
		// Calculate new positional information
		setRelativeFormationPositions(this.relativeFormationPositions, nearestAgentIdx - 1);
		this.desiredRelativePositions = getDesiredPositions (winnerIdx, false, false);
		this.desiredAbsolutePositions = getDesiredPositions (winnerIdx, false, true);

		// Get virtual center controller and its last position
		FootballPlayerController virtualCenterController = agents[agents.Length - 1].GetComponent<FootballPlayerController>();
		PointInfo lastCenterPos = virtualCenterController.getLastPosInfo ();
		// Move center in the same way the nearest agent moves, anchoring the formation to the agent
		Vector2 desiredCenterPosition = getDesiredPosition(agents.Length - 2);
		Vector3 desiredCenter3D = new Vector3 (desiredCenterPosition.x, agentHeight, desiredCenterPosition.y);
		// Get copy of formation rectangle
		VirtualStructureRectangle tmp = new VirtualStructureRectangle(formationRectangle);
		// Update rectangle with new desired center
		tmp.updateRectangle (desiredCenter3D);

		// Check if new rectangle gets out of the bounding polygon in case the agent is not already near the boundary
		if (lastPos.currentTime > 30.0f && !tmp.isInPolygon (nearestAgentController.getWorld ().boundingPolygon)) {
			// If it is not entirely inside, restore changes and return false
			agents[nearestAgentIdx].transform.position = prevPos;
			this.relativeFormationPositions = prevRelativeFormationPositions;
			this.desiredAbsolutePositions = prevDesiredAbsolute;
			this.desiredRelativePositions = prevDesiredRelative;
			return false;
		}


		// Otherwise make all necessary changes to variables of the closest agent and formation center
		agents[nearestAgentIdx].transform.position = prevPos;
		agents [nearestAgentIdx].transform.LookAt(agents[nearestAgentIdx].transform.position + (target - agents[nearestAgentIdx].transform.position).normalized);
		PointInfo nextPos = new PointInfo (nextPoint.pos, Vector3.zero, nextPoint.orientation, nextPoint.currentTime + vehicleDt);
		nearestAgentController.setNextPosInfo (nextPoint);

		agents [agents.Length - 1].transform.position = desiredCenter3D;
		lastCenterPos.pos = desiredCenter3D;
		lastCenterPos.vel = nextPoint.vel;
		lastCenterPos.orientation = nextPoint.orientation;
		lastCenterPos.currentTime = nextPoint.currentTime;
		formationRectangle = tmp;
		agents [agents.Length - 1].transform.LookAt(agents[agents.Length - 1].transform.position + (target - agents[agents.Length - 1].transform.position).normalized);
		virtualCenterController.setLastPosInfo (lastCenterPos);
		nextPos = new PointInfo (lastCenterPos.pos, Vector3.zero, lastCenterPos.orientation, lastCenterPos.currentTime +vehicleDt);
		virtualCenterController.setNextPosInfo (nextPos);
		virtualCenterController.setPlay (false);
		return true;
	}


}
