using System.Collections.Generic;
using UnityEngine;

public class ShootingPlanner {

	private World world;
	private float maxHealth = 10.0f;
	private int comradeCount; // Number of agents in team 1
	private int enemyCount; // Number of agents in team 2
	private WeaponTypeEnum weaponType;
	private FloydWarshall shortestPathsFinder;
	private float[,] shortestDists;

	public ShootingPlanner(World world, WeaponTypeEnum weaponType) {
		this.world = world;
		comradeCount = world.startPositions.Length;
		enemyCount = world.enemyPositions.Length;
		this.weaponType = weaponType;
	}

	public List<ShooterOneStepPlan[]> getPlan() {

		if (shortestPathsFinder == null) {
			shortestPathsFinder = new FloydWarshall (world.visibilityGraph);
			shortestDists = shortestPathsFinder.findShortestPaths ();
		}

		// Get initial greedy solution
		List<ShooterOneStepPlan[]> initialSolution;
		switch (weaponType) {
		case WeaponTypeEnum.Shotgun:

			initialSolution = getShotgunInitialSolution ();
			return initialSolution;
		
		case WeaponTypeEnum.Rifle:
			break;
		}

		return null;
	}

	private List<ShooterOneStepPlan[]> getShotgunInitialSolution() {

		int enemyStartIndex = world.graphVertices.Count - enemyCount;
		int comradeStartIndex = enemyStartIndex - comradeCount;

		// Find "meeting" point
		int meetingVertexIndex = 0;
		for (int i = 0; i < world.graphVertices.Count; i++) {

			float[] health = new float[comradeCount + enemyCount];
			for (int j = 0; j < health.Length; j++)
				health[j] = maxHealth;

			World.VisibilityVertex[] startVertices = new World.VisibilityVertex[comradeCount];
			List<int>[] paths = new List<int>[comradeCount];
			for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++) {
				paths [comradeIndex] = shortestPathsFinder.reconstructShortestPath (comradeStartIndex + comradeIndex, i);
				startVertices [comradeIndex] = world.graphVertices [comradeStartIndex + comradeIndex];
			}
			State currState = new State (startVertices, health);

			bool allReachedFinish = false;
			int[] prevPivots = new int[comradeCount];
			while (!allReachedFinish) {

				allReachedFinish = true;
				for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++) {

					if (prevPivots [comradeIndex] == paths [comradeIndex].Count - 1 || !(currState.health[comradeIndex] > 0.0f)) {
						continue;
					} else {
						allReachedFinish = false;
					}

					float maxTravelDist = world.vehicle.maxVelocity;
                    Vector2 nextVertex = world.graphVertices[paths[comradeIndex][prevPivots[comradeIndex] + 1]].vertex;
                    Vector2 currVertex = world.graphVertices[paths[comradeIndex][prevPivots[comradeIndex]]].vertex;
                    float prevStepRemDist = (currState.positions[comradeIndex].vertex - world.graphVertices[paths[comradeIndex][prevPivots[comradeIndex]]].vertex).magnitude;
                    float remainingDist = (nextVertex - currVertex).magnitude - prevStepRemDist;
                    bool reachedFinish = false;
                    while (maxTravelDist > remainingDist) {

                        prevPivots[comradeIndex]++;
                        if (prevPivots[comradeIndex] == paths[comradeIndex].Count - 1) {
                            reachedFinish = true;
                            break;
                        }

                        maxTravelDist -= remainingDist;

                        nextVertex = world.graphVertices [paths [comradeIndex] [prevPivots [comradeIndex] + 1]].vertex;
						currVertex = world.graphVertices [paths [comradeIndex] [prevPivots [comradeIndex]]].vertex;
						remainingDist = (nextVertex - currVertex).magnitude;
					}

					var prevPosition = world.graphVertices [paths [comradeIndex] [prevPivots [comradeIndex]]].vertex;
					if (reachedFinish) {
						currState.positions [comradeIndex] = world.graphVertices [i];
						continue;
					}

					float currTravelLineLength = world.visibilityGraph [paths [comradeIndex] [prevPivots [comradeIndex]]] [paths [comradeIndex] [prevPivots [comradeIndex] + 1]];
					currState.positions [comradeIndex].vertex = (world.graphVertices [paths [comradeIndex] [prevPivots [comradeIndex] + 1]].vertex - prevPosition) *
						(maxTravelDist / currTravelLineLength) + prevPosition;



				}

				shoot (currState, 2.0f, 1.0f/5, 1.0f, 1.0f/20);
			}


		}



		// Find the best enemies traversing order

		return null;
	}

	/*
	private PointInfo simulateMove (PointInfo curPointInfo, Vector3 goalPoint)
	{
		Vector3 path = goalPoint - curPointInfo.pos;
		float dist = path.magnitude; // distance from current point to goal
		float max_dt_dist = maxVelocity * dt; // distance that can be travelled in dt 
		float part_dist = max_dt_dist / dist;
		float time = dt;
		// If we can travel more in dt than the distance left to the goal, then adjust part_dist and time needed to travel
		if (part_dist >= 1.0) {
			part_dist = 1f;
			time /= part_dist;
		}
		Vector3 newPath = path * part_dist; // Get the proportion of the path to the goal to be travelled
		float xVel = (float)System.Math.Round((System.Double)newPath.x/dt, 2, System.MidpointRounding.AwayFromZero);
		float zVel = (float)System.Math.Round((System.Double)newPath.z/dt, 2, System.MidpointRounding.AwayFromZero);
		return new PointInfo (curPointInfo.pos + newPath, new Vector3(xVel, 0, zVel), Vector3.Normalize(path), curPointInfo.currentTime + time	);
	}
	*/

	private void shoot(State state, float d0_comrade, float k_comrade, float d0_enemy, float k_enemy) {

		int enemyStartIndex = world.graphVertices.Count - enemyCount;
		int comradeStartIndex = enemyStartIndex - comradeCount;

		for (int i = 0; i < state.health.Length; i++) {

            if (!(state.health[i] > 0.0f)) {
                continue;
            }

			float minDistance = float.MaxValue;
			int minIndex = -1;
			if (i < comradeCount) {
				// The comrade shoots
				for (int j = 0; j < enemyCount; j++) {

                    if (!(state.health[comradeCount + j] > 0.0f)) {
                        continue;
                    }

					// find closest if not wall
					float currDistance = (state.positions[i].vertex - world.graphVertices[j + enemyStartIndex].vertex).magnitude;
					if (currDistance < minDistance && canSee(state.positions[i], world.graphVertices[j + enemyStartIndex])) {
						minDistance = currDistance;
						minIndex = j;
					}
				}

				if (minIndex != -1) {
					state.health [comradeCount + minIndex] -= damage (minDistance, d0_comrade, k_comrade);
					state.health [comradeCount + minIndex] = System.Math.Max (state.health [comradeCount + minIndex], 0.0f);
				}
				continue;
			}

			// The enemy shoots
			int currEnemyIndex = i - comradeCount;
			for (int j = 0; j < comradeCount; j++) {

                if (!(state.health[j] > 0.0f)) {
                    continue;
                }
                // find closest if not wall
                float currDistance = (state.positions[j].vertex - world.graphVertices[enemyStartIndex+currEnemyIndex].vertex).magnitude;
				if (currDistance < minDistance && canSee(state.positions[j], world.graphVertices[enemyStartIndex+currEnemyIndex])) {
					minDistance = currDistance;
					minIndex = j;
				}
			}

			if (minIndex != -1) {
				state.health [minIndex] -= damage (minDistance, d0_enemy, k_enemy);
				state.health [minIndex] = System.Math.Max (state.health [minIndex], 0.0f);
			}

		}
	}

	private bool canSee(World.VisibilityVertex v1, World.VisibilityVertex v2) {

		var visible = true;
		if (v1.obstacleVertex && v2.obstacleVertex && v1.obstacleIndex == v2.obstacleIndex &&
		    Raycasting.insidePolygon ((v1.vertex.x + v2.vertex.x) / 2, (v1.vertex.y + v2.vertex.y) / 2, world.obstacles [v1.obstacleIndex])) {
			return false;
		} else {
			for (var k = 0; k < world.obstacleEdges.Count; k++) {
				if (Raycasting.segmentsIntersect (v1.vertex, v2.vertex, world.obstacleEdges [k].vertex1, world.obstacleEdges [k].vertex2)) {
					return false;
				}
			}
		}

		return true;
	}

	private float damage(float r, float d0, float k) {
		return (float)(d0 * System.Math.Exp (-r*k));
	}

	private float shotgunDamage(float r) {
		return (float)(2 * System.Math.Exp (-r / 5));
	}

	private float rifleDamage(float r) {
		return (float)System.Math.Exp (-r / 20);
	}


	public class ShooterOneStepPlan {
		
		public List<Vector2> positions { get; set; }

		public int targetAgentIndex { get; set; }

		public ShooterOneStepPlan(List<Vector2> positions, int targetAgentIndex) {
			this.positions = positions;
			this.targetAgentIndex = targetAgentIndex;
		}
	}

	public class State {
		//public readonly int[] positionIndices;

		public readonly World.VisibilityVertex[] positions;
		public readonly float[] health;
		public float utility { get; set; }

		public State(World.VisibilityVertex[] positions, float[] health) {
			this.positions = positions;
			this.health = health;
		}

		public State(World.VisibilityVertex[] positions, float[] health, float utility) {
			this.positions = positions;
			this.health = health;
			this.utility = utility;
		}

	}
}