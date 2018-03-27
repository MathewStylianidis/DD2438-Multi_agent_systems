using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ShootingPlanner {

    public const float DEATH_UTILITY_FACTOR = 5.0f;
    public const float HEALTH_UTILITY_FACTOR = 3.0f;
    public const float LOG_HEALTH_UTILITY_FACTOR = 3.0f;
    public const float DEATHS_UTILITY_RATIO_BASE = 0.1f;
    public const float HEALTH_UTILITY_RATIO_BASE = 0.2f;
    public const float LOG_HEALTH_UTILITY_RATIO_BASE = 0.2f;
    //public const float TERMINAL_STATE_BASE = 1000000.0f;
    public const double HEALTH_LOG_BASE = Math.E;
    public const int MAX_NUMBER_OF_VISITS = 3;

    private World world;
	private float maxHealth = 10.0f;
	private int comradeCount;       // Number of agents in team 1
	private int enemyCount;         // Number of agents in team 2
    private int comradeStartIndex;  // First index of comrade vertices in world.graphVertices
    private int enemyStartIndex;    // First index of enemy vertices in world.graphVertices
    private WeaponTypeEnum weaponType;
	private FloydWarshall shortestPathsFinder;
	private float[,] shortestDists;


    public ShootingPlanner(World world, WeaponTypeEnum weaponType) {
		this.world = world;
		comradeCount = world.startPositions.Length;
		enemyCount = world.enemyPositions.Length;
        enemyStartIndex = world.graphVertices.Count - enemyCount;
        comradeStartIndex = enemyStartIndex - comradeCount;
        this.weaponType = weaponType;
	}

    public List<OneStepPlan> getPlan()
    {

        if (shortestPathsFinder == null)
        {
            shortestPathsFinder = new FloydWarshall(world.visibilityGraph);
            shortestDists = shortestPathsFinder.findShortestPaths();
        }

        // Get initial greedy solution
        List<OneStepPlan> initialSolution;
        switch (weaponType)
        {
            case WeaponTypeEnum.Shotgun:
                initialSolution = getShotgunInitialSolution();
                return initialSolution;

            case WeaponTypeEnum.Rifle:
                initialSolution = getRifleInitialSolution();
                return initialSolution;
            
        }

        return null;
    }

	private List<OneStepPlan> getShotgunInitialSolution() {

        List<OneStepPlan> result = null;
        ShooterOneStepPlan[] shooterOneStepPlans;
        OneStepPlan oneStepPlan;


		// Find "meeting" point
		int meetingVertexIndex = 0;
        State bestMeetingState = null;
        float bestMeetingUtility = float.NegativeInfinity;
		for (int i = 0; i < world.graphVertices.Count; i++) {

            List<OneStepPlan> currResult = new List<OneStepPlan>();
            
			World.VisibilityVertex[] startVertices = new World.VisibilityVertex[comradeCount];
			List<int>[] paths = new List<int>[comradeCount];
			for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++) {
				paths [comradeIndex] = shortestPathsFinder.reconstructShortestPath (comradeStartIndex + comradeIndex, i);
				startVertices [comradeIndex] = world.graphVertices [comradeStartIndex + comradeIndex];
            }

            float[] health = new float[comradeCount + enemyCount];
            for (int j = 0; j < health.Length; j++)
                health[j] = maxHealth;
            State currState = new State (startVertices, health);

			bool allReachedFinish = false;
			int[] prevPivots = new int[comradeCount];
			while (!allReachedFinish) {
                
                shooterOneStepPlans = new ShooterOneStepPlan[comradeCount];
                oneStepPlan = new OneStepPlan(shooterOneStepPlans);
                
                allReachedFinish = true;
				for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++) {
                    
                    List<Vector2> currComradePositions = new List<Vector2>();
                    shooterOneStepPlans[comradeIndex] = new ShooterOneStepPlan(currComradePositions, currState.health[comradeIndex]);

                    if (prevPivots [comradeIndex] == paths [comradeIndex].Count - 1 || !(currState.health[comradeIndex] > 0.0f)) {

                        var timeLeft = 1.0f;
                        if (prevPivots[comradeIndex] == paths[comradeIndex].Count - 1)
                        {
                            while (timeLeft > 0.0f)
                            {
                                currComradePositions.Add(world.graphVertices[paths[comradeIndex].Last()].vertex);
                                timeLeft -= world.vehicle.dt;
                            }
                        }
                        else if (!(currState.health[comradeIndex] > 0.0f))
                        {
                            //currComradePositions.Add(shooterOneStepPlans[comradeIndex].positions.Last());
                            currComradePositions.AddRange(currResult.Last().shooterPlans[comradeIndex].positions);
                        }

                        continue;
					} else {
						allReachedFinish = false;
					}

                    float maxTravelDist = world.vehicle.maxVelocity;
                    Vector2 nextVertex = world.graphVertices[paths[comradeIndex][prevPivots[comradeIndex] + 1]].vertex;
                    Vector2 currVertex = world.graphVertices[paths[comradeIndex][prevPivots[comradeIndex]]].vertex;
                    float prevStepRemDist = (currState.positions[comradeIndex].vertex - currVertex).magnitude;
                    float remainingDist = (nextVertex - currVertex).magnitude - prevStepRemDist;
                    bool reachedFinish = false;

                    float stepTimeLeft = 1.0f;
                    float accumulatedTime = 0.0f;
                    
                    float travelTime;
                    float prevStepRemTime;

                    while (maxTravelDist > remainingDist) {

                        // we're at point prevPosition + prevStepRemDist

                        prevStepRemTime = prevStepRemDist / world.vehicle.maxVelocity;

                        travelTime = remainingDist / world.vehicle.maxVelocity;
                        while (accumulatedTime + world.vehicle.dt < travelTime)
                        {
                            accumulatedTime += world.vehicle.dt;
                            stepTimeLeft -= world.vehicle.dt;

                            if (stepTimeLeft <= 0.0f)
                            {
                                Debug.Log("FUCKKKK !!!");
                                break;
                            }
                            
                            // calculate position, add it
                            currComradePositions.Add((nextVertex - currVertex) * ((prevStepRemTime + accumulatedTime) / (prevStepRemTime + travelTime)) + currVertex);
                        }
                        accumulatedTime = accumulatedTime + world.vehicle.dt - travelTime;
                        // calculate position, add it


                        prevPivots[comradeIndex]++;
                        if (prevPivots[comradeIndex] == paths[comradeIndex].Count - 1) {
                            remainingDist = 0.0f;
                            reachedFinish = true;
                            break;
                        }

                        maxTravelDist -= remainingDist;
                        prevStepRemDist = 0.0f;

                        nextVertex = world.graphVertices [paths [comradeIndex] [prevPivots [comradeIndex] + 1]].vertex;
						currVertex = world.graphVertices [paths [comradeIndex] [prevPivots [comradeIndex]]].vertex;
						remainingDist = (nextVertex - currVertex).magnitude;
                        
					}

                    prevStepRemTime = prevStepRemDist / world.vehicle.maxVelocity;

                    travelTime = remainingDist / world.vehicle.maxVelocity;
                    while (accumulatedTime + world.vehicle.dt < travelTime)
                    {
                        accumulatedTime += world.vehicle.dt;
                        stepTimeLeft -= world.vehicle.dt;

                        if (stepTimeLeft <= 0.0f)
                        {
                            break;
                        }

                        // calculate position, add it
                        //currComradePositions.Add((nextVertex - currVertex) * (accumulatedTime / travelTime) + currVertex);

                        currComradePositions.Add((nextVertex - currVertex) * ((prevStepRemTime + accumulatedTime) / (prevStepRemTime + travelTime)) + currVertex);
                    }
                    accumulatedTime = accumulatedTime + world.vehicle.dt - travelTime;

                    var prevPosition = world.graphVertices [paths [comradeIndex] [prevPivots [comradeIndex]]].vertex;
					if (reachedFinish) {
                        currState.positions[comradeIndex] = world.graphVertices[i];
                        
                        while (stepTimeLeft> 0.0f)
                        {
                            currComradePositions.Add(currState.positions[comradeIndex].vertex);
                            stepTimeLeft -= 0.1f;
                        }

                        continue;
					}

					float currTravelLineLength = world.visibilityGraph [paths [comradeIndex] [prevPivots [comradeIndex]]] [paths [comradeIndex] [prevPivots [comradeIndex] + 1]];
                    currState.positions[comradeIndex] = new World.VisibilityVertex((world.graphVertices[paths[comradeIndex][prevPivots[comradeIndex] + 1]].vertex - prevPosition) *
                        ((maxTravelDist + prevStepRemDist) / currTravelLineLength) + prevPosition, false);

                    currComradePositions.Add(currState.positions[comradeIndex].vertex);

                }

                if (!allReachedFinish)
                {

                    float[] enemyHealths = new float[enemyCount];
                    oneStepPlan.enemyHealths = enemyHealths;
                    for (int enemIndex = 0; enemIndex < enemyCount; enemIndex++)
                    {
                        enemyHealths[enemIndex] = currState.health[comradeCount + enemIndex];
                    }

                    int[] comradeTargets;
                    int[] enemyTargets;
                    float enemiesTotalHealth;
                    shoot(currState, 2.0f, 1.0f / 5, 1.0f, 1.0f / 20, out comradeTargets, out enemyTargets, out enemiesTotalHealth);

                    oneStepPlan.enemyTargetIndices = enemyTargets;

                    for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                        shooterOneStepPlans[comradeIndex].targetAgentIndex = comradeTargets[comradeIndex];

                    currResult.Add(oneStepPlan);
                }

            }

            // Estimate utility of currState
            float currUtility = estimateUtility(currState, isRelative: true);
            if (currUtility > bestMeetingUtility) {
                bestMeetingUtility = currUtility;
                bestMeetingState = currState;
                meetingVertexIndex = i;
                result = currResult;
            }
            
        }

        // Find the best enemies traversing order

        float bestPermUtility = float.NegativeInfinity;
        List<OneStepPlan> bestPermuResult = null;
        System.Collections.IList bestPermu = null;
        State bestPermuState = null;

        var enemyIndices = Enumerable.Range(0, enemyCount).Where(i => bestMeetingState.health[comradeCount + i] > 0.0f).ToList();
        foreach (var permu in UtilityClass.Permutate(enemyIndices, enemyIndices.Count))
        {
            List<int> path = new List<int>();
            List<int> reconstructedPath = shortestPathsFinder.reconstructShortestPath(meetingVertexIndex, enemyStartIndex + (int)permu[0]);
            path.AddRange(reconstructedPath);
            for (int i = 0; i < permu.Count - 1; i++)
            {
                reconstructedPath = shortestPathsFinder.reconstructShortestPath(enemyStartIndex + (int)permu[i], enemyStartIndex + (int)permu[i + 1]);
                reconstructedPath.RemoveAt(0);
                path.AddRange(reconstructedPath);
            }

            // Simulate reconstructed path
            float[] health = new float[comradeCount + enemyCount];
            for (int j = 0; j < health.Length; j++)
                health[j] = bestMeetingState.health[j];
            World.VisibilityVertex[] startVertices = new World.VisibilityVertex[comradeCount];
            for (int i = 0; i < startVertices.Length; i++)
                startVertices[i] = new World.VisibilityVertex(bestMeetingState.positions[i]);
            State curState = new State(startVertices, health);

            List<OneStepPlan> currPermuResult = new List<OneStepPlan>();


            int prevPivot = 0;
            float currTotalHealth = 1.0f;
            bool reachedFinish = false;
            while (!reachedFinish && currTotalHealth > 0.0f)
            {
                shooterOneStepPlans = new ShooterOneStepPlan[comradeCount];
                oneStepPlan = new OneStepPlan(shooterOneStepPlans);

                for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                {
                    List<Vector2> curComradePositions = new List<Vector2>();
                    shooterOneStepPlans[comradeIndex] = new ShooterOneStepPlan(curComradePositions, curState.health[comradeIndex]);
                }

                float maxTravelDist = world.vehicle.maxVelocity;
                Vector2 nextVertex = world.graphVertices[path[prevPivot + 1]].vertex;
                Vector2 currVertex = world.graphVertices[path[prevPivot]].vertex;
                float prevStepRemDist = (curState.positions[0].vertex - currVertex).magnitude;
                float remainingDist = (nextVertex - currVertex).magnitude - prevStepRemDist;
                
                float stepTimeLeft = 1.0f;
                float accumulatedTime = 0.0f;
                float travelTime;

                float prevStepRemTime;

                while (maxTravelDist > remainingDist)
                {

                    prevStepRemTime = prevStepRemDist / world.vehicle.maxVelocity;

                    // we're at point prevPosition + prevStepRemDist
                    travelTime = remainingDist / world.vehicle.maxVelocity;
                    while (accumulatedTime + world.vehicle.dt < travelTime)
                    {
                        accumulatedTime += world.vehicle.dt;
                        stepTimeLeft -= world.vehicle.dt;

                        if (stepTimeLeft <= 0.0f)
                        {
                            Debug.Log("FUCKKKK !!!");
                            break;
                        }

                        // calculate position, add it (for each comrade)

                        for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                        {
                            shooterOneStepPlans[comradeIndex].positions.Add((nextVertex - currVertex) * ((prevStepRemTime + accumulatedTime) / (prevStepRemTime + travelTime)) + currVertex);
                        }
                    }
                    accumulatedTime = accumulatedTime + world.vehicle.dt - travelTime;


                    prevPivot++;
                    if (prevPivot == path.Count - 1)
                    {
                        reachedFinish = true;
                        break;
                    }

                    maxTravelDist -= remainingDist;
                    prevStepRemDist = 0.0f;

                    nextVertex = world.graphVertices[path[prevPivot + 1]].vertex;
                    currVertex = world.graphVertices[path[prevPivot]].vertex;
                    remainingDist = (nextVertex - currVertex).magnitude;
                }

                prevStepRemTime = prevStepRemDist / world.vehicle.maxVelocity;

                travelTime = remainingDist / world.vehicle.maxVelocity;
                while (accumulatedTime + world.vehicle.dt < travelTime)
                {
                    accumulatedTime += world.vehicle.dt;
                    stepTimeLeft -= world.vehicle.dt;

                    if (stepTimeLeft <= 0.0f)
                    {
                        break;
                    }

                    // calculate position, add it (for each fucker)
                    
                    for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                    {
                        shooterOneStepPlans[comradeIndex].positions.Add((nextVertex - currVertex) * ((prevStepRemTime + accumulatedTime) / (prevStepRemTime + travelTime)) + currVertex);
                    }
                }
                accumulatedTime = accumulatedTime + world.vehicle.dt - travelTime;
                
                var prevPosition = world.graphVertices[path[prevPivot]].vertex;
                if (reachedFinish)
                {
                    // TODO: Change positions so that healthiest fucker is in the front

                    for(int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                        curState.positions[comradeIndex] = world.graphVertices[path[path.Count - 1]];
                    
                    while (stepTimeLeft > 0.0f)
                    {
                        for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                        {
                            shooterOneStepPlans[comradeIndex].positions.Add(curState.positions[comradeIndex].vertex);
                        }
                        stepTimeLeft -= 0.1f;
                    }

                    continue;
                }

                float currTravelLineLength = world.visibilityGraph[path[prevPivot]][path[prevPivot + 1]];

                // TODO: Change positions so that healthiest fucker is in the front

                for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                {
                    curState.positions[comradeIndex] = new World.VisibilityVertex((world.graphVertices[path[prevPivot + 1]].vertex - prevPosition) *
                        ((maxTravelDist + prevStepRemDist) / currTravelLineLength) + prevPosition, false);
                    shooterOneStepPlans[comradeIndex].positions.Add(curState.positions[comradeIndex].vertex);
                }


                float[] enemyHealths = new float[enemyCount];
                oneStepPlan.enemyHealths = enemyHealths;
                for (int enemIndex = 0; enemIndex < enemyCount; enemIndex++)
                {
                    enemyHealths[enemIndex] = curState.health[comradeCount + enemIndex];
                }

                int[] comradeTargets;
                int[] enemyTargets;
                float enemiesTotalHealth;
                currTotalHealth = shoot(curState, 2.0f, 1.0f / 5, 1.0f, 1.0f / 20, out comradeTargets, out enemyTargets, out enemiesTotalHealth);
                
                oneStepPlan.enemyTargetIndices = enemyTargets;

                for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                    shooterOneStepPlans[comradeIndex].targetAgentIndex = comradeTargets[comradeIndex];

                currPermuResult.Add(oneStepPlan);

                if (!(enemiesTotalHealth > 0.0f))
                {
                    reachedFinish = true;
                }
            }


            // Adding the final state information
            shooterOneStepPlans = new ShooterOneStepPlan[comradeCount];
            oneStepPlan = new OneStepPlan(shooterOneStepPlans);
            for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
            {
                List<Vector2> curComradePositions = new List<Vector2>();
                shooterOneStepPlans[comradeIndex] = new ShooterOneStepPlan(curComradePositions, curState.health[comradeIndex]);
            }
            for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
            {
                shooterOneStepPlans[comradeIndex].positions.Add(curState.positions[comradeIndex].vertex);
            }
            float[] lastEnemyHealths = new float[enemyCount];
            oneStepPlan.enemyHealths = lastEnemyHealths;
            for (int enemIndex = 0; enemIndex < enemyCount; enemIndex++)
            {
                lastEnemyHealths[enemIndex] = curState.health[comradeCount + enemIndex];
            }
            currPermuResult.Add(oneStepPlan);


            // Estimate utility of curState
            float curUtility = estimateUtility(curState, isRelative: true);
            if (curUtility > bestPermUtility)
            {
                bestPermUtility = curUtility;
                bestPermuState = curState;
                bestPermu = permu;
                bestPermuResult = currPermuResult;
            }

        }

        result.AddRange(bestPermuResult);

        return result;
	}


    private List<OneStepPlan> getRifleInitialSolution()
    {
        List<OneStepPlan> result = new List<OneStepPlan>();
        
        Dictionary<int, int> numberOfPointVisits = new Dictionary<int, int>();
        for (int i = 0; i < world.graphVertices.Count; i++)
        {
            numberOfPointVisits.Add(i, 0);
        }

        int[] prevCombination = new int[comradeCount];
        for (int i = 0; i < comradeCount; i++)
        {
            prevCombination[i] = comradeStartIndex + i;
            numberOfPointVisits[prevCombination[i]] = 1;
        }

        Dictionary<int[], StateVisits> stateVisits = new Dictionary<int[], StateVisits>(new CombinationsEqualityComparer())
        {
            { prevCombination, new StateVisits() { numberOfVisits = 1 } }
        };
        
        World.VisibilityVertex[] startVertices = new World.VisibilityVertex[comradeCount];
        for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
        {
            startVertices[comradeIndex] = world.graphVertices[comradeStartIndex + comradeIndex];
        }
        float[] health = new float[comradeCount + enemyCount];
        for (int j = 0; j < health.Length; j++)
            health[j] = maxHealth;
        State prevBestState = new State(startVertices, health);

        bool terminalStateReached = false;
        while (!terminalStateReached)
        {
            // Select feasible vertices to go to
            List<int>[] oneStepAccessVertices = new List<int>[comradeCount];
            for (int i = 0; i < comradeCount; i++)
            {
                oneStepAccessVertices[i] = new List<int>();
                for (int j = 0; j < world.graphVertices.Count; j++)
                {
                    var comraringValue = shortestDists[prevCombination[i], j];
                    if (comraringValue <= world.vehicle.maxVelocity*1.0f)
                    {
                        oneStepAccessVertices[i].Add(j);
                    }
                }
            }

            bool currTerminalStateReached = false;
            OneStepPlan bestOneStepPlan = null;
            int[] bestCombination = new int[comradeCount];
            State currBestState = null;
            float bestUtility = float.NegativeInfinity;

            int[] accessVertIndices = new int[comradeCount];
            int[] newCombination = new int[comradeCount];
            bool allCombinationsExhausted = false;
            while (!allCombinationsExhausted)
            {
                // New combination that we're assessing
                bool isInvalidCombination = false;
                for (int j = 0; j < comradeCount; j++)
                {
                    newCombination[j] = oneStepAccessVertices[j][accessVertIndices[j]];
                    if (prevBestState == null || prevBestState.health == null || newCombination == null || prevCombination == null)
                    {
                        Debug.Log("Fuck");
                        isInvalidCombination = true;
                        break;
                    }

                    if (!(prevBestState.health[j] > 0.0f) && newCombination[j] != prevCombination[j])
                    {
                        isInvalidCombination = true;
                        break;
                    }

                    for (int k = 0; k < comradeCount; k++)
                    {
                        if (j != k && newCombination[j] == newCombination[k])
                        {
                            isInvalidCombination = true;
                            break;
                        }
                    }
                }
                
                StateVisits combVisits;
                if (!isInvalidCombination && (!stateVisits.TryGetValue(newCombination, out combVisits) || combVisits.numberOfVisits < MAX_NUMBER_OF_VISITS))
                {
                    ShooterOneStepPlan[] shooterOneStepPlans = new ShooterOneStepPlan[comradeCount];
                    OneStepPlan oneStepPlan = new OneStepPlan(shooterOneStepPlans);
                    
                    int[] prevPivots = new int[comradeCount];

                    // Simulation

                    for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                    {
                        List<Vector2> currComradePositions = new List<Vector2>();
                        shooterOneStepPlans[comradeIndex] = new ShooterOneStepPlan(currComradePositions, prevBestState.health[comradeIndex]);

                        if (prevCombination[comradeIndex] == newCombination[comradeIndex] || !(prevBestState.health[comradeIndex] > 0.0f))
                        {
                            var timeLeft = 1.0f;
                            if (prevCombination[comradeIndex] == newCombination[comradeIndex])
                            {
                                while (timeLeft > 0.0f)
                                {
                                    currComradePositions.Add(world.graphVertices[prevCombination[comradeIndex]].vertex);
                                    timeLeft -= world.vehicle.dt;
                                }
                            }
                            else if (!(prevBestState.health[comradeIndex] > 0.0f))
                            {
                                //currComradePositions.Add(shooterOneStepPlans[comradeIndex].positions.Last());
                                currComradePositions.Add(prevBestState.positions[comradeIndex].vertex);
                            }

                            continue;
                        }

                        List<int> path = shortestPathsFinder.reconstructShortestPath(prevCombination[comradeIndex], newCombination[comradeIndex]);
                        
                        float maxTravelDist = world.vehicle.maxVelocity * 1.0f;
                        Vector2 nextVertex = world.graphVertices[path[prevPivots[comradeIndex] + 1]].vertex;
                        Vector2 currVertex = world.graphVertices[path[prevPivots[comradeIndex]]].vertex;
                        float remainingDist = (nextVertex - currVertex).magnitude;

                        float stepTimeLeft = 1.0f;
                        float accumulatedTime = 0.0f;

                        float travelTime;
                        float prevStepRemTime;

                        while (maxTravelDist >= remainingDist)
                        {
                            // we're at point prevPosition + prevStepRemDist
                            travelTime = remainingDist / world.vehicle.maxVelocity;
                            while (accumulatedTime + world.vehicle.dt < travelTime)
                            {
                                accumulatedTime += world.vehicle.dt;
                                stepTimeLeft -= world.vehicle.dt;
                                if (stepTimeLeft <= 0.0f)
                                {
                                    Debug.Log("FUCKKKK !!!");
                                    break;
                                }
                                // calculate position, add it (for each comrade)
                                currComradePositions.Add((nextVertex - currVertex) * ((accumulatedTime) / (travelTime)) + currVertex);
                            }
                            accumulatedTime = accumulatedTime + world.vehicle.dt - travelTime;


                            prevPivots[comradeIndex]++;
                            if (prevPivots[comradeIndex] == path.Count - 1)
                            {
                                remainingDist = 0.0f;
                                break;
                            }

                            maxTravelDist -= remainingDist;

                            nextVertex = world.graphVertices[path[prevPivots[comradeIndex] + 1]].vertex;
                            currVertex = world.graphVertices[path[prevPivots[comradeIndex]]].vertex;
                            remainingDist = (nextVertex - currVertex).magnitude;
                        }

                        // Adding missing dt-points
                        while (stepTimeLeft > 0.0f)
                        {
                            Vector2 lastVertex = world.graphVertices[path.Last()].vertex;
                            if (Vector2.Distance(lastVertex, nextVertex) > 0.000001f)
                            {
                                Debug.Log("FUCKKKK !!!");
                            }
                            currComradePositions.Add(world.graphVertices[path.Last()].vertex);
                            stepTimeLeft -= 0.1f;
                        }

                    }
                    
                    // Create a new state

                    World.VisibilityVertex[] currVertices = new World.VisibilityVertex[comradeCount];
                    for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                    {
                        currVertices[comradeIndex] = world.graphVertices[newCombination[comradeIndex]];
                    }
                    float[] currHealths = new float[comradeCount + enemyCount];
                    for (int j = 0; j < currHealths.Length; j++)
                        currHealths[j] = prevBestState.health[j];
                    State currState = new State(currVertices, currHealths);
                    
                    float[] enemyHealths = new float[enemyCount];
                    oneStepPlan.enemyHealths = enemyHealths;
                    for (int enemIndex = 0; enemIndex < enemyCount; enemIndex++)
                    {
                        enemyHealths[enemIndex] = currState.health[comradeCount + enemIndex];
                    }
                    
                    // TODO: If some fuckers are at one point - change positions so that healthiest fucker is in the front
                    
                    int[] comradeTargets;
                    int[] enemyTargets;
                    float enemiesTotalHealth;
                    // Shooting
                    float currTotalHealth = shoot(currState, 1.0f, 1.0f / 20, 2.0f, 1.0f / 5, out comradeTargets, out enemyTargets, out enemiesTotalHealth);

                    oneStepPlan.enemyTargetIndices = enemyTargets;

                    for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                        shooterOneStepPlans[comradeIndex].targetAgentIndex = comradeTargets[comradeIndex];

                    if (!(enemiesTotalHealth > 0.0f) || !(currTotalHealth > 0.0f))
                    {
                        currTerminalStateReached = true;
                        //allCombinationsExhausted = true;
                    }

                    float currUtility = estimateUtility(currState, isRelative: true);

                    int currCombPointVisitsTotal = 0;
                    int numberOfVisibleOpponents = 0;
                    float sumShootingDist = 0.0f;
                    int numShoots = 0;
                    float sumShortDistToOp = 0.0f;
                    float sumDistToComrades = 0.0f;
                    int numberComradesAlive = 0;
                    for (int j = 0; j < comradeCount; j++)
                    {
                        if (!(currState.health[j] > 0.0f))
                        {
                            continue;
                        }

                        numberComradesAlive++;
                        sumDistToComrades += Enumerable.Range(0, comradeCount).Where(ind => currState.health[ind] > 0.0f)
                            .Sum(sumInd => shortestDists[newCombination[j], newCombination[sumInd]]);

                        currCombPointVisitsTotal += numberOfPointVisits[newCombination[j]];

                        if (comradeTargets[j] != -1)
                        {
                            int numRepeat = comradeTargets.Count(x => x == comradeTargets[j]);
                            numberOfVisibleOpponents += (numRepeat * numRepeat);
                            numShoots++;
                            sumShootingDist += Vector2.Distance(currState.positions[j].vertex, world.graphVertices[enemyStartIndex + comradeTargets[j]].vertex);
                        }
                        else
                        {
                            float minDistToOp = float.PositiveInfinity;
                            for (int k = 0; k < enemyCount; k++)
                            {
                                if (shortestDists[newCombination[j], enemyStartIndex + k] < minDistToOp)
                                {
                                    minDistToOp = shortestDists[newCombination[j], enemyStartIndex + k];
                                }
                            }
                            sumShortDistToOp += minDistToOp;
                        }
                    }
                    currUtility -= 0.2f * currCombPointVisitsTotal;
                    currUtility += 1.0f * numberOfVisibleOpponents;
                    if (numShoots != 0)
                    {
                        currUtility += 0.3f * sumShootingDist / numShoots;
                    }
                    else
                    {
                        currUtility -= 50.0f * sumShortDistToOp / numberComradesAlive;
                    }
                    currUtility -= 100.0f * sumDistToComrades / numberComradesAlive;
                    
                    // Adding Gaussian noise
                    double u1 = 1.0 - WorldController.rand.NextDouble();
                    double u2 = 1.0 - WorldController.rand.NextDouble();
                    double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
                    currUtility += (float)(randStdNormal * 0.1);

                    if (currUtility > bestUtility)
                    {
                        // Record best values
                        bestUtility = currUtility;
                        bestOneStepPlan = oneStepPlan;
                        currBestState = currState;
                        terminalStateReached = currTerminalStateReached;
                        
                        for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
                        {
                            bestCombination[comradeIndex] = newCombination[comradeIndex];
                        }
                    }
                }

                // Generate a new combination
                int i = comradeCount - 1;
                accessVertIndices[i]++;
                while (accessVertIndices[i] >= oneStepAccessVertices[i].Count)
                {
                    accessVertIndices[i] = 0;
                    i--;
                    if (i < 0)
                    {
                        allCombinationsExhausted = true;
                        break;
                    }

                    accessVertIndices[i]++;
                }
            }


            StateVisits bestCombVisits;
            if (!stateVisits.TryGetValue(bestCombination, out bestCombVisits))
            {
                bestCombVisits = new StateVisits();
                stateVisits.Add(bestCombination, bestCombVisits);
            }
            bestCombVisits.numberOfVisits++;

            prevBestState = currBestState;
            result.Add(bestOneStepPlan);

            for (int comradeIndex = 0; comradeIndex < comradeCount; comradeIndex++)
            {
                prevCombination[comradeIndex] = bestCombination[comradeIndex];
                numberOfPointVisits[prevCombination[comradeIndex]]++;
            }

            if (terminalStateReached)
            {
                // Adding the last state info with zero healths

                ShooterOneStepPlan[] shooterLastStepPlans = new ShooterOneStepPlan[comradeCount];
                OneStepPlan lastStepPlan = new OneStepPlan(shooterLastStepPlans);

                float[] lastEnemyHealths = new float[enemyCount];
                for (int i = 0; i < enemyCount; i++)
                {
                    lastEnemyHealths[i] = prevBestState.health[comradeCount + i];
                }
                lastStepPlan.enemyHealths = lastEnemyHealths;

                for (int i = 0; i < comradeCount; i++)
                {
                    shooterLastStepPlans[i] = new ShooterOneStepPlan(new List<Vector2> { prevBestState.positions[i].vertex }, prevBestState.health[i]);
                    // prevBestState.positions.Select(p => p.vertex).ToList()
                }

                result.Add(lastStepPlan);
            }
        }
        
        return result;
    }
    
	private float shoot(State state, float d0_comrade, float k_comrade, float d0_enemy, float k_enemy, out int[] comradeTargets, out int[] enemyTargets, out float enemiesTotalHealth) {
        enemiesTotalHealth = 0.0f;
        float ourTotalHealth = 0.0f;

        comradeTargets = Enumerable.Repeat(-1, comradeCount).ToArray();
        enemyTargets = Enumerable.Repeat(-1, enemyCount).ToArray();

        for (int i = 0; i < state.health.Length; i++)
        {
            if (!(state.health[i] > 0.0f))
            {
                continue;
            }

            float minDistance = float.MaxValue;
            int minIndex = -1;
            if (i < comradeCount)
            {
                // Total health to be returned
                ourTotalHealth += state.health[i];

                // The comrade shoots
                for (int j = 0; j < enemyCount; j++)
                {

                    if (!(state.health[comradeCount + j] > 0.0f))
                    {
                        continue;
                    }

                    // find closest if not wall
                    float currDistance = (state.positions[i].vertex - world.graphVertices[j + enemyStartIndex].vertex).magnitude;
                    if (currDistance < minDistance && canSee(state.positions[i], world.graphVertices[j + enemyStartIndex]))
                    {
                        minDistance = currDistance;
                        minIndex = j;
                    }
                }

                if (minIndex != -1)
                {
                    state.health[comradeCount + minIndex] -= damage(minDistance, d0_comrade, k_comrade);
                    state.health[comradeCount + minIndex] = Math.Max(state.health[comradeCount + minIndex], 0.0f);
                    comradeTargets[i] = minIndex;
                }
                continue;
            }


            int currEnemyIndex = i - comradeCount;

            enemiesTotalHealth += state.health[i];

            // The enemy shoots
            for (int j = 0; j < comradeCount; j++)
            {

                if (!(state.health[j] > 0.0f))
                {
                    continue;
                }
                // find closest if not wall
                float currDistance = (state.positions[j].vertex - world.graphVertices[enemyStartIndex + currEnemyIndex].vertex).magnitude;
                if (currDistance < minDistance && canSee(state.positions[j], world.graphVertices[enemyStartIndex + currEnemyIndex]))
                {
                    minDistance = currDistance;
                    minIndex = j;
                }
            }

            if (minIndex != -1)
            {
                float ourHealthBeforeDamage = state.health[minIndex];

                state.health[minIndex] -= damage(minDistance, d0_enemy, k_enemy);
                state.health[minIndex] = Math.Max(state.health[minIndex], 0.0f);
                enemyTargets[currEnemyIndex] = minIndex;

                ourTotalHealth -= (ourHealthBeforeDamage - state.health[minIndex]);
            }

        }

        return ourTotalHealth;
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
		return (float)(d0 * Math.Exp (-r*k));
	}

	private float shotgunDamage(float r) {
		return (float)(2 * Math.Exp (-r / 5));
	}

	private float rifleDamage(float r) {
        return (float)Math.Exp(-r / 20);
	}

    private float estimateUtility(State state, bool isRelative = true) {
        float ourTotalHealth = 0.0f;
        float ourLogarithmHealth = 0.0f;
        int ourDeaths = 0;
        for (int i = 0; i < comradeCount; i++)
        {
            if (state.health[i] > 0.0f)
            {
                ourTotalHealth += state.health[i];
                ourLogarithmHealth += (float)Math.Log(1.0 + state.health[i], HEALTH_LOG_BASE);
            }
            else
            {
                ourDeaths++;
            }
        }

        float theirTotalHealth = 0.0f;
        float theirLogarithmHealth = 0.0f;
        int theirDeaths = 0;
        for (int i = 0; i < enemyCount; i++)
        {
            if (state.health[comradeCount + i] > 0.0f)
            {
                theirTotalHealth += state.health[comradeCount + i];
                theirLogarithmHealth += (float)Math.Log(1.0 + state.health[comradeCount+i], HEALTH_LOG_BASE);
            }
            else
            {
                theirDeaths++;
            }
        }

        /*
        if (ourDeaths == 0)
        {
            return ourTotalHealth / theirTotalHealth + theirDeaths * DEAD_UTILITY;
        }
        if (ourDeaths == comradeCount)
        {
            return -TERMINAL_STATE_BASE + theirTotalHealth + theirDeaths * DEAD_UTILITY;
        }
        else if (theirDeaths == enemyCount)
        {
            return TERMINAL_STATE_BASE - ourTotalHealth - ourDeaths * DEAD_UTILITY;
        }
        */

        if (isRelative)
        {

            return DEATH_UTILITY_FACTOR * ((DEATHS_UTILITY_RATIO_BASE + theirDeaths) / (DEATHS_UTILITY_RATIO_BASE + ourDeaths)) +
                HEALTH_UTILITY_FACTOR * (HEALTH_UTILITY_RATIO_BASE + ourTotalHealth) / (HEALTH_UTILITY_RATIO_BASE + theirTotalHealth) +
                LOG_HEALTH_UTILITY_FACTOR * (LOG_HEALTH_UTILITY_RATIO_BASE + ourLogarithmHealth) / (LOG_HEALTH_UTILITY_RATIO_BASE + theirLogarithmHealth);
        }

        return DEATH_UTILITY_FACTOR * ((theirDeaths - ourDeaths)) +
            HEALTH_UTILITY_FACTOR * (ourTotalHealth - theirTotalHealth) +
            LOG_HEALTH_UTILITY_FACTOR * (ourLogarithmHealth - theirLogarithmHealth);

    }

    private float estimateAbsoluteUtility(State state)
    {
        float ourTotalHealth = 0.0f;
        int ourDeaths = 0;
        for (int i = 0; i < comradeCount; i++)
        {
            if (state.health[i] > 0.0f)
            {
                ourTotalHealth += state.health[i];
            }
            else
            {
                ourDeaths++;
            }
        }

        float theirTotalHealth = 0.0f;
        int theirDeaths = 0;
        for (int i = 0; i < enemyCount; i++)
        {
            if (state.health[comradeCount + i] > 0.0f)
            {
                theirTotalHealth += state.health[comradeCount + i];
            }
            else
            {
                theirDeaths++;
            }
        }

        /*
        if (ourDeaths == 0)
        {
            return ourTotalHealth / theirTotalHealth + theirDeaths * DEAD_UTILITY;
        }
        if (ourDeaths == comradeCount)
        {
            return -TERMINAL_STATE_BASE + theirTotalHealth + theirDeaths * DEAD_UTILITY;
        }
        else if (theirDeaths == enemyCount)
        {
            return TERMINAL_STATE_BASE - ourTotalHealth - ourDeaths * DEAD_UTILITY;
        }
        */

        return DEATH_UTILITY_FACTOR * ((theirDeaths - ourDeaths)) +
            HEALTH_UTILITY_FACTOR * (ourTotalHealth - theirTotalHealth);
    }

    public class ShooterOneStepPlan {
		
        public float health { get; set; }

		public List<Vector2> positions { get; set; }

		public int targetAgentIndex { get; set; }

		public ShooterOneStepPlan(List<Vector2> positions, float health) {
			this.positions = positions;
            this.health = health;
        }
	}

    public class OneStepPlan
    {
        public readonly ShooterOneStepPlan[] shooterPlans;

        public float[] enemyHealths { get; set; }

        public int[] enemyTargetIndices { get; set; }

        public OneStepPlan(ShooterOneStepPlan[] shooterPlans)
        {
            this.shooterPlans = shooterPlans;
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


    private class CombinationsEqualityComparer : IEqualityComparer<int[]>
    {
        public bool Equals(int[] x, int[] y)
        {
            if (x.Length != y.Length)
            {
                return false;
            }
            for (int i = 0; i < x.Length; i++)
            {
                if (x[i] != y[i])
                {
                    return false;
                }
            }
            return true;
        }

        public int GetHashCode(int[] obj)
        {
            int result = 17;
            for (int i = 0; i < obj.Length; i++)
            {
                result = result * 23 + obj[i];
                /*
                unchecked
                {
                    result = result * 23 + obj[i];
                }
                */
            }
            return result;
        }
    }

    private class StateVisits
    {
        public int numberOfVisits { get; set; }
    }
}