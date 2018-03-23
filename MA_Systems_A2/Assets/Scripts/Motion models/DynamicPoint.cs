using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DynamicPoint : BaseModel {

	private float aMax;

	public DynamicPoint(float velocityMax, float dt, float aMax) :base(velocityMax, dt) {
		this.aMax = aMax;
	}

	public override PointInfo moveTowards (PointInfo curPointInfo, Vector3 goalPoint)
	{
		Vector3 path = goalPoint - curPointInfo.pos;
		float currentAcceleration = aMax;
		Vector3 deltaVel = path.normalized * currentAcceleration * dt;
		Vector3 newVel = Vector3.ClampMagnitude (curPointInfo.vel + deltaVel, maxVelocity);
		float xMove = newVel.x * dt;
		float yMove = newVel.z * dt;

		Vector3 newPosition = new Vector3 (curPointInfo.pos.x + xMove, curPointInfo.pos.y, curPointInfo.pos.z + yMove);
		Vector3 newOrientation = path.normalized;
		float xVel = (float)System.Math.Round((System.Double)xMove/dt, 2, System.MidpointRounding.AwayFromZero);
		float zVel = (float)System.Math.Round((System.Double)yMove/dt, 2, System.MidpointRounding.AwayFromZero);
		return new PointInfo (newPosition, new Vector3(xVel, 0f, zVel), newOrientation, curPointInfo.currentTime + dt);
	}


		
	/// <summary>
	/// Goes from curPointInfo to goalPointInfo with a decreasing velocity and without caring for the goal velocity direction
	/// </summary>
	public override PointInfo moveTowardsWithDecreasingVelocity (PointInfo curPointInfo, PointInfo goalPointInfo, World world, bool collisionCheck = true, float constant = 1.0f)
	{
		float tolerance = 0.01f;
		Vector3 path = goalPointInfo.pos - curPointInfo.pos;
		float dist = path.magnitude;
		PointInfo nextPointInfo = moveTowards(curPointInfo, goalPointInfo.pos);

		if (curPointInfo.pos == goalPointInfo.pos || Vector3.Distance(nextPointInfo.pos, goalPointInfo.pos) < tolerance)
			return goalPointInfo;

		float tmp = maxVelocity;
		// get desired velocity
		maxVelocity = maxVelocity / (1 + constant /(dist + 1e-40f));
		// if desired velocity is smaller than the one that can be achieved then lower velocity as much as possible
		maxVelocity = curPointInfo.vel.magnitude - aMax > maxVelocity ? curPointInfo.vel.magnitude - aMax : maxVelocity;
		nextPointInfo = moveTowards(curPointInfo,goalPointInfo.pos);
		maxVelocity = tmp;

		return nextPointInfo;
	}

	/// <summary>
	/// Completes the path from the current point denoted by curPointInfo to the goal point denoted by goalPointInfo
	/// respecting the constraints of the given motion model.
	/// </summary>
	public override List<PointInfo> completePath (PointInfo curPointInfo, PointInfo goalPointInfo, World world, bool collisionCheck = true)
	{
		float tau = getOptimalTau (goalPointInfo, curPointInfo);
		return getPath(goalPointInfo, curPointInfo, tau);
	}

	private float getOptimalTau(PointInfo goalPointInfo, PointInfo startPointInfo,
		/*params for Newton*/	float t0 = 20, int max_n = 10, float r = 2.0f) {
		float p_x0 = startPointInfo.pos.x;
		float p_y0 = startPointInfo.pos.z;
		float v_x0 = startPointInfo.vel.x;
		float v_y0 = startPointInfo.vel.z;

		float p_x1 = goalPointInfo.pos.x;
		float p_y1 = goalPointInfo.pos.z;
		float v_x1 = goalPointInfo.vel.x;
		float v_y1 = goalPointInfo.vel.z;

		float a = ((24*r*v_x1*v_x0)+(12*r*v_x1*(v_x1-v_x0))-(4*r*Mathf.Pow(2*(v_x1-v_x0)-3*v_x0,2))-(4*r*Mathf.Pow(2*(v_y1-v_y0)-3*v_y0,2)));
		float b = (-24*r*v_x1*(p_x1-p_x0) + 24*r*(2*(v_x1-v_x0) - 3*v_x0)*(p_x1-p_x0) + 24*r*(2*(v_y1-v_y0) - 3*v_y0)*(p_y1-p_y0));
		float c = -36*r*((p_x1-p_x0)*(p_x1-p_x0) + (p_y1-p_y0)*(p_y1-p_y0));

		float[] coefficients = new float[5];
		coefficients [0] = c;
		coefficients [1] = b;
		coefficients [2] = a;
		coefficients [3] = 0;
		coefficients [4] = 1;
		return NewtonSolver.Newton(coefficients, t0, max_n);
	}



	private List<PointInfo> getPath(PointInfo goalPointInfo, PointInfo startPointInfo, float tau, float r = 2.0f) {

		float p_x0 = startPointInfo.pos.x;
		float p_y0 = startPointInfo.pos.z;
		float v_x0 = startPointInfo.vel.x;
		float v_y0 = startPointInfo.vel.z;

		float p_x1 = goalPointInfo.pos.x;
		float p_y1 = goalPointInfo.pos.z;
		float v_x1 = goalPointInfo.vel.x;
		float v_y1 = goalPointInfo.vel.z;

		float d1Tau = 2*r*(6*(p_x1-p_x0-v_x0*tau)/(tau*tau*tau) - 3*(v_x1-v_x0)/(tau*tau));
		float d2Tau = 2*r*(6*(p_y1-p_y0-v_y0*tau)/(tau*tau*tau) - 3*(v_y1-v_y0)/(tau*tau));
		float d3Tau = 2*r*((2/tau)*(v_x1-v_x0) - 3*(p_x1-p_x0-v_x0*tau)/(tau*tau));
		float d4Tau = 2*r*((2/tau)*(v_y1-v_y0) - 3*(p_y1-p_y0-v_y0*tau)/(tau*tau));

		float a_x_curr = (d3Tau-(1e-7f-tau)*d1Tau)/r;
		float a_y_curr = (d4Tau-(1e-7f-tau)*d2Tau)/r;

		if (Mathf.Sqrt(a_x_curr*a_x_curr + a_y_curr*a_y_curr) > aMax) {
			return null;
		}

		float v_x_curr = v_x1 - Mathf.Pow(1e-7f-tau,2)*d1Tau/(2*r) + (1e-7f-tau)*(d3Tau)/r;
		float v_y_curr = v_y1 - Mathf.Pow(1e-7f-tau,2)*d2Tau/(2*r) + (1e-7f-tau)*(d4Tau)/r;
		if (Mathf.Sqrt(v_x_curr*v_x_curr + v_y_curr*v_y_curr) > maxVelocity){
			return null;
		}

		float p_prev_x = p_x0;
		float p_prev_y = p_y0;

		float v_prev_x = v_x0;
		float v_prev_y = v_y0;

		List<PointInfo> path = new List<PointInfo>();
		PointInfo prevPoint = startPointInfo;
		for (float t = dt; t < tau; t+=dt) {
			float p_x_curr = p_x1 + v_x1*(t-tau) + Mathf.Pow(t-tau,2)*d3Tau/(2*r) - t*d1Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Mathf.Pow(tau,3)*d1Tau/(6*r);
			float p_y_curr = p_y1 + v_y1*(t-tau) + Mathf.Pow(t-tau,2)*d4Tau/(2*r) - t*d2Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Mathf.Pow(tau,3)*d2Tau/(6*r);

			v_x_curr = v_x1 - Mathf.Pow(t-tau,2)*d1Tau/(2*r) + (t-tau)*(d3Tau)/r;
			v_y_curr = v_y1 - Mathf.Pow(t-tau,2)*d2Tau/(2*r) + (t-tau)*(d4Tau)/r;

			//	if (Mathf.Sqrt(v_x_curr*v_x_curr + v_y_curr*v_y_curr) > maxVelocity) {
				//return null;
			//}

			Vector3 curPos = new Vector3 (p_x_curr, startPointInfo.pos.y, p_y_curr);
			Vector3 curVel = new Vector3 (v_x_curr, 0.0f, v_y_curr);
			Vector3 curOri = curVel.normalized;
			PointInfo newPoint = new PointInfo (curPos,curVel , curOri, prevPoint.currentTime + dt);
			path.Add (newPoint);
			return path;
			prevPoint = newPoint;
			//if (Math.sqrt((v_x_curr-v_prev_x)*(v_x_curr-v_prev_x) + (v_y_curr-v_prev_y)*(v_y_curr-v_prev_y))/dt > map.vehicle_a_max) {
			//	return false;
			//}

			//a_x_curr = (d3Tau-(t-tau)*d1Tau)/r;
			//a_y_curr = (d4Tau-(t-tau)*d2Tau)/r;

			//if (Math.sqrt(a_x_curr*a_x_curr + a_y_curr*a_y_curr) > map.vehicle_a_max) {
			//	return false;
			//}

			p_prev_x = p_x_curr;
			p_prev_y = p_y_curr;
			//v_prev_x = v_x_curr;
			//v_prev_y = v_y_curr;
		}

		v_x_curr = v_x1 - Mathf.Pow(-1e-3f,2)*d1Tau/(2*r) + (-1e-3f)*(d3Tau)/r;
		v_y_curr = v_y1 - Mathf.Pow(-1e-3f,2)*d2Tau/(2*r) + (-1e-3f)*(d4Tau)/r;
		if (Mathf.Sqrt(v_x_curr*v_x_curr + v_y_curr*v_y_curr) > maxVelocity){
			return null;
		}

		float t_crit = (d3Tau+d1Tau*tau+d4Tau+d2Tau*tau)/(d1Tau+d2Tau);
		if (t_crit > 0 && t_crit < tau) {
			float a_x_crit = (d3Tau-(t_crit-tau)*d1Tau)/r;
			float a_y_crit = (d4Tau-(t_crit-tau)*d2Tau)/r;
			if (Mathf.Sqrt(a_x_crit*a_x_crit + a_y_crit*a_y_crit) > maxVelocity) {
				return null;
			}
		}

		a_x_curr = (d3Tau-(-1e-7f)*d1Tau)/r;
		a_y_curr = (d4Tau-(-1e-7f)*d2Tau)/r;

		if (Mathf.Sqrt(a_x_curr*a_x_curr + a_y_curr*a_y_curr) > maxVelocity) {
			return null;
		}

		return path;
	}

	/*
	public override List<PointInfo> completePath (PointInfo curPointInfo, PointInfo goalPointInfo, World world, bool collisionCheck = true)
	{
		float radius1 = Mathf.Pow(curPointInfo.vel.magnitude,2) / aMax;
		float radius2 = Mathf.Pow(goalPointInfo.vel.magnitude, 2) / aMax;
		List<List<PointInfo>> pathLists = new List<List<PointInfo>>();
		pathLists.Add(RL(curPointInfo, goalPointInfo, radius1, radius2, world, collisionCheck));
		pathLists.Add(RR(curPointInfo, goalPointInfo, radius1, radius2, world, collisionCheck));
		pathLists.Add(LR(curPointInfo, goalPointInfo, radius1, radius2, world, collisionCheck));
		pathLists.Add(LL(curPointInfo, goalPointInfo, radius1, radius2, world, collisionCheck));

		int minIdx = 0;
		int minSize = int.MaxValue;
		for (int i = 0; i < pathLists.Count; i++)
			if (pathLists[i] != null && pathLists[i].Count < minSize)
			{
				minIdx = i;
				minSize = pathLists[i].Count;
			}
		return pathLists[minIdx];
	}

	private List<PointInfo> RL(PointInfo curPointInfo, PointInfo goalPointInfo, float radius1, float radius2, World world, bool collisionCheck)
	{
		Vector3 dir = Vector3.Cross(Vector3.up, curPointInfo.vel).normalized;
		Vector3 p1 = curPointInfo.pos + dir * radius1;
		Vector3 dir2 = Vector3.Cross(Vector3.up, goalPointInfo.vel).normalized;
		Vector3 p2 = goalPointInfo.pos - dir2 * radius2;
		float D = (p2 - p1).magnitude;
		float alpha = radius1 + radius2;
		float theta = Mathf.Acos(alpha / D);
		float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
		if ((p2 - p1).z < 0)
			baseAngle = -baseAngle;
		Vector3 tp1 = p1 + new Vector3(Mathf.Cos(theta + baseAngle), 0f, Mathf.Sin(theta + baseAngle)) * radius1;
		Vector3 tp2 = p2 - new Vector3(Mathf.Cos(theta + baseAngle), 0f, Mathf.Sin(theta + baseAngle)) * radius2;

		//GameObject tmp = new GameObject ();
		//LineRenderer lineRenderer = tmp.AddComponent<LineRenderer> ();
		//lineRenderer.widthMultiplier = Visualizer.widthMultiplier;
		//lineRenderer.useWorldSpace = true;
		//lineRenderer.SetPosition (0, tp1);
		//lineRenderer.SetPosition (1, tp2);
		//tmp.transform.SetParent (GameObject.Find ("Visualizer").transform);


		return dubinPath(curPointInfo, goalPointInfo, radius1, radius2, tp1, tp2, p1, p2, true, false, world, collisionCheck);
	}



	private List<PointInfo> dubinPath(PointInfo curPointInfo, PointInfo goalPointInfo, float radius1, float radius2, Vector3 tp1, Vector3 tp2, Vector3 p1, Vector3 p2, bool rightStart, bool rightGoal, World world, bool collisionCheck)
	{
		Vector3 tangentLine = tp2 - tp1;
		float epsilon = 1e-10f;

		if(collisionCheck && (Raycasting.insideObstacle (tp1.x, tp1.z, world.obstacles) || Raycasting.insideObstacle (tp2.x, tp2.z, world.obstacles)))
			return null;
				
		float arcLength = getArcLength(curPointInfo.pos - p1, tp1 - p1, rightStart, radius1);

		float movingDts = arcLength / (dt * curPointInfo.vel.magnitude);
		int moveCount = (int)Mathf.Ceil(movingDts);
		float phi1 = arcLength / radius1;
		float angleStep = phi1 / moveCount;

		List<PointInfo> pointList = new List<PointInfo>();
		PointInfo parent = curPointInfo;
		float baseAngle = Vector3.Angle(Vector3.right, curPointInfo.pos - p1)*Mathf.Deg2Rad;
		if ((curPointInfo.pos - p1).z < 0)
			baseAngle = -baseAngle;

		int rotateRight = rightStart ? -1 : 1;
		if(moveCount > 0)
			for (int i = 1; i < moveCount; i++)
			{
				Vector3 aDir = (p1 - parent.pos).normalized;
				float x = p1.x + radius1 * Mathf.Cos(baseAngle + rotateRight * i * angleStep);
				float z = p1.z + radius1 * Mathf.Sin(baseAngle + rotateRight * i * angleStep);
				Vector3 newPosition = new Vector3(x, parent.pos.y, z);
				if(collisionCheck && (Raycasting.insideObstacle (newPosition.x, newPosition.z, world.obstacles)))
					return null;
				float xVel = (float)System.Math.Round((System.Double)(x - parent.pos.x ) / dt, 2, System.MidpointRounding.AwayFromZero);
				float zVel = (float)System.Math.Round((System.Double)(z - parent.pos.z) / dt, 2, System.MidpointRounding.AwayFromZero);
				pointList.Add(new PointInfo(newPosition, new Vector3(xVel, 0, zVel), aDir, parent.currentTime + dt));
				parent = pointList[pointList.Count - 1];
			}
		pointList.Add(new PointInfo(tp1, curPointInfo.vel.magnitude * tangentLine.normalized, tangentLine.normalized, parent.currentTime + dt));
		parent = pointList[pointList.Count - 1];
		//Debug.Log (tp1);
		float movingDtsDacc = (goalPointInfo.vel - parent.vel).magnitude / aMax;
		float moveCountDacc = (int)Mathf.Ceil(movingDtsDacc);

		float coefficient = goalPointInfo.vel.magnitude > parent.vel.magnitude ? 1 : -1;
		float dist = 0;
		float vel = parent.vel.magnitude;
		for(int i = 0; i < moveCountDacc; i++)
		{
			dist += vel * dt;
			vel = vel + coefficient * aMax * dt;
		}

		if (dist > tangentLine.magnitude)
			return null;

		movingDts = (tangentLine.magnitude - dist) / (parent.vel.magnitude * dt);
		moveCount = (int)Mathf.Ceil(movingDts);
		//Debug.Log ("2");
		// Move on tangent line
		for (int i = 1; i < moveCount; i++)
		{
			Vector3 newPosition = tp1 + i * dt * parent.vel.magnitude * tangentLine.normalized;
			if(collisionCheck && (Raycasting.insideObstacle (newPosition.x, newPosition.z, world.obstacles)))
				return null;
			pointList.Add(new PointInfo(newPosition, tangentLine.normalized * parent.vel.magnitude, tangentLine.normalized, parent.currentTime + dt));
			parent = pointList[pointList.Count - 1];

		}
		pointList.Add(new PointInfo(tp2 - (tangentLine.normalized * dist), tangentLine.normalized * parent.vel.magnitude, tangentLine.normalized, parent.currentTime + dt));
		parent = pointList[pointList.Count - 1];
		//Debug.Log (tp2 - (tangentLine.normalized * dist));
		//Debug.Log ("3");
		for (int i = 1; i < moveCountDacc; i++)
		{
			vel = (parent.vel.magnitude + coefficient* aMax) * dt;
			Vector3 newPosition = parent.pos + tangentLine.normalized*vel;
			if(collisionCheck && (Raycasting.insideObstacle (newPosition.x, newPosition.z, world.obstacles)))
				return null;
			pointList.Add(new PointInfo(newPosition, tangentLine.normalized * vel /dt, tangentLine.normalized, parent.currentTime + dt));
			parent = pointList[pointList.Count - 1];
		}
		// Add tp2 to list
		pointList.Add(new PointInfo(tp2, goalPointInfo.vel.magnitude * tangentLine.normalized, tangentLine.normalized, parent.currentTime + dt));
		parent = pointList[pointList.Count - 1];
		 

		arcLength = getArcLength(tp2 - p2, goalPointInfo.pos - p2, rightGoal, radius2);
		movingDts = arcLength / (dt * goalPointInfo.vel.magnitude);
		moveCount = (int)Mathf.Ceil(movingDts);
		phi1 = arcLength / radius2;
		angleStep = phi1 / moveCount;
		baseAngle = Vector3.Angle(Vector3.right, tp2 - p2) * Mathf.Deg2Rad;
		if ((tp2 - p2).z < 0)
			baseAngle = -baseAngle;
		rotateRight = rightGoal ? -1 : 1;
		//Debug.Log ("3");
		for (int i = 1; i < moveCount; i++)
		{
			Vector3 aDir = (p2 - parent.pos).normalized;
			float x = p2.x + radius2 * Mathf.Cos(baseAngle + rotateRight * i * angleStep);
			float z = p2.z + radius2 * Mathf.Sin(baseAngle + rotateRight * i * angleStep);
			Vector3 newPosition = new Vector3(x, parent.pos.y, z);
			if(collisionCheck && (Raycasting.insideObstacle (newPosition.x, newPosition.z, world.obstacles)))
				return null;
			float xVel = (float)System.Math.Round((System.Double)(x - parent.pos.x) / dt, 2, System.MidpointRounding.AwayFromZero);
			float zVel = (float)System.Math.Round((System.Double)(z - parent.pos.z) / dt, 2, System.MidpointRounding.AwayFromZero);
			pointList.Add(new PointInfo(newPosition, new Vector3(xVel, 0, zVel), aDir, parent.currentTime + dt));
			parent = pointList[pointList.Count - 1];
			if (float.IsNaN (newPosition.x))
				Debug.Log ("4");
		}

		//Add tp1 to list
		pointList.Add(goalPointInfo);
		//Debug.Log (goalPointInfo.pos);
		return pointList;
	}


	private float getArcLength(Vector3 v1, Vector3 v2, bool right, float radius)
	{
		float theta = Mathf.Atan2(v1.x, v1.z) - Mathf.Atan2(v2.x, v2.z);
		if (theta < 0f && !right)
			theta = theta + 2 * Mathf.PI;
		else if (theta > 0 && right)
			theta = theta - 2 * Mathf.PI;
		return Mathf.Abs(theta * radius);
	}




	private List<PointInfo> LR(PointInfo curPointInfo, PointInfo goalPointInfo, float radius1, float radius2, World world, bool collisionCheck)
	{
		Vector3 dir = Vector3.Cross(Vector3.up, curPointInfo.vel).normalized;
		Vector3 p1 = curPointInfo.pos - dir * radius1;
		Vector3 dir2 = Vector3.Cross(Vector3.up, goalPointInfo.vel).normalized;
		Vector3 p2 = goalPointInfo.pos + dir2 * radius2;

		float D = (p2 - p1).magnitude;
		float alpha = radius1 + radius2;
		float theta = -Mathf.Acos(alpha / D);
		float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
		if ((p2 - p1).z < 0)
			baseAngle = -baseAngle;
		Vector3 tp1 = p1 + new Vector3(Mathf.Cos(theta + baseAngle), 0, Mathf.Sin(theta + baseAngle)) * radius1;
		Vector3 tp2 = p2 - new Vector3(Mathf.Cos(theta + baseAngle), 0, Mathf.Sin(theta + baseAngle)) * radius2;
		return dubinPath(curPointInfo, goalPointInfo, radius1, radius2, tp1, tp2, p1, p2, false, true, world, collisionCheck);
	}

	private List<PointInfo> RR(PointInfo curPointInfo, PointInfo goalPointInfo, float radius1, float radius2, World world, bool collisionCheck)
	{
		Vector3 dir = Vector3.Cross(Vector3.up, curPointInfo.vel).normalized;
		Vector3 p1 = curPointInfo.pos + dir * radius1;
		Vector3 dir2 = Vector3.Cross(Vector3.up, goalPointInfo.vel).normalized;
		Vector3 p2 = goalPointInfo.pos + dir2 * radius2;

		float D = (p2 - p1).magnitude;
		float H = Mathf.Sqrt(Mathf.Pow(D,2) - Mathf.Pow(radius1-radius2,2));
		float Y = Mathf.Sqrt(Mathf.Pow(H,2) + Mathf.Pow(radius2, 2));
		float theta = Mathf.Acos((Mathf.Pow(radius1, 2) + Mathf.Pow(D, 2) - Mathf.Pow(Y, 2)) / (2 * radius1 * D));
		float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
		if ((p2-p1).z < 0)
			baseAngle = -baseAngle;
		Vector3 tp1 = p1 + new Vector3(Mathf.Cos(theta+ baseAngle), 0, Mathf.Sin(theta+ baseAngle)) * radius1;
		Vector3 tp2 = p2 + new Vector3(Mathf.Cos(theta+ baseAngle), 0, Mathf.Sin(theta+ baseAngle)) * radius2;
		return dubinPath(curPointInfo, goalPointInfo, radius1, radius2, tp1, tp2, p1, p2, true, true, world, collisionCheck);
	}

	private List<PointInfo> LL(PointInfo curPointInfo, PointInfo goalPointInfo, float radius1, float radius2, World world, bool collisionCheck)
	{
		Vector3 dir = Vector3.Cross(Vector3.up, curPointInfo.vel).normalized;
		Vector3 p1 = curPointInfo.pos - dir * radius1;
		Vector3 dir2 = Vector3.Cross(Vector3.up, goalPointInfo.vel).normalized;
		Vector3 p2 = goalPointInfo.pos - dir2 * radius2;

		float D = (p2 - p1).magnitude;
		float H = Mathf.Sqrt(Mathf.Pow(D, 2) - Mathf.Pow(radius1 - radius2, 2));
		float Y = Mathf.Sqrt(Mathf.Pow(H, 2) + Mathf.Pow(radius2, 2));
		float theta = Mathf.Acos((Mathf.Pow(radius1, 2) + Mathf.Pow(D, 2) - Mathf.Pow(Y, 2)) / (2 * radius1 * D));
		float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
		if ((p2 - p1).z < 0)
			baseAngle = -baseAngle;
		Vector3 tp1 = p1 + new Vector3(Mathf.Cos(-theta + baseAngle), 0, Mathf.Sin(-theta + baseAngle)) * radius1;
		Vector3 tp2 = p2 + new Vector3(Mathf.Cos(-theta + baseAngle), 0, Mathf.Sin(-theta + baseAngle)) * radius2;
		return dubinPath(curPointInfo, goalPointInfo, radius1, radius2, tp1, tp2, p1, p2, false, false, world, collisionCheck);
	}
	*/
}
