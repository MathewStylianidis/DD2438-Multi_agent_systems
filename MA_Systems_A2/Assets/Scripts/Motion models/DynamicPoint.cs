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
		Vector3 deltaVel = path.normalized * aMax * dt;
		Vector3 newVel = Vector3.ClampMagnitude (curPointInfo.vel + deltaVel, maxVelocity);

		Debug.Log (curPointInfo.vel);
		Debug.Log (aMax);
		Debug.Log (newVel);
		float xMove = newVel.x * dt;
		float yMove = newVel.z * dt;
		Vector3 newPosition = new Vector3 (curPointInfo.pos.x + xMove, curPointInfo.pos.y, curPointInfo.pos.z + yMove);
		Vector3 newOrientation = path.normalized;
		float xVel = (float)System.Math.Round((System.Double)xMove/dt, 2, System.MidpointRounding.AwayFromZero);
		float zVel = (float)System.Math.Round((System.Double)yMove/dt, 2, System.MidpointRounding.AwayFromZero);
		return new PointInfo (newPosition, new Vector3(xVel, 0f, zVel), newOrientation, curPointInfo.currentTime + dt);
	}

	public override List<PointInfo> completePath (PointInfo curPointInfo, Vector3 goalPoint, World world, bool collisionCheck = true)
	{
		List<PointInfo> path = new List<PointInfo> ();
		while (Vector3.Distance (goalPoint, curPointInfo.pos) != 0) {
			curPointInfo = moveTowards (curPointInfo, goalPoint);
			if (collisionCheck && Raycasting.insideObstacle (curPointInfo.pos.x, curPointInfo.pos.z, world.obstacles))
				return null;
			path.Add (curPointInfo);
		}
		return path;
	}
}
