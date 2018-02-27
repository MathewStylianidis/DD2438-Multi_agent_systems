using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinematicPoint : BaseModel {

	public KinematicPoint(float velocityMax, float dt) :base(velocityMax, dt) {}

	public override PointInfo moveTowards (PointInfo curPointInfo, Vector3 goalPoint)
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

	public override List<PointInfo> completePath (PointInfo curPointInfo, Vector3 goalPoint)
	{
		List<PointInfo> path = new List<PointInfo> ();

		return path;
	}


}
