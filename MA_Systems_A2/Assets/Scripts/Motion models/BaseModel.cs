using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class BaseModel {

	protected float maxVelocity;
	protected float dt;

	/// <summary>
	/// Brings the point which can be reached from curPointInfo to goalPoint in dt
	/// given a certain motion model.
	/// </summary>
	public abstract PointInfo moveTowards (PointInfo curPointInfo, Vector3 goalPoint);

	/// <summary>
	/// Moves from curPointInfo to goalPointInfo with a decreasing velocity without caring about the goal direction of velocity
	/// aiming to have a goal velocity of zero.
	/// </summary>
	public abstract PointInfo moveTowardsWithDecreasingVelocity (PointInfo curPointInfo, PointInfo goalPointInfo, World world, bool collisionCheck = true, float constant = 1.0f);

	/// <summary>
	/// Completes the path from the current point denoted by curPointInfo to the goal point denoted by goalPointInfo
	/// respecting the constraints of the given motion model.
	/// </summary>
	public abstract List<PointInfo> completePath (PointInfo curPointInfo, PointInfo goalPointInfo, World world, bool collisionCheck);

	public float getDt() {return dt;}

	public BaseModel(float maxVel, float dt) {
		this.maxVelocity = maxVel;
		this.dt = dt;
	}

}
