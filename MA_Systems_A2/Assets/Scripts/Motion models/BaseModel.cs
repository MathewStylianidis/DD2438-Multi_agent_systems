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
	/// Gets the coordinates of a trivial path from curPointInfo and goalPoint 
	/// (I.e. straight line for kinematic point model)
	/// </summary>
	public abstract List<PointInfo> completePath (PointInfo curPointInfo, PointInfo goalPointInfo, World world, bool collisionCheck, float constant = 1.0f);

	public float getDt() {return dt;}

	public BaseModel(float maxVel, float dt) {
		this.maxVelocity = maxVel;
		this.dt = dt;
	}

}
