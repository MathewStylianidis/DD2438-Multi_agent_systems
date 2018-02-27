using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointInfo {
	public Vector3 pos;
	public Vector3 vel;
	public Vector3 orientation;
	public float currentTime;

	public PointInfo(Vector3 pos, Vector3 vel, Vector3 orientation, float currentTime) {
		this.pos = pos;
		this.vel = vel;
		this.orientation = orientation;
		this.currentTime = currentTime;
	}
}