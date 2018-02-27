using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Raycasting {

	// v11 -> v12 & v21 -> v22
	static public bool segmentsIntersect(Vector2 v11, Vector2 v12, Vector2 v21, Vector2 v22) {

		var det = (v12.x - v11.x) * (v22.y - v21.y) - (v22.x - v21.x) * (v12.y - v11.y);
		if (det == 0) {
			return false;
		}

		var lambda = ((v22.y - v21.y) * (v22.x - v11.x) + (v21.x - v22.x) * (v22.y - v11.y)) / det;
		var gamma = ((v11.y - v12.y) * (v22.x - v11.x) + (v12.x - v11.x) * (v22.y - v11.y)) / det;
		return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
	}

	private float dist(Vector2 v1, Vector2 v2) {
		return Mathf.Sqrt ((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y));
	}

	// Point (x,y) is inside polygon vs
	static public bool insidePolygon(float x, float y, Vector2[] vs) {
		// ray-casting algorithm

		double tol = 1e-7;
		var inside = false;
		var j = vs.Length - 1;
		for (var i = 0; i < vs.Length; i++) {
			var xi = vs [i].x;
			var yi = vs [i].y;
			var xj = vs [j].x;
			var yj = vs [j].y;

			// On an edge
			if (Mathf.Abs ((xi + xj) / 2 - x) < tol && Mathf.Abs ((yi + yj) / 2 - y) < tol) {
				return false;
			}

			var intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
			if (intersect) {
				inside = !inside;
			}
			j = i;
		}

		return inside;
	}


}
