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
		
	/// <summary>
	/// Checks if (x,y) is inside the polygon given by <vs>, assuming the polygon is convex.
	/// </summary>
	static public bool insidePolygon(float x, float y, Vector2[] vs, double tol = 1e-7) {
		// ray-casting algorithm
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

	/// <summary>
	/// Given the obstacles of the enviroment checks if a point is within any of those obstacles.
	/// </summary>
	static public bool insideObstacle(float x, float y,  List<Vector2[]> obstacles) {
		for(int i = 0; i  < obstacles.Count; i++)
			if(insidePolygon(x, y, obstacles[i]))
				return true;
		return false;
	}


	/// <summary>
	/// Odd-even method for checking if point is inside polygon.
	/// Works for all the types of polygons but is more computationally expensive.
	/// </summary>
	static public bool insidePolygon2(float x, float y, Vector2[] poly) {
		List<float[]> vertices = new List<float[]> ();
		int vertexCount = poly.Length;
		float[] p = { x, y };
		for (int i = 0; i < vertexCount; i++) {
			vertices.Add (new float[2]);
			vertices [i] [0] = poly [i].x;
			vertices [i] [1] = poly [i].y;
		}

		if (vertexCount < 3)
			return false; // Not a polygon

		return isOdd (vertices, p, vertexCount);
		
	}


	static private bool isOdd(List<float[]> vertices, float[] point, int vertexCount, float xMax = 100000.0f) {
		float[] extremePoint = {xMax, point[1]};
		int intersectCount = 0;
		int i = 0;

		do {
			int next = (i + 1) % vertexCount;
			if(intersectsSide(vertices[i], vertices[next], point, extremePoint)) {
				if(getOrientation(vertices[i], point, vertices[next]) == 0)
					return onSegment(vertices[i], point, vertices[next]);
				intersectCount++;
			}
			i = next;
			
		} while(i != 0);

		return intersectCount % 2 == 1;
	}

	static private bool intersectsSide(float[] vertex1, float[] vertex2, float[] point, float[] extremePoint) {
		int o1, o2, o3, o4;
		o1 = getOrientation (vertex1, vertex2, point);
		o2 = getOrientation (vertex1, vertex2, extremePoint);
		o3 = getOrientation (point, extremePoint, vertex1);
		o4 = getOrientation (point, extremePoint, vertex2);
		if (o1 != o2 && o3 != o4)
			return true;
		if (o1 == 0 && onSegment (vertex1, point, vertex2))
			return true;
		if (o2 == 0 && onSegment (vertex1, extremePoint, vertex2))
			return true;
		if (o3 == 0 && onSegment (point, vertex1, extremePoint))
			return true;
		if (o4 == 0 && onSegment (point, vertex2, extremePoint))
			return true;
		return false;
	}

	static private int getOrientation(float[] vertex1, float[] vertex2, float[] vertex3) {
		float orientation = (vertex2 [1] - vertex1 [1]) * (vertex3 [0] - vertex2 [0]) -
		                    (vertex2 [0] - vertex1 [0]) * (vertex3 [1] - vertex2 [1]);
		if (orientation == 0f)
			return 0;
		else if (orientation > 0f)
			return 1;
		else
			return 2;
	}

	static private bool onSegment(float[] vertex1, float[] vertex2, float[] vertex3) {
		if (vertex2 [0] <= Mathf.Max (vertex1 [0], vertex3 [0]) && vertex2 [0] >= Mathf.Min (vertex1 [0], vertex3 [0])
		   && vertex2 [1] <= Mathf.Max (vertex1 [1], vertex3 [1]) && vertex2 [1] >= Mathf.Min (vertex1 [1], vertex3 [1]))
			return true;
		return false;
	}
}
