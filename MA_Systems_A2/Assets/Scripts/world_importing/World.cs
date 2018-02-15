// Avoid warnings because fields in the TempMap class are seemingly never assigned
#pragma warning disable 649
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Serialization;

[System.Serializable]
public class World {
	public Vector2[] boundingPolygon;
	public List<Vector2[]> obstacles;
	public Vector2[] goalPositions;
	public Vector2[] startPositions;
	public Vector2[] enemyPositions;
	public Vector2[] formationPositions;
	public Vector2[] pointsOfInterest;
	public float sensorRange;

	public VehicleInfo vehicle;

	class TempMap {
		public float[][] bounding_polygon;
		public float[][] obstacle_1;
		public float[][] obstacle_2;
		public float[][] obstacle_3;
		public float[][] obstacle_4;
		public float[][] obstacle_5;
		public float[][] obstacle_6;
		public float[][] obstacle_7;
		public float[][] obstacle_8;
		public float[][] obstacle_9;
		public float[][] obstacle_10;
		public float[][] goal_positions;
		public float[][] start_positions;
		public float[][] enemy_positions;
		public float[][] formation_positions;
		public float[][] points_of_interest;
		public float vehicle_L;
		public float vehicle_a_max;
		public float vehicle_dt;
		public float vehicle_omega_max;
		public float vehicle_phi_max;
		public float vehicle_t;
		public float vehicle_v_max;
		public float sensor_range;
	}

	[System.Serializable]
	public class VehicleInfo {
		public float length;
		public float maxAcceleration;
		public float dt;
		public float maxOmega;
		public float maxPhi;
		public float t;
		public float maxVelocity;
	}

	public static World FromJson (string jsonData) {
		var tmp = TinyJsonDeserializer.Deserialize(jsonData, typeof(TempMap)) as TempMap;
		var map = new World();
		map.goalPositions = (tmp.goal_positions ?? new float[0][]).Select(p => new Vector2(p[0], p[1])).ToArray();
		map.startPositions = (tmp.start_positions ?? new float[0][]).Select(p => new Vector2(p[0], p[1])).ToArray();
		map.enemyPositions = (tmp.enemy_positions ?? new float[0][]).Select(p => new Vector2(p[0], p[1])).ToArray();
		map.formationPositions = (tmp.formation_positions ?? new float[0][]).Select(p => new Vector2(p[0], p[1])).ToArray();
		map.pointsOfInterest = (tmp.points_of_interest ?? new float[0][]).Select(p => new Vector2(p[0], p[1])).ToArray();
		map.boundingPolygon = tmp.bounding_polygon.Select(p => new Vector2(p[0], p[1])).ToArray();
		map.obstacles = new [] {
			tmp.obstacle_1, tmp.obstacle_2, tmp.obstacle_3, tmp.obstacle_4, tmp.obstacle_5,
			tmp.obstacle_6, tmp.obstacle_7, tmp.obstacle_8, tmp.obstacle_9, tmp.obstacle_10, 
		}.Where(x => x != null).Select(o => o.Select(p => new Vector2(p[0], p[1])).ToArray()).ToList();
		map.sensorRange = tmp.sensor_range;
		map.vehicle = new VehicleInfo {
			length = tmp.vehicle_L,
			maxAcceleration = tmp.vehicle_a_max,
			dt = tmp.vehicle_dt,
			maxOmega = tmp.vehicle_omega_max,
			maxPhi = tmp.vehicle_phi_max,
			t = tmp.vehicle_t,
			maxVelocity = tmp.vehicle_v_max
		};
		return map;
	}

	void DrawPolygon (Vector2[] poly, Color color) {
		for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i, i++) {
			Debug.DrawLine(poly[j], poly[i], color);
		}
	}

	/** Checks if \a p is inside the polygon.
	 * \author http://unifycommunity.com/wiki/index.php?title=PolyContainsPoint (Eric5h5)
	 */
	static bool ContainsPoint (Vector2[] polyPoints, Vector2 p) {
		int j = polyPoints.Length-1;
		bool inside = false;
		
		for (int i = 0; i < polyPoints.Length; j = i++) {
			if (((polyPoints[i].y <= p.y && p.y < polyPoints[j].y) || (polyPoints[j].y <= p.y && p.y < polyPoints[i].y)) &&
			    (p.x < (polyPoints[j].x - polyPoints[i].x) * (p.y - polyPoints[i].y) / (polyPoints[j].y - polyPoints[i].y) + polyPoints[i].x))
				inside = !inside;
		}
		return inside;
	}

	public bool IsPointInsideObstacle (Vector2 p) {
		bool inside = !ContainsPoint(boundingPolygon, p);
		for (int i = 0; i < obstacles.Count; i++) {
			inside |= ContainsPoint(obstacles[i], p);
		}
		return inside;
	}

	static Color ObstacleColor = new Color(228/255f,26/255f,28/255f);
	static Color StartColor = new Color(77/255f,175/255f,74/255f);
	static Color GoalColor = new Color(228/255f,26/255f,28/255f);
	static Color EnemyColor = new Color(228/255f,26/255f,28/255f);
	static Color InterestingColor = new Color(152/255f,78/255f,163/255f);

	public void DebugDraw () {
		if (boundingPolygon == null || obstacles == null) return;

		DrawPolygon(boundingPolygon, ObstacleColor);
		for (int i = 0; i < obstacles.Count; i++) DrawPolygon(obstacles[i], ObstacleColor);

		foreach (var p in startPositions) Draw.Debug.CircleXY(p, 0.1f, StartColor);
		foreach (var p in goalPositions) Draw.Debug.CircleXY(p, 0.1f, GoalColor);
		foreach (var p in enemyPositions) Draw.Debug.CircleXY(p, 0.1f, EnemyColor);
		foreach (var p in pointsOfInterest) Draw.Debug.CircleXY(p, 0.1f, InterestingColor);
	}
}
