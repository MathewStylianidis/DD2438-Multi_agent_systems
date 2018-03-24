using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VisibilityGraph {

	//public 

	public static void initVisibilityGraph (World world, IEnumerable<Vector2> additionalPoints = null) {

		var vertices = new List<World.VisibilityVertex> ();
		var obstEdges = new List<Segment> ();

		for (var i = 0; i < world.obstacles.Count; i++) {
			var obstacle = world.obstacles[i];
			var prevVertex = obstacle [obstacle.Length - 1];
			foreach (var obstVertex in obstacle) {
				obstEdges.Add (new Segment (prevVertex, obstVertex));
				vertices.Add (new World.VisibilityVertex (obstVertex, true, i));
				prevVertex = obstVertex;
			}
		}

		if (additionalPoints != null) {
			foreach (var point in additionalPoints) {
				vertices.Add(new World.VisibilityVertex (point, false));
			}
		}

		var visibilityGraph = new float[vertices.Count][];
		for (var i = 0; i < vertices.Count; i++) {
			var v1 = vertices [i];
			visibilityGraph [i] = new float[vertices.Count];
			for (var j = 0; j < vertices.Count; j++) {
				if (i == j) {
					visibilityGraph [i] [j] = 0;
					continue;
				}

				var v2 = vertices [j];

				var visible = true;

				// Check if the two vertices are of the same polygon and their middle point is inside the polygon
				// (this case is not handled by the intersection checking below)
				if (v1.obstacleVertex && v2.obstacleVertex && v1.obstacleIndex == v2.obstacleIndex &&
					Raycasting.insidePolygon ((v1.vertex.x + v2.vertex.x) / 2, (v1.vertex.y + v2.vertex.y) / 2, world.obstacles [v1.obstacleIndex])) {
					visible = false;
				} else {
					for (var k = 0; k < obstEdges.Count; k++) {
						if (Raycasting.segmentsIntersect (v1.vertex, v2.vertex, obstEdges [k].vertex1, obstEdges [k].vertex2)) {
							visible = false;
							break;
						}
					}
				}

				if (visible) {
					visibilityGraph [i] [j] = Vector2.Distance(v1.vertex, v2.vertex);
				} else {
					visibilityGraph [i] [j] = float.MaxValue;
				}
			}
		}

		world.visibilityGraph = visibilityGraph;
		world.graphVertices = vertices;
		world.obstacleEdges = obstEdges;

	}


	public class Segment {
		public readonly Vector2 vertex1;
		public readonly Vector2 vertex2;

		public Segment(Vector2 vertex1, Vector2 vertex2) {
			this.vertex1 = vertex1;
			this.vertex2 = vertex2;
		}
	}
}
