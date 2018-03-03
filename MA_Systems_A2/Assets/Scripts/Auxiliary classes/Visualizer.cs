using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Visualizer : MonoBehaviour {

	public static float widthMultiplier = 0.1f;

	/// <summary>
	/// Visualizes the VRP solution
	/// </summary>
	public static void visualizeVRPsolution(List<World.VisibilityVertex> graphVertices, List<int> solution, int vehicles, int obstacleVertexCount) {
		int count = 0;

		for (int i = 0; i < solution.Count - 1; i++) {
			// If we reached a goal then stop drawing the current vehicle's path
			if (solution[i] >= graphVertices.Count - vehicles)
				continue;
			
	
			GameObject tmp = new GameObject ();
			LineRenderer lineRenderer = tmp.AddComponent<LineRenderer> ();
			Vector2 start = graphVertices [solution [i]].vertex;
			Vector2 dest = graphVertices [solution [i + 1]].vertex;
			lineRenderer.widthMultiplier = Visualizer.widthMultiplier;
			lineRenderer.useWorldSpace = true;
			lineRenderer.SetPosition (0, new Vector3 (start.x, 0, start.y));
			lineRenderer.SetPosition (1, new Vector3 (dest.x, 0, dest.y));
			tmp.transform.SetParent (GameObject.Find ("Visualizer").transform);

		}


	}


	/// <summary>
	/// Visualizes a trajectory given by x and y
	/// </summary>
	public static void visualizeTrajectory(float[] x, float[] y) {
		Vector3 prevPoint = new Vector3 (x [0], 0, y [0]);
		for (int i = 1; i < x.Length; i++) {
			Vector3 newPoint = new Vector3 (x [i], 0, y [i]);
			GameObject tmp = new GameObject ();
			LineRenderer lineRenderer = tmp.AddComponent<LineRenderer> ();
			lineRenderer.widthMultiplier = Visualizer.widthMultiplier;
			lineRenderer.useWorldSpace = true;
			lineRenderer.SetPosition (0, prevPoint);
			lineRenderer.SetPosition (1, newPoint);
			prevPoint = newPoint;
			tmp.transform.SetParent (GameObject.Find ("Visualizer").transform);
		}
	}

}
