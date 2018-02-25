using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Visualizer : MonoBehaviour {

	public static float widthMultiplier = 0.1f;

	/// <summary>
	/// Visualizes the VRP solution
	/// </summary>
	public static void visualizeVRPsolution(List<Vector2> graphVertices, List<int> solution, int vehicles, int obstacleVertexCount) {
		int count = 0;
		for (int i = 0; i < solution.Count - 1; i++) {
			// If we reached a goal then stop drawing the current vehicle's path
			if (solution[i] >= solution.Count - vehicles)
				continue;
			// Otherwise draw line from node i to i + 1
			GameObject tmp = new GameObject();
			LineRenderer lineRenderer = tmp.AddComponent<LineRenderer> ();
			Vector2 start = graphVertices [obstacleVertexCount + solution [i] - 1];
			Vector2 dest = graphVertices[obstacleVertexCount + solution[i + 1] - 1];
			lineRenderer.widthMultiplier = Visualizer.widthMultiplier;
			lineRenderer.useWorldSpace = true;
			lineRenderer.SetPosition (0, new Vector3(start.x, 0, start.y));
			lineRenderer.SetPosition (1, new Vector3(dest.x, 0, dest.y));
			tmp.transform.SetParent (GameObject.Find("Visualizer").transform);
		}


	}

}
