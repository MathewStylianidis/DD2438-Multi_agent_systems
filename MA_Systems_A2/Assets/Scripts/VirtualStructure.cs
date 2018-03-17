using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualStructure : BaseFormationController {

	public static Vector2 getCenter(Vector2[] agentsPos) {
		// Return null if there are less than 2 agents in the structure
		if (agentsPos.Length < 2)
			throw new System.Exception("agentsPos parameter should have more than 1 element.");
		
		float xMax, xMin, yMax, yMin;
		xMax = yMax = float.MinValue;
		xMin = yMin = float.MaxValue;
		for (int i = 0; i < agentsPos.Length; i++) {
			if (agentsPos [i].x < xMin)
				xMin = agentsPos [i].x;
			else if (agentsPos [i].x > xMax)
				xMax = agentsPos [i].x;
			if (agentsPos [i].y < yMin)
				yMin = agentsPos [i].y;
			else if (agentsPos [i].y > yMax)
				yMax = agentsPos [i].y;
		}
		return new Vector2 ((xMax + xMin) / 2, (yMax + yMin) / 2);
	}
}
