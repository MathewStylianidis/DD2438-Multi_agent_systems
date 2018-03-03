using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/// <summary>
/// Class with several static utility functions
/// </summary>
public class UtilityClass {

	/// <summary>
	/// Turns degrees to a vector showing to that direction.
	/// </summary>
	public static Vector3 rads2Vec(float rads) {
		return new Vector3 (Mathf.Cos(rads), 0f, Mathf.Sin(rads));
	}
	 
}
