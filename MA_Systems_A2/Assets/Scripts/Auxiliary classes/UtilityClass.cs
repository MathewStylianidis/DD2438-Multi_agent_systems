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

	/// <summary>
	/// Gets matrix for rotation of <rads> around x
	/// </summary>
	public static float[][] getRotationYMatrix(float rads) {
		float[][] rotMatrix = new float[2][];
		for (int i = 0; i < 2; i++)
			rotMatrix [i] = new float[2];
		rotMatrix [0] [0] = rotMatrix [1] [1] = Mathf.Cos (rads);
		rotMatrix [1] [0] = Mathf.Sin (rads);
		rotMatrix [0] [1] = -Mathf.Sin (rads);
		return rotMatrix;		
	}

	/// <summary>
	/// Rotates Vector2 given a rotation matrix
	/// </summary>
	public static Vector2 rotateVector(float[][] rotMatrix, Vector2 vector) {
		Vector2 rotatedVector = new Vector2 (0f, 0f);
		rotatedVector.x = rotMatrix [0] [0] * vector.x + rotMatrix [0] [1] * vector.y;
		rotatedVector.y = rotMatrix [1] [0] * vector.x + rotMatrix [1] [1] * vector.y;
		return rotatedVector;
	}


    public static void RotateRight(IList sequence, int count)
    {
        object tmp = sequence[count - 1];
        sequence.RemoveAt(count - 1);
        sequence.Insert(0, tmp);
    }

    public static IEnumerable<IList> Permutate(IList sequence, int count)
    {
        if (count == 1) yield return sequence;
        else
        {
            for (int i = 0; i < count; i++)
            {
                foreach (var perm in Permutate(sequence, count - 1))
                    yield return perm;
                RotateRight(sequence, count);
            }
        }
    }

}
