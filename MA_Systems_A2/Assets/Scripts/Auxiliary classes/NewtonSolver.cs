using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewtonSolver {

	public static float[] eval(float[] a, float t) {
		// f(x) = a0+ a1x + ... + anxn
		int n = a.Length - 1;
		List<float> b = new List<float>();
		List<float> c = new List<float>();

		for (int i = 0; i <= n; i++) {
			b.Add(0);
			c.Add(0);
		}

		b[n] = a[n];
		c[n] = b[n];
		for (int k = n-1; k >= 1; k--) {
			b[k] = a[k] + t*b[k+1];
			c[k] = b[k] + t*c[k+1];
		}
		b[0] = a[0] + t*b[1];

		float[] result = new float[2];
		result [0] = b [0];
		result [1] = c [1];

		return result;
	}

	// Simple Newton
	public static float Newton(float[] coeff, float x0, int max_n) {
		float x_prev;
		for (var i = 0; i < max_n; i++) {
			x_prev = x0;
			float[] fdf = eval(coeff, x0);
			x0 = x0 - fdf[0]/fdf[1];

			if (Mathf.Abs (x_prev - x0) < 0.01f) {
				break;
			}
		}
		return x0;
	}
}
