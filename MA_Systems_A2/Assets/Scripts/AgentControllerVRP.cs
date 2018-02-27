using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentControllerVRP : MonoBehaviour {

	private List<PointInfo> route;

	// Use this for initialization
	void Start () {
		GameObject gameController = GameObject.Find ("GameController");
		if (gameController != null) {
			WorldController worldController = gameController.GetComponent<WorldController> ();
			if (worldController != null) {
				route = worldController.getRoute (this.transform.GetSiblingIndex());
			}
		}
	}
	
	// Update is called once per frame
	void Update () {
		
	}
}
