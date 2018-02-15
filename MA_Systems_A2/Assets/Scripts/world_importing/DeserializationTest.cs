using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DeserializationTest : MonoBehaviour {
	public TextAsset data;
	World world;

	// Use this for initialization
	void Start () {
		world = World.FromJson(data.text);
	}
	
	// Update is called once per frame
	void Update () {
		
	}

	public void OnDrawGizmos () {
		if (world != null) world.DebugDraw();
	}
}
