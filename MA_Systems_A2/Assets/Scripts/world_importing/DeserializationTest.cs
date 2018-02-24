using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DeserializationTest : MonoBehaviour {
	public TextAsset data;
	public TextAsset data2;
	World world;

	// Use this for initialization
	void Start () {
		world = World.FromJson(data.text, data2.text);
	}
	
	// Update is called once per frame
	void Update () {
		
	}

	public void OnDrawGizmos () {
		if (world != null) world.DebugDraw();
	}
}
