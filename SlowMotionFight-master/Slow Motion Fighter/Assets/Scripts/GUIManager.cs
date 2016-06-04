using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class GUIManager : MonoBehaviour {

	public Slider timeScale;
	public Text timeScaleValue;
	public Slider followSpeed;
	public Text followSpeedValue;

	public static GUIManager instance;

	void Awake(){
		instance = this;
	}

	// Use this for initialization
	void Start () {
		Time.timeScale = timeScale.value;
		timeScaleValue.text = "timeScale = " + timeScale.value;
		followSpeedValue.text = "followSpeed = " + followSpeed.value;
	}
	
	// Update is called once per frame
	void Update () {
	}

	public void onTimeScaleChanged(){
		Time.timeScale = timeScale.value;
		if (Time.timeScale > 0.2f) {
			Time.fixedDeltaTime = 0.01f;
		} else {
			Time.fixedDeltaTime = 0.04F * timeScale.value;
		}
		timeScaleValue.text = "timeScale = " + timeScale.value;
	}

	public void onFollowSpeedChanged(){
		followSpeedValue.text = "followSpeed = " + followSpeed.value;
	}
}
