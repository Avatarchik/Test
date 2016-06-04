using UnityEngine;
using System.Collections;

public class MotionPathCreator : MonoBehaviour {

    public int touchIkID;
    public GameObject motionPathPrefeb;
    public Bipedal ownerBip;

    private RaycastHit hit;
    private SplineTrailRenderer trailReference = null;
    private Vector2 startMousePos;
    private Vector2 currMousePos;
    private Vector3 dragAxis;
    private Plane dragPlane;

    private float distance = 0;
    private bool playerSelected = false;
    // Use this for initialization
    void Start() {

    }

    public void startDraw()
    {
        playerSelected = true;
        if (trailReference != null)
        {
            trailReference.Clear();
            Destroy(trailReference.gameObject);
            trailReference = null;
        }
    }

    public void updateDraw(Vector3 inputPos)
    {
      //  if (playerSelected)
     //   {
            if (trailReference == null)
            {
                distance = 0;
                trailReference = ((GameObject)Instantiate(motionPathPrefeb, this.transform.position, Quaternion.identity)).GetComponent<SplineTrailRenderer>();
                trailReference.maxLength = 0;
                /*
                dragAxis = Vector3.Cross(Vector3.up, (Camera.main.transform.position - this.transform.position)).normalized;
                Vector3 n = Vector3.Cross(dragAxis, Vector3.up).normalized;
                dragPlane = new Plane(n, this.transform.position);*/
            }
            else if (trailReference != null)
            {

                trailReference.maxLength = trailReference.spline.Length();
                /*
                float PLDis = 0;
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                dragPlane.Raycast(ray, out PLDis);
                */
                trailReference.transform.position = inputPos;
            }
       // }
    }

    public void endDraw()
    {
       // playerSelected = false;
    }

    // Update is called once per frame
    void LateUpdate () {
        /*
		if (Input.GetMouseButtonDown (0)) {
			startMousePos = Input.mousePosition;

			hit = new RaycastHit ();
			if (!Physics.Raycast (Camera.main.ScreenPointToRay (Input.mousePosition), out hit, Mathf.Infinity))
				return;

			if (hit.collider != this.GetComponent<Collider> ())
				return;

			playerSelected = true;
			if (trailReference != null) {
				trailReference.Clear ();
				Destroy (trailReference.gameObject);
				trailReference = null;
			}
			//ownerBip.getCameraControler().allowRotation = false;
		} else if (Input.GetMouseButton (0) && playerSelected) {
			currMousePos = Input.mousePosition;

			if (trailReference == null && (currMousePos - startMousePos).magnitude > 20) {
				distance = 0;
				trailReference = ((GameObject)Instantiate (motionPathPrefeb, this.transform.position, Quaternion.identity)).GetComponent<SplineTrailRenderer> ();
				trailReference.maxLength = 0;

				dragAxis = Vector3.Cross (Vector3.up, (Camera.main.transform.position - this.transform.position)).normalized;
				Vector3 n = Vector3.Cross (dragAxis, Vector3.up).normalized;
				dragPlane = new Plane (n, this.transform.position);
			} else if (trailReference != null) {
				
				trailReference.maxLength = trailReference.spline.Length(); 

				float PLDis = 0;
				Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
				dragPlane.Raycast(ray, out PLDis);

				trailReference.transform.position = ray.GetPoint(PLDis);
			}
		}else if(Input.GetMouseButtonUp(0)){
			playerSelected = false;
			//ownerBip.getCameraControler().allowRotation = true;
		}*/

		if (trailReference != null && trailReference.maxLength > 0.1f) {
			float length = trailReference.spline.Length (); 
			distance = Mathf.Clamp (distance + GUIManager.instance.followSpeed.value * Time.deltaTime, 0, length);
			trailReference.maxLength = Mathf.Max(length - distance + 0.5f, 0);
			Vector3 tpos = trailReference.spline.FindPositionFromDistance (distance);
			ownerBip.setHandTouchState (HandTouchState.touch, null, tpos, (HandTouchMode)touchIkID);
			//ownerBip.setLookState (HeadLookState.lookTarget, null, tpos);

			if (trailReference.maxLength <= 0.5f) {
				trailReference.Clear ();
				Destroy (trailReference.gameObject);
				trailReference = null;

				ownerBip.setHandTouchState (HandTouchState.touch, null, Vector3.zero, (HandTouchMode)touchIkID);
				//ownerBip.setLookState (HeadLookState.none, null, Vector3.zero);
			}
		}
	}
}
