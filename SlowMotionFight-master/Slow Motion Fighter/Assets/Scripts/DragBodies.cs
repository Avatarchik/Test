using UnityEngine;
using System.Collections;
using CartWheelCore;

public class DragBodies : MonoBehaviour {
	
	public float spring = 600.0f;
	public float damper = 20.0f;
	public float distance = 1;
	public float drag = 10.0f;
	public float angularDrag = 5.0f;
	public Material lineMat;
	
	private Bipedal bip = null;
	private RaycastHit[] hit;
	private CWRigidBody[] currDragBody;
	private GameObject[] go;
	private LineRenderer[] line;
	private SpringJoint[] springJoint;
	private Vector3[] springJointStartPos;
	private Vector3[] dragStartPosition;
	private Vector3[] dragEndPosition;
	private float[] currDragLen;
	private float[] dragMoveSpeed;
	private int[] dragTouchID;
	
	private Vector2[] startMousePos;
	private Vector2[] oldMousePos;
	private Vector2[] currMousePos;

	private int dragNum = 0;
	private Vector3 dragAxis;
	private Plane dragPlane;
	private int layerMask;

	// Use this for initialization
	void Awake () {
		bip = gameObject.GetComponent(typeof(Bipedal)) as Bipedal;
		
		hit = new RaycastHit[2];
		go = new GameObject[2];
		go[0] = null;
		go[1] = null;
		line = new LineRenderer[2];
		springJoint = new SpringJoint[2];
		currDragBody = new CWRigidBody[2];
		springJointStartPos = new Vector3[2];
		dragStartPosition = new Vector3[2];
		dragEndPosition = new Vector3[2];
		currDragLen = new float[2];
		dragMoveSpeed = new float[2];
		dragTouchID = new int[2];
		dragTouchID[0] = -1;
		dragTouchID[1] = -1;
		
		startMousePos = new Vector2[2];
		oldMousePos = new Vector2[2];
		currMousePos = new Vector2[2];
		currMousePos[0] = Vector3.zero;
		currMousePos[1] = Vector3.zero;

		layerMask = ~(1 << (int)ColliderLayer.ignoreAllLayer | 1 << (int)ColliderLayer.ignoreRaycastLayer);
	}
	
	void LateUpdate () {
		//if(RFGameManager.instance.getGameState() != RFGameState.idle) return;

		//if(bip.getIsMoveing()) return;
		//if (Input.touchCount >2)
		//return;

		currDragLen[0] = 0;
		currDragLen[1] = 0;
		dragMoveSpeed[0] = 0;
		dragMoveSpeed[1] = 0;
		//foreach(Touch touch in Input.touches){
			//int i = touch.fingerId;
			int i = 0;
			//if(i < 0 || i >= 2) continue;
			
			if(Input.GetMouseButtonDown(0)){
				dragTouchID[i] = -1;
				startMousePos[i] = Input.mousePosition;
				
				hit[i] = new RaycastHit();
			if (!Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition),  out hit[i], Mathf.Infinity, layerMask))
					//continue;
					return;
				
				if (!hit[i].rigidbody || hit[i].rigidbody.isKinematic || !hit[i].rigidbody.gameObject.transform.IsChildOf(this.transform))
					//continue;
					return;
				
				currDragBody[i] = hit[i].rigidbody.GetComponent("CWRigidBody") as CWRigidBody;
				if(currDragBody[i] == null) 
					//continue;
					return;

			//if(hit[i].point.y > 95)
				//return;

				//Debug.Log(i +" dddddddddddddd "+currDragBody[i].name);
				dragTouchID[i] = i;
				springJointStartPos[i] = currDragBody[i].getLocalCoordinatesPoint(hit[i].point);

			}else if(Input.GetMouseButton(0)){
			//}else if(touch.phase == TouchPhase.Moved || touch.phase == TouchPhase.Stationary){
				oldMousePos[i] = currMousePos[i];
					currMousePos[i] = Input.mousePosition;
				dragMoveSpeed[i] = (oldMousePos[i] - currMousePos[i]).magnitude;
				
				if(dragTouchID[i] < 0)
					//continue;
					return;
				
				if (go[i] == null && (currMousePos[i] - startMousePos[i]).magnitude > 15){
					go[i] = new GameObject("Rigidbody dragger");
					Rigidbody body = go[i].AddComponent <Rigidbody>() as Rigidbody;
					body.isKinematic = true;
					springJoint[i] = go[i].AddComponent<SpringJoint>() as SpringJoint;
					springJoint[i].transform.position = hit[i].point;
					springJoint[i].anchor = Vector3.zero;
					springJoint[i].spring = spring;
					springJoint[i].damper = damper;
					springJoint[i].maxDistance = distance;
					springJoint[i].connectedBody = hit[i].rigidbody;
					springJoint[i].connectedBody.drag = drag;
					springJoint[i].connectedBody.angularDrag = angularDrag;
					
					line[i] = go[i].AddComponent<LineRenderer>() as LineRenderer;
					line[i].material = lineMat;
					line[i].SetVertexCount(2);
					line[i].SetWidth(0.1f, 0.1f);
					line[i].SetColors(Color.yellow, Color.yellow);
					line[i].SetPosition(0, hit[i].point);
					line[i].SetPosition(1, hit[i].point);

					dragNum += 1;
					if((dragNum < 2 && dragTouchID[i] >= 0) || (dragNum > 1 && dragTouchID[i] == 1)){
						dragAxis = Vector3.Cross(SimGlobals.upAxis, (Camera.main.transform.position - hit[i].point)).normalized;
						Vector3 n = Vector3.Cross(dragAxis, SimGlobals.upAxis).normalized;
						dragPlane = new Plane(n, hit[i].point);
					}
					
				}else if (go[i] != null){
					dragStartPosition[i] = currDragBody[i].getWorldCoordinatesPoint(springJointStartPos[i]);

					float PLDis = 0;
					Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
					dragPlane.Raycast(ray, out PLDis);
					
					dragEndPosition[i] = ray.GetPoint(PLDis);
					//dragEndPosition[i] = ray.GetPoint(hit[i].distance);
					//if(dragEndPosition[i].y > 95){
						//dragEndPosition[i].y = 95;
					//}
					springJoint[i].transform.position = dragEndPosition[i];
					line[i].SetPosition(0, dragStartPosition[i]);
					line[i].SetPosition(1, dragEndPosition[i]);
					
					currDragLen[i] = (dragStartPosition[i] - dragEndPosition[i]).magnitude;

					bip.setIsDragging(true, !currDragBody[i].name.Contains("Arm"));
					bip.getCameraControler().allowRotation = false;
				}
			}else if(Input.GetMouseButtonUp(0)){
				if(go[i] != null){
					springJoint[i].connectedBody.drag = currDragBody[i].getDrag();
					springJoint[i].connectedBody.angularDrag = currDragBody[i].getAngularDrag();
					springJoint[i].connectedBody = null;
					springJoint[i] = null;
					GameObject.Destroy(go[i]);
					go[i] = null;
					dragTouchID[i] = -1;
					dragNum -= 1;
					//Debug.Log("aaaaaaaa "+i+" aaa "+Input.touchCount);
					
					bip.setHandTouchState(HandTouchState.touch, null, dragStartPosition[i], HandTouchMode.autoHand);
					
					if(go[0] == null && go[1] == null){
					bip.getCameraControler().allowRotation = true;
						bip.setIsDragging(false, true);
						dragNum = 0;
						dragTouchID[0] = -1;
						dragTouchID[1] = -1;
					}
				}
			}
		//}
	}
	
	public CWRigidBody getCurrDragBody(int id){
		return currDragBody[id];
	}
	public Vector3 getSpringJointStartPos(int id){
		return springJointStartPos[id];
	}
	public Vector3 getDragStartPosition(int id){
		return dragStartPosition[id];
	}
	public Vector3 getDragEndPosition(int id){
		return dragEndPosition[id];
	}
	
	public float getCurrDragLen(int id){
		return currDragLen[id];
	}
	public float getDragMoveSpeed(int id){
		return dragMoveSpeed[id];
	}
	
	public int getMaxDragMoveSpeedID(){
		if(dragMoveSpeed[0] >= dragMoveSpeed[1]){
			return 0;
		}else{
			return 1;
		}
	}
	public int getMinDragMoveSpeedID(){
		if(dragMoveSpeed[0] <= dragMoveSpeed[1]){
			return 0;
		}else{
			return 1;
		}
	}
	
	public int[] getDragTouchID(){
		return dragTouchID;
	}
	
	public Plane getDragPlane(){
		return dragPlane;
	}
}
