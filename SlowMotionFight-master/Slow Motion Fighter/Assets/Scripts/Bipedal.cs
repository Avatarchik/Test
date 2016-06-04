using UnityEngine;
using System.Collections;
using CartWheelCore;
using CartWheelCore.Control;
using CartWheelCore.Math;
using RootMotion.FinalIK;
using PhatRobit;

public enum ColliderArea{
	none = 0,
	outArea = 1,
	nearArea = 2,
	farArea = 3
}

public enum HeadLookState{
	none = 0,
	idle = 1,
	lookTarget = 2
}

public enum HandTouchState{
	none = 0,
	touch = 1,
	scratch = 2,
	resist = 3,
	hit = 4,
	grab = 5
}

public enum HandTouchMode{
	leftHand = 0,
	rightHand = 1,
	doubleHand = 2,
	autoHand = 3,
	autoDoubleHand = 4,
	leftFoot = 5,
	rightFoot = 6,
	spine = 7
}

public enum ClimbLadderState{
	none = 0,
	startToClimbStand = 1,
	climbStand = 2,
	climbUp = 3,
	climbDown = 4,
	climbUpEnd1 = 5,
	climbUpEnd2 = 6,
	climbDownStart1 = 7,
	climbDownStart2 = 8,
	climbDownStart3 = 9,
	climbDownStart4 = 10,
	climbDownStart5 = 11,
	climbUpToStand = 31,
	climbDownToStand = 41
}

public enum DefenseWalkDirection{
	none = 0,
	defense_stop = 1,
	defense_forward = 2,
	defense_backward = 3,
	defense_left = 4,
	defense_right = 5,
	walk_forward = 6,
	walk_back = 7
}

public enum BipedalActions
{
	standard = 0,
	stand_normal = 1,
	forward_walk = 2,
	forward_run = 3,
	forward_somersault = 4,
	jump = 5,
	back_walk = 6,
	stand_defense = 7,
	punch1 = 8,
	punch2 = 9,
	punch3 = 10,
	punch4 = 11,
	kick1 = 12,
	kick2 = 13,
	kick3 = 14,
	forward_jump = 15,
	standup1 = 16,
	standup2 = 17,
	standup3 = 18,
	standup4 = 19,
	fall_down = 20,
	shin_up = 21,
	climb_stand = 22,
	climb_up = 23,
	climb_down = 24,
	climb_up_end1 = 25,
	climb_up_end2 = 26,
	climb_down_start1 = 27,
	climb_down_start2 = 28,
	climb_down_start3 = 29,
	hang_stand = 30,
	hang_up = 31,
	hang_down = 32,
	hang_swing_front = 33,
	hang_swing_back = 34,
	hang_to_jump = 35,
	falling = 36,
	touch_down1 = 37,
	defense_walk = 38,
	free_walk = 39,
	hurt = 40,
	inWaterStand = 41,
	inWaterSwiming = 42,
	weaponHit1 = 43,
	weaponHit2 = 44,
	weaponHit3 = 45
}

public class Bipedal : MonoBehaviour {

	public TextAsset actionDataFile;
	public string type;
	public Vector3 leftAxis;
	public Vector3 upAxis;
	public Vector3 frontAxis;

	public BipedIK bipedIK;
	public NavMeshAgent agent;
	public GameObject nearCollider;

	public GameObject pelvis;
	public GameObject waist;
	public GameObject torso;
	public GameObject head;
	public GameObject lUpperArm;
	public GameObject lLowerArm;
	public GameObject rUpperArm;
	public GameObject rLowerArm;
	public GameObject lUpperLeg;
	public GameObject lLowerLeg;
	public GameObject rUpperLeg;
	public GameObject rLowerLeg;
	public GameObject lFoot;
	public GameObject rFoot;
	public GameObject lHand;
	public GameObject rHand;

	//[HideInInspector]public ColdWeapon weapon = null;
	//[HideInInspector]public Shoes lShoes = null;
	//[HideInInspector]public Shoes rShoes = null;
	public AudioClip[] bipedalSounds;
	
	private GameObject targetObj;

	protected AudioSource lHandSoundSource;
	protected AudioSource rHandSoundSource;
	protected AudioSource lFootSoundSource;
	protected AudioSource rFootSoundSource;
	protected AudioSource headSoundSource;
		
	protected ActionEditor actionEditor = null;
	
	protected Character character;
	protected IKVMCController controller;
	protected BehaviourController behaviour;
	protected bool characterIsReady;

	protected HeadLookState lookState;
	protected GameObject lookObject;
	protected Vector3 lookLocalPos;
	protected HandTouchState handTouchState;
	protected GameObject leftHandTouchObject;
	protected GameObject rightHandTouchObject;
	protected Vector3 leftHandTouchLocalPos;
	protected Vector3 rightHandTouchLocalPos;

	protected GameObject leftFootTouchObject;
	protected GameObject rightFootTouchObject;
	protected Vector3 leftFootTouchLocalPos;
	protected Vector3 rightFootTouchLocalPos;
	protected GameObject spineTouchObject;
	protected Vector3 spineTouchLocalPos;
	
	protected NavMeshPath agentPath;
	protected DragBodies dragBody;
	protected Camera currCamera;
	//protected MobileSimpleRpgCamera cameraControler;
	protected SimpleRpgCamera cameraControler;
	
	protected HitFeedback hurtFeedback = null;
	protected ArrayList pickerTargets;
	protected GameObject currentPickTarget = null;
	protected int currentPickHandCount = 0;
	protected ClimbLadderState climbState = ClimbLadderState.none;
	protected DefenseWalkDirection walkDirection = DefenseWalkDirection.walk_forward;

	protected bool isDead = false;
	private GameObject walkToAimTarget = null;
	private Vector3 walkToAimPosition;

	protected GameObject weaponHitTarget = null;
	protected Vector3 weaponHitPos = Vector3.zero;
	protected int weaponHitRandNum = 0;

	private float startInAirTimer;
	private float rootIdleTimer = 0;
	protected bool isIndojo = true;
	protected bool isFallDown = false;
	protected bool isStanding = false;
	protected bool isStableStanding = false;
	protected bool isMoveing = false;
	protected bool isJumping = false;
	protected bool isFighting = false;
	protected bool isDefense = false;
	protected bool isHurting = false;
	protected bool isInWater = false;
	protected bool isReaching = false;
	protected bool isPicking = false;
	protected bool isUpdatePickerPosition = false;
	protected bool isWalkToAim = false;
	protected bool isInAir = false;
	protected bool isDragging = false;
	protected bool isVigilance = false;
	protected bool isIgnoreCollisionWithOthers = false;

	private int currentPathIndex = 0;
	private Vector3 currentStartPosition = Vector3.zero;
	protected float life = 100;

	private FullBodyBipedIK fbbik;
	private AimIK aimik;
	private LookAtIK lookik;

	private SimBiData actionData;

	private float lookTargetOffsetRange = 0.5f;
	private float backToIdleTimer = 0;
	private float lookDragTimer = 0;
	private float standUpTimer = 0;
	private float startFallDownTimer = 0;
	private float fightingTimer = 0;
	private float hurtTimer = 0;
	private float addExpTimer = 0;
	private float foodTouchGroundTimer = 0;
	private float leftFoodTouchGroundTimer = 0;
	private float rightFoodTouchGroundTimer = 0;
	private float leftResistTimer = 0;
	private float rightResistTimer = 0;
	private float leftResistMaxTimer = 0;
	private float rightResistMaxTimer = 0;
	private bool startLeftResist = false;
	private bool startRightResist = false;
	private float normalStepTimer = 0;
	private float belenceErrorTimer = 0;

	private bool isAvoidMove = false;
	private bool isLeftTouchRay = false;
	private bool isRightTouchRay = false;
	private bool isDoubleTouchRay = false;
	private int walkAvoidSign = 0;
	
	// Use this for initialization
	void Awake () {

		lHandSoundSource = lHand.GetComponent<AudioSource>();
		rHandSoundSource = rHand.GetComponent<AudioSource>();
		lFootSoundSource = lFoot.GetComponent<AudioSource>();
		rFootSoundSource = rFoot.GetComponent<AudioSource>();
		headSoundSource = head.GetComponent<AudioSource>();

		targetObj = GameObject.Find("TargetCube") as GameObject;

		actionEditor = this.GetComponent("ActionEditor") as ActionEditor;

		fbbik = GetComponent<FullBodyBipedIK>();
		lookik = GetComponent<LookAtIK>();
		aimik = GetComponent<AimIK>();

		lookState = HeadLookState.none;
		lookObject = null;
		lookLocalPos = Vector3.zero;
		handTouchState = HandTouchState.none;
		leftHandTouchObject = null;
		rightHandTouchObject = null;
		leftHandTouchLocalPos = Vector3.zero;
		rightHandTouchLocalPos = Vector3.zero;

		leftFootTouchObject = null;
		rightFootTouchObject = null;
		leftFootTouchLocalPos = Vector3.zero;
		rightFootTouchLocalPos = Vector3.zero;

		spineTouchObject = null;
		spineTouchLocalPos = Vector3.zero;
		
		dragBody = this.GetComponent("DragBodies") as DragBodies;
		
		currCamera = Camera.main;
		cameraControler = currCamera.GetComponent<SimpleRpgCamera>();
		
		SimGlobals.upAxis = -1*Physics.gravity.normalized;
		
		pickerTargets = new ArrayList();
		characterIsReady = false;

		if (agent != null) {
			//agent.updatePosition = false;
			//agent.updateRotation = false;
			agentPath = new NavMeshPath();
		}
	}

	void Start(){
		if(actionDataFile != null){
			actionData = new SimBiData();
			actionData.onDataLoadFinished += new SimBiData.DataLoaderHandler(this.ResponseLoadFinishedEvent);
			actionData.loadFromFile(actionDataFile.text);
		}
	}
	
	private void ResponseLoadFinishedEvent(int index){
		initCharacter(actionData);
	}
	
	private void characterInitState(){
		controller.gotoState(0);
	}
	
	virtual protected void startCharacter(){
	}
	
	public void initCharacter(SimBiData statesData){
		character = new Character();
		character.ownerObject = this.gameObject;
		character.setName("Bipedal");
		character.setRoot(pelvis.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(waist.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(torso.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(head.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(lUpperArm.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(rUpperArm.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(lLowerArm.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(rLowerArm.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(lUpperLeg.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(rUpperLeg.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(lLowerLeg.GetComponent("CWRigidBody") as CWRigidBody);
		character.addArticulatedRigidBody(rLowerLeg.GetComponent("CWRigidBody") as CWRigidBody);
		if(lFoot != null)character.addArticulatedRigidBody(lFoot.GetComponent("CWRigidBody") as CWRigidBody);
		if(rFoot != null)character.addArticulatedRigidBody(rFoot.GetComponent("CWRigidBody") as CWRigidBody);
		if(lHand != null)character.addArticulatedRigidBody(lHand.GetComponent("CWRigidBody") as CWRigidBody);
		if(rHand != null)character.addArticulatedRigidBody(rHand.GetComponent("CWRigidBody") as CWRigidBody);
		character.computeMass();
		character.setLocalAxis(leftAxis, upAxis, frontAxis);

		//Physics.IgnoreCollision(lLowerLeg.GetComponent<Collider>(), rLowerLeg.GetComponent<Collider>());
		//Physics.IgnoreCollision(lUpperLeg.GetComponent<Collider>(), rUpperLeg.GetComponent<Collider>());
		//Physics.IgnoreCollision(lLowerLeg.GetComponent<Collider>(), rUpperLeg.GetComponent<Collider>());
		//Physics.IgnoreCollision(rLowerLeg.GetComponent<Collider>(), lUpperLeg.GetComponent<Collider>());

		for(int i=0;i<character.getArticulatedRigidBodyCount();i++){
			character.getArticulatedRigidBody(i).onCollisionEnterHandler += new CWRigidBody.CollisionFeedbackHandler(this.ResponseHitLimbsFeedback);
			for(int j=0;j<character.getArticulatedRigidBodyCount();j++){
				if(character.getArticulatedRigidBody(i)!=character.getArticulatedRigidBody(j) && character.getArticulatedRigidBody(i).gameObject.GetComponent<Collider>() != null && character.getArticulatedRigidBody(j).gameObject.GetComponent<Collider>() != null){
					Physics.IgnoreCollision(character.getArticulatedRigidBody(i).gameObject.GetComponent<Collider>(), character.getArticulatedRigidBody(j).gameObject.GetComponent<Collider>());
				}
			}
		}
		//ignoreCollisionTrunkWithLowerArm(false);

		controller = new IKVMCController(character);
		controller.setStatesData(statesData);
		controller.bipedIK = bipedIK;

		if(actionEditor!=null){
			actionEditor.setController(controller);
		}
		
		controller.onStartNewState += new IKVMCController.TransitionToStateHandler(this.ResponseStartNewStateEvent);
		controller.onCurrentStateEnded += new IKVMCController.TransitionToStateHandler(this.ResponseStateEndedEvent);
		controller.onHitOtherSBCharacter += new IKVMCController.HitFeedbackHandler(this.ResponseHitOthersFeedback);
		
		behaviour = new BehaviourController(character, controller);
		behaviour.initializeDefaultParameters();
		controller.setBehaviour(behaviour);
		behaviour.conTransitionPlan();
		
		characterInitState();
		
		startCharacter();
		
		characterIsReady = true;
		Debug.Log("----------------------- start ---------------------------"+name);
	}
	
	void FixedUpdate () {
		if(!characterIsReady) return;

		isAvoidMove = false;
		bipedIK.references.root.localPosition = Vector3.zero;
		bipedIK.references.root.localRotation = Quaternion.identity;
		if(nearCollider!=null)nearCollider.transform.position = pelvis.transform.position;

		if (agent != null) {
			if((pelvis.transform.position - agent.transform.position).magnitude > 1){
				agent.SetDestination(pelvis.transform.position);
			}else{
				agent.transform.position = pelvis.transform.position;
			}
		}
		
		setRBStateFromEngine();
		
		controller.performPreTasks(SimGlobals.dt);
		
		if(controller.performPostTasks(SimGlobals.dt)){
		}
		
		if(controller.isCharacterStandUp() && (getCurrentStateID() == (int)BipedalActions.stand_normal || getCurrentStateID() == (int)BipedalActions.stand_defense)){
			isStanding = true;
			if(!controller.getStanceBelence()){
				isStableStanding = true;
			}else{
				isStableStanding = false;
			}

			if(isJumping){
				isJumping = false;
				//endAllReach();
			}
		}else{
			isStanding = false;
			isStableStanding = false;
		}
		
		if(!isFallDown){
			if(!isMoveing && !isFighting && !isInWater && (!controller.isFootInContactWithTheGround() || controller.getFallDownAngle() > 70)){
				//if(controller.standFootBaseNormal.magnitude < 0.001f && controller.swingFootBaseNormal.magnitude < 0.001f){
				//if(getCurrentStateID() != (int)BipedalActions.stand_normal){
					//startStand();
				//}
				foodTouchGroundTimer += 50 * Time.deltaTime;
				if(foodTouchGroundTimer > 20){
					//behaviour.maxStepHeight = 0.5f;
					behaviour.rootExternalForceKp = 0.00001f;
					behaviour.rootExternalForceKd = 0.00001f;
					behaviour.rootExternalTorqueKp = 0.00001f;
					behaviour.rootExternalTorqueKd = 0.00001f;
					foodTouchGroundTimer = 31;
				}
				//behaviour.bodySpringKp = 100;
				//behaviour.bodySpringKd = 10;
			}else{
				foodTouchGroundTimer = 0;
				/*if(isHurting){
					behaviour.maxStepHeight = 0.8f;
				}else{
					behaviour.maxStepHeight = 1.0f;
				}*/
			}
			/*
			if (controller.isLFootTouchedTheGround ()) {
				//leftFoodTouchGroundTimer += 50 * Time.deltaTime;
				//if(leftFoodTouchGroundTimer > 20){

					Vector3 desAxis = controller.getCurrentHeading() * SimGlobals.upAxis;
					Vector3 currAxis = controller.getLFoot ().getOrientation() * character.getLocalUpAxis();
					Vector3 rotAxis = Vector3.Cross(currAxis, desAxis).normalized;
					float ang = Mathf.Acos(Vector3.Dot(currAxis, desAxis)) * Mathf.Rad2Deg;
					Vector3 torque = (10 * ang) * rotAxis - 100 * controller.getLFoot ().getAngularVelocity();
					controller.getLFoot ().addExternalTorque (torque);

					currAxis = controller.getLUpperLeg().getOrientation() * character.getLocalUpAxis();
					rotAxis = Vector3.Cross(currAxis, desAxis).normalized;
					ang = Mathf.Acos(Vector3.Dot(currAxis, desAxis)) * Mathf.Rad2Deg;
					torque = (10 * ang) * rotAxis - 100 * controller.getLUpperLeg().getAngularVelocity();
					controller.getLUpperLeg().addExternalTorque (0.5f*torque);
					leftFoodTouchGroundTimer = 31;
				//}
			}else{
				leftFoodTouchGroundTimer = 0;
			}
			if (controller.isRFootTouchedTheGround ()) {
				//rightFoodTouchGroundTimer += 50 * Time.deltaTime;
				//if (rightFoodTouchGroundTimer > 20) {

					Vector3 desAxis = controller.getCurrentHeading() * Quaternion.identity * SimGlobals.upAxis;
					Vector3 currAxis = controller.getRFoot ().getOrientation() * character.getLocalUpAxis();
					Vector3 rotAxis = Vector3.Cross(currAxis, desAxis).normalized;
					float ang = Mathf.Acos(Vector3.Dot(currAxis, desAxis)) * Mathf.Rad2Deg;
					Vector3 torque = (10 * ang) * rotAxis - 100 * controller.getRFoot ().getAngularVelocity();
					controller.getRFoot ().addExternalTorque (torque);

					currAxis = controller.getRUpperLeg().getOrientation() * character.getLocalUpAxis();
					rotAxis = Vector3.Cross(currAxis, desAxis).normalized;
					ang = Mathf.Acos(Vector3.Dot(currAxis, desAxis)) * Mathf.Rad2Deg;
					torque = (10 * ang) * rotAxis - 100 * controller.getRUpperLeg().getAngularVelocity();
					controller.getRUpperLeg().addExternalTorque (0.5f*torque);
					rightFoodTouchGroundTimer = 31;
				//}
			} else {
				rightFoodTouchGroundTimer = 0;
			}*/

			if(isDragging){
				foreach(int i in dragBody.getDragTouchID()){
					if(i < 0) continue;
					if(dragBody.getCurrDragBody(i) == controller.getHead()){
						setLookState(HeadLookState.none, null, Vector3.zero);
						if(i==0 && dragBody.getCurrDragLen(i) > 0.1f){
							if(dragBody.getDragMoveSpeed(i) > 2){
								addExpTimer += 100 * Time.deltaTime;
								if(addExpTimer > Mathf.Clamp(400/dragBody.getDragMoveSpeed(i), 40, 200)){
									//RFGameManager.instance.addExperience(2 + GameAttribute.gameAttribute.loadPlayerLevel());
									addExpTimer = 0;
								}
							}
						}
					}else{
						if(dragBody.getCurrDragLen(i) > 0.1f){
							if(dragBody.getDragMoveSpeed(i) > 2){
								lookDragTimer = 0;
								setLookState(HeadLookState.lookTarget, dragBody.getCurrDragBody(i).gameObject, dragBody.getSpringJointStartPos(i));
								if(i==0){
									addExpTimer += 100 * Time.deltaTime;
									if(addExpTimer > Mathf.Clamp(400/dragBody.getDragMoveSpeed(i), 40, 200)){
										//RFGameManager.instance.addExperience(2 + GameAttribute.gameAttribute.loadPlayerLevel());
										addExpTimer = 0;
									}
								}
							}else{
								if(lookDragTimer > 30 && Random.Range(0, 200) < 5){
									setLookState(HeadLookState.none, null, Vector3.zero);
								}
								lookDragTimer += 50*Time.deltaTime;
							}
						}else{
							if(Random.Range(0, 100) < 5){
								setLookState(HeadLookState.lookTarget, currCamera.gameObject, Vector3.zero);
							}
						}
					}
					
					if(!dragBody.getCurrDragBody(i).name.Contains("Arm")){
						/*if(dragBody.getCurrDragLen(i) > 0.1f){
							if(Random.Range(0, 200) < 5){
								setHandTouchState(HandTouchState.touch, dragBody.getCurrDragBody(i).gameObject, dragBody.getSpringJointStartPos(i), HandTouchMode.autoHand);
							}
							if(dragBody.getDragMoveSpeed(i) > 40 && Random.Range(0, 100) < 10){
								setHandTouchState(HandTouchState.touch, null, dragBody.getDragStartPosition(i), HandTouchMode.autoHand);
							}
						}*/
					}else{
						if(dragBody.getCurrDragLen(i) > 1.0f && dragBody.getDragEndPosition(i)[controller.getWorldUpAxisID()] < head.transform.position[controller.getWorldUpAxisID()]){
							//behaviour.rootExternalTorqueKp = 50;
							//behaviour.rootExternalTorqueKd = 400;
							setDirectionWithTargetPosition(dragBody.getDragEndPosition(i));
						}
					}
				}

				behaviour.belenceError = 0.2f;

				if(controller.standFootBaseNormal.magnitude > 0.001f && controller.swingFootBaseNormal.magnitude > 0.001f){
				}else{
					if(getCurrentStateID() != (int)BipedalActions.stand_normal && getCurrentStateID() != (int)BipedalActions.stand_defense){
						Debug.Log("dragging startStand ======");
						startStand();
					}
				}
			}else{
				/*if (isDead || controller.isCharacterFallDown ()) {
					if (isDead) {
						startFallDown ();
						endHurt ();
					} else {
						startFallDownTimer += 50 * Time.deltaTime;
						if (startFallDownTimer > 10) {
							startFallDown ();
							endHurt ();
						}
					}
				} else {
					startFallDownTimer = 0;
				}*/

				//setHandTouchState(HandTouchState.touch, targetObj, Vector3.zero, HandTouchMode.doubleHand);
				//setLookState(HeadLookState.lookTarget, targetObj, Vector3.zero);
				if (getCurrentStateID () == (int)BipedalActions.stand_normal && !controller.getStanceBelence ()) {
					backToIdleTimer += 50 * Time.deltaTime;
					if (backToIdleTimer > 20 && lookState == HeadLookState.none) {
						setLookState (HeadLookState.idle, null, Vector3.zero);
						setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
					}
				}else{
					backToIdleTimer = 0;
					//if(controller.getStanceBelence()){
						//setLookState(HeadLookState.none, null, Vector3.zero);
					//}
				}
				
				if(controller.standFootBaseNormal.magnitude > 0.001f && controller.swingFootBaseNormal.magnitude > 0.001f){
					//if(isInAir && !isPicking && getCurrentStateID() == (int)BipedalActions.falling){
						//Debug.Log("ttttttttttttt startTouchDown ======");
						//startTouchDown();
					//}
					startInAirTimer = 0;
					isInAir = false;

					if (getCurrentStateID() == (int)BipedalActions.forward_run || getCurrentStateID() == (int)BipedalActions.forward_walk) {
						float sh = controller.swingFootFrontPosition [controller.getWorldUpAxisID ()] - controller.standFootBasePosition [controller.getWorldUpAxisID ()];
						if (sh > 0.3f) {
							behaviour.time = 0.6f;
							behaviour.swingStepLength = 0.7f;
							behaviour.standStepLength = -0.6f;
							//velDSagittal = 4;
							behaviour.stepHeight = 0.8f;
							behaviour.bodySpringKp = 800;
							behaviour.bodySpringKd = 50;
						} else if (sh < -0.3f) {
							behaviour.time = 0.6f;
							behaviour.swingStepLength = 0.6f;
							behaviour.standStepLength = -0.6f;
							//velDSagittal = 3;
							behaviour.stepHeight = 0.6f;
						}
						if (isInWater) {
							character.getRoot ().addExternalForce (controller.getCurrentHeading () * (600 * character.getLocalFrontAxis ()));
						}
					}
					
					if (controller.isTwoFootTouchedGround() && getCurrentStateID() == (int)BipedalActions.stand_normal || getCurrentStateID() == (int)BipedalActions.stand_defense) {
						float sh = controller.swingFootBasePosition [controller.getWorldUpAxisID ()] - controller.standFootBasePosition [controller.getWorldUpAxisID ()];
						if (Mathf.Abs(sh) > 0.01f) {
							behaviour.rootExternalTorqueKp = 50;
							behaviour.rootExternalTorqueKd = 500;
							behaviour.rootExternalForceKp = 3000;
							behaviour.rootExternalForceKd = 300;
							behaviour.belenceError = 0.5f;
						}
					}
				}else{
					if(startInAirTimer > 20){
						if(isInWater){
							if(!isHurting && getCurrentStateID()!=(int)BipedalActions.inWaterStand && getCurrentStateID()!=(int)BipedalActions.inWaterSwiming){
								//Debug.Log("ttttttttttttt startInWaterStand ====== "+50*Time.deltaTime);
								if (type == "Player") {
									startInWaterStand ();
								} else {
									startInWaterSwiming ();
								}
							}

							float f = Random.Range(50, 120)*Mathf.Sin(3*startInAirTimer * Mathf.Deg2Rad);
							waist.GetComponent<Rigidbody>().AddForce(f*SimGlobals.upAxis);
							if(getCurrentStateID()==(int)BipedalActions.inWaterSwiming){
								Vector3 faxis = controller.getCharacterFrame()*character.getLocalFrontAxis();
								Vector3 uaxis = Vector3.Cross(controller.getCurrentHeading()*character.getLocalFrontAxis(), -1*SimGlobals.upAxis);
								Vector3 axis = (Vector3.Project(faxis, uaxis) + Vector3.Project(faxis, -1*SimGlobals.upAxis)).normalized;
								float ang = Vector3.Angle(axis, -1*SimGlobals.upAxis);
								if(Vector3.Dot(axis, uaxis) > 0){
									ang = -ang;
								}
								pelvis.GetComponent<Rigidbody>().AddTorque(30*ang*(controller.getCharacterFrame()*character.getLocalUpAxis()) - 300*pelvis.GetComponent<Rigidbody>().angularVelocity);
							}

							isInAir = false;
						}else{
							/*if(!isInAir && !isFighting && !isPicking){
								if(getCurrentStateID()!=(int)BipedalActions.falling){
									//Debug.Log("ttttttttttttt startFalling ======");
									startFalling();
								}
								isInAir = true;
							}*/
						}
					}
					startInAirTimer += 50*Time.deltaTime;
				}
			}

			if(!isFighting && (handTouchState == HandTouchState.none || handTouchState == HandTouchState.touch)){
				RaycastHit raycastHit = new RaycastHit();
				Vector3 rayStartPos = torso.transform.position + torso.transform.rotation*(0.5f*character.getLocalUpAxis());
				Vector3 rayDir = 3*character.getLocalLeftAxis();
				Vector3 rayEndPos = rayStartPos + torso.transform.rotation*rayDir;
				Debug.DrawLine(rayStartPos, rayEndPos, Color.blue);
				if(Physics.Linecast(rayStartPos, rayEndPos, out raycastHit, behaviour.stepLayerMask)){
					Vector3 hitPos = rayStartPos + torso.transform.rotation*(0.5f*raycastHit.distance*rayDir.normalized);
					isLeftTouchRay = true;
					setHandTouchState(HandTouchState.touch, null, hitPos, HandTouchMode.leftHand);

					if(isWalkToAim && isMoveing){
						behaviour.time = 0.5f;
						behaviour.stepHeight = 0.8f;
						behaviour.velDCoronal = -3;
						behaviour.swingStepLength = 0.001f;
						behaviour.standStepLength = 0.001f;
						isAvoidMove = true;
					}
				}else{
					if(isLeftTouchRay){
						isLeftTouchRay = false;
						setHandTouchState(HandTouchState.touch, null, Vector3.zero, HandTouchMode.leftHand);
					}
				}

				rayDir = -3*character.getLocalLeftAxis();
				rayEndPos = rayStartPos + torso.transform.rotation*rayDir;
				Debug.DrawLine(rayStartPos, rayEndPos, Color.blue);
				if(Physics.Linecast(rayStartPos, rayEndPos, out raycastHit, behaviour.stepLayerMask)){
					Vector3 hitPos = rayStartPos + torso.transform.rotation*(0.5f*raycastHit.distance*rayDir.normalized);
					isRightTouchRay = true;
					setHandTouchState(HandTouchState.touch, null, hitPos, HandTouchMode.rightHand);

					if(isWalkToAim && isMoveing){
						behaviour.time = 0.5f;
						behaviour.stepHeight = 0.8f;
						behaviour.velDCoronal = 3;
						behaviour.swingStepLength = 0.001f;
						behaviour.standStepLength = 0.001f;
						isAvoidMove = true;
					}
				}else{
					if(isRightTouchRay){
						isRightTouchRay = false;
						setHandTouchState(HandTouchState.touch, null, Vector3.zero, HandTouchMode.rightHand);
					}
				}

				rayDir = 3*character.getLocalFrontAxis();
				rayEndPos = rayStartPos + torso.transform.rotation*rayDir;
				Debug.DrawLine(rayStartPos, rayEndPos, Color.blue);
				if(!isHurting && Physics.Linecast(rayStartPos, rayEndPos, out raycastHit, behaviour.stepLayerMask)){
					Vector3 hitPosLeft = rayStartPos + torso.transform.rotation*(0.5f*raycastHit.distance*(rayDir+1.5f*character.getLocalLeftAxis()).normalized);
					Vector3 hitPosRight = rayStartPos + torso.transform.rotation*(0.5f*raycastHit.distance*(rayDir-1.5f*character.getLocalLeftAxis()).normalized);
					isDoubleTouchRay = true;
					setHandTouchState(HandTouchState.touch, null, Vector3.zero, HandTouchMode.doubleHand);
					leftHandTouchLocalPos = hitPosLeft;
					rightHandTouchLocalPos = hitPosRight;

					if(isMoveing){
						if (isWalkToAim) {
							if (walkAvoidSign == 0) {
								Vector3 aimPos = agentPath.corners [currentPathIndex];
								Vector3 vec = aimPos - torso.transform.position;
								float dot = Vector3.Dot (vec, torso.transform.rotation * character.getLocalLeftAxis ());
								walkAvoidSign = (dot > 0) ? 3 : -3;
							}
						} else {
							walkAvoidSign = 0;
						}
						behaviour.time = 0.5f;
						behaviour.stepHeight = 0.8f;
						behaviour.velDCoronal = walkAvoidSign;
						behaviour.velDSagittal = -3;
						behaviour.swingStepLength = 0.001f;
						behaviour.standStepLength = 0.001f;
						isAvoidMove = true;
						//Debug.Log("adsassa === behaviour.velDCoronal === "+behaviour.velDCoronal);
					}else if(isInWater){
						if(raycastHit.collider.gameObject.layer == (int)ColliderLayer.sceneRefLayer){
							if(getCurrentStateID() == (int)BipedalActions.inWaterSwiming){
								startRun();
							}
						}
					}
				}else{
					walkAvoidSign = 0;
					if(isDoubleTouchRay){
						isDoubleTouchRay = false;
						setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
					}
				}
			}
		}else{
			if (!isDead && getCurrentStateID () == (int)BipedalActions.fall_down) {
				setLookState (HeadLookState.none, null, Vector3.zero);
				setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
				standUpTimer += 50 * Time.deltaTime;
				if (standUpTimer > 50) {
					Vector3 vec = controller.getCharacterFrame () * character.getLocalFrontAxis ();
					if (Vector3.Dot (vec, SimGlobals.upAxis) > 0) {
						float ang = Vector3.Angle (vec, SimGlobals.upAxis);
						if (ang > 20) {
							Vector3 rotAxis = Vector3.Cross (vec, SimGlobals.upAxis).normalized;
							Vector3 torque = (50 * ang) * rotAxis - 500 * character.getRoot ().getAngularVelocity ();
							character.getRoot ().addExternalTorque (torque);
						} else {
							if (Random.Range (0, 100) < 2) {
								startStandUp ((int)BipedalActions.standup3);
							}
						}
					} else {
						float ang = Vector3.Angle (vec, -1 * SimGlobals.upAxis);
						if (ang > 20) {
							Vector3 rotAxis = Vector3.Cross (vec, -1 * SimGlobals.upAxis).normalized;
							Vector3 torque = (50 * ang) * rotAxis - 500 * character.getRoot ().getAngularVelocity ();
							character.getRoot ().addExternalTorque (torque);
						} else {
							if (Random.Range (0, 100) < 2) {
								startStandUp ((int)BipedalActions.standup4);
							}
						}
					}
				}
			} else if (isStanding || isMoveing) {
				checkForIsFallDownFalse ();
			}
		}
		/*
		if(!isReaching){
			if (getCurrentStateID () == (int)BipedalActions.stand_normal) {
				if (isJumping || (isInAir && !isHurting)) {
					checkForReach (pelvis.transform.position, PickerDirection.up, true);
				} else if (isFighting) {
					if (weapon != null) {
						//currentPickTarget = weapon.gameObject;
						//checkForReach (pelvis.transform.position, PickerDirection.up);
					}
				}
			} else if (getCurrentStateID () == (int)BipedalActions.stand_defense) {
				
			}else if(getCurrentStateID()==(int)BipedalActions.climb_up){
				if(controller.getPhase() >= 1){
					checkForReach(torso.transform.position, PickerDirection.up);
				}
			}else if(getCurrentStateID()==(int)BipedalActions.climb_down){
				//if(controller.swingFootBaseNormal.magnitude > 0){
				//	startStand();
				//	endAllReach();
				//	endAllPick();
				//}else{
					if(controller.getPhase() >= 1){
						checkForReach(pelvis.transform.position, PickerDirection.down);
					}
				//}
			}else if(getCurrentStateID()==(int)BipedalActions.climb_up_end1 || getCurrentStateID()==(int)BipedalActions.shin_up){
				if(controller.getPhase() >= 1){
					checkForReach(pelvis.transform.position, PickerDirection.up);
				}
			}else if(getCurrentStateID()==(int)BipedalActions.climb_down_start1){
				if(controller.getPhase() >= 1){
					checkForStartClimbDown();
				}
			}else if(getCurrentStateID()==(int)BipedalActions.climb_down_start2 || getCurrentStateID()==(int)BipedalActions.climb_down_start3){
				if(controller.getPhase() >= 1){
					if(controller.getLFoot().isFixedPosition()){
						checkForStartClimbDown(SimGlobals.LEFT_STANCE);
					}else if(controller.getRFoot().isFixedPosition()){
						checkForStartClimbDown(SimGlobals.RIGHT_STANCE);
					}
				}
			}else if(getCurrentStateID()==(int)BipedalActions.hang_up){
				if(controller.getPhase() >= 1){
					checkForReach(controller.getStanceHand().getWorldBottomPosition(), PickerDirection.up, true);
				}
			}else if(getCurrentStateID()==(int)BipedalActions.hang_down){
				//if(controller.swingFootBaseNormal.magnitude > 0){
				//	startStand();
				//	endAllReach();
				//	endAllPick();
				//}else{
					if(controller.getPhase() >= 1){
						checkForReach(controller.getStanceHand().getWorldBottomPosition(), PickerDirection.down, true);
					}
				//}
			}else if(getCurrentStateID()==(int)BipedalActions.hang_to_jump){
				if(controller.getPhase() >= 1){
					if(isPicking){
						endAllPick();
						//setDirection(hangJumpIsRight);
						//if(hangJumpIsRight){
							//controller.getTorso().rigidbody.AddForce(new Vector3(500, 500, 0), ForceMode.Impulse);
						//}else{
						//	controller.getTorso().rigidbody.AddForce(new Vector3(-500, 500, 0), ForceMode.Impulse);
						//}
					}else{
						GameObject target = getClosestPickerTarget(pelvis.transform.position, PickerDirection.up);
						if(target != null && target.GetComponent<Collider>().tag == "picker_rope"){
							if(!isFaceToPosition(target.transform.position, 0.5f)){
								startStand();
							}
						}else{
							startStand();
						}
					}
				}
			}
		}else{
			updateReach();
		}*/
		/*
		if(isPicking && isUpdatePickerPosition){
			Vector3 pickerPos = Vector3.zero;
			GameObject target;
			if(controller.getLHand().isFixedPosition()){
				target = controller.getLHand().getFixedTarget();
				Picker picker = target.GetComponent("Picker") as Picker;
				pickerPos = picker.getWorldPickerPosition(picker.leftHandAtPickerIndex);
				//target.GetComponent<Rigidbody>().AddForceAtPosition(-0.05f*controller.getLHand().getFixPositionForce(), pickerPos);
				controller.getLHand().updateFixedPosition(pickerPos);
			}
			if(controller.getRHand().isFixedPosition()){
				target = controller.getRHand().getFixedTarget();
				Picker picker = target.GetComponent("Picker") as Picker;
				pickerPos = picker.getWorldPickerPosition(picker.rightHandAtPickerIndex);
				//target.GetComponent<Rigidbody>().AddForceAtPosition(-0.05f*controller.getRHand().getFixPositionForce(), pickerPos);
				controller.getRHand().updateFixedPosition(pickerPos);
			}
			if(controller.getLFoot().isFixedPosition()){
				target = controller.getLFoot().getFixedTarget();
				Picker picker = target.GetComponent("Picker") as Picker;
				pickerPos = picker.getWorldPickerPosition(picker.leftFootAtPickerIndex);
				//target.GetComponent<Rigidbody>().AddForceAtPosition(-0.05f*controller.getLFoot().getFixPositionForce(), pickerPos);
				controller.getLFoot().updateFixedPosition(pickerPos);
			}
			if(controller.getRFoot().isFixedPosition()){
				target = controller.getRFoot().getFixedTarget();
				Picker picker = target.GetComponent("Picker") as Picker;
				pickerPos = picker.getWorldPickerPosition(picker.rightFootAtPickerIndex);
				//target.GetComponent<Rigidbody>().AddForceAtPosition(-0.05f*controller.getRFoot().getFixPositionForce(), pickerPos);
				controller.getRFoot().updateFixedPosition(picker.getWorldPickerPosition(picker.rightFootAtPickerIndex));
			}
		}
		*/
		if(isWalkToAim && (isMoveing || isInWater) && currentPathIndex < agentPath.corners.Length){
			Vector3 aimPos = agentPath.corners[currentPathIndex];
			aimPos[controller.getWorldUpAxisID()] = 0;

			Vector3 rootPos = pelvis.transform.position;
			rootPos[controller.getWorldUpAxisID()] = 0;
			Vector3 toAimVec = aimPos - currentStartPosition;
			float aimDis = toAimVec.magnitude;

			if(aimDis > 1){
				Vector3 dir = (aimPos - rootPos).normalized;
				if(getCurrentStateID()==(int)BipedalActions.back_walk){
					setDirectionWithDirection(-1*dir);
				}else{
					setDirectionWithDirection(dir);
				}

				float rootDis = (rootPos - currentStartPosition).magnitude;
				if (rootDis > aimDis * 0.9f) {
					if (currentPathIndex < agentPath.corners.Length - 1) {
						currentPathIndex++;
						currentStartPosition = rootPos;
					} else {
						walkToAimEnd (SimGlobals.LEFT_STANCE);
					}
				}
			}else{
				currentPathIndex++;
				//currentStartPosition = pelvis.transform.position;
				//currentStartPosition[controller.getWorldUpAxisID()] = 0;
			}
		}
		/*
		if (isFighting) {
			if(isStanding || isMoveing || isInAir){
				checkForIsFightingFalse();
			}
		}*/
		
		if(isHurting){
			getHurt();
			hurtTimer += 50*Time.deltaTime;
			if(hurtTimer > 50 && (controller.isCharacterStandUp() || isInWater) /*&& controller.getDoubleStanceCOMError().magnitude < controller.getBelenceError()*/){
				if(Random.Range(0, 100) < 5){
					endHurt();
				}
			}
		}
		/*
		if (weapon != null) {
			if (isInWater) {
				Vector3 pos = head.transform.position + controller.getCurrentHeading () * new Vector3 (2, 0, 0.3f);
				Vector3 dir = new Vector3 (0, 1, 0.5f);
				if (type == "Enemy") {
					dir = new Vector3 (0, -1, -0.5f);
				}
				controller.setWristOrientationWithDirection (false, controller.getCurrentHeading () * dir.normalized);
				setHandTouchState (HandTouchState.touch, null, pos, HandTouchMode.rightHand);
			} else {
				if (getCurrentStateID () == (int)BipedalActions.stand_defense) {
					//controller.setWristOrientationWithDirection (true, controller.getCurrentHeading () * new Vector3 (-1, 1, 1).normalized);
					//Vector3 pos = character.getCOM () + controller.getCurrentHeading () * new Vector3 (0, 1.5f, 1f);
					//setHandTouchState (HandTouchState.touch, null, pos, HandTouchMode.rightHand);
					Vector3 dir = new Vector3 (-1, 1, 1);
					if (type == "Enemy") {
						dir = new Vector3 (1, -1, -1);
					}
					controller.setWristOrientationWithDirection (false, controller.getCurrentHeading () * dir.normalized);
					//}
				}else if (isMoveing) {
					Vector3 pos = character.getCOM () + controller.getCurrentHeading () * new Vector3 (2, 0, 0.3f);
					Vector3 dir = new Vector3 (0.5f, 1, 1);
					if (type == "Enemy") {
						dir = new Vector3 (-0.5f, -1, -1);
					}
					controller.setWristOrientationWithDirection (false, controller.getCurrentHeading () * dir.normalized);
					//rHand.transform.rotation = Quaternion.Lerp(rHand.transform.rotation, controller.getCurrentHeading ()*j.getInitialWorldRotation(), 0.2f);
					setHandTouchState (HandTouchState.touch, null, pos, HandTouchMode.rightHand);
				} else {
					if (handTouchState == HandTouchState.touch) {
						setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.rightHand);
					}
				}
			}
		}*/
		/*
		if (lShoes != null && rShoes != null) {
			if (isDragging || isHurting || isMoveing) {
				behaviour.maxStepHeight = lShoes.length;
				behaviour.stepHeight = controller.getCurrentState ().stepHeight + 0.3f;
				behaviour.belenceError = 0.05f;
				belenceErrorTimer = 0;
			} else if (isFighting) {
				behaviour.stepHeight = 0;
				behaviour.belenceError = 0.25f;
				belenceErrorTimer = 0;
			}else {
				behaviour.maxStepHeight = lShoes.length;
				behaviour.stepHeight = controller.getCurrentState ().stepHeight + 0.3f;
				belenceErrorTimer += 50 * Time.deltaTime;
				if (belenceErrorTimer > 30) {
					behaviour.belenceError = 0.4f;
					belenceErrorTimer = 31;
				}
			}

			if (!isFallDown) {
				if (controller.isLFootTouchedTheGround ()) {
					Vector3 currAxis = (controller.getLFoot ().getWorldBottomPosition () - lShoes.bottom.transform.position).normalized;
					Vector3 rotAxis = Vector3.Cross (currAxis, SimGlobals.upAxis).normalized;
					float ang = Mathf.Acos (Vector3.Dot (currAxis, SimGlobals.upAxis)) * Mathf.Rad2Deg;
					Vector3 torque = (20 * ang) * rotAxis - 100 * controller.getLFoot ().getAngularVelocity ();
					controller.getLFoot ().addExternalTorque (torque);
				}
				if (controller.isRFootTouchedTheGround ()) {
					Vector3 currAxis = (controller.getRFoot ().getWorldBottomPosition () - rShoes.bottom.transform.position).normalized;
					Vector3 rotAxis = Vector3.Cross (currAxis, SimGlobals.upAxis).normalized;
					float ang = Mathf.Acos (Vector3.Dot (currAxis, SimGlobals.upAxis)) * Mathf.Rad2Deg;
					Vector3 torque = (20 * ang) * rotAxis - 100 * controller.getRFoot ().getAngularVelocity ();
					controller.getRFoot ().addExternalTorque (torque);
				}

				if (controller.isFootInContactWithTheGround ()) {
					float height = Mathf.Abs (controller.standFootBasePosition.y - character.getRoot ().transform.position.y);
					if (height < 3) {
						character.getRoot ().addExternalForce ((2000 * (3.0f - height) - 250 * (character.getRoot ().GetComponent<Rigidbody> ().velocity.y)) * SimGlobals.upAxis);
					}
					behaviour.rootExternalTorqueKp = 50;
					behaviour.rootExternalTorqueKd = 500;
					behaviour.rootExternalForceKp = 3000;
					behaviour.rootExternalForceKd = 300;
				}
			}

			if (!isAvoidMove && (getCurrentStateID () == (int)BipedalActions.forward_run || getCurrentStateID () == (int)BipedalActions.forward_walk)) {
				float sh = controller.swingFootFrontPosition [controller.getWorldUpAxisID ()] - controller.standFootBasePosition [controller.getWorldUpAxisID ()];
				if (sh > 0.3f) {
					behaviour.time = 0.6f;
					behaviour.swingStepLength = 0.7f;
					behaviour.standStepLength = -0.6f;
					behaviour.velDSagittal = 6;
					behaviour.stepHeight = controller.getCurrentState ().stepHeight + 0.5f;
					behaviour.bodySpringKp = 800;
					behaviour.bodySpringKd = 60;
					behaviour.comOffsetSagittal = 0.3f;
					behaviour.coronalWidth = -0.3f;
					if(sh > 0.75f){
						behaviour.time = 0.7f;
						behaviour.velDSagittal = 9;
						behaviour.comOffsetSagittal = 0.4f;
						behaviour.coronalWidth = -0.4f;
					}
					normalStepTimer = 0;
				} else if (sh < -0.3f) {
					behaviour.time = 0.6f;
					behaviour.swingStepLength = 0.6f;
					behaviour.standStepLength = -0.6f;
					behaviour.velDSagittal = 4;
					behaviour.stepHeight = controller.getCurrentState ().stepHeight + 0.5f;
					behaviour.comOffsetSagittal = 0.2f;
					normalStepTimer = 0;
				} else {
					normalStepTimer += 50 * Time.deltaTime;
					if (normalStepTimer > 30) {
						if (getCurrentStateID () == (int)BipedalActions.forward_run) {
							behaviour.time = 0.5f;
							behaviour.velDSagittal = (type == "Player") ? 9 : 8;
							behaviour.comOffsetSagittal = 0.1f;
						} else if (getCurrentStateID () == (int)BipedalActions.forward_walk) {
							behaviour.time = 0.6f;
							behaviour.velDSagittal = (type == "Player") ? 5 : 4;
							behaviour.comOffsetSagittal = 0.1f;
						}
						normalStepTimer = 31;
					}
				}
			}

		}*/
		
		updateHeadLookState();
		updateHandTouchState();
	}
	
	protected void updateHeadLookState(){
		if(lookState == HeadLookState.none){
			controller.lookAtTargetPosition = Vector3.zero;
		}else if(lookState == HeadLookState.idle){
			//controller.setLookAtWeight(0, 1);
			controller.lookAtTargetPosition = character.getRoot().getWorldCoordinatesPoint(5.0f * (character.getLocalFrontAxis() + 0.0f*character.getLocalUpAxis()));

			float rand = rootIdleTimer * Mathf.Deg2Rad;
			/*if(isStanding){
				Vector3 rootIdleForce = Vector3.zero;
				rootIdleForce[controller.getLocalFrontAxisID()] = 40 * Mathf.Sin(rand);
				rootIdleForce[controller.getLocalLeftAxisID()] = 40 * Mathf.Cos(rand);
				character.getRoot().addExternalForce(controller.getCharacterFrame() * rootIdleForce);
			}*/
			
			Vector3 lookTargetOffset = Vector3.zero;
			lookTargetOffset[controller.getLocalUpAxisID()] = lookTargetOffsetRange * Mathf.Sin(rand);
			lookTargetOffset[controller.getLocalLeftAxisID()] = lookTargetOffsetRange * Mathf.Cos(rand);
			controller.lookAtTargetPosition += lookTargetOffset;
			
			rootIdleTimer += 1.2f;
			if(rootIdleTimer > 360){
				rootIdleTimer = 0;
				lookTargetOffsetRange = Random.Range(0.1f, 1.0f);
			}
		}else if(lookState == HeadLookState.lookTarget){
			//controller.setLookAtWeight(0.5f, 1);
			if(lookObject != null){
				controller.lookAtTargetPosition = lookObject.transform.position + lookObject.transform.rotation * lookLocalPos;
			}else{
				controller.lookAtTargetPosition = lookLocalPos;
			}
		}
	}
	
	public void setLookState(HeadLookState state, GameObject obj, Vector3 pos){
		lookState = state;
		lookObject = obj;
		lookLocalPos = pos;
	}
	
	protected void updateHandTouchState(){
		if (handTouchState == HandTouchState.none) {
			controller.leftArmReachTargetPosition = Vector3.zero;
			controller.rightArmReachTargetPosition = Vector3.zero;
		} else if (handTouchState == HandTouchState.touch) {
			if (leftHandTouchObject == null) {
				controller.leftArmReachTargetPosition = leftHandTouchLocalPos;
			} else {
				controller.leftArmReachTargetPosition = leftHandTouchObject.transform.position + leftHandTouchObject.transform.rotation * leftHandTouchLocalPos;
			}
			if (rightHandTouchObject == null) {
				controller.rightArmReachTargetPosition = rightHandTouchLocalPos;
			} else {
				controller.rightArmReachTargetPosition = rightHandTouchObject.transform.position + rightHandTouchObject.transform.rotation * rightHandTouchLocalPos;
			}
			if (leftFootTouchObject == null) {
				controller.leftLegReachTargetPosition = leftFootTouchLocalPos;
			} else {
				controller.leftLegReachTargetPosition = leftFootTouchObject.transform.position + leftFootTouchObject.transform.rotation * leftFootTouchLocalPos;
			}
			if (rightFootTouchObject == null) {
				controller.rightLegReachTargetPosition = rightFootTouchLocalPos;
			} else {
				controller.rightLegReachTargetPosition = rightFootTouchObject.transform.position + rightFootTouchObject.transform.rotation * rightFootTouchLocalPos;
			}
			if (spineTouchObject == null) {
				controller.shineTargetPosition = spineTouchLocalPos;
			} else {
				controller.shineTargetPosition = spineTouchObject.transform.position + spineTouchObject.transform.rotation * spineTouchLocalPos;
			}
		} else if (handTouchState == HandTouchState.scratch) {
		} else if (handTouchState == HandTouchState.resist) {
			if (leftHandTouchObject != null) {
				Vector3 armVec = (leftHandTouchObject.transform.position - controller.getLeftShoulder ().getChildJointWorldPosition ()).normalized;
				controller.leftArmReachTargetPosition = controller.getLeftShoulder ().getChildJointWorldPosition () + 0.5f * controller.getArmLength () * armVec;
				/*if (rightHandTouchObject == null) {
					behaviour.setUpperBodyPose (1, 0, 50, 60);
				} else {
					behaviour.setUpperBodyPose (0, 0, 50, 60);
				}*/
				if (!startLeftResist && (controller.getLHand ().getWorldBottomPosition () - leftHandTouchObject.transform.position).magnitude < 1.5f * controller.getArmLength ()) {
					leftResistTimer += 50 * Time.deltaTime;
					if (leftResistTimer >= leftResistMaxTimer) {
						startLeftResist = true;
						leftResistTimer = 0;
						leftResistMaxTimer = Random.Range (7, 15);
					}
				}
				if (startLeftResist) {
					controller.leftArmReachTargetPosition = controller.getLeftShoulder ().getChildJointWorldPosition () + controller.getArmLength () * armVec;

					leftResistTimer += 50 * Time.deltaTime;
					if (leftResistTimer >= leftResistMaxTimer) {
						startLeftResist = false;
						leftResistTimer = 0;
						leftResistMaxTimer = Random.Range (7, 15);
					}
				}
			} else {
				leftResistTimer = 0;
				leftResistMaxTimer = 0;
				startLeftResist = false;
				controller.leftArmReachTargetPosition = Vector3.zero;
			}
			if (rightHandTouchObject != null) {
				Vector3 armVec = (rightHandTouchObject.transform.position - controller.getRightShoulder ().getChildJointWorldPosition ()).normalized;
				controller.rightArmReachTargetPosition = controller.getRightShoulder ().getChildJointWorldPosition () + 0.5f * controller.getArmLength () * armVec;
				/*if (leftHandTouchObject == null) {
					behaviour.setUpperBodyPose (1, 0, -50, -60);
				} else {
					behaviour.setUpperBodyPose (0, 0, 50, 60);
				}*/
				if (!startRightResist && (controller.getRHand ().getWorldBottomPosition () - rightHandTouchObject.transform.position).magnitude < 1.5f * controller.getArmLength ()) {
					rightResistTimer += 50 * Time.deltaTime;
					if (rightResistTimer >= rightResistMaxTimer) {
						startRightResist = true;
						rightResistTimer = 0;
						rightResistMaxTimer = Random.Range (7, 15);
					}
				}
				if (startRightResist) {
					controller.rightArmReachTargetPosition = controller.getRightShoulder ().getChildJointWorldPosition () + controller.getArmLength () * armVec;

					rightResistTimer += 50 * Time.deltaTime;
					if (rightResistTimer >= rightResistMaxTimer) {
						startRightResist = false;
						rightResistTimer = 0;
						rightResistMaxTimer = Random.Range (7, 15);
					}
				}
			} else {
				rightResistTimer = 0;
				rightResistMaxTimer = 0;
				startRightResist = false;
				controller.rightArmReachTargetPosition = Vector3.zero;
			}
		} else if (handTouchState == HandTouchState.hit) {
			if (leftHandTouchObject != null) {
				Vector3 armVec = (leftHandTouchObject.transform.position - controller.getLeftShoulder ().getChildJointWorldPosition ()).normalized;
				controller.leftArmReachTargetPosition = controller.getLeftShoulder ().getChildJointWorldPosition () + 0.5f * controller.getArmLength () * armVec;
				
				if ((controller.getLHand ().getWorldBottomPosition () - leftHandTouchObject.transform.position).magnitude < 1.2f * controller.getArmLength ()) {
					startLeftResist = true;
					leftResistTimer = 0;
					leftResistMaxTimer = Random.Range (10, 15);
				}
				if (startLeftResist) {
					controller.leftArmReachTargetPosition = controller.getLeftShoulder ().getChildJointWorldPosition () + controller.getArmLength () * armVec;

					leftResistTimer += 50 * Time.deltaTime;
					if (leftResistTimer >= leftResistMaxTimer) {
						startLeftResist = false;
						leftResistTimer = 0;
					}
				}
			} else {
				leftResistTimer = 0;
				leftResistMaxTimer = 0;
				startLeftResist = false;
				controller.leftArmReachTargetPosition = Vector3.zero;
			}
			if (rightHandTouchObject != null) {
				Vector3 armVec = (rightHandTouchObject.transform.position - controller.getRightShoulder ().getChildJointWorldPosition ()).normalized;
				controller.rightArmReachTargetPosition = controller.getRightShoulder ().getChildJointWorldPosition () + 0.5f * controller.getArmLength () * armVec;
				if ((controller.getRHand ().getWorldBottomPosition () - rightHandTouchObject.transform.position).magnitude < 1.2f * controller.getArmLength ()) {
					startRightResist = true;
					rightResistTimer = 0;
					rightResistMaxTimer = Random.Range (10, 15);
				}
				if (startRightResist) {
					controller.rightArmReachTargetPosition = controller.getRightShoulder ().getChildJointWorldPosition () + controller.getArmLength () * armVec;

					rightResistTimer += 50 * Time.deltaTime;
					if (rightResistTimer >= rightResistMaxTimer) {
						startRightResist = false;
						rightResistTimer = 0;
					}
				}
			} else {
				rightResistTimer = 0;
				rightResistMaxTimer = 0;
				startRightResist = false;
				controller.rightArmReachTargetPosition = Vector3.zero;
			}
		}else if(handTouchState == HandTouchState.grab){
			
		}
	}

	public void resetResist(){
		//behaviour.setUpperBodyPose (0, 0, 0, 0);
		//behaviour.setUpperBodyPose (1, 0, 0, 0);
		startLeftResist = false;
		leftResistTimer = 0;
		rightResistTimer = 0;
		leftResistMaxTimer = 0;
		rightResistMaxTimer = 0;
		startRightResist = false;
	}
	
	public void setHandTouchState(HandTouchState state, GameObject obj, Vector3 pos, HandTouchMode mode){
		handTouchState = state;
		if (state == HandTouchState.none || mode == HandTouchMode.doubleHand) {
			leftHandTouchObject = obj;
			rightHandTouchObject = obj;
			leftHandTouchLocalPos = pos;
			rightHandTouchLocalPos = pos;
		} else if (mode == HandTouchMode.autoHand) {
			Vector3 wpos = (obj == null) ? pos : obj.transform.position + obj.transform.rotation * pos;
			float len1 = (controller.getLeftShoulder ().getChildJointWorldPosition () - wpos).magnitude;
			float len2 = (controller.getRightShoulder ().getChildJointWorldPosition () - wpos).magnitude;
			if (len1 < len2) {
				leftHandTouchObject = obj;
				leftHandTouchLocalPos = pos;
				if (rightHandTouchObject == leftHandTouchObject && (leftHandTouchLocalPos - rightHandTouchLocalPos).magnitude < 0.01f) {
					rightHandTouchObject = null;
					rightHandTouchLocalPos = Vector3.zero;
				}
			} else {
				rightHandTouchObject = obj;
				rightHandTouchLocalPos = pos;
				if (leftHandTouchObject == rightHandTouchObject && (rightHandTouchLocalPos - leftHandTouchLocalPos).magnitude < 0.01f) {
					leftHandTouchObject = null;
					leftHandTouchLocalPos = Vector3.zero;
				}
			}
		} else if (mode == HandTouchMode.autoDoubleHand) {
			Vector3 wpos = (obj == null) ? pos : obj.transform.position + obj.transform.rotation * pos;
			float len1 = (controller.getLeftShoulder ().getChildJointWorldPosition () - wpos).magnitude;
			float len2 = (controller.getRightShoulder ().getChildJointWorldPosition () - wpos).magnitude;
			if (Mathf.Abs (len1 - len2) < 0.4f * controller.getArmLength ()) {
				leftHandTouchObject = obj;
				leftHandTouchLocalPos = pos;
				rightHandTouchObject = obj;
				rightHandTouchLocalPos = pos;
			} else if (len1 < len2) {
				leftHandTouchObject = obj;
				leftHandTouchLocalPos = pos;
				rightHandTouchObject = null;
				rightHandTouchLocalPos = Vector3.zero;
			} else {
				rightHandTouchObject = obj;
				rightHandTouchLocalPos = pos;
				leftHandTouchObject = null;
				leftHandTouchLocalPos = Vector3.zero;
			}
		} else if (mode == HandTouchMode.leftHand) {
			leftHandTouchObject = obj;
			leftHandTouchLocalPos = pos;
			//rightHandTouchObject = null;
			//rightHandTouchLocalPos = Vector3.zero;
		} else if (mode == HandTouchMode.rightHand) {
			rightHandTouchObject = obj;
			rightHandTouchLocalPos = pos;
			//leftHandTouchObject = null;
			//leftHandTouchLocalPos = Vector3.zero;
		} else if (mode == HandTouchMode.leftFoot) {
			leftFootTouchObject = obj;
			leftFootTouchLocalPos = pos;
		} else if (mode == HandTouchMode.rightFoot) {
			rightFootTouchObject = obj;
			rightFootTouchLocalPos = pos;
		} else if (mode == HandTouchMode.spine) {
			spineTouchObject = obj;
			spineTouchLocalPos = pos;
		}
	}

	public HandTouchState getHandTouchState(){
		return handTouchState;
	}
	
	IEnumerator isFightingToFalse(){
		//if (weapon == null) {
			yield return new WaitForSeconds (0.4f);
		//} else {
			//yield return new WaitForSeconds (0.5f);
			//weapon.endAttack ();
		//}
		isFighting = false;
	}

	IEnumerator lockCurrentHeading(){
		yield return new WaitForSeconds(0.3f);
		controller.setDesiredHeading (controller.getCurrentHeading());
		controller.setIsFreeHeading(false);
	}

	private void checkForIsFightingFalse(){
		fightingTimer += 50*Time.deltaTime;
		if(fightingTimer > 50){
			if(isFighting){
				isFighting = false;
				fightingTimer = 0;
				Debug.Log("===== checkForIsFightingFalse === ");
			}
		}
	}
	
	private void checkForIsFallDownFalse(){
		standUpTimer += 50*Time.deltaTime;
		if(standUpTimer > 10){
			isFallDown = false;
			standUpTimer = 0;
		}
	}

	virtual protected void ResponseStartNewStateEvent(object sender, System.EventArgs e){

		isMoveing = false;
		isHurting = false;
		switch(getCurrentStateID()){
			case (int)BipedalActions.stand_normal: 
			StartCoroutine("lockCurrentHeading");
		        break;
			case (int)BipedalActions.forward_walk: 
				isMoveing = true;
		        break;
			case (int)BipedalActions.forward_run: 
				isMoveing = true;
		        break;
			case (int)BipedalActions.forward_somersault: 
				//isJumping = true;
		        break;
			case (int)BipedalActions.jump: 
				isJumping = true;
		        break;
			case (int)BipedalActions.back_walk: 
				isMoveing = true;
		        break;
			case (int)BipedalActions.stand_defense: 
			StartCoroutine("lockCurrentHeading");
		        break;
			case (int)BipedalActions.defense_walk: 
				isMoveing = true;
		        break;
			case (int)BipedalActions.free_walk:
				isMoveing = true;
				break;
			case (int)BipedalActions.punch1: 
				isFighting = true;
		        break;
			case (int)BipedalActions.punch2: 
				isFighting = true;
		        break;
			case (int)BipedalActions.punch3: 
				isFighting = true;
		        break;
			case (int)BipedalActions.punch4: 
				isFighting = true;
		        break;
			case (int)BipedalActions.kick1: 
				isFighting = true;
		        break;
			case (int)BipedalActions.kick2: 
				isFighting = true;
		        break;
			case (int)BipedalActions.kick3: 
				isFighting = true;
		        break;
			case (int)BipedalActions.forward_jump: 
				isJumping = true;
		        break;
			case (int)BipedalActions.standup1: 
				isFighting = true;
		        break;
			case (int)BipedalActions.standup2: 
				isFighting = true;
		        break;
			case (int)BipedalActions.standup3: 
		        break;
			case (int)BipedalActions.standup4: 
		        break;
			case (int)BipedalActions.fall_down: 
				isFallDown = true;
		        break;
			case (int)BipedalActions.hurt: 
				isHurting = true;
		        break;
			case (int)BipedalActions.inWaterStand: 
				//pelvis.GetComponent<BuoyancyForce>().enabled = false;
				StartCoroutine("lockCurrentHeading");
				break;
			case (int)BipedalActions.inWaterSwiming: 
				//pelvis.GetComponent<BuoyancyForce>().enabled = false;
				StartCoroutine("lockCurrentHeading");
				break;
			case (int)BipedalActions.weaponHit1: 
				isFighting = true;
				break;
			case (int)BipedalActions.weaponHit2: 
				isFighting = true;
				break;
			case (int)BipedalActions.weaponHit3: 
				isFighting = true;
				break;
		}
		fightingTimer = 0;
		standUpTimer = 0;
		if(actionEditor!=null){
       		actionEditor.updateState();
		}
    }
	
	virtual protected void ResponseStateEndedEvent(object sender, System.EventArgs e){
		
		switch(getCurrentStateID()){
			case (int)BipedalActions.stand_normal: 
		        break;
			case (int)BipedalActions.forward_walk: 
		        break;
			case (int)BipedalActions.forward_run: 
		        break;
			case (int)BipedalActions.forward_somersault: 
		        break;
			case (int)BipedalActions.jump: 
		        break;
			case (int)BipedalActions.back_walk: 
		        break;
			case (int)BipedalActions.stand_defense: 
			//	behaviour.bodySpringKp = 0;
		        break;
			case (int)BipedalActions.defense_walk: 
		        break;
			case (int)BipedalActions.free_walk:
				break;
			case (int)BipedalActions.punch1: 
				StartCoroutine("isFightingToFalse");
		        break;
			case (int)BipedalActions.punch2: 
				StartCoroutine("isFightingToFalse");
		        break;
			case (int)BipedalActions.punch3: 
				StartCoroutine("isFightingToFalse");
		        break;
			case (int)BipedalActions.punch4: 
				StartCoroutine("isFightingToFalse");
		        break;
			case (int)BipedalActions.kick1: 
				StartCoroutine("isFightingToFalse");
		        break;
			case (int)BipedalActions.kick2: 
				StartCoroutine("isFightingToFalse");
		        break;
			case (int)BipedalActions.kick3: 
				StartCoroutine("isFightingToFalse");
		        break;
			case (int)BipedalActions.forward_jump: 
		        break;
			case (int)BipedalActions.standup1: 
				StartCoroutine("isFightingToFalse");
				break;
			case (int)BipedalActions.standup2: 
				StartCoroutine("isFightingToFalse");
				break;
			case (int)BipedalActions.standup3: 
				break;
			case (int)BipedalActions.standup4: 
		        break;
			case (int)BipedalActions.fall_down: 
		        break;
			case (int)BipedalActions.hurt: 
				isHurting = false;
		        break;
			case (int)BipedalActions.climb_up_end2:
				if(isPicking){
					//endAllPick();
					//startWalkToAim(null, 1, true, SimGlobals.LEFT_STANCE);
				}
		        break;
			case (int)BipedalActions.weaponHit1: 
				StartCoroutine("isFightingToFalse");
				break;
			case (int)BipedalActions.weaponHit2: 
				StartCoroutine("isFightingToFalse");
				break;
			case (int)BipedalActions.weaponHit3: 
				StartCoroutine("isFightingToFalse");
				break;
		}

		fightingTimer = 0;
		standUpTimer = 0;
    }
	
	virtual protected void ResponseHitOthersFeedback(HitFeedback feedback, ContactPointInfo cp){
	}

	virtual protected void ResponseHitLimbsFeedback(HitFeedback feedback, ContactPointInfo cp){
		if(controller != null && !isFallDown && !isReaching && !isPicking){
		//	Debug.Log(feedback.ARB.name + " mmmmmmmmmm  "+feedback.force.magnitude + " mmmmmm  "+controller.getV().x);
			if(((feedback.feedbackARB == controller.getLFoot() && Vector3.Dot(cp.n, SimGlobals.upAxis) > 0.7f) || (feedback.feedbackARB == controller.getRFoot() && Vector3.Dot(cp.n, SimGlobals.upAxis) > 0.7f)) && getCurrentStateID()==(int)BipedalActions.falling){
				//if(feedback.force.magnitude > 23 && controller.getV().x > 0){
				//	startTouchRoll();
				//}else{
				if (hurtFeedback == null) {
					//startHurt (feedback);
					/*RFGameManager.instance.addExperience (4 + GameAttribute.gameAttribute.loadPlayerLevel ());
					if (type == "Player" && lShoes != null && rShoes != null) {
						lShoes.reduceShoesWastageRate (0.005f * feedback.force.magnitude);
						rShoes.reduceShoesWastageRate (0.005f * feedback.force.magnitude);
					}*/
				} else {
					if (feedback.power > hurtFeedback.power) {
						//startHurt (feedback);
						/*RFGameManager.instance.addExperience (4 + GameAttribute.gameAttribute.loadPlayerLevel ());
						if (type == "Player" && lShoes != null && rShoes != null) {
							lShoes.reduceShoesWastageRate (0.005f * feedback.force.magnitude);
							rShoes.reduceShoesWastageRate (0.005f * feedback.force.magnitude);
						}*/
					} else {
						startStand ();
					}
				}
				//}
			}else if((feedback.feedbackARB == controller.getHead() || feedback.feedbackARB == controller.getTorso() || feedback.feedbackARB == character.getRoot()) && (getCurrentStateID()==(int)BipedalActions.falling || getCurrentStateID()==(int)BipedalActions.falling)){
				if(hurtFeedback == null){
					//startHurt(feedback);
					//RFGameManager.instance.addExperience(4 + GameAttribute.gameAttribute.loadPlayerLevel());
				}else{
					if(feedback.power > hurtFeedback.power){
						//startHurt(feedback);
						//RFGameManager.instance.addExperience(4 + GameAttribute.gameAttribute.loadPlayerLevel());
					} else {
						startStand ();
					}
				}
			}
		}
	}
	
	virtual public void startHurt(HitFeedback feedback){
		if(isFallDown) return;
		
		hurtTimer = 0;
		controller.setIsFreeHeading(true);
		controller.gotoState((int)BipedalActions.hurt);
		
		hurtFeedback = feedback;
		controller.lookAtTargetPosition = Vector3.zero;
		controller.setDoubleStanceMode(false);
		character.setBodySpringStuff(feedback.feedbackARB, feedback.bodySpringKp, feedback.bodySpringKd, true);
		
		if(!feedback.feedbackARB.name.Contains("Arm") && !feedback.feedbackARB.name.Contains("Leg")){
			setHandTouchState(HandTouchState.touch, feedback.feedbackARB.gameObject, feedback.localPos, HandTouchMode.autoHand);
		}
		
		if(hurtFeedback.isLookHit){
			if(hurtFeedback.feedbackARB.name.Contains("Arm")){
				setLookState(HeadLookState.lookTarget, hurtFeedback.feedbackARB.gameObject, hurtFeedback.localPos);
			}else{
				setLookState(HeadLookState.none, null, Vector3.zero);
			}
		}else{
			setLookState(HeadLookState.none, null, Vector3.zero);
		}
	}
	
	virtual public void getHurt(){
		if(hurtFeedback != null){
			if(!isFallDown && !hurtFeedback.isTouchDown){
				//Debug.Log("ffffffffffhhhhhhhhhhhhh");
				//endAllPick();
			}
		//	Debug.Log("hhhhhhhhhhhhh");
			behaviour.belenceError = 0.05f;
			behaviour.impactBody = hurtFeedback.feedbackARB;
			behaviour.bodySpringKp = hurtFeedback.bodySpringKp;
			behaviour.bodySpringKd = hurtFeedback.bodySpringKd;
			behaviour.rootExternalForceKp = hurtFeedback.rootExternalForceKp;
			behaviour.rootExternalForceKd = hurtFeedback.rootExternalForceKd;
			behaviour.rootExternalTorqueKp = hurtFeedback.rootExternalTorqueKp;
			behaviour.rootExternalTorqueKd = hurtFeedback.rootExternalTorqueKd;
		}
	}
	virtual public void endHurt(){
		if(hurtFeedback != null && hurtFeedback.isLookHit){
			setLookState(HeadLookState.none, null, Vector3.zero);
		}
		setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		hurtFeedback = null;
		hurtTimer = 0;
		//Debug.Log("========== is end hurt ======== ");
		if(!isFallDown){
			//Debug.Log("eeeeeeeeeeeeeeee");
			startStand();
		}
	}
	
	virtual public void startStand(){
		controller.gotoState((int)BipedalActions.stand_normal, true, controller.getStance());
	}
	virtual public void startWalk(bool immediately = true){
		setLookState(HeadLookState.none, null, Vector3.zero);
		setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		controller.gotoState((int)BipedalActions.forward_walk, immediately);
	}
	virtual public void startWalkBack(bool immediately = true){
		setLookState(HeadLookState.none, null, Vector3.zero);
		setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		controller.gotoState((int)BipedalActions.back_walk, immediately);
	}
	virtual public void startRun(bool immediately = true){	
		setLookState(HeadLookState.none, null, Vector3.zero);
		setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		controller.gotoState((int)BipedalActions.forward_run, immediately);
	}
	virtual public void startDefenseStand(){
		setLookState(HeadLookState.none, null, Vector3.zero);
		setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		controller.gotoState((int)BipedalActions.stand_defense);
	}
	virtual public void startDefenseWalk(float velS, float velC){
		setLookState(HeadLookState.none, null, Vector3.zero);
		setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		behaviour.velDSagittal = velS;
		behaviour.velDCoronal = velC;
		controller.gotoState((int)BipedalActions.defense_walk);
	}
	virtual public void startFreeWalk(float velS, float VelC){
		setLookState(HeadLookState.none, null, Vector3.zero);
		setHandTouchState(HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		behaviour.velDSagittal = velS;
		behaviour.velDCoronal = VelC;
		controller.gotoState((int)BipedalActions.free_walk);
	}
	virtual public void startJump(){
		controller.gotoState((int)BipedalActions.jump);
	}
	virtual public void startForwardJump(){
		controller.gotoState((int)BipedalActions.forward_jump, false);
	}
	virtual public void startForwardSomersault(){	
		controller.gotoState((int)BipedalActions.forward_somersault, false);
	}
	virtual public void startFallDown(){
		controller.setIsFreeHeading(true);
		controller.gotoState((int)BipedalActions.fall_down);
	}
	virtual public void startStandUp(int standID){
		controller.gotoState(standID);
	}
	virtual public void startInWaterStand(){
		controller.gotoState((int)BipedalActions.inWaterStand);
	}
	virtual public void startInWaterSwiming(){
		controller.gotoState((int)BipedalActions.inWaterSwiming);
	}

	
	private void setRBStateFromEngine(){
		CWRigidBody arb;
		Quaternion bodyQua;
		
		for (int i=0;i<character.getArticulatedRigidBodyCount();i++){
			arb = character.getArticulatedRigidBody(i);
			arb.setCMPosition(arb.GetComponent<Rigidbody>().position);
			
			bodyQua = arb.GetComponent<Rigidbody>().rotation;
			arb.setOrientation(bodyQua);
			
			arb.setCMVelocity(arb.GetComponent<Rigidbody>().velocity);
			arb.setAngularVelocity(arb.GetComponent<Rigidbody>().angularVelocity);
			arb.CWRBUpdate();
		}
	}
	
	public Character getCharacter(){
		return character;
	}
	public IKVMCController getController(){
		return controller;
	}
	public BehaviourController getBehaviour(){
		return behaviour;
	}

	public float getLife(){
		return life;
	}

	public void setLife(float num){
		life = Mathf.Clamp(num, 0, 100);

		if(life > 0){
			isDead = false;
		}else{
			isDead = true;
		}
	}

	public void addLife(float num){
		life = Mathf.Clamp(life + num, 0, 100);

		if(life <= 0){
			isDead = true;
		}else{
			isDead = false;
		}
	}
	
	public int getCurrentStateID(){
		return controller.getFSMState();
	}
	
	public bool getCharacterIsReady(){
		return characterIsReady;
	}

	public bool getIsDead(){
		return isDead;
	}

	public bool getIsIndojo(){
		return isIndojo;
	}
	public void setIsIndojo(bool indojo){
		isIndojo = indojo;
	}
		
	public bool getIsFallDown(){
		return isFallDown;
	}
	
	public bool getIsStanding(){
		return isStanding;
	}

	public bool getIsStableStanding(){
		return isStableStanding;
	}
	
	public bool getIsMoveing(){
		return isMoveing;
	}
	
	public bool getIsJumping(){
		return isJumping;
	}
	
	public bool getIsFighting(){
		return isFighting;
	}

	public bool getIsDefense(){
		return isDefense;
	}
	public void setIsDefense(bool d){
		isDefense = d;
	}
	
	public bool getIsHurting(){
		return isHurting;
	}

	public bool getIsInWater(){
		return isInWater;
	}
	public void setIsInWater(bool isIn){
		isInWater = isIn;
	}
	
	public bool getIsVigilance(){
		return isVigilance;
	}
	public void setIsVigilance(bool isvig){
		isVigilance = isvig;
	}
	
	public bool getIsReaching(){
		return isReaching;
	}
	
	public bool getIsPicking(){
		return isPicking;
	}

	public bool getIsWalkToAim(){
		return isWalkToAim;
	}
	
	public bool getIsDragging(){
		return isDragging;
	}
	public void setIsDragging(bool drag, bool isFreeHeading){
		isDragging = drag;
		if (!drag) {
			StartCoroutine("lockCurrentHeading");
			setLookState (HeadLookState.none, null, Vector3.zero);
			setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		} else {
			controller.setIsFreeHeading (isFreeHeading);
		}
	}
	public DragBodies getDragBody(){
		return dragBody;
	}

	public void setIsIgnoreCollisionWithOthers(bool flag){
		isIgnoreCollisionWithOthers = flag;
	}

	public float getStandUpTimer(){
		return standUpTimer;
	}
	
	public GameObject getCurrentPickTarget(){
		return currentPickTarget;
	}
	
	public HitFeedback getHurtFeedback(){
		return hurtFeedback;
	}

	public AudioSource getHandSoundSource(){
		return lHandSoundSource;
	}
	
	public SimpleRpgCamera getCameraControler(){
		return cameraControler;
	}
	/*
	public bool isHavePickerTarget(GameObject target){
		for(int i=0; i<pickerTargets.Count; i++){
			GameObject obj = pickerTargets[i] as GameObject;
			if(obj == target){
				return true;
			}
		}
		return false;
	}
	
	public void addPickerTarget(GameObject target){
		if(!isHavePickerTarget(target)){
			pickerTargets.Add(target);
		}
	}
	public void removePickerTarget(GameObject target){
		if(isHavePickerTarget(target)){
			if(currentPickTarget == target){
				endAllReach();
				endAllPick();
				startStand();
			}
			pickerTargets.Remove(target);
		}
	}

	public GameObject getClosestPickerTarget(Vector3 closestPosition, PickerDirection dir){
		int targetCount = pickerTargets.Count;
		if(targetCount == 0) return null;
		float leastDis = 1000000;
		GameObject leastObj = null;
		int pid = -1;
		Vector3 closestPickerPos;
		for(int i=0; i<targetCount; i++){
			GameObject obj = pickerTargets[i] as GameObject;
			Picker picker = obj.GetComponent("Picker") as Picker;
			closestPickerPos = picker.getClosestPickerPosition(closestPosition, dir, ref pid);
			float dis = (closestPosition - closestPickerPos).magnitude;
			if(pid >= 0 && dis < leastDis){
				leastDis = dis;
				leastObj = pickerTargets[i] as GameObject;
			}
		}
		
		return leastObj;
	}*/
	/*
	public void startReach(GameObject target, bool lHandIgnoreCollision = true, bool rHandIgnoreCollision = true, bool lFootIgnoreCollision = true, bool rFootIgnoreCollision = true){
		isReaching = true;
		currentPickTarget = target;
		currentPickHandCount = 0;
		if(currentPickTarget != null){
			//Picker picker = currentPickTarget.GetComponent("Picker") as Picker;
			GameObject colliderObj = null;
			if (currentPickTarget.tag == "picker_grabs") {
				isUpdatePickerPosition = false;
				colliderObj = currentPickTarget.transform.parent.gameObject;
			} else if (currentPickTarget.tag == "picker_ladder_ground") {
				isUpdatePickerPosition = false;
				colliderObj = currentPickTarget;
			} else if (currentPickTarget.tag == "picker_rope") {
				isUpdatePickerPosition = true;
				colliderObj = null;
			} else if (currentPickTarget.tag == "picker_props_ground") {
				isUpdatePickerPosition = false;
				colliderObj = currentPickTarget;
			} else if (currentPickTarget.tag == "picker_weapons") {
				isUpdatePickerPosition = true;
				colliderObj = null;
				ColdWeapon wp = currentPickTarget.GetComponent<ColdWeapon> ();
				foreach (GameObject obj in wp.colliders) {
					Physics.IgnoreCollision(lLowerArm.GetComponent<Collider>(), obj.GetComponent<Collider>());
					Physics.IgnoreCollision(rLowerArm.GetComponent<Collider>(), obj.GetComponent<Collider>());
				}
			}
			if(colliderObj != null){
				if(lHandIgnoreCollision)
					Physics.IgnoreCollision(lLowerArm.GetComponent<Collider>(), colliderObj.GetComponent<Collider>());
				if(rHandIgnoreCollision)
					Physics.IgnoreCollision(rLowerArm.GetComponent<Collider>(), colliderObj.GetComponent<Collider>());
				if(lFootIgnoreCollision)
					Physics.IgnoreCollision(lLowerLeg.GetComponent<Collider>(), colliderObj.GetComponent<Collider>());
				if(rFootIgnoreCollision)
					Physics.IgnoreCollision(rLowerLeg.GetComponent<Collider>(), colliderObj.GetComponent<Collider>());
			}
		}
	}
	*/
	/*public void updateReach(){
		if(isReaching && currentPickTarget != null){
			Picker picker = currentPickTarget.GetComponent("Picker") as Picker;
			if(picker.enableLeftHand && !controller.getLHand().isFixedPosition()){
				if(picker.upperClosestLimb != null){
					controller.leftArmReachTargetPosition = picker.getClosestPickerPositionWithLimbs(picker.upperClosestLimb, picker.upperDirection, ref picker.leftHandAtPickerIndex);
					//Debug.Log("leftArmReachTargetPosition  === "+ controller.leftArmReachTargetPosition);
				}
				if((controller.leftArmReachTargetPosition - controller.getLHand().getWorldBottomPosition()).magnitude<=picker.pickDistance){
					startPick(controller.getLHand(), controller.leftArmReachTargetPosition);
				}
			}
			if(picker.enableRightHand && !controller.getRHand().isFixedPosition()){
				if(picker.upperClosestLimb != null){
					controller.rightArmReachTargetPosition = picker.getClosestPickerPositionWithLimbs(picker.upperClosestLimb, picker.upperDirection, ref picker.rightHandAtPickerIndex);
					//Debug.Log("rightArmReachTargetPosition  === "+ controller.rightArmReachTargetPosition);
				}
				if((controller.rightArmReachTargetPosition - controller.getRHand().getWorldBottomPosition()).magnitude<=picker.pickDistance){
					startPick(controller.getRHand(), controller.rightArmReachTargetPosition);
				}
			}
			if(picker.enableLeftFoot && !controller.getLFoot().isFixedPosition()){
				if(picker.lowerClosestLimb != null){
					controller.leftLegReachTargetPosition = picker.getClosestPickerPositionWithLimbs(picker.lowerClosestLimb, picker.lowerDirection, ref picker.leftFootAtPickerIndex);
					//Debug.Log("leftLegReachTargetPosition  === "+ controller.leftLegReachTargetPosition);
				}
				if((controller.leftLegReachTargetPosition - controller.getLFoot().getWorldBottomPosition()).magnitude<=picker.pickDistance){
					startPick(controller.getLFoot(), controller.leftLegReachTargetPosition);
				}
			}
			if(picker.enableRightFoot && !controller.getRFoot().isFixedPosition()){
				if(picker.lowerClosestLimb != null){
					controller.rightLegReachTargetPosition = picker.getClosestPickerPositionWithLimbs(picker.lowerClosestLimb, picker.lowerDirection, ref picker.rightFootAtPickerIndex);
					//Debug.Log("rightLegReachTargetPosition  === "+ controller.rightLegReachTargetPosition);
				}
				if((controller.rightLegReachTargetPosition - controller.getRFoot().getWorldBottomPosition()).magnitude<=picker.pickDistance){
					startPick(controller.getRFoot(), controller.rightLegReachTargetPosition);
				}
			}
		}
	}
	*/
	/*public void endAllReach(){
		isReaching = false;
		//isJumping = false;
		controller.leftArmReachTargetPosition = Vector3.zero;
		controller.rightArmReachTargetPosition = Vector3.zero;
		controller.leftLegReachTargetPosition = Vector3.zero;
		controller.rightLegReachTargetPosition = Vector3.zero;
	}
	*/
/*
	public void startPick(CWRigidBody hand, Vector3 targetPos){
		if(hand.isFixedPosition()) return;
		
		isJumping = false;
		currentPickHandCount += 1;
		Picker picker = currentPickTarget.GetComponent("Picker") as Picker;
		hand.addFixedPositionJoint(currentPickTarget, targetPos, 500, 10);
		if(currentPickHandCount >= picker.getHandCount()){
			isPicking = true;
			if (currentPickTarget.tag == "picker_grabs") {
				if (climbState == ClimbLadderState.climbUpEnd2) {
					endAllReach ();
					endPick (controller.getLHand ());
					endPick (controller.getRHand ());
					controller.getCurrentState ().startManualTransition ();
				} else if (climbState == ClimbLadderState.climbDownStart1) {
					endAllReach ();
					if (hand == controller.getLFoot ()) {
						startClimbDownStart1 (SimGlobals.RIGHT_STANCE);
					} else if (hand == controller.getRFoot ()) {
						startClimbDownStart1 (SimGlobals.LEFT_STANCE);
					}
				} else if (climbState == ClimbLadderState.climbDownStart2) {
					endAllReach ();
					if (controller.getLFoot ().isFixedPosition ()) {
						endPick (controller.getLFoot ());
						startClimbDownStart3 (SimGlobals.RIGHT_STANCE);
					} else if (controller.getRFoot ().isFixedPosition ()) {
						endPick (controller.getRFoot ());
						startClimbDownStart3 (SimGlobals.LEFT_STANCE);
					}
				}
			} else if (currentPickTarget.tag == "picker_ladder_ground") {
				if (climbState == ClimbLadderState.none) {
					endAllReach ();
					startClimbStand ();
				} else if (climbState == ClimbLadderState.startToClimbStand) {
					endAllReach ();
					if (Mathf.Abs (picker.leftHandAtPickerIndex - picker.leftFootAtPickerIndex) > 2 || Mathf.Abs (picker.rightHandAtPickerIndex - picker.leftFootAtPickerIndex) > 2) {
						endPick (controller.getLHand (), true);
						endPick (controller.getRHand (), true);
						checkForReach (pelvis.transform.position);
					} else {
						//if(!checkOnPickerTop(pelvis.transform.position)){
						climbState = ClimbLadderState.climbStand;
						picker.updateLimbsEnable (false, false, false, false);
						//}
					}
				} else if (climbState == ClimbLadderState.climbUp) {
					endAllReach ();
					if (!checkOnPickerTop (pelvis.transform.position)) {
						endPick (controller.getSwingHand (), true);
						endPick (controller.getStanceFoot (), true);
						controller.getCurrentState ().startManualTransition ();
					}
				} else if (climbState == ClimbLadderState.climbDown) {
					endAllReach ();
					if (!checkOnPickerStart (pelvis.transform.position)) {
						endPick (controller.getSwingHand (), true);
						endPick (controller.getStanceFoot (), true);
						controller.getCurrentState ().startManualTransition ();
					}
				} else if (climbState == ClimbLadderState.climbUpEnd1) {
					endAllReach ();
					endPick (controller.getLHand ());
					endPick (controller.getRHand ());
					if (hand == controller.getLFoot ()) {
						endPick (controller.getRFoot ());
						startClimbUpEnd2 (SimGlobals.RIGHT_STANCE);
					} else if (hand == controller.getRFoot ()) {
						endPick (controller.getLFoot ());
						startClimbUpEnd2 (SimGlobals.LEFT_STANCE);
					}
				} else if (climbState == ClimbLadderState.climbUpToStand) {
					endAllReach ();
					if (!checkOnPickerTop (pelvis.transform.position)) {
						climbState = ClimbLadderState.climbStand;
						picker.updateLimbsEnable (false, false, false, false);
						controller.gotoState ((int)BipedalActions.climb_stand);
					}
				} else if (climbState == ClimbLadderState.climbDownToStand) {
					endAllReach ();
					if (!checkOnPickerStart (pelvis.transform.position)) {
						climbState = ClimbLadderState.climbStand;
						picker.updateLimbsEnable (false, false, false, false);
						controller.gotoState ((int)BipedalActions.climb_stand);
					}
				} else if (climbState == ClimbLadderState.climbDownStart1) {
					endAllReach ();
					if (hand == controller.getLFoot ()) {
						startClimbDownStart1 (SimGlobals.RIGHT_STANCE);
					} else if (hand == controller.getRFoot ()) {
						startClimbDownStart1 (SimGlobals.LEFT_STANCE);
					}
				} else if (climbState == ClimbLadderState.climbDownStart2) {
					endAllReach ();
					if (controller.getLFoot ().isFixedPosition ()) {
						endPick (controller.getLFoot ());
						startClimbDownStart2 (SimGlobals.LEFT_STANCE);
					} else if (controller.getRFoot ().isFixedPosition ()) {
						endPick (controller.getRFoot ());
						startClimbDownStart2 (SimGlobals.LEFT_STANCE);
					}
				} else if (climbState == ClimbLadderState.climbDownStart3) {
					endAllReach ();
					climbState = ClimbLadderState.climbDownStart4;
				} else if (climbState == ClimbLadderState.climbDownStart4) {
					endAllReach ();
					climbState = ClimbLadderState.climbDownStart5;
				}
			} else if (currentPickTarget.tag == "picker_rope") {
				if (climbState == ClimbLadderState.none) {
					endAllReach ();
					endPick (controller.getLHand ());
					startHangStand ();
				} else if (climbState == ClimbLadderState.startToClimbStand) {
					endAllReach ();
					climbState = ClimbLadderState.climbStand;
				} else if (climbState == ClimbLadderState.climbUp) {
					endAllReach ();
					if (!checkOnPickerTop (controller.getStanceHand ().getWorldBottomPosition (), PickerDirection.up, true)) {
						endPick (controller.getStanceHand (), true);
						controller.getCurrentState ().startManualTransition ();
					} else {
						climbState = ClimbLadderState.climbStand;
						picker.updateLimbsEnable (false, false, false, false);
						if (hand == controller.getLHand ()) {
							controller.setIsLeftReverseStance (false);
						} else if (hand == controller.getRHand ()) {
							controller.setIsLeftReverseStance (true);
						}
						controller.gotoState ((int)BipedalActions.hang_stand);
					}
				} else if (climbState == ClimbLadderState.climbUpToStand) {
					endAllReach ();
					//if(!checkOnPickerTop(controller.getSwingHand().getWorldBottomPosition(), PickerDirection.none, true)){
					climbState = ClimbLadderState.climbStand;
					picker.updateLimbsEnable (false, false, false, false);
					if (hand == controller.getLHand ()) {
						controller.setIsLeftReverseStance (false);
					} else if (hand == controller.getRHand ()) {
						controller.setIsLeftReverseStance (true);
					}
					controller.gotoState ((int)BipedalActions.hang_stand);
					//}
				} else if (climbState == ClimbLadderState.climbDown) {
					endAllReach ();
					if (!checkOnPickerStart (hand.getWorldBottomPosition (), PickerDirection.none, true)) {
						endPick (controller.getStanceHand (), true);
						controller.getCurrentState ().startManualTransition ();
					}
				} else if (climbState == ClimbLadderState.climbDownToStand) {
					endAllReach ();
					if (!checkOnPickerStart (hand.getWorldBottomPosition (), PickerDirection.none, true)) {
						climbState = ClimbLadderState.climbStand;
						picker.updateLimbsEnable (false, false, false, false);
						if (hand == controller.getLHand ()) {
							controller.setIsLeftReverseStance (true);
						} else if (hand == controller.getRHand ()) {
							controller.setIsLeftReverseStance (false);
						}
						controller.gotoState ((int)BipedalActions.hang_stand);
					}
				}
			} else if (currentPickTarget.tag == "picker_weapons") {
				
			}
		}
	}
	
	public void endPick(CWRigidBody hand, bool isIgnoreCollision = false){
		hand.removeFixedPositionJoint();
		if(currentPickTarget != null){
			GameObject colliderObj = null;
			if(currentPickTarget.tag == "picker_grabs"){
				colliderObj = currentPickTarget.transform.parent.gameObject;
			}else if(currentPickTarget.tag == "picker_ladder_ground"){
				colliderObj = currentPickTarget;
			}
			if(colliderObj != null){
				Physics.IgnoreCollision(hand.gameObject.GetComponent<Collider>(), colliderObj.GetComponent<Collider>(), isIgnoreCollision);
			}
		}
	}
	public void endAllPick(){
		endPick(controller.getLHand(), false);
		endPick(controller.getRHand(), false);
		endPick(controller.getLFoot(), false);
		endPick(controller.getRFoot(), false);
		
		climbState = ClimbLadderState.none;
		isPicking = false;
		currentPickTarget = null;
	}*/
	/*
	protected bool checkOnPickerStart(Vector3 closestPosition, PickerDirection dir = PickerDirection.none, bool renewTarget = false){
		GameObject target = (currentPickTarget != null && !renewTarget)?currentPickTarget:getClosestPickerTarget(closestPosition, dir);
		if(target != null){
			Picker picker = target.GetComponent("Picker") as Picker;
			if(target.GetComponent<Collider>().tag == "picker_ladder_ground"){
				if(picker.leftFootAtPickerIndex == 0 || picker.rightFootAtPickerIndex == 0){
					endAllReach();
					endAllPick();
					startStand();
					return true;
				}
			}else if(target.GetComponent<Collider>().tag == "picker_rope"){
				if(picker.pickerID == picker.relatedPickerCount - 1){
					endAllReach();
					endAllPick();
					startStand();
					return true;
				}
			}
		}
		return false;
	}*/
	/*
	protected bool checkOnPickerTop(Vector3 closestPosition, PickerDirection dir = PickerDirection.none, bool renewTarget = false){
		GameObject target = (currentPickTarget != null && !renewTarget)?currentPickTarget:getClosestPickerTarget(closestPosition, dir);
		if(target != null){
			Picker picker = target.GetComponent("Picker") as Picker;
			if(target.GetComponent<Collider>().tag == "picker_ladder_ground"){
				if(picker.leftHandAtPickerIndex == picker.getPickerPositionCount()-1 && picker.rightHandAtPickerIndex == picker.getPickerPositionCount()-1){
					endAllReach();
					if(picker.leftFootAtPickerIndex > picker.rightFootAtPickerIndex){
						endPick(controller.getRFoot());
						startClimbUpEnd1(SimGlobals.LEFT_STANCE);
					}else{
						endPick(controller.getLFoot());
						startClimbUpEnd1(SimGlobals.RIGHT_STANCE);
					}
					return true;
				}
			}else if(target.GetComponent<Collider>().tag == "picker_rope"){
				if(picker.pickerID <= 1){
					return true;
				}
			}
		}
		return false;
	}*/
	/*
	protected bool checkForReach(Vector3 closestPosition, PickerDirection dir = PickerDirection.none, bool renewTarget = false){
		GameObject target = (currentPickTarget != null && !renewTarget)?currentPickTarget:getClosestPickerTarget(closestPosition, dir);
		if(target != null){
			//Debug.Log("aaaaaaaaa  "+target.name);
			Picker picker = target.GetComponent("Picker") as Picker;
			if (target.tag == "picker_grabs" && isFaceToPosition (target.transform.position)) {
				if (climbState == ClimbLadderState.none) {
					picker.updateLimbsEnable (true, true, false, false);
					picker.updateClosestToPickerPosition (controller.getTorso (), PickerDirection.up, null, PickerDirection.none);
					int pid = -1;
					picker.getClosestPickerPositionWithLimbs (picker.upperClosestLimb, picker.upperDirection, ref pid);
					if (pid >= 0) {
						startReach (target, true, true, false, false);
						return true;
					}
				} else if (climbState == ClimbLadderState.climbUpEnd1) {
					climbState = ClimbLadderState.climbUpEnd2;
					picker.updateLimbsEnable (false, false, false, true);
					picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
					controller.rightLegReachTargetPosition = picker.getClosestPickerPosition (controller.getLHand ().getWorldBottomPosition (), PickerDirection.none, ref picker.rightFootAtPickerIndex);
					startReach (target, false, false, false, true);
					return true;
				}
			} else if (target.tag == "picker_ladder_ground" && isFaceToPosition (target.transform.position)) {
				if (climbState == ClimbLadderState.none) {
					picker.updateLimbsEnable (true, true, false, false);
					picker.updateClosestToPickerPosition (controller.getTorso (), PickerDirection.up, null, PickerDirection.none);
					picker.pickDistance = 0.6f;
					int pid = -1;
					Vector3 closestPos = picker.getClosestPickerPositionWithLimbs (picker.upperClosestLimb, picker.upperDirection, ref pid);
					if (pid >= 0) {
						float dis = Mathf.Abs ((closestPos - picker.upperClosestLimb.getCMPosition ()).x);
						if ((isStanding || isMoveing) && dis > controller.getArmLength ()) {
							//startWalkToAim(target, dis - controller.getArmLength(), true, -1);
						} else {
							startReach (target, true, true, false, false);
						}
						return true;
					}
				} else if (climbState == ClimbLadderState.startToClimbStand) {
					picker.updateLimbsEnable (true, true, false, false);
					picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
					picker.pickDistance = 0.8f;
					controller.leftArmReachTargetPosition = picker.getClosestPickerPositionWithPickerID (picker.leftFootAtPickerIndex, 1, PickerDirection.up, ref picker.leftHandAtPickerIndex);
					controller.rightArmReachTargetPosition = picker.getClosestPickerPositionWithPickerID (picker.leftFootAtPickerIndex, 1, PickerDirection.up, ref picker.rightHandAtPickerIndex);
					if (picker.leftHandAtPickerIndex >= 0 && picker.rightHandAtPickerIndex >= 0) {
						startReach (target);
						return true;
					}
				} else if (climbState == ClimbLadderState.climbUp || climbState == ClimbLadderState.climbUpToStand) {
					if (controller.getStance () == SimGlobals.LEFT_STANCE) {
						picker.updateLimbsEnable (true, false, false, true);
						picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.leftArmReachTargetPosition = picker.getClosestPickerPosition (controller.getRHand ().getWorldBottomPosition (), PickerDirection.up, ref picker.leftHandAtPickerIndex);
						controller.rightLegReachTargetPosition = picker.getClosestPickerPosition (controller.getLFoot ().getWorldBottomPosition (), PickerDirection.up, ref picker.rightFootAtPickerIndex);
						
						if (picker.leftHandAtPickerIndex < 0 || (picker.leftHandAtPickerIndex >= 0 && Mathf.Abs (picker.leftHandAtPickerIndex - picker.leftFootAtPickerIndex) > 3)) {
							controller.leftArmReachTargetPosition = picker.getClosestPickerPosition (controller.getRHand ().getWorldBottomPosition (), PickerDirection.none, ref picker.leftHandAtPickerIndex);
						}
						if (picker.rightFootAtPickerIndex == picker.rightHandAtPickerIndex) {
							controller.rightLegReachTargetPosition = picker.getClosestPickerPosition (controller.getLFoot ().getWorldBottomPosition (), PickerDirection.none, ref picker.rightFootAtPickerIndex);
						}
						if (picker.leftHandAtPickerIndex >= 0 && picker.rightFootAtPickerIndex >= 0) {
							startReach (target);
							return true;
						}
					} else {
						picker.updateLimbsEnable (false, true, true, false);
						picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.rightArmReachTargetPosition = picker.getClosestPickerPosition (controller.getLHand ().getWorldBottomPosition (), PickerDirection.up, ref picker.rightHandAtPickerIndex);
						controller.leftLegReachTargetPosition = picker.getClosestPickerPosition (controller.getRFoot ().getWorldBottomPosition (), PickerDirection.up, ref picker.leftFootAtPickerIndex);
						
						if (picker.rightHandAtPickerIndex < 0 || (picker.rightHandAtPickerIndex >= 0 && Mathf.Abs (picker.rightHandAtPickerIndex - picker.rightFootAtPickerIndex) > 3)) {
							controller.rightArmReachTargetPosition = picker.getClosestPickerPosition (controller.getLHand ().getWorldBottomPosition (), PickerDirection.none, ref picker.rightHandAtPickerIndex);
						}
						if (picker.leftFootAtPickerIndex == picker.leftHandAtPickerIndex) {
							controller.leftLegReachTargetPosition = picker.getClosestPickerPosition (controller.getRFoot ().getWorldBottomPosition (), PickerDirection.none, ref picker.leftFootAtPickerIndex);
						}
						if (picker.rightHandAtPickerIndex >= 0 && picker.leftFootAtPickerIndex >= 0) {
							startReach (target);
							return true;
						}
					}
				} else if (climbState == ClimbLadderState.climbDown || climbState == ClimbLadderState.climbDownToStand) {
					if (controller.getStance () == SimGlobals.LEFT_STANCE) {
						picker.updateLimbsEnable (true, false, false, true);
						picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.6f;
						controller.leftArmReachTargetPosition = picker.getClosestPickerPosition (controller.getRHand ().getWorldBottomPosition (), PickerDirection.down, ref picker.leftHandAtPickerIndex);
						controller.rightLegReachTargetPosition = picker.getClosestPickerPosition (controller.getLFoot ().getWorldBottomPosition (), PickerDirection.down, ref picker.rightFootAtPickerIndex);
						
						if (picker.rightFootAtPickerIndex >= 0 && Mathf.Abs (picker.rightFootAtPickerIndex - picker.rightHandAtPickerIndex) > 3) {
							controller.rightLegReachTargetPosition = picker.getClosestPickerPosition (controller.getLFoot ().getWorldBottomPosition (), PickerDirection.none, ref picker.rightFootAtPickerIndex);
						}
						if (picker.leftHandAtPickerIndex == picker.leftFootAtPickerIndex) {
							controller.leftArmReachTargetPosition = picker.getClosestPickerPosition (controller.getRHand ().getWorldBottomPosition (), PickerDirection.none, ref picker.leftHandAtPickerIndex);
						}
						if (picker.leftHandAtPickerIndex >= 0 && picker.rightFootAtPickerIndex >= 0) {
							startReach (target);
							return true;
						}
					} else {
						picker.updateLimbsEnable (false, true, true, false);
						picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.6f;
						controller.rightArmReachTargetPosition = picker.getClosestPickerPosition (controller.getLHand ().getWorldBottomPosition (), PickerDirection.down, ref picker.rightHandAtPickerIndex);
						controller.leftLegReachTargetPosition = picker.getClosestPickerPosition (controller.getRFoot ().getWorldBottomPosition (), PickerDirection.down, ref picker.leftFootAtPickerIndex);
						
						if (picker.leftFootAtPickerIndex >= 0 && Mathf.Abs (picker.leftFootAtPickerIndex - picker.leftHandAtPickerIndex) > 3) {
							controller.leftLegReachTargetPosition = picker.getClosestPickerPosition (controller.getRFoot ().getWorldBottomPosition (), PickerDirection.none, ref picker.leftFootAtPickerIndex);
						}
						if (picker.rightHandAtPickerIndex == picker.rightFootAtPickerIndex) {
							controller.rightArmReachTargetPosition = picker.getClosestPickerPosition (controller.getLHand ().getWorldBottomPosition (), PickerDirection.none, ref picker.rightHandAtPickerIndex);
						}
						if (picker.rightHandAtPickerIndex >= 0 && picker.leftFootAtPickerIndex >= 0) {
							startReach (target);
							return true;
						}
					}
				} else if (climbState == ClimbLadderState.climbUpEnd1) {
					//climbState = ClimbLadderState.climbUpEnd2;
					if (controller.getLFoot ().isFixedPosition ()) {
						picker.updateLimbsEnable (false, false, false, true);
						picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.rightLegReachTargetPosition = picker.getClosestPickerPosition (controller.getLHand ().getWorldBottomPosition (), PickerDirection.none, ref picker.rightFootAtPickerIndex);
					} else {
						picker.updateLimbsEnable (false, false, true, false);
						picker.updateClosestToPickerPosition (null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.leftLegReachTargetPosition = picker.getClosestPickerPosition (controller.getLHand ().getWorldBottomPosition (), PickerDirection.none, ref picker.leftFootAtPickerIndex);
					}
					startReach (target);
					return true;
				}
			} else if (target.tag == "picker_rope") {
				if (climbState == ClimbLadderState.none && isFaceToPosition (target.transform.position, 0.5f)) {
					picker.updateLimbsEnable (true, true, false, false);
					picker.updateClosestToPickerPosition (controller.getTorso (), PickerDirection.up, null, PickerDirection.none);
					picker.pickDistance = 1.0f;
					startReach (target, false, false, false, false);
					return true;
				} else if (climbState == ClimbLadderState.startToClimbStand) {
					picker.updateLimbsEnable (true, false, false, false);
					picker.updateClosestToPickerPosition (controller.getRHand (), PickerDirection.down, null, PickerDirection.none);
					picker.pickDistance = 1.0f;
					int pid = -1;
					picker.getClosestPickerPositionWithLimbs (controller.getRHand (), PickerDirection.down, ref pid);
					if (pid < 0) {
						picker.updateClosestToPickerPosition (controller.getRHand (), PickerDirection.none, null, PickerDirection.none);
					}
					startReach (target, false, false, false, false);
					return true;
				} else if (climbState == ClimbLadderState.climbUp || climbState == ClimbLadderState.climbUpToStand) {
					if (controller.getStance () == SimGlobals.LEFT_STANCE) {
						picker.updateLimbsEnable (false, true, false, false);
						picker.updateClosestToPickerPosition (controller.getRHand (), PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 1.0f;
						startReach (target);
						return true;
					} else {
						picker.updateLimbsEnable (true, false, false, false);
						picker.updateClosestToPickerPosition (controller.getLHand (), PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 1.0f;
						startReach (target);
						return true;
					}
				} else if (climbState == ClimbLadderState.climbDown || climbState == ClimbLadderState.climbDownToStand) {
					if (controller.getStance () == SimGlobals.LEFT_STANCE) {
						picker.updateLimbsEnable (false, true, false, false);
						picker.updateClosestToPickerPosition (controller.getRHand (), PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 1.0f;
						startReach (target);
						return true;
					} else {
						picker.updateLimbsEnable (true, false, false, false);
						picker.updateClosestToPickerPosition (controller.getLHand (), PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 1.0f;
						startReach (target);
						return true;
					}
				}
			} else if (target.tag == "picker_weapons") {
				picker.updateLimbsEnable (true, false, false, false);
				picker.updateClosestToPickerPosition (controller.getRHand (), PickerDirection.none, null, PickerDirection.none);
				int pid = -1;
				picker.getClosestPickerPositionWithLimbs (picker.upperClosestLimb, picker.upperDirection, ref pid);
				if (pid >= 0) {
					startReach (target, true, true, false, false);
					return true;
				}
			} else {
				return false;
			}
		}else{
			if(getCurrentStateID() == (int)BipedalActions.hang_stand && climbState == ClimbLadderState.startToClimbStand){
				checkForReach(controller.getRHand().getWorldBottomPosition(), PickerDirection.none, true);
			}else if(getCurrentStateID() == (int)BipedalActions.hang_up){
				checkForReach(controller.getStanceHand().getWorldBottomPosition(), PickerDirection.none, true);
			}
		}
		return false;
	}
	
	protected bool checkForStartClimbDown(int stance = SimGlobals.LEFT_STANCE){
		GameObject target = (currentPickTarget != null)?currentPickTarget:getClosestPickerTarget(pelvis.transform.position, PickerDirection.none);
		if(target != null){
			Picker picker = target.GetComponent("Picker") as Picker;
			if(target.GetComponent<Collider>().tag == "picker_grabs"){
				if(climbState == ClimbLadderState.none && isStanding){
					int pid = -1;
					Vector3 closestPos = picker.getClosestPickerPosition(pelvis.transform.position, PickerDirection.down, ref pid);
					if(pid == picker.getPickerPositionCount()-1){
						//float dis = Mathf.Abs((closestPos - pelvis.transform.position).x);
						if(isFaceToPosition(closestPos)){
							startWalkToAim(target, closestPos, false, -1);
						}else{
							startWalkToAim(target, closestPos, false, -1);
						}
						return true;
					}
				}else if(climbState == ClimbLadderState.climbDownStart1){
					if(stance == SimGlobals.LEFT_STANCE){
						picker.updateLimbsEnable(false, false, true, false);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.leftLegReachTargetPosition = walkToAimPosition;
						startReach(target, false, false, true, true);
						return true;
					}else if(stance == SimGlobals.RIGHT_STANCE){
						picker.updateLimbsEnable(false, false, false, true);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.rightLegReachTargetPosition = walkToAimPosition;
						startReach(target, false, false, true, true);
						return true;
					}
				}else if(climbState == ClimbLadderState.climbDownStart2 || climbState == ClimbLadderState.climbDownStart3){
					picker.updateLimbsEnable(true, true, false, false);
					picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
					picker.pickDistance = 0.8f;
					controller.leftArmReachTargetPosition = walkToAimPosition;
					controller.rightArmReachTargetPosition = walkToAimPosition;
					startReach(target, true, true, false, false);
					return true;
				}
			}else if(target.GetComponent<Collider>().tag == "picker_ladder_ground"){
				if(climbState == ClimbLadderState.none && isStanding){
					int pid = -1;
					Vector3 closestPos = picker.getClosestPickerPosition(pelvis.transform.position, PickerDirection.down, ref pid);
					if(pid == picker.getPickerPositionCount()-1){
						//float dis = Mathf.Abs((closestPos - pelvis.transform.position).x);
						if(isFaceToPosition(closestPos)){
							startWalkToAim(target, closestPos, false, -1);
						}else{
							startWalkToAim(target, closestPos, false, -1);
						}
						return true;
					}
				}else if(climbState == ClimbLadderState.climbDownStart1){
					if(stance == SimGlobals.LEFT_STANCE){
						picker.updateLimbsEnable(false, false, true, false);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.leftLegReachTargetPosition = walkToAimPosition;
						startReach(target, false, false, true, true);
						return true;
					}else if(stance == SimGlobals.RIGHT_STANCE){
						picker.updateLimbsEnable(false, false, false, true);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.rightLegReachTargetPosition = walkToAimPosition;
						startReach(target, false, false, true, true);
						return true;
					}
				}else if(climbState == ClimbLadderState.climbDownStart2){
					picker.updateLimbsEnable(true, true, false, false);
					picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
					picker.leftHandAtPickerIndex = picker.getPickerPositionCount()-1;
					picker.rightHandAtPickerIndex = picker.getPickerPositionCount()-1;
					picker.pickDistance = 0.8f;
					controller.leftArmReachTargetPosition = walkToAimPosition;
					controller.rightArmReachTargetPosition = walkToAimPosition;
					startReach(target, true, true, false, false);
					return true;
				}else if(climbState == ClimbLadderState.climbDownStart3){
					if(stance == SimGlobals.LEFT_STANCE){
						picker.updateLimbsEnable(false, false, true, false);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.leftLegReachTargetPosition = picker.getClosestPickerPosition(walkToAimPosition, PickerDirection.down, ref picker.leftFootAtPickerIndex);
						startReach(target);
						return true;
					}else if(stance == SimGlobals.RIGHT_STANCE){
						picker.updateLimbsEnable(false, false, false, true);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.rightLegReachTargetPosition = picker.getClosestPickerPosition(walkToAimPosition, PickerDirection.down, ref picker.rightFootAtPickerIndex);
						startReach(target);
						return true;
					}
				}else if(climbState == ClimbLadderState.climbDownStart4){
					if(stance == SimGlobals.LEFT_STANCE){
						picker.updateLimbsEnable(false, false, false, true);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.rightLegReachTargetPosition = picker.getClosestPickerPosition(picker.getWorldPickerPosition(picker.leftFootAtPickerIndex), PickerDirection.down, ref picker.rightFootAtPickerIndex);
						startReach(target);
						return true;
					}else if(stance == SimGlobals.RIGHT_STANCE){
						picker.updateLimbsEnable(false, false, true, false);
						picker.updateClosestToPickerPosition(null, PickerDirection.none, null, PickerDirection.none);
						picker.pickDistance = 0.8f;
						controller.leftLegReachTargetPosition = picker.getClosestPickerPosition(picker.getWorldPickerPosition(picker.rightFootAtPickerIndex), PickerDirection.down, ref picker.leftFootAtPickerIndex);
						startReach(target);
						return true;
					}
				}
			}
		}
		
		return false;
	}
	*/
	protected bool isFaceToPosition(Vector3 pos, float diff = 0){
		/*Vector3 dir = pos - pelvis.transform.position;
		if(dir.x > diff && character.directionIsRight){
			return true;
		}else if(dir.x < -diff && !character.directionIsRight){
			return true;
		}*/
		return false;
	}

	public void setRootPosition(Vector3 pos, Quaternion qua){
		pelvis.transform.position = pos;
		pelvis.transform.rotation = qua;
		setRBStateFromEngine();
		controller.updateDAndV ();
		controller.setDesiredHeading (character.getHeading());
	}
	
	public void setDirectionWithDirection(Vector3 dir){
		Vector3 axis = (Vector3.Project(dir, Vector3.right) + Vector3.Project(dir, Vector3.forward)).normalized;
		float ang = Vector3.Angle(axis, character.getLocalFrontAxis());
		if(Vector3.Dot(Vector3.Cross(character.getLocalFrontAxis(), axis), SimGlobals.upAxis) < 0){
			ang = -ang;
		}
		controller.setIsFreeHeading(false);
		controller.setDesiredHeading(ang);
	}
	public void setDirectionWithTargetPosition(Vector3 pos){
		controller.setIsFreeHeading(false);
		controller.setDesiredHeading(pos);
	}
	
	public void ignoreCollisionWithOtherBipedal(Bipedal other){
		for(int i=0;i<character.getArticulatedRigidBodyCount();i++){
			for(int j=0;j<other.getCharacter().getArticulatedRigidBodyCount();j++){
				if(character.getArticulatedRigidBody(i).gameObject.GetComponent<Collider>() != null && other.getCharacter().getArticulatedRigidBody(j).gameObject.GetComponent<Collider>() != null){
					Physics.IgnoreCollision(character.getArticulatedRigidBody(i).gameObject.GetComponent<Collider>(), other.getCharacter().getArticulatedRigidBody(j).gameObject.GetComponent<Collider>());
				}
			}
		}
		Physics.IgnoreCollision (head.GetComponent<Collider>(), other.head.GetComponent<Collider>(), false);
	}
	
	public void ignoreCollisionARBWithOtherBipedal(CWRigidBody arb, Bipedal other, string tag = "", bool ignore = true){
		if (arb.GetComponent<Collider> () == null)
			return;
		for (int i = 0; i < other.getCharacter ().getArticulatedRigidBodyCount (); i++) {
			if (tag != "") {
				if (other.getCharacter ().getArticulatedRigidBody (i).GetComponent<Collider> () != null && other.getCharacter ().getArticulatedRigidBody (i).GetComponent<Collider> ().tag == tag) {
					Physics.IgnoreCollision (arb.GetComponent<Collider> (), other.getCharacter ().getArticulatedRigidBody (i).GetComponent<Collider> (), ignore);
				}
			} else {
				if (other.getCharacter ().getArticulatedRigidBody (i).GetComponent<Collider> () != null) {
					Physics.IgnoreCollision (arb.GetComponent<Collider> (), other.getCharacter ().getArticulatedRigidBody (i).GetComponent<Collider> (), ignore);
				}
			}
		}
	}
	
	public void ignoreCollisionTrunkWithLowerArm(bool ignore = false){
		//if (weapon == null) {
			Physics.IgnoreCollision (pelvis.GetComponent<Collider> (), lLowerArm.GetComponent<Collider> (), ignore);
			Physics.IgnoreCollision (pelvis.GetComponent<Collider> (), rLowerArm.GetComponent<Collider> (), ignore);
		Physics.IgnoreCollision (waist.GetComponent<Collider> (), lLowerArm.GetComponent<Collider> (), ignore);
		Physics.IgnoreCollision (waist.GetComponent<Collider> (), rLowerArm.GetComponent<Collider> (), ignore);
			Physics.IgnoreCollision (torso.GetComponent<Collider> (), lLowerArm.GetComponent<Collider> (), ignore);
			Physics.IgnoreCollision (torso.GetComponent<Collider> (), rLowerArm.GetComponent<Collider> (), ignore);
			Physics.IgnoreCollision (head.GetComponent<Collider> (), lLowerArm.GetComponent<Collider> (), ignore);
			Physics.IgnoreCollision (head.GetComponent<Collider> (), rLowerArm.GetComponent<Collider> (), ignore);
		/*} else {
			for (int i = 0; i < weapon.colliders.Length; i++) {
				Physics.IgnoreCollision (pelvis.GetComponent<Collider> (), weapon.colliders[i].GetComponent<Collider> (), ignore);
				Physics.IgnoreCollision (torso.GetComponent<Collider> (), weapon.colliders[i].GetComponent<Collider> (), ignore);
				Physics.IgnoreCollision (head.GetComponent<Collider> (), weapon.colliders[i].GetComponent<Collider> (), ignore);
				Physics.IgnoreCollision (lUpperLeg.GetComponent<Collider> (), weapon.colliders[i].GetComponent<Collider> (), ignore);
				Physics.IgnoreCollision (lLowerLeg.GetComponent<Collider> (), weapon.colliders[i].GetComponent<Collider> (), ignore);
				Physics.IgnoreCollision (rUpperLeg.GetComponent<Collider> (), weapon.colliders[i].GetComponent<Collider> (), ignore);
				Physics.IgnoreCollision (rLowerLeg.GetComponent<Collider> (), weapon.colliders[i].GetComponent<Collider> (), ignore);
			}
		}*/
	}
	/*
	public void startClimbStand(){
		
		Picker picker = currentPickTarget.GetComponent("Picker") as Picker;
		picker.pickDistance = 0.8f;
		
		if(climbState == ClimbLadderState.none){
			climbState = ClimbLadderState.startToClimbStand;
			controller.gotoState((int)BipedalActions.climb_stand);
			picker.updateLimbsEnable(false, false, true, true);
			picker.updateClosestToPickerPosition(null, PickerDirection.none, controller.getLFoot(), PickerDirection.none);
			startReach(currentPickTarget);
		}else if(climbState == ClimbLadderState.climbUp){
			climbState = ClimbLadderState.climbUpToStand;
		}else if(climbState == ClimbLadderState.climbDown){
			climbState = ClimbLadderState.climbDownToStand;
		}
	}
	
	public void startClimbUp(){
		if(climbState == ClimbLadderState.startToClimbStand){
			if(controller.getLFoot().isFixedPosition()){
				climbState = ClimbLadderState.climbUp;
				controller.gotoState((int)BipedalActions.climb_up, true, SimGlobals.LEFT_STANCE);
				endPick(controller.getLHand(), true);
				endPick(controller.getRFoot(), true);
			}else if(controller.getRFoot().isFixedPosition()){
				climbState = ClimbLadderState.climbUp;
				controller.gotoState((int)BipedalActions.climb_up, true, SimGlobals.RIGHT_STANCE);
				endPick(controller.getRHand(), true);
				endPick(controller.getLFoot(), true);
			}
		}else{
			climbState = ClimbLadderState.climbUp;
			if(controller.getLHand().getWorldBottomPosition().y < controller.getRHand().getWorldBottomPosition().y){
				controller.gotoState((int)BipedalActions.climb_up, true, SimGlobals.LEFT_STANCE);
				endPick(controller.getLHand(), true);
				endPick(controller.getRFoot(), true);
			}else{
				controller.gotoState((int)BipedalActions.climb_up, true, SimGlobals.RIGHT_STANCE);
				endPick(controller.getRHand(), true);
				endPick(controller.getLFoot(), true);
			}
		}
	}
	
	public void startClimbDown(){
		if(climbState == ClimbLadderState.startToClimbStand){
			if(controller.getLFoot().isFixedPosition()){
				climbState = ClimbLadderState.climbDown;
				controller.gotoState((int)BipedalActions.climb_down, true, SimGlobals.LEFT_STANCE);
				endPick(controller.getLHand(), true);
				endPick(controller.getRFoot(), true);
			}else if(controller.getRFoot().isFixedPosition()){
				climbState = ClimbLadderState.climbDown;
				controller.gotoState((int)BipedalActions.climb_down, true, SimGlobals.RIGHT_STANCE);
				endPick(controller.getRHand(), true);
				endPick(controller.getLFoot(), true);
			}
		}else{
			climbState = ClimbLadderState.climbDown;
			if(controller.getLFoot().getWorldBottomPosition().y < controller.getRFoot().getWorldBottomPosition().y){
				controller.gotoState((int)BipedalActions.climb_down, true, SimGlobals.LEFT_STANCE);
				endPick(controller.getLHand(), true);
				endPick(controller.getRFoot(), true);
			}else{
				controller.gotoState((int)BipedalActions.climb_down, true, SimGlobals.RIGHT_STANCE);
				endPick(controller.getRHand(), true);
				endPick(controller.getLFoot(), true);
			}
		}
	}
	
	public void startShinUp(){
		climbState = ClimbLadderState.climbUpEnd1;
		controller.gotoState((int)BipedalActions.shin_up);
	}
	
	public void startClimbUpEnd1(int stance){
		climbState = ClimbLadderState.climbUpEnd1;
		controller.gotoState((int)BipedalActions.climb_up_end1, true, stance);
	}
	
	public void startClimbUpEnd2(int stance){
		climbState = ClimbLadderState.climbUpEnd2;
		controller.gotoState((int)BipedalActions.climb_up_end2, true, stance);
	}
	
	public void startClimbDownStart1(int stance){
		climbState = ClimbLadderState.climbDownStart2;
		controller.gotoState((int)BipedalActions.climb_down_start1, true, stance);
	}
	
	public void startClimbDownStart2(int stance){
		climbState = ClimbLadderState.climbDownStart3;
		controller.gotoState((int)BipedalActions.climb_down_start2, true, stance);
		checkForStartClimbDown(stance);
	}
	
	public void startClimbDownStart3(int stance){
		climbState = ClimbLadderState.climbDownStart3;
		controller.gotoState((int)BipedalActions.climb_down_start3, true, stance);
		checkForStartClimbDown(stance);
	}
	
	public void startHangStand(){
		if(climbState == ClimbLadderState.none){
			climbState = ClimbLadderState.startToClimbStand;
			controller.setIsLeftReverseStance(true);
			controller.gotoState((int)BipedalActions.hang_stand);
			checkForReach(controller.getRHand().getWorldBottomPosition(), PickerDirection.down, true);
		}else if(climbState == ClimbLadderState.climbUp){
			climbState = ClimbLadderState.climbUpToStand;
		}else if(climbState == ClimbLadderState.climbDown){
			climbState = ClimbLadderState.climbDownToStand;
		}else if(climbState == ClimbLadderState.climbStand){
			controller.gotoState((int)BipedalActions.hang_stand);
		}
	}
	
	public void startHangUp(){
		climbState = ClimbLadderState.climbUp;
		if(controller.getLHand().getWorldBottomPosition().y < controller.getRHand().getWorldBottomPosition().y){
			controller.gotoState((int)BipedalActions.hang_up, true, SimGlobals.RIGHT_STANCE);
			endPick(controller.getLHand(), true);
		}else{
			controller.gotoState((int)BipedalActions.hang_up, true, SimGlobals.LEFT_STANCE);
			endPick(controller.getRHand(), true);
		}
	}
	
	public void startHangDown(){
		climbState = ClimbLadderState.climbDown;
		if(controller.getLHand().getWorldBottomPosition().y < controller.getRHand().getWorldBottomPosition().y){
			controller.gotoState((int)BipedalActions.hang_down, true, SimGlobals.LEFT_STANCE);
			endPick(controller.getRHand(), true);
		}else{
			controller.gotoState((int)BipedalActions.hang_down, true, SimGlobals.RIGHT_STANCE);
			endPick(controller.getLHand(), true);
		}
	}*/
	/*
	public void startHangSwing(bool isFront){
		if(isFront){
			if(character.directionIsRight){
				controller.getLHand().rigidbody.AddForce(new Vector3(400, 0, 0));
				controller.getRHand().rigidbody.AddForce(new Vector3(400, 0, 0));
			}else{
				controller.getLHand().rigidbody.AddForce(new Vector3(-400, 0, 0));
				controller.getRHand().rigidbody.AddForce(new Vector3(-400, 0, 0));
			}
			controller.gotoState((int)BipedalActions.hang_swing_front);
		}else{
			if(character.directionIsRight){
				controller.getLHand().rigidbody.AddForce(new Vector3(-400, 0, 0));
				controller.getRHand().rigidbody.AddForce(new Vector3(-400, 0, 0));
			}else{
				controller.getLHand().rigidbody.AddForce(new Vector3(400, 0, 0));
				controller.getRHand().rigidbody.AddForce(new Vector3(400, 0, 0));
			}
			controller.gotoState((int)BipedalActions.hang_swing_back);
		}
	}*/
	
	public void startHangToJump(bool isRight){
		controller.gotoState((int)BipedalActions.hang_to_jump);
	}
	
	public void startFalling(){
		controller.gotoState((int)BipedalActions.falling);
	}
	
	public void startTouchDown(){
		controller.gotoState((int)BipedalActions.touch_down1);
	}
	
	//public void startTouchRoll(){
		//controller.gotoState((int)BipedalActions.touch_roll);
	//}
	
	public void startWalkToAim(GameObject target, Vector3 endpos, bool isForward, int startStance){
		isWalkToAim = true;
		walkToAimTarget = target;
		walkToAimPosition = endpos;

		if (agent != null) {
			currentPathIndex = 0;
			currentStartPosition = pelvis.transform.position;
			currentStartPosition[controller.getWorldUpAxisID()] = 0;
			agentPath.ClearCorners();
			agent.CalculatePath(endpos, agentPath);
			if(agentPath.status == NavMeshPathStatus.PathComplete){
				/*Debug.Log("aaaaaaaa == "+agentPath.corners.Length);
				for(int i =0; i<agentPath.corners.Length; i++){
					Debug.Log("++++++ == "+agentPath.corners[i]);
				}*/
				controller.gotoState((int)BipedalActions.forward_walk, true, startStance);
			}
		}
		/*
		if(isForward){
			controller.gotoState((int)BipedalActions.forward_walk, true, startStance);
		}else{
			controller.gotoState((int)BipedalActions.back_walk, true, startStance);
		}*/
	}

	public void startRunToAim(GameObject target, Vector3 endpos, int startStance){
		isWalkToAim = true;
		walkToAimTarget = target;
		walkToAimPosition = endpos;

		//Debug.Log ("aa ===== startRunToAim === "+endpos);
		if (agent != null) {
			currentPathIndex = 0;
			currentStartPosition = pelvis.transform.position;
			currentStartPosition[controller.getWorldUpAxisID()] = 0;
			agentPath.ClearCorners();
			agent.CalculatePath(endpos, agentPath);
			if (agentPath.status == NavMeshPathStatus.PathComplete) {
				if (isInWater) {
					if (!isMoveing) {
						startInWaterSwiming ();
					}
				} else {
					controller.gotoState ((int)BipedalActions.forward_run, true, startStance);
				}
			}
		}
	}

	public void updateRunToAim(Vector3 endpos){
		walkToAimPosition = endpos;
		
		if (agent != null) {
			currentPathIndex = 0;
			currentStartPosition = pelvis.transform.position;
			currentStartPosition[controller.getWorldUpAxisID()] = 0;
			agentPath.ClearCorners();
			agent.CalculatePath(endpos, agentPath);
		}
	}

	public void walkToAimEnd(int endStance = SimGlobals.LEFT_STANCE){
		isWalkToAim = false;
		startStand();
		if(walkToAimTarget != null){
			if(walkToAimTarget.GetComponent<Collider>().tag == "picker_grabs"){
				climbState = ClimbLadderState.climbDownStart1;
				//checkForStartClimbDown(endStance);
			}else if(walkToAimTarget.GetComponent<Collider>().tag == "picker_ladder_ground"){
				/*if(walkToAimDistance > 0){
					startReach(walkToAimTarget);
				}else{
					climbState = ClimbLadderState.climbDownStart1;
					checkForStartClimbDown(endStance);
				}*/
			}
		}
	}

	public void onEnterWater(GameObject obj){
		if (obj.name == "pelvis") {
			isInWater = true;
		}
	}

	public void onExitWater(GameObject obj){
		if (obj.name == "pelvis") {
			isInWater = false;
		}
	}
	/*
	public void addWeapon(GameObject wp, GameObject hd){
		if(this.weapon != null) return;

		wp.transform.parent = hd.transform;
		wp.transform.localPosition = Vector3.zero;
		wp.transform.localRotation = Quaternion.identity;
		this.weapon = wp.GetComponent<ColdWeapon>();
		weapon.bipOwner = this;
		weapon.parentHand = hd.GetComponent<CWRigidBody>();
		hd.GetComponent<Rigidbody> ().mass = hd.GetComponent<CWRigidBody> ().getMass () + weapon.mass;
	}

	public void removeWeapon(GameObject hd){
		if(this.weapon == null) return;

		weapon.transform.parent = null;

		weapon.bipOwner = null;
		weapon.parentHand = null;
		weapon.endAttack();
		this.weapon = null;

		hd.GetComponent<Rigidbody> ().mass = hd.GetComponent<CWRigidBody> ().getMass ();
		hd.GetComponent<Rigidbody> ().centerOfMass = hd.GetComponent<CWRigidBody> ().getCenterOfMass ();
		hd.GetComponent<Rigidbody> ().inertiaTensor = hd.GetComponent<CWRigidBody> ().getInertiaTensor ();
	}

	public void addShoes(GameObject lsh, GameObject rsh){
		if (this.lShoes != null || this.rShoes != null)
			return;

		//Debug.Log ("qqqqqqqq " + controller.getLFoot ().GetComponent<Rigidbody> ().centerOfMass + " ==== " + controller.getLFoot ().GetComponent<Rigidbody> ().inertiaTensor + " == "+controller.getLFoot ().GetComponent<Rigidbody> ().inertiaTensorRotation);
		lsh.transform.parent = controller.getLFoot().transform;
		lsh.transform.localPosition = Vector3.zero;
		lsh.transform.localRotation = Quaternion.identity;
		//lsh.transform.localScale = new Vector3 (0.01f, 0.01f, 0.01f);
		this.lShoes = lsh.GetComponent<Shoes>();
		lShoes.bipOwner = this;
		lShoes.parentFoot = controller.getLFoot().GetComponent<CWRigidBody> ();
		controller.getLFoot ().GetComponent<Rigidbody> ().ResetCenterOfMass ();
		controller.getLFoot ().GetComponent<Rigidbody> ().ResetInertiaTensor ();
		//lLowerLeg.GetComponent<Rigidbody> ().mass = lLowerLeg.GetComponent<Rigidbody> ().mass + lShoes.mass;

		//Debug.Log ("bbbbbbbbbb " + controller.getLFoot ().GetComponent<Rigidbody> ().centerOfMass + " ==== " + controller.getLFoot ().GetComponent<Rigidbody> ().inertiaTensor + " == "+controller.getLFoot ().GetComponent<Rigidbody> ().inertiaTensorRotation);

		//Vector3 newBomttom = controller.getLFoot ().transform.InverseTransformPoint(lShoes.bottom.transform.position);
		controller.additionHeight = lShoes.length;
		controller.iPStepScale = 1.7f;
		//Debug.Log ("sfghjdsgfhj === " + controller.additionHeight);
		//controller.getLFoot ().setLocalBottomPosition (newBomttom);
		//bipedIK.references.leftFoot.transform.localPosition = newBomttom;

		rsh.transform.parent = controller.getRFoot().transform;
		rsh.transform.localPosition = Vector3.zero;
		rsh.transform.localRotation = Quaternion.identity;
		//rsh.transform.localScale = new Vector3 (0.01f, 0.01f, 0.01f);
		this.rShoes = rsh.GetComponent<Shoes>();
		rShoes.bipOwner = this;
		rShoes.parentFoot = controller.getRFoot ().GetComponent<CWRigidBody> ();
		controller.getRFoot ().GetComponent<Rigidbody> ().ResetCenterOfMass ();
		controller.getRFoot ().GetComponent<Rigidbody> ().ResetInertiaTensor ();
		//rLowerLeg.GetComponent<Rigidbody> ().mass = rLowerLeg.GetComponent<Rigidbody> ().mass + rShoes.mass;

		//newBomttom = controller.getRFoot ().transform.InverseTransformPoint (rShoes.bottom.transform.position);
		//controller.getRFoot ().setLocalBottomPosition (newBomttom);
		//bipedIK.references.rightFoot.transform.localPosition = newBomttom;
	}

	public void removeShoes(Shoes sh){
		controller.additionHeight = 0;
		controller.iPStepScale = 1;
		if (sh == this.lShoes) {
			lShoes.transform.parent = null;
			lShoes.bipOwner = null;
			lShoes.parentFoot = null;
			this.lShoes = null;
			controller.getLFoot ().GetComponent<Rigidbody> ().centerOfMass = controller.getLFoot ().getCenterOfMass ();
			controller.getLFoot ().GetComponent<Rigidbody> ().inertiaTensor = controller.getLFoot ().getInertiaTensor ();
		} else if (sh == this.rShoes) {
			rShoes.transform.parent = null;
			rShoes.bipOwner = null;
			rShoes.parentFoot = null;
			this.rShoes = null;
			controller.getRFoot ().GetComponent<Rigidbody> ().centerOfMass = controller.getRFoot ().getCenterOfMass ();
			controller.getRFoot ().GetComponent<Rigidbody> ().inertiaTensor = controller.getRFoot ().getInertiaTensor ();
		}
	}*/

	public bool canLookObject(GameObject obj){
		Vector3 vec = (obj.transform.position - pelvis.transform.position).normalized;
		Vector3 head = controller.getCurrentHeading () * character.getLocalFrontAxis ();
		float ang = Mathf.Abs (Vector3.Angle (head, vec));
		if (ang < 70) {
			return true;
		}
		return false;
	}
}