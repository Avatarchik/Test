using UnityEngine;
using System.Collections;
using CartWheelCore.Math;
using RootMotion.FinalIK;

/**
	Controller Inspired by Simbicon and Virtual Model Control ideas. Uses IK to control swing foot placement. It assumes
	a pretty specilized character architecture: Legs look like this: 
		root -> hips (3 dof) -> knees (1 dof, around x-axis, for now) -> ankles (doesn't matter how many dofs) -> (may or may not have toes, etc).

	An IKVMCController is a Simbicon Controller. The targets for the swing hip and swing knee are computed using IK, based on a target
	foot location (which is given by the inverted pendulum prediction), and the stance ~hip, knee and ankle torques are computed in part
	by the desired velocity of the CM. Gravity compensation can be added to any rb in the body (propagated to the root). Also, "torque bubbling"
	can be added for any chain that starts at the root.

	TODO:

		- try out some stairs, slopes
		- try some IK for the arm, while walking
		- try some static balance, arms moving...
		- get a less 'hacky' implementation of the desired foot placement position (at least in the sagittal plane).

		- test out 'external' foot placement accuracy - DONE!
		- make walk more natural - how much is simbicon, how much is T = J' * f for push-off? How much does the vertical trajectory of swing foot matter? - DONE!
		- eliminate/reduce the torso/head wobbling. - DONE!
		- stance leg/swing leg velocity/position control through T = J' * f - DONE!
		- get some decent toe-off - Pretty much DONE!
		- torque bubbling - DONE!
		- gravity compensation - DONE!
		- check to see why the crouched walk happens sometime - stance knee should be straight, shouldn't it? - DONE!
		- check to see why swing hip initially goes inwards! - DONE!
		- make sure character can walk in any direction - DONE!
		- limit the extent to which stance ankle 'rolls' on the heel - DONE!
		- how much should the strength of the root be? What works best?
			- doesn't actually matter too much. 1 seems to work best, so we'll go with that.



		- THINGS TO TRY STILL:
			- try to somehow bring the upper body into the motion - maybe simulate the force being applied
			to the CM - i.e. t = dpCM/dq * f, where CM now only takes into account the chain from the head/torso
			to the stance foot - i don't know...
			- upper body - can it be used to make it slow down/speed up/whatever faster?
			- maybe com J'?
*/
namespace CartWheelCore.Control{
	public class IKVMCController : SimBiController {
	
		//keep track of the swing knee and swing hip
		private CWRigidBody currentTouchedBody;
		private CWJoint neck, waist;
		private CWJoint leftShoulder, rightShoulder, leftElbow, rightElbow, leftKnee, rightKnee, leftWrist, rightWrist;
		private CWJoint swingKnee, swingHip, standKnee, standHip, swingAnkle, standAnkle;
		private int neckIndex, waistIndex, torsoIndex;
		private int leftShoulderIndex, rightShoulderIndex, leftElbowIndex, rightElbowIndex;
		private int lKneeIndex, rKneeIndex, lAnkleIndex, rAnkleIndex;
		//keep track of the stance ankle, stance knee and stance hip
		private int stanceKneeIndex, swingKneeIndex, stanceAnkleIndex, swingAnkleIndex;

		private BehaviourController behaviour;
		//if the controller is in double-stance mode, then the swing foot won't have an IK target
		private bool doubleStanceMode;
		private bool doIPBelence;
		private bool stanceBelence;
		private bool isComputeIPStep;
		private float belenceError;
		private float velDSagittal;
		private float velDCoronal;
		private float stepHeight;
		private float swingStepLength;
		private float standStepLength;
		private float rootExternalTorqueKp;
		private float rootExternalTorqueKd;
		private float rootExternalForceKp;
		private float rootExternalForceKd;
		private float fixedPositionKp;
		private float fixedPositionKd;
		private float velSagittalInAir;
		private float bodySpringKp;
		private float bodySpringKd;
		private bool continueTrajectory;
		
		private Vector3 fixedPosition;
		private float legWidth;
		private float legLength;
		private float armLength;
		private float checkBelenceTimer = 0;
		
		//this is a desired foot trajectory that we may wish to follow, expressed separately, for the 3 components,
		//and relative to the current location of the CM
		public GenericTrajectory<float> standFootSagittalTrajectory;
		public GenericTrajectory<float> swingFootSagittalTrajectory;
		public GenericTrajectory<float> standFootCoronalTrajectory;
		public GenericTrajectory<float> swingFootCoronalTrajectory;
		public GenericTrajectory<float> standFootHeightTrajectory;
		public GenericTrajectory<float> swingFootHeightTrajectory;
		
		//public GenericTrajectory<float> swingFootTrajectoryDeltaSagittal;
		//public GenericTrajectory<float> swingFootTrajectoryDeltaHeight;
		//this is the vector that specifies the plane of rotation for the swing leg, relative to the root...
		
		public BipedIK bipedIK = null;
		public Vector3 leftArmReachTargetPosition = Vector3.zero;
		public Vector3 rightArmReachTargetPosition = Vector3.zero;
		
		public Vector3 leftLegReachTargetPosition = Vector3.zero;
		public Vector3 rightLegReachTargetPosition = Vector3.zero;

		public Vector3 lookAtTargetPosition = Vector3.zero;
		public Vector3 shineTargetPosition = Vector3.zero;
		private float lookAtBodyWeight = 0.5f;
		private float lookAtHeadWeight = 1;
		private bool isComputeLeftLegIK = false;
		private bool isComputeRightLegIK = false;
		private bool isComputeShineIK = false;
		
		//this variable can be used to quickly alter the desired height, if panic ensues...
		public float swingLegPanicHeight;
		public float additionHeight;
		public float iPStepScale;
		
		public float swingFootFrontLength;
		public Vector3 swingFootFrontPosition;
		public Vector3 swingFootBasePosition;
		public Vector3 standFootBasePosition;
		public Vector3 swingFootBaseNormal;
		public Vector3 standFootBaseNormal;
		public Vector3 swingToStandBaseVector;

		/**
			constructor
		*/
		public IKVMCController(Character b) : base(b){
		//	Debug.Log("IKVMCController===================================================");
			currentTouchedBody = null;

			neckIndex = character.getJointIndex("torso_head");
			waistIndex = character.getJointIndex("pelvis_waist");
			torsoIndex = character.getJointIndex ("waist_torso");
			lKneeIndex = character.getJointIndex("lKnee");
			rKneeIndex = character.getJointIndex("rKnee");
			lAnkleIndex = character.getJointIndex("lAnkle");
			rAnkleIndex = character.getJointIndex("rAnkle");

			neck = character.getJointByName("torso_head");
			waist = character.getJointByName("pelvis_waist");
			leftShoulder = character.getJointByName("lShoulder");
			rightShoulder = character.getJointByName("rShoulder");
			leftElbow = character.getJointByName("lElbow");
			rightElbow = character.getJointByName("rElbow");
			leftKnee = character.getJointByName ("lKnee");
			rightKnee = character.getJointByName ("rKnee");
			leftWrist = character.getJointByName ("lWrist");
			rightWrist = character.getJointByName ("rWrist");
			
			leftShoulderIndex = character.getJointIndex(leftShoulder);
			rightShoulderIndex = character.getJointIndex(rightShoulder);
			leftElbowIndex = character.getJointIndex(leftElbow);
			rightElbowIndex = character.getJointIndex(rightElbow);
			
			swingFootSagittalTrajectory = new GenericTrajectory<float>();
			standFootSagittalTrajectory = new GenericTrajectory<float>();
			standFootCoronalTrajectory = new GenericTrajectory<float>();
			swingFootCoronalTrajectory = new GenericTrajectory<float>();
			standFootHeightTrajectory = new GenericTrajectory<float>();
			swingFootHeightTrajectory = new GenericTrajectory<float>();
			//swingFootTrajectoryDeltaSagittal = new GenericTrajectory<float>();
			//swingFootTrajectoryDeltaHeight = new GenericTrajectory<float>();
			
			velDSagittal = 0;
			velDCoronal = 0;
			iPStepScale = 0.5f;
			belenceError = 0.2f;
			
			rootExternalTorqueKp = 0;
			rootExternalTorqueKd = 0;
			rootExternalForceKp = 0;
			rootExternalForceKd = 0;
			fixedPositionKp = 0;
			fixedPositionKd = 0;
			velSagittalInAir = 0;
			bodySpringKp = 0;
			bodySpringKd = 0;
			fixedPosition = Vector3.zero;
		
			swingLegPanicHeight = 0;
			swingFootFrontPosition = Vector3.zero;
			swingFootBasePosition = Vector3.zero;
			standFootBasePosition = Vector3.zero;
			swingFootBaseNormal = Vector3.zero;
			standFootBaseNormal = Vector3.zero;
			swingToStandBaseVector = Vector3.zero;
		
			additionHeight = 0;
			legWidth = (b.getJoint(lHipIndex).getChildJointWorldPosition() - b.getJoint(rHipIndex).getChildJointWorldPosition()).magnitude;
			legLength = (b.getJoint(lHipIndex).getChildJointWorldPosition() - lFoot.getWorldBottomPosition()).magnitude;
			armLength = (b.getJoint(leftShoulderIndex).getChildJointWorldPosition() - lHand.getWorldBottomPosition()).magnitude;
			
			swingFootFrontLength = legLength * 0.7f;
			
			Debug.Log("aaaaaaaaaaaalegLength  "+legLength);
		//	Debug.Log("bbbbbbbbbbbb  "+armLength);
			
			doubleStanceMode = false;
			stanceBelence = false;
			doIPBelence = true;
			isComputeIPStep = false;
			continueTrajectory = false;
		
			behaviour = null;
		}
		
		public void setDoubleStanceMode(bool flag){
			doubleStanceMode = flag;
		}
		
		public bool getDoubleStanceMode(){
			return doubleStanceMode;
		}
				
		public bool getStanceBelence(){
			return stanceBelence;
		}
		
		public float getBelenceError(){
			return belenceError;
		}
		
		public bool getIsComputeIPStep(){
			return isComputeIPStep;
		}
		
		public void setDoIPBelence(bool flag){
			doIPBelence = flag;
		}
		public bool getDoIPBelence(){
			return doIPBelence;
		}
		
		public float getLegLength(){
			return legLength;
		}
		public void setLegLength(float len){
			legLength = len;
		}
		
		public float getArmLength(){
			return armLength;
		}
		
		public float getVelSagittal(){
			return velDSagittal;
		}
		
		public float getVelDCoronal(){
			return velDCoronal;
		}

		public float getSwingStepLength(){
			return swingStepLength;
		}
		public float getStandStepLength(){
			return standStepLength;
		}

		public float getStepHeight(){
			return stepHeight;
		}

		public CWJoint getSwingHip(){
			return swingHip;
		}
		
		public CWJoint getStandHip(){
			return standHip;
		}
		
		public CWJoint getLeftShoulder(){
			return leftShoulder;
		}
		
		public CWJoint getRightShoulder(){
			return rightShoulder;
		}
		
		public CWJoint getLeftElbow(){
			return leftElbow;
		}
		
		public CWJoint getRightElbow(){
			return rightElbow;
		}

		public CWJoint getLeftKnee(){
			return leftKnee;
		}

		public CWJoint getRightKnee(){
			return rightKnee;
		}

		public CWJoint getLeftWrist(){
			return leftWrist;
		}

		public CWJoint getRightWrist(){
			return rightWrist;
		}
		
		public CWRigidBody getCurrentTouchedBody(){
			return currentTouchedBody;
		}

		public void setLookAtWeight(float bodyWeight, float headWeight){
			lookAtBodyWeight = bodyWeight;
			lookAtHeadWeight = headWeight;
		}
		
		/*
		public void setSwingFootTrajectoryDeltaSagittal( GenericTrajectory<float> traj ) {
			swingFootTrajectoryDeltaSagittal.copy( traj );
			//notifyObservers();
		}
		public GenericTrajectory<float> getSwingFootTrajectoryDeltaSagittal() {
			return swingFootTrajectoryDeltaSagittal;
		}
		
		public void setSwingFootTrajectoryDeltaHeight( GenericTrajectory<float> traj ) {
			swingFootTrajectoryDeltaHeight.copy( traj );
			//notifyObservers();
		}
		public GenericTrajectory<float> getSwingFootTrajectoryDeltaHeight() {
			return swingFootTrajectoryDeltaHeight;
		}
		
		public Vector3 computeSwingFootDelta( float phiToUse = -1, int stanceToUse = -1 ) {

			if( phiToUse < 0 )
				phiToUse = phi;
			if( phiToUse > 1 )
				phiToUse = 1;
			if( stanceToUse < 0 )
				stanceToUse = stance;
			if( stanceToUse > 1 )
				stanceToUse = 1;
	
			return new Vector2(
				swingFootTrajectoryDeltaSagittal.evaluate_catmull_rom(phiToUse),
				swingFootTrajectoryDeltaHeight.evaluate_catmull_rom(phiToUse));
		}*/

		public void setBehaviour( BehaviourController behaviour_disown ) {
			this.behaviour = behaviour_disown;
		}
	
		public BehaviourController getBehaviour() { return behaviour; }

		public void setElbowIsFree(bool isLeft, bool isFree){
			if(isFree){
				if(isLeft){
					leftElbow.setAngularXMotion(ConfigurableJointMotion.Free);
					leftElbow.setAngularYMotion(ConfigurableJointMotion.Free);
					leftElbow.setAngularZMotion(ConfigurableJointMotion.Free);
				}else{
					rightElbow.setAngularXMotion(ConfigurableJointMotion.Free);
					rightElbow.setAngularYMotion(ConfigurableJointMotion.Free);
					rightElbow.setAngularZMotion(ConfigurableJointMotion.Free);
				}
			}else{
				if(isLeft){
					leftElbow.setAngularXMotion(ConfigurableJointMotion.Limited);
					leftElbow.setAngularYMotion(ConfigurableJointMotion.Locked);
					leftElbow.setAngularZMotion(ConfigurableJointMotion.Locked);
				}else{
					rightElbow.setAngularXMotion(ConfigurableJointMotion.Limited);
					rightElbow.setAngularYMotion(ConfigurableJointMotion.Locked);
					rightElbow.setAngularZMotion(ConfigurableJointMotion.Locked);
				}
			}
		}

		public bool getElbowIsFree(bool isLeft){
			if (isLeft) {
				return leftElbow.getAngularXMotion() == ConfigurableJointMotion.Free;
			} else {
				return rightElbow.getAngularXMotion() == ConfigurableJointMotion.Free;
			}
		}

		public void setKneeIsFree(bool isLeft, bool isFree){
			if(isFree){
				if(isLeft){
					leftKnee.setAngularXMotion(ConfigurableJointMotion.Limited);
					leftKnee.setAngularYMotion(ConfigurableJointMotion.Free);
					leftKnee.setAngularZMotion(ConfigurableJointMotion.Free);
				}else{
					rightKnee.setAngularXMotion(ConfigurableJointMotion.Limited);
					rightKnee.setAngularYMotion(ConfigurableJointMotion.Free);
					rightKnee.setAngularZMotion(ConfigurableJointMotion.Free);
				}
			}else{
				if(isLeft){
					leftKnee.setAngularXMotion(ConfigurableJointMotion.Limited);
					leftKnee.setAngularYMotion(ConfigurableJointMotion.Locked);
					leftKnee.setAngularZMotion(ConfigurableJointMotion.Locked);
				}else{
					rightKnee.setAngularXMotion(ConfigurableJointMotion.Limited);
					rightKnee.setAngularYMotion(ConfigurableJointMotion.Locked);
					rightKnee.setAngularZMotion(ConfigurableJointMotion.Locked);
				}
			}
		}
		public bool getKneeIsFree(bool isLeft){
			if (isLeft) {
				return leftKnee.getAngularYMotion() == ConfigurableJointMotion.Free;
			} else {
				return rightKnee.getAngularYMotion() == ConfigurableJointMotion.Free;
			}
		}

		public void setWristOrientation(bool isLeft, Quaternion rot){
			if (isLeft) {
				leftWrist.setTargetOrientation (Quaternion.Inverse (lHand.getOrientation ()) * rot * leftWrist.getInitialWorldRotation ());
			} else {
				rightWrist.setTargetOrientation (Quaternion.Inverse (rHand.getOrientation ()) * rot * rightWrist.getInitialWorldRotation ());
			}
		}
		public void setWristOrientationWithPosition(bool isLeft, Vector3 pos){
			if (isLeft) {
				leftWrist.setTargetOrientation (Quaternion.Inverse (lHand.getOrientation ()) * (Quaternion.LookRotation ((pos - leftWrist.getChildJointWorldPosition ()).normalized)) * leftWrist.getInitialWorldRotation ());
			} else {
				rightWrist.setTargetOrientation (Quaternion.Inverse (rHand.getOrientation ()) * (Quaternion.LookRotation ((pos - rightWrist.getChildJointWorldPosition ()).normalized)) * rightWrist.getInitialWorldRotation ());
			}
		}
		public void setWristOrientationWithDirection(bool isLeft, Vector3 dir){
			if (isLeft) {
				leftWrist.setTargetOrientation (Quaternion.Inverse (lHand.getOrientation ()) * Quaternion.LookRotation (dir) * leftWrist.getInitialWorldRotation ());
			} else {
				rightWrist.setTargetOrientation (Quaternion.Inverse (rHand.getOrientation ()) * Quaternion.LookRotation (dir) * rightWrist.getInitialWorldRotation ());
			}
		}
		
		/**
			returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
			on the assumption that the character will come to a stop by taking a step at that location. The step location
			is expressed in the character's frame coordinates.
		*/
		public Vector3 computeIPStepLocation(){
			/*Vector3 step = new Vector3();
			float h = Mathf.Abs(comPosition[worldUpAxisID] - stanceFoot.getWorldBottomPosition()[worldUpAxisID]);
			step[localFrontAxisID] = v[localFrontAxisID] * Mathf.Sqrt(h/9.8f + v[localFrontAxisID] * v[localFrontAxisID] / (4*9.8f*9.8f)) * iPStepScale;
			step[localLeftAxisID] = v[localLeftAxisID] * Mathf.Sqrt(h/9.8f + v[localLeftAxisID] * v[localLeftAxisID] / (4*9.8f*9.8f)) * iPStepScale;
			step[localUpAxisID] = 0;
			return step;*/

			Vector3 step = new Vector3();
			float h = Mathf.Abs(comPosition[worldUpAxisID] - stanceFoot.getWorldBottomPosition()[worldUpAxisID]);
			float vl = (doubleStanceCOMError [localLeftAxisID] + v [localLeftAxisID]) / 2;
			//float vl = (v [localLeftAxisID]);
			step [localLeftAxisID] = vl * Mathf.Sqrt (h / 9.8f + vl * vl / (4 * 9.8f * 9.8f)) * 0.8f* iPStepScale;

			vl = (doubleStanceCOMError [localFrontAxisID] + v [localFrontAxisID]) / 2;
			//vl = (v [localFrontAxisID]);
			var x = (vl>0)?0.5f:0.6f;
			step [localFrontAxisID] = vl * Mathf.Sqrt (h / 9.8f + vl * vl / (4 * 9.8f * 9.8f)) * x * iPStepScale;
			step[localUpAxisID] = 0;
			return step;
		}
		
		public void computeIKStandLegTargets(float dt){
			Vector3 pNow = getStandFootTargetLocation(phi, comPosition, characterFrame);

			if (stance == SimGlobals.LEFT_STANCE) {
				computeIKLeftLegTargets(pNow, dt);
			} else {
				computeIKRightLegTargets(pNow, dt);
			}
		}
		
		/**
			This method is used to compute the target angles for the swing hip and swing knee that help 
			to ensure (approximately) precise foot-placement control.
		*/
		public void computeIKSwingLegTargets(float dt){
			//note, V is already expressed in character frame coordinates.
			Vector3 pNow = getSwingFootTargetLocation(phi, comPosition, characterFrame);
			
			if (stance == SimGlobals.LEFT_STANCE) {
				computeIKRightLegTargets(pNow, dt);
			} else {
				computeIKLeftLegTargets(pNow, dt);
			}
		}
		
		public void computeIKLeftLegTargets(Vector3 targetPos, float dt, bool isAddForce = false){
			if (bipedIK == null) {
				CWJoint lknee = character.getJoint(lKneeIndex);
				CWJoint lHip = character.getJoint(lHipIndex);
				Vector3 parentAxis = lknee.getParentJointPosition () - lHip.getChildJointPosition ();
				Vector3 childAxis = lLowerLeg.getLocalBottomPosition () - lknee.getChildJointPosition ();
			
				computeIKQandW (lHipIndex, lKneeIndex, parentAxis, character.getLocalLeftAxis (), character.getLocalLeftAxis (), childAxis, targetPos, false, targetPos, dt);
			} else {
				isComputeLeftLegIK = true;
				bipedIK.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);
				bipedIK.SetIKPosition(AvatarIKGoal.LeftFoot, targetPos);
				bipedIK.SetIKRotation(AvatarIKGoal.LeftFoot, currentHeading);
				ReducedCharacterState rs = new ReducedCharacterState(desiredPose);

				Debug.DrawLine(bipedIK.references.leftCalf.position, bipedIK.references.leftThigh.position, Color.red);
				Debug.DrawLine(bipedIK.references.leftFoot.position, bipedIK.references.leftCalf.position, Color.green);
				
				rs.setJointRelativeOrientation(bipedIK.references.leftCalf.localRotation, lKneeIndex);
				rs.setJointRelativeOrientation(bipedIK.references.leftThigh.localRotation, lHipIndex);

				if (isAddForce) {
					Vector3 f = -fixedPositionKp * (lFoot.getWorldBottomPosition() - targetPos) - fixedPositionKd * lFoot.getCMVelocity();
					lFoot.addExternalForce (f);
				}
			}
		}
		
		public void computeIKRightLegTargets(Vector3 targetPos, float dt, bool isAddForce = false){
			if (bipedIK == null) {
				CWJoint rknee = character.getJoint(rKneeIndex);
				CWJoint rHip = character.getJoint(rHipIndex);
				Vector3 parentAxis = rknee.getParentJointPosition () - rHip.getChildJointPosition ();
				Vector3 childAxis = rLowerLeg.getLocalBottomPosition () - rknee.getChildJointPosition ();
			
				computeIKQandW (rHipIndex, rKneeIndex, parentAxis, character.getLocalLeftAxis (), character.getLocalLeftAxis (), childAxis, targetPos, false, targetPos, dt);
			}else{
				isComputeRightLegIK = true;
				bipedIK.SetIKPositionWeight(AvatarIKGoal.RightFoot, 1);
				bipedIK.SetIKPosition(AvatarIKGoal.RightFoot, targetPos);
				bipedIK.SetIKRotation(AvatarIKGoal.RightFoot, currentHeading);
				ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
				
				Debug.DrawLine(bipedIK.references.rightCalf.position, bipedIK.references.rightThigh.position, Color.red);
				Debug.DrawLine(bipedIK.references.rightFoot.position, bipedIK.references.rightCalf.position, Color.green);
				
				rs.setJointRelativeOrientation(bipedIK.references.rightCalf.localRotation, rKneeIndex);
				rs.setJointRelativeOrientation(bipedIK.references.rightThigh.localRotation, rHipIndex);

				if (isAddForce) {
					Vector3 f = -fixedPositionKp * (rFoot.getWorldBottomPosition() - targetPos) - fixedPositionKd * rFoot.getCMVelocity();
					rFoot.addExternalForce (f);
				}
			}
		}

		public void computeLookAtTarget (Vector3 targetPos){
			if (bipedIK != null) {
				if(targetPos.magnitude>0){
					bipedIK.SetLookAtWeight(1,lookAtBodyWeight,lookAtHeadWeight,0,0.5f,0.6f,0.5f);
					bipedIK.SetLookAtPosition(targetPos);

					Debug.DrawLine(bipedIK.references.pelvis.position, bipedIK.references.spine[0].position, Color.red);
					Debug.DrawLine(bipedIK.references.spine[0].position, bipedIK.references.head.position, Color.green);

					ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
					rs.setJointRelativeOrientation(bipedIK.references.spine[0].localRotation*rs.getJointRelativeOrientation (waistIndex), waistIndex);
					rs.setJointRelativeOrientation(bipedIK.references.spine[1].localRotation*rs.getJointRelativeOrientation (torsoIndex), torsoIndex);
					rs.setJointRelativeOrientation(bipedIK.references.head.localRotation*rs.getJointRelativeOrientation (neckIndex), neckIndex);
				}else{
					bipedIK.SetLookAtWeight(0,0,0,0,0,0,0);
				}
			}
		}

		public void computeShineIKTarget(Vector3 targetPos){
			if (bipedIK != null) {
				if(targetPos.magnitude>0){
					isComputeShineIK = true;
					bipedIK.SetSpineWeight (1);
					bipedIK.SetSpinePosition (targetPos);

					ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
					rs.setJointRelativeOrientation(bipedIK.references.spine[0].localRotation*rs.getJointRelativeOrientation (waistIndex), waistIndex);
					rs.setJointRelativeOrientation(bipedIK.references.spine[1].localRotation*rs.getJointRelativeOrientation (torsoIndex), torsoIndex);

					Vector3 f = -fixedPositionKp * (torso.getCMPosition() - targetPos) - fixedPositionKd * torso.getCMVelocity();
					torso.addExternalForce (f);
				}else{
					bipedIK.SetSpineWeight (0);
				}
			}
		}
		
		public void computeIKLeftArmTargets(Vector3 targetPos, float dt){
			if (bipedIK == null) {
				Vector3 parentAxis = leftElbow.getParentJointPosition () - leftShoulder.getChildJointPosition ();
				Vector3 childAxis = leftElbow.getChild ().getLocalBottomPosition () - leftElbow.getChildJointPosition ();
			
				//Debug.DrawLine(leftElbow.getParentJointWorldPosition() + new Vector3(0,0,-1), leftShoulder.getChildJointWorldPosition() + new Vector3(0,0,-1), Color.red);
				//Debug.DrawLine(leftElbow.getChild().getWorldBottomPosition() + new Vector3(0,0,-1), leftElbow.getChildJointWorldPosition() + new Vector3(0,0,-1), Color.red);
			
				computeIKQandW (leftShoulderIndex, leftElbowIndex, parentAxis, -1 * character.getLocalLeftAxis (), character.getLocalLeftAxis (), childAxis, targetPos, false, targetPos, dt);
			}else{
				bipedIK.SetIKPositionWeight(AvatarIKGoal.LeftHand, 1);
				bipedIK.SetIKPosition(AvatarIKGoal.LeftHand, targetPos);
				bipedIK.SetIKRotation(AvatarIKGoal.LeftHand, currentHeading);
				ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
				
				Debug.DrawLine(bipedIK.references.leftForearm.position, bipedIK.references.leftUpperArm.position, Color.red);
				Debug.DrawLine(bipedIK.references.leftHand.position, bipedIK.references.leftForearm.position, Color.green);
				
				rs.setJointRelativeOrientation(bipedIK.references.leftForearm.localRotation, leftElbowIndex);
				rs.setJointRelativeOrientation(bipedIK.references.leftUpperArm.localRotation, leftShoulderIndex);

				Vector3 f = -fixedPositionKp * (lHand.getWorldBottomPosition() - targetPos) - fixedPositionKd * lHand.getCMVelocity();
				lHand.addExternalForce (f);
			}
		}
		
		public void computeIKRightArmTargets(Vector3 targetPos, float dt){
			if (bipedIK == null) {
				Vector3 parentAxis = rightElbow.getParentJointPosition () - rightShoulder.getChildJointPosition ();
				Vector3 childAxis = rightElbow.getChild ().getLocalBottomPosition () - rightElbow.getChildJointPosition ();
			
				//Debug.DrawLine(rightElbow.getParentJointWorldPosition() + new Vector3(0,0,-1), rightShoulder.getChildJointWorldPosition() + new Vector3(0,0,-1), Color.red);
				//Debug.DrawLine(rightElbow.getChild().getWorldBottomPosition() + new Vector3(0,0,-1), rightElbow.getChildJointWorldPosition() + new Vector3(0,0,-1), Color.red);
			
				computeIKQandW (rightShoulderIndex, rightElbowIndex, parentAxis, -1 * character.getLocalLeftAxis (), character.getLocalLeftAxis (), childAxis, targetPos, false, targetPos, dt);
			}else{
				bipedIK.SetIKPositionWeight(AvatarIKGoal.RightHand, 1);
				bipedIK.SetIKPosition(AvatarIKGoal.RightHand, targetPos);
				bipedIK.SetIKRotation(AvatarIKGoal.RightHand, currentHeading);
				ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
				
				Debug.DrawLine(bipedIK.references.rightForearm.position, bipedIK.references.rightUpperArm.position, Color.red);
				Debug.DrawLine(bipedIK.references.rightHand.position, bipedIK.references.rightForearm.position, Color.green);
				
				rs.setJointRelativeOrientation(bipedIK.references.rightForearm.localRotation, rightElbowIndex);
				rs.setJointRelativeOrientation(bipedIK.references.rightUpperArm.localRotation, rightShoulderIndex);

				Vector3 f = -fixedPositionKp * (rHand.getWorldBottomPosition() - targetPos) - fixedPositionKd * rHand.getCMVelocity();
				rHand.addExternalForce (f);
			}
		}
		
		override protected void transitionToState(int stateIndex, int startStance = -1){
			
			if(behaviour==null) return;
			base.transitionToState(stateIndex, startStance);

			this.setDoubleStanceMode(((!behaviour.doubleStanceMode)?behaviour.doubleStanceMode:currentState.doubleStanceMode));
			this.setDoIPBelence(currentState.doIPBelence);
			this.comOffsetSagittal = (behaviour.comOffsetSagittal > 0) ? behaviour.comOffsetSagittal : currentState.comOffsetSagittal;
			this.comOffsetCoronal = (behaviour.comOffsetCoronal > 0) ? behaviour.comOffsetCoronal : currentState.comOffsetCoronal;
			this.duration = (behaviour.time > 0)?behaviour.time:currentState.getDuration();
			this.belenceError = (behaviour.belenceError > 0)?behaviour.belenceError:currentState.belenceError;
			float velSpeed = 0;
			if(Mathf.Abs(behaviour.velDSagittal) > 0){
				if(Mathf.Abs(behaviour.velDSagittal) > Mathf.Abs(this.velDSagittal)){
					this.velDSagittal = Mathf.SmoothDamp(this.velDSagittal, behaviour.velDSagittal, ref velSpeed, 0.03f);
				}else{
					this.velDSagittal = behaviour.velDSagittal;
				}
			}else{
				if(Mathf.Abs(currentState.velSagittal) > Mathf.Abs(this.velDSagittal)){
					this.velDSagittal = Mathf.SmoothDamp(this.velDSagittal, currentState.velSagittal, ref velSpeed, 0.03f);
				}else{
					this.velDSagittal = currentState.velSagittal;
				}
			}
			if(Mathf.Abs(behaviour.velDCoronal) > 0){
				if(Mathf.Abs(behaviour.velDCoronal) > Mathf.Abs(this.velDCoronal)){
					this.velDCoronal = Mathf.SmoothDamp(this.velDCoronal, behaviour.velDCoronal, ref velSpeed, 0.03f);
				}else{
					this.velDCoronal = behaviour.velDCoronal;
				}
			}else{
				if(Mathf.Abs(currentState.velCoronal) > Mathf.Abs(this.velDCoronal)){
					this.velDCoronal = Mathf.SmoothDamp(this.velDCoronal, currentState.velCoronal, ref velSpeed, 0.03f);
				}else{
					this.velDCoronal = currentState.velCoronal;
				}
			}
			//Debug.Log(currentState.velSagittal + " aaaaaaa  " + this.velDSagittal);
			this.rootExternalTorqueKp = (behaviour.rootExternalTorqueKp > 0)?behaviour.rootExternalTorqueKp:currentState.rootExternalTorqueKp;
			this.rootExternalTorqueKd = (behaviour.rootExternalTorqueKd > 0)?behaviour.rootExternalTorqueKd:currentState.rootExternalTorqueKd;
			this.rootExternalForceKp = (behaviour.rootExternalForceKp > 0)?behaviour.rootExternalForceKp:currentState.rootExternalForceKp;
			this.rootExternalForceKd = (behaviour.rootExternalForceKd > 0)?behaviour.rootExternalForceKd:currentState.rootExternalForceKd;
			this.fixedPositionKp = (behaviour.fixedPositionKp > 0)?behaviour.fixedPositionKp:currentState.fixedPositionKp;
			this.fixedPositionKd = (behaviour.fixedPositionKd > 0)?behaviour.fixedPositionKd:currentState.fixedPositionKd;
			this.velSagittalInAir = (behaviour.velSagittalInAir > 0)?behaviour.velSagittalInAir:currentState.velSagittalInAir;
			this.bodySpringKp = (behaviour.bodySpringKp > 0)?behaviour.bodySpringKp:currentState.bodySpringKp;
			this.bodySpringKd = (behaviour.bodySpringKd > 0)?behaviour.bodySpringKd:currentState.bodySpringKd;
			if(behaviour.impactBody == null){
				character.setBodySpringStuff(bodySpringKp, bodySpringKd);
			}else{
				character.setBodySpringStuff(behaviour.impactBody, bodySpringKp, bodySpringKd, true);
			}
			this.continueTrajectory = currentState.continueTrajectory;

			this.stepHeight = (behaviour.stepHeight > 0) ? behaviour.stepHeight : currentState.stepHeight;
			this.swingStepLength = (Mathf.Abs(behaviour.swingStepLength) > 0)?behaviour.swingStepLength:currentState.swingStepLength;
			this.standStepLength = (Mathf.Abs(behaviour.standStepLength) > 0)?behaviour.standStepLength:currentState.standStepLength;
			if(Mathf.Abs(this.swingStepLength) < 0.01f && Mathf.Abs(this.standStepLength) < 0.01f){
				isComputeIPStep = true;
			}else{
				isComputeIPStep = false;
			}
			behaviour.requestSagittalStepWidth(currentState.sagittalStepWidth);
			behaviour.requestCoronalStepWidth ((behaviour.coronalWidth > 0) ? behaviour.coronalWidth : currentState.coronalStepWidth);
			
			behaviour.doubleStanceMode = true;
			behaviour.time = 0;
			behaviour.belenceError = 0;
			behaviour.velDSagittal = 0;
			behaviour.velDCoronal = 0;
			behaviour.comOffsetSagittal = 0;
			behaviour.comOffsetCoronal = 0;
			behaviour.stepHeight = 0;
			behaviour.swingStepLength = 0;
			behaviour.standStepLength = 0;
			behaviour.coronalWidth = 0;
			behaviour.rootExternalTorqueKp = 0;
			behaviour.rootExternalTorqueKd = 0;
			behaviour.rootExternalForceKp = 0;
			behaviour.rootExternalForceKd = 0;
			behaviour.fixedPositionKp = 0;
			behaviour.fixedPositionKd = 0;
			behaviour.velSagittalInAir = 0;
			behaviour.bodySpringKp = 0;
			behaviour.bodySpringKd = 0;
			behaviour.impactBody = null;
		}
		
		private void updateStanceLegAngle(){
			if(standFootBasePosition[worldUpAxisID] > swingFootBasePosition[worldUpAxisID]){
				float difference = (standFootBasePosition[worldUpAxisID] - swingFootBasePosition[worldUpAxisID]);
				difference = Mathf.Clamp(difference, 0, behaviour.maxStepHeight);
				
				Vector3 legTopPos = standHip.getChildJointWorldPosition();
				Vector3 targetPos = standFootBasePosition;
				targetPos[worldUpAxisID] = legTopPos[worldUpAxisID] - Mathf.Abs(legLength - difference);

				if(bipedIK == null){
					Vector3 parentAxis = standKnee.getParentJointPosition() - standHip.getChildJointPosition();
					Vector3 childAxis = standKnee.getChild().getLocalBottomPosition() - standKnee.getChildJointPosition();
				
					computeIKQandW(stanceHipIndex, stanceKneeIndex, parentAxis, character.getLocalLeftAxis(), character.getLocalLeftAxis(), childAxis, targetPos, false, targetPos, SimGlobals.dt);
				}else{
					if(stance == SimGlobals.LEFT_STANCE){
						isComputeLeftLegIK = true;
						bipedIK.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);
						bipedIK.SetIKPosition(AvatarIKGoal.LeftFoot, targetPos);
						bipedIK.SetIKRotation(AvatarIKGoal.LeftFoot, currentHeading);
						ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
						
						Debug.DrawLine(bipedIK.references.leftCalf.position, bipedIK.references.leftThigh.position, Color.red);
						Debug.DrawLine(bipedIK.references.leftFoot.position, bipedIK.references.leftCalf.position, Color.red);
						
						rs.setJointRelativeOrientation(bipedIK.references.leftCalf.localRotation, stanceKneeIndex);
						rs.setJointRelativeOrientation(bipedIK.references.leftThigh.localRotation, stanceHipIndex);
					}else{
						isComputeRightLegIK = true;
						bipedIK.SetIKPositionWeight(AvatarIKGoal.RightFoot, 1);
						bipedIK.SetIKPosition(AvatarIKGoal.RightFoot, targetPos);
						bipedIK.SetIKRotation(AvatarIKGoal.RightFoot, currentHeading);
						ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
						
						Debug.DrawLine(bipedIK.references.rightCalf.position, bipedIK.references.rightThigh.position, Color.red);
						Debug.DrawLine(bipedIK.references.rightFoot.position, bipedIK.references.rightCalf.position, Color.red);
						
						rs.setJointRelativeOrientation(bipedIK.references.rightCalf.localRotation, stanceKneeIndex);
						rs.setJointRelativeOrientation(bipedIK.references.rightThigh.localRotation, stanceHipIndex);
					}
				}
			}
		}
		
		private void updateDoubleStanceLegAngle(){
			Vector3 leftFootPos = (stance == SimGlobals.LEFT_STANCE)?standFootBasePosition:swingFootBasePosition;
			Vector3 rightFootPos = (stance == SimGlobals.LEFT_STANCE)?swingFootBasePosition:standFootBasePosition;
			
			//leftFootPos.x = comPosition.x + 2;
			//rightFootPos.x = comPosition.x - 2;
			//Debug.Log("aaaaaaaaa  "+leftFootPos.y);
			//Debug.Log("bbbbbbbbb  "+rightFootPos.y);
			if(leftFootPos[worldUpAxisID] > rightFootPos[worldUpAxisID] + 0.00001f){
				float difference = (leftFootPos[worldUpAxisID] - rightFootPos[worldUpAxisID]);
				difference = Mathf.Clamp(difference, 0, behaviour.maxStepHeight);
				///Debug.Log("aaaaaaaaaa "+difference);
				CWJoint lHip = character.getJoint(lHipIndex);
				Vector3 legTopPos = lHip.getChildJointWorldPosition();
				Vector3 targetPos = leftFootPos;
				targetPos[worldUpAxisID] = legTopPos[worldUpAxisID] - Mathf.Abs(legLength - difference);
				computeIKLeftLegTargets(targetPos, SimGlobals.dt);
			}else if(leftFootPos[worldUpAxisID] < rightFootPos[worldUpAxisID] - 0.00001f){
				float difference = (rightFootPos[worldUpAxisID] - leftFootPos[worldUpAxisID]);
				difference = Mathf.Clamp(difference, 0, behaviour.maxStepHeight);
				//Debug.Log("bbbbbbbbbb "+difference);
				CWJoint rHip = character.getJoint(rHipIndex);
				Vector3 legTopPos = rHip.getChildJointWorldPosition();
				Vector3 targetPos = rightFootPos;
				targetPos[worldUpAxisID] = legTopPos[worldUpAxisID] - Mathf.Abs(legLength - difference);
				computeIKRightLegTargets(targetPos, SimGlobals.dt);
			}
		}
		
		override public void computeTorques(){
			//get the correct references for the swing knee and hip, as well as an estimate of the starting position for the swing foot

			isComputeShineIK = false;
			isComputeLeftLegIK = false;
			isComputeRightLegIK = false;
			//if (stanceBelence && d [localFrontAxisID] > 0) {
				//behaviour.hipBend = -150 * d [localFrontAxisID];
				//Debug.Log("vvbvbvbvbvbvbvbv "+d [localFrontAxisID]);
			//}
			computeDoubleStanceBelence();

			//if (Mathf.Abs(behaviour.hipBend) > 0) {
			//	behaviour.setHipBend (behaviour.hipBend, behaviour.hipBend);
			//} else {
			//	behaviour.setHipBend (0, 0);
			//}

			//evaluate the target orientation for every joint, using the SIMBICON state information
			evaluateJointTargets();
			
			//now overwrite the target angles for the swing hip and the swing knee in order to ensure foot-placement control
			if (!doubleStanceMode || stanceBelence) {
				computeIKSwingLegTargets (SimGlobals.dt);
				if (isComputeIPStep) {
					updateStanceLegAngle ();
					/*if (!getKneeIsFree (true)) {
						setKneeIsFree (true, true);
					}
					if (!getKneeIsFree (false)) {
						setKneeIsFree (false, true);
					}*/
				} else {
					/*if (getKneeIsFree (true)) {
						setKneeIsFree (true, false);
					}
					if (getKneeIsFree (false)) {
						setKneeIsFree (false, false);
					}*/
					computeIKStandLegTargets (SimGlobals.dt);
					if (isAddFootJoint && !stanceFoot.isFixedPosition ()) {
						float contactPosDis = (stanceFoot.getWorldBottomPosition () - standFootBasePosition).magnitude;
						if (Vector3.Dot (standFootBaseNormal, -1 * Physics.gravity.normalized) > 0.7f && contactPosDis < 0.5f) {
							Debug.Log("=== isAddFootJoint === "+standFootBasePosition);
							stanceFoot.addFixedPositionJoint (null, standFootBasePosition, 2000, 50);
						}
					}
				}
			} else if (doubleStanceMode && doIPBelence) {
				/*if (!getKneeIsFree (true)) {
					setKneeIsFree (true, true);
				}
				if (!getKneeIsFree (false)) {
					setKneeIsFree (false, true);
				}*/
				updateDoubleStanceLegAngle ();
			} else {
				/*if (getKneeIsFree (true)) {
					setKneeIsFree (true, false);
				}
				if (getKneeIsFree (false)) {
					setKneeIsFree (false, false);
				}*/
			}
			
			if (leftArmReachTargetPosition.magnitude > 0) {
				/*if (!getElbowIsFree (true)) {
					setElbowIsFree (true, true);
				}*/
				computeIKLeftArmTargets (leftArmReachTargetPosition, SimGlobals.dt);
			} else {
				/*if (getElbowIsFree (true)) {
					setElbowIsFree (true, false);
				}*/
				if (bipedIK != null) {
					bipedIK.SetIKPositionWeight(AvatarIKGoal.LeftHand, 0);
				}
			}
			if(rightArmReachTargetPosition.magnitude > 0){
				/*if(!getElbowIsFree(false)){
					setElbowIsFree(false, true);
				}*/
				computeIKRightArmTargets(rightArmReachTargetPosition, SimGlobals.dt);
			} else {
				/*if (getElbowIsFree (false)) {
					setElbowIsFree (false, false);
				}*/
				if (bipedIK != null) {
					bipedIK.SetIKPositionWeight(AvatarIKGoal.RightHand, 0);
				}
			}
			
			if(leftLegReachTargetPosition.magnitude > 0){
				computeIKLeftLegTargets(leftLegReachTargetPosition, SimGlobals.dt, true);
			}
			if(rightLegReachTargetPosition.magnitude > 0){
				computeIKRightLegTargets(rightLegReachTargetPosition, SimGlobals.dt, true);
			}
			if (!isComputeLeftLegIK) {
				bipedIK.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 0);
			}
			if (!isComputeRightLegIK) {
				bipedIK.SetIKPositionWeight(AvatarIKGoal.RightFoot, 0);
			}

			computeLookAtTarget (lookAtTargetPosition);
			computeShineIKTarget (shineTargetPosition);

			computePDTorques();
			computeRootExternalTorque();
			computeRootExternalForce();

			behaviour.hipBend = 0;
		}
		
		protected void computeRootExternalTorque(){
			float ang;
			Vector3 currAxis, desAxis, rotAxis, torque;
			if(!isFreeHeading){
				currAxis = currentHeading * character.getLocalFrontAxis();
				desAxis = desiredHeading * character.getLocalFrontAxis();
				rotAxis = Vector3.Cross(currAxis, desAxis).normalized;
				ang = Vector3.Angle(currAxis, desAxis);
				torque = (rootExternalTorqueKp * ang) * rotAxis;
				root.addExternalTorque(torque);
				//waist.getChild ().addExternalTorque (0.5f*torque);
				//torso.addExternalTorque (0.5f*torque);
				
				if(Vector3.Angle(currAxis, desAxis) > 30){
					this.doubleStanceMode = false;
					behaviour.doubleStanceMode = false;
				}

				Debug.DrawRay(root.getCMPosition(), 10*currAxis, Color.black);
			}
			
			currAxis = characterFrame * character.getLocalUpAxis();
			//Debug.DrawLine(head.getCMPosition(), head.getCMPosition() + 10*currAxis, Color.red);
			desAxis = currentHeading * qRootD * SimGlobals.upAxis;
			//Debug.DrawLine(head.getCMPosition(), head.getCMPosition() + 10*desAxis, Color.white);
			rotAxis = Vector3.Cross(currAxis, desAxis).normalized;
			ang = Mathf.Acos(Vector3.Dot(currAxis, desAxis)) * Mathf.Rad2Deg;
			torque = (rootExternalTorqueKp * ang) * rotAxis - rootExternalTorqueKd * root.getAngularVelocity();
			root.addExternalTorque(torque);

			if (isComputeShineIK) {
				currAxis = waist.getChild ().getOrientation () * character.getLocalUpAxis ();
				desAxis = bipedIK.references.spine [0].rotation * SimGlobals.upAxis;
				rotAxis = Vector3.Cross (currAxis, desAxis).normalized;
				ang = Mathf.Acos (Vector3.Dot (currAxis, desAxis)) * Mathf.Rad2Deg;
				torque = (rootExternalTorqueKp * ang) * rotAxis - rootExternalTorqueKd * waist.getChild ().getAngularVelocity ();
				waist.getChild ().addExternalTorque (0.5f*torque);

				currAxis = torso.getOrientation () * character.getLocalUpAxis ();
				desAxis = bipedIK.references.spine [1].rotation * SimGlobals.upAxis;
				rotAxis = Vector3.Cross (currAxis, desAxis).normalized;
				ang = Mathf.Acos (Vector3.Dot (currAxis, desAxis)) * Mathf.Rad2Deg;
				torque = (rootExternalTorqueKp * ang) * rotAxis - rootExternalTorqueKd * torso.getAngularVelocity ();
				torso.addExternalTorque (0.5f*torque);
			}
		}
		
		protected void computeRootExternalForce(){
			if(isCharacterStandUp()){
				//fixedPosition = root.getCMPosition();
				Vector3 vel = velDSagittal * character.getLocalFrontAxis() + velDCoronal * character.getLocalLeftAxis();
				Vector3 f = rootExternalForceKp * doubleStanceCOMError - rootExternalForceKd * (comVelocity - characterFrame * vel);
				f[worldUpAxisID] = 0;
				root.addExternalForce(f);
				//Debug.DrawLine(root.getCMPosition(),root.getCMPosition()+(Vector3)f,Color.blue);
			}
			/*else{
				Vector3 f = -fixedPositionKp*(new Vector3(root.getCMPosition().x, 0, root.getCMPosition().z) - new Vector3(fixedPosition.x, 0, fixedPosition.z))-fixedPositionKd*(comVelocity - (characterFrame * (velSagittalInAir * character.getLocalFrontAxis())));
				f.y = 0;
				root.addExternalForce(f);
				Debug.DrawLine(root.getCMPosition(),root.getCMPosition()+(Vector3)f,Color.black);
			}*/
		}
		/*
		protected void computeUpperBodyPos(){
			float error = -1 * doubleStanceCOMError;
			float targetPos = behaviour.getUpperBodyPos() + error;
			behaviour.setUpperBodyPose(targetPos);
		}*/
		
		/**
			returns a panic level which is 0 if val is between minG and maxG, 1 if it's
			smaller than minB or larger than maxB, and linearly interpolated 
		*/
		private float getValueInFuzzyRange(float val, float minB, float minG, float maxG, float maxB){
			if (val <= minB || val >= maxB)
				return 1;
			if (val >= minG && val <= maxG)
				return 0;
			if (val > minB && val < minG)
				return (minG - val) / (minG - minB);
			if (val > maxG && val < maxB)
				return (val - maxG) / (maxB - maxG);
			//the input was probably wrong, so return panic...
			return 1;
		}
		
		protected void computeDoubleStanceBelence(){
			if(!doubleStanceMode || !doIPBelence){
				stanceBelence = false;
				//checkBelenceTimer = 0;
				return;
			}
			Vector3 footWidth = root.getLocalCoordinatesVector(lFoot.getWorldBottomPosition() - rFoot.getWorldBottomPosition());
			if(doubleStanceCOMError.magnitude >= belenceError || Mathf.Abs(footWidth[localFrontAxisID]) > 2 * belenceError || (footWidth[localLeftAxisID] > -belenceError)){
				stanceBelence = true;
				/*if((Quaternion.Inverse(characterFrame)*(lFoot.getWorldBottomPosition() - comPosition))[localFrontAxisID] < -0.5f || (Quaternion.Inverse(characterFrame)*(rFoot.getWorldBottomPosition() - comPosition))[localFrontAxisID] < -0.5f){
					checkBelenceTimer += 50*Time.deltaTime;
					if(checkBelenceTimer > 20){
						behaviour.hipBend = -100;
					}
				}else{
					checkBelenceTimer = 0;
				}*/
			}else{
				//checkBelenceTimer = 0;
				stanceBelence = false;
			}
		}
		
		override public void blendOutTorques(){
		}
		
		/**
			updates the indexes of the swing and stance hip, knees and ankles
		*/
		public void updateSwingAndStanceReferences(){
			stanceHipIndex = ((stance == SimGlobals.LEFT_STANCE) ? (lHipIndex) : (rHipIndex));
			swingHipIndex = ((stance == SimGlobals.LEFT_STANCE) ? (rHipIndex) : (lHipIndex));
			swingKneeIndex = ((stance == SimGlobals.LEFT_STANCE) ? (rKneeIndex) : (lKneeIndex));
			stanceKneeIndex = ((stance == SimGlobals.LEFT_STANCE) ? (lKneeIndex) : (rKneeIndex));
			stanceAnkleIndex = ((stance == SimGlobals.LEFT_STANCE) ? (lAnkleIndex) : (rAnkleIndex));
			swingAnkleIndex = ((stance == SimGlobals.LEFT_STANCE) ? (rAnkleIndex) : (lAnkleIndex));
		
			swingHip = character.getJoint(swingHipIndex);
			swingKnee = character.getJoint(swingKneeIndex);
			swingAnkle = character.getJoint(swingAnkleIndex);
			standHip = character.getJoint(stanceHipIndex);
			standKnee = character.getJoint(stanceKneeIndex);
			standAnkle = character.getJoint(stanceAnkleIndex);
		}
		
		public Vector3 getStandFootTargetLocation(float t, Vector3 com, Quaternion charFrameToWorld){
			Vector3 step = Vector3.zero;
			step[localFrontAxisID] = standFootSagittalTrajectory.evaluate_catmull_rom(t);
			float halfLegWidth = (stance == SimGlobals.LEFT_STANCE)?legWidth/2:-legWidth/2;
			step[localLeftAxisID] = localLeftAxisSign*halfLegWidth + standFootCoronalTrajectory.evaluate_catmull_rom(t);
			step = charFrameToWorld*step;
			step = com + step;
			
			if(t <= 1){
				Vector3 legTopPos = standHip.getChildJointWorldPosition();
				if(swingToStandBaseVector[localFrontAxisID] < 0 && standFootBasePosition[worldUpAxisID] > swingFootBasePosition[worldUpAxisID]){
					float difference = (standFootBasePosition[worldUpAxisID] - swingFootBasePosition[worldUpAxisID]);
					//difference = Mathf.Clamp(difference, 0, behaviour.maxStepHeight);
					step[worldUpAxisID] = legTopPos[worldUpAxisID] - Mathf.Abs(legLength - difference);
				}else{
					step[worldUpAxisID] = legTopPos[worldUpAxisID] - legLength;
				}
			}else{
				step[worldUpAxisID] = standFootBasePosition[worldUpAxisID];
			}
		//	Debug.DrawRay(step, new Vector3(0,-5,0), Color.yellow);
			return step;
		}
		
		/**
			This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
			is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
			initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
		*/
		public Vector3 getSwingFootTargetLocation(float t, Vector3 com, Quaternion charFrameToWorld){
			Vector3 step = Vector3.zero;
			step[localFrontAxisID] = swingFootSagittalTrajectory.evaluate_catmull_rom(t);
			float halfLegWidth = (stance == SimGlobals.LEFT_STANCE)?-legWidth/2:legWidth/2;
			step[localLeftAxisID] = localLeftAxisSign*halfLegWidth + swingFootCoronalTrajectory.evaluate_catmull_rom(t);
			if(t>0.7f && standFootBasePosition[worldUpAxisID] < swingFootFrontPosition[worldUpAxisID] - 0.01f){
				velDSagittal = 0.1f;
				step[localFrontAxisID] -= swingFootFrontLength/2;
			}
			//now transform this vector into world coordinates
			step = charFrameToWorld*step;
			
			//add it to the com location
			step = com + step;
			//finally, set the desired height of the foot
			if(t <= 1){
				float difference = Mathf.Abs(swingFootFrontPosition[worldUpAxisID] - standFootBasePosition[worldUpAxisID]);
				if(t<0.7f && difference < behaviour.maxStepHeight && standFootBasePosition[worldUpAxisID] >= swingFootBasePosition[worldUpAxisID] && standFootBasePosition[worldUpAxisID] <= swingFootFrontPosition[worldUpAxisID]){
					step [worldUpAxisID] = swingFootHeightTrajectory.evaluate_catmull_rom (t) + swingLegPanicHeight + swingFootFrontPosition [worldUpAxisID] + additionHeight;
				}else{
					step[worldUpAxisID] = Mathf.Clamp(swingFootHeightTrajectory.evaluate_catmull_rom(t) + swingLegPanicHeight, 0, behaviour.maxStepHeight) + swingFootBasePosition[worldUpAxisID] + additionHeight;
				}
			}else{
				step[worldUpAxisID] = swingFootBasePosition[worldUpAxisID];
			}
		//	Debug.DrawRay(step, new Vector3(0,-5,0), Color.yellow);
			//step += (Vector3)computeSwingFootDelta(t);
			return step;
		}
		
		/**
			This method is used to compute the desired orientation and angular velocity for a parent RB and a child RB, relative to the grandparent RB and
			parent RB repsectively. The input is:
				- the index of the joint that connects the grandparent RB to the parent RB, and the index of the joint between parent and child
		
				- the distance from the parent's joint with its parent to the location of the child's joint, expressed in parent coordinates
		
				- two rotation normals - one that specifies the plane of rotation for the parent's joint, expressed in grandparent coords, 
				  and the other specifies the plane of rotation between the parent and the child, expressed in parent's coordinates.
		
			    - The position of the end effector, expressed in child's coordinate frame
		
				- The desired position of the end effector, expressed in world coordinates
		
				- an estimate of the desired position of the end effector, in world coordinates, some dt later - used to compute desired angular velocities
		*/
		public void computeIKQandW(int parentJIndex, int childJIndex, Vector3 parentAxis, Vector3 parentNormal, Vector3 childNormal, Vector3 childEndEffector, Vector3 wP, bool computeAngVelocities, Vector3 futureWP, float dt){
			
			//this is the joint between the grandparent RB and the parent
			CWJoint parentJoint = character.getJoint(parentJIndex);
			//this is the grandparent - most calculations will be done in its coordinate frame
			CWRigidBody gParent = parentJoint.getParent();
			//this is the reduced character space where we will be setting the desired orientations and ang vels.
			ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
		
			//the desired relative orientation between parent and grandparent
			Quaternion qParent = Quaternion.identity;
			//and the desired relative orientation between child and parent
			Quaternion qChild = Quaternion.identity;
		
			TwoLinkIK.getIKOrientations(this, parentJoint, gParent, parentJoint.getParentJointPosition(), gParent.getLocalCoordinatesPoint(wP), parentNormal, parentAxis, childNormal, childEndEffector, ref qParent, ref qChild);
		
			rs.setJointRelativeOrientation(qChild, childJIndex);
			rs.setJointRelativeOrientation(qParent, parentJIndex);
		/*
			if (computeAngVelocities){
				Vector3 wParentD = Vector3.zero;
				Vector3 wChildD = Vector3.zero;
				//the joint's origin will also move, so take that into account, by subbing the offset by which it moves to the
				//futureTarget (to get the same relative position to the hip)
				Quaternion parentQua = gParent.getOrientation();
				Vector3 velOffset = Vector3.Cross(gParent.getAngularVelocity(), parentQua*parentJoint.getParentJointPosition()) + gParent.getCMVelocity();
		
				Quaternion qParentF = Quaternion.identity;
				Quaternion qChildF = Quaternion.identity;
				TwoLinkIK.getIKOrientations(parentJoint.getParentJointPosition(), gParent.getLocalCoordinatesPoint(futureWP + velOffset * -dt), parentNormal, parentAxis, childNormal, childEndEffector, ref qParentF, ref qChildF);
		
				Quaternion qDiff = qParentF * MathLib.getComplexConjugate(qParent);
				wParentD = new Vector3(qDiff.x,qDiff.y,qDiff.z) * 2.0f/dt;
				//the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we'll account for that
				wParentD -= MathLib.getComplexConjugate(parentQua) * gParent.getAngularVelocity();
		
				qDiff = qChildF * MathLib.getComplexConjugate(qChild);
				wChildD = new Vector3(qDiff.x,qDiff.y,qDiff.z) * 2.0f/dt;
		
				//make sure we don't go overboard with the estimates, in case there are discontinuities in the trajectories...
				/*wChildD.x = Mathf.Clamp(wChildD.x, -5.0f, 5.0f);
				wChildD.y = Mathf.Clamp(wChildD.y, -5.0f, 5.0f);
				wChildD.z = Mathf.Clamp(wChildD.z, -5.0f, 5.0f);
				wParentD.x = Mathf.Clamp(wParentD.x, -5.0f, 5.0f);
				wParentD.y = Mathf.Clamp(wParentD.y, -5.0f, 5.0f);
				wParentD.z = Mathf.Clamp(wParentD.z, -5.0f, 5.0f);*/
				/*
				rs.setJointRelativeAngVelocity(wChildD.z, childJIndex);
				rs.setJointRelativeAngVelocity(wParentD.z, parentJIndex);
			}*/
		}
		
		override public void performPreTasks(float dt) { 
			behaviour.simStepPlan(SimGlobals.dt);
			
			bodyTouchedTheGround = false;
			lFootTouchedTheGround = false;
			rFootTouchedTheGround = false;
			twoFootTouchedGround = false;
			int footTouchedNum = 0;
			//see if anything else other than the feet touch the ground...
			ContactPointInfo cp;
			if(root.contactPoints.Count>0){
				cp = (ContactPointInfo)root.contactPoints[0];
				if(cp.collider2 != null && cp.collider2.tag.Contains("ground")){
					currentTouchedBody = root;
					bodyTouchedTheGround = true;
				}
			}
			if(!bodyTouchedTheGround && torso.contactPoints.Count>0){
				cp = (ContactPointInfo)torso.contactPoints[0];
				if(cp.collider2 != null && cp.collider2.tag.Contains("ground")){
					currentTouchedBody = torso;
					bodyTouchedTheGround = true;
				}
			}
			if(!bodyTouchedTheGround && head.contactPoints.Count>0){
				cp = (ContactPointInfo)head.contactPoints[0];
				if(cp.collider2 != null && cp.collider2.tag.Contains("ground")){
					currentTouchedBody = head;
					bodyTouchedTheGround = true;
				}
			}
			
			if(lFoot.contactPoints.Count>0){
				cp = (ContactPointInfo)lFoot.contactPoints[0];
				if(cp.collider2 != null && cp.collider2.tag.Contains("ground") && Vector3.Dot(cp.n, SimGlobals.upAxis)>0.7f){
					footTouchedNum ++;
					lFootTouchedTheGround = true;
				}
			}
			if(rFoot.contactPoints.Count>0){
				cp = (ContactPointInfo)rFoot.contactPoints[0];
				if(cp.collider2 != null && cp.collider2.tag.Contains("ground") && Vector3.Dot(cp.n, SimGlobals.upAxis)>0.7f){
					footTouchedNum ++;
					rFootTouchedTheGround = true;
				}
			}
			if(footTouchedNum > 1){
				twoFootTouchedGround = true;
			}
			//Debug.Log("aaaaaaaaaaaaa  "+bodyTouchedTheGround);
			//Debug.Log("bbbbbbbbbbbbb  "+footTouchedTheGround);
			//Debug.Log("ccccccccccccc  "+twoFootTouchedGround);
			base.performPreTasks(dt);
		}
		
		override public bool performPostTasks(float dt) {
			bool newState = base.performPostTasks(dt);
			
			if( newState ) {
				behaviour.conTransitionPlan();
			}
		
			return newState;
		}
	}
}
