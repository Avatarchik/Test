using UnityEngine;
using System.Collections;
using CartWheelCore.Math;
using System.IO;

namespace CartWheelCore.Control{
	
	
	/**
	 * A simbicon controller is a fancy PoseController. The root (i.e. pelvis or torso), as well as the two hips are controlled
	 * relative to a semi-global coordinate frame (it needs only have the y-axis pointing up), but all other joints are controlled
	 * exactely like in a normal PoseController. The SimBiController operates on a biped, which means that we can make special
	 * assumptions about it: it must have two joints, lHip and rHip, connecting the root to the left and right upper-legs,
	 * and it must also have two feet (lFoot and rFoot) as rigid bodies in the articulated linkage.
	 */
	public class SimBiController : PoseController {
		
		/**
			These are quantities that are set only once
		*/
		//we will keep a reference to the left and right feet to be able to determine when the stance switches
		protected CWRigidBody lUpperLeg;
		protected CWRigidBody rUpperLeg;
		protected CWRigidBody lLowerLeg;
		protected CWRigidBody rLowerLeg;
		protected CWRigidBody lFoot;
		protected CWRigidBody rFoot;
		protected CWRigidBody lHand;
		protected CWRigidBody rHand;
		//we will also keep a reference to the root of the figure, to be able to identify the semi-global coordinate frame quickly
		protected CWRigidBody root;
		protected CWRigidBody torso;
		protected CWRigidBody head;
		//we also need to keep track of the joint indices in the articulated figure for the two hips, since they are special
		protected int lHipIndex;
		protected int rHipIndex;
		//this is a collection of the states that are used in the controller
		protected SimBiData statesData;
		
		protected Quaternion qRootD;
		
		/**
			these are quantities that get updated throughout the simulation
		*/
		//this value indicates which side is the stance side. 
		protected int stance;
		protected float duration;
		//a pointer to the swing and stance feet
		protected CWRigidBody stanceFoot;
		protected CWRigidBody swingFoot;
		protected CWRigidBody stanceHand;
		protected CWRigidBody swingHand;
		//keep track of the swing and stance hip indices
		protected int stanceHipIndex;
		protected int swingHipIndex;
		//this is the index of the controller that is currently active
		protected int FSMStateIndex;
		protected int prevFSMState;
		protected int nextFSMState;
		protected SimBiConState currentState;
		
		//this is the world relative velocity of the COM
		protected Vector3 comVelocity;
		//and this is the position
		protected Vector3 comPosition;
		
		//this is the vector from the cm of the stance foot to the cm of the character
		protected Vector3 d;
		//this is the velocity of the cm of the character, in character frame
		protected Vector3 v;		
		protected float phi;
		
		protected float comOffsetSagittal;
		protected float comOffsetCoronal;
		//this is the distance between the COM and the midpoint between the feet
		protected Vector3 doubleStanceCOMError;
		protected float fallDownAngle;
		
		protected bool isFreeHeading;
		protected bool isLeftReverseStance;
		protected bool isAddFootJoint;
	
		//this variable, updated everytime the controller state is advanced in time, is set to true if any body other than the feet are in contact
		//with the ground, false otherwise. A higer level process can determine if the controller failed or not, based on this information.
		protected bool bodyTouchedTheGround;
		protected bool lFootTouchedTheGround;
		protected bool rFootTouchedTheGround;
		protected bool twoFootTouchedGround;
		
		public delegate void TransitionToStateHandler(object sender, System.EventArgs e);
		public event TransitionToStateHandler onStartNewState;
		public event TransitionToStateHandler onCurrentStateEnded;
		
		public delegate void HitFeedbackHandler(HitFeedback feedback, ContactPointInfo cp);
		public event HitFeedbackHandler onHitOtherSBCharacter;
		
		public void gotoState(int index, bool immediately = true, int startStance = -1){
			if(prevFSMState!=index){
				if(immediately){
					transitionToState(index, startStance);
				}else{
					nextFSMState = index;
					startingStance = startStance;
				}
			}
		}
		
		/**
			This method is used to set the current FSM state of the controller to that of the index that
			is passed in.
		*/
		protected void setFSMStateTo(int index){
			if (index<0)
				FSMStateIndex = -1;
			else if( index >= statesData.getStateCount() )
				FSMStateIndex = statesData.getStateCount()-1;
			else
				FSMStateIndex = index;
			
			if(FSMStateIndex>=0){
				currentState = statesData.getState(FSMStateIndex);
			}
		}
		
		/**
			This method should be called when the controller transitions to this state.
		*/
		virtual protected void transitionToState(int stateIndex, int startStance = -1){
			if(prevFSMState >= 0 && prevFSMState != stateIndex){
				nextFSMState = -1;
				if(isAddFootJoint && (currentState.swingStepLength != 0 || currentState.standStepLength != 0)){
					lFoot.removeFixedPositionJoint();
					rFoot.removeFixedPositionJoint();
				}
				this.onCurrentStateEnded(this, new System.EventArgs());
			}
			setFSMStateTo(stateIndex);
			if(startStance < 0){
				setStance(currentState.getStateStance(this.stance));
			}else{
				setStance(startStance);
			}
			this.phi = 0;
			
			if(prevFSMState != stateIndex){
				this.onStartNewState(this, new System.EventArgs());
			}
			prevFSMState = FSMStateIndex;
		}
		
		/**
			This method is used to resolve the names (map them to their index) of the joints.
		*/
		protected void resolveJoints(SimBiConState state){
			string tmpName;
			for (int i=0;i<state.getExternalForceCount();i++){
				ExternalForce ef = state.getExternalForce(i);
			
				//deal with the SWING_XXX' case
				if(ef.bName.StartsWith("SWING_")){
					tmpName = ef.bName.Substring(6);
					tmpName = tmpName.Insert(0,"r");
					ef.leftStanceARBIndex = character.getARBIndexByName(tmpName);
					//if (ef.leftStanceARB==null)
						//throwError("Cannot find arb %s\n", tmpName);
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"l");
					ef.rightStanceARBIndex = character.getARBIndexByName(tmpName);
					//if (ef.rightStanceARB==null)
						//throwError("Cannot find arb %s\n", tmpName);
					continue;
				}
				//deal with the STANCE_XXX' case
				if(ef.bName.StartsWith("STANCE_")){
					tmpName = ef.bName.Substring(7);
					tmpName = tmpName.Insert(0,"l");
					ef.leftStanceARBIndex = character.getARBIndexByName(tmpName);
					//if (ef.leftStanceARB==null)
						//throwError("Cannot find arb %s\n", tmpName);
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"r");
					ef.rightStanceARBIndex = character.getARBIndexByName(tmpName);
					//if (ef.rightStanceARB==null)
						//throwError("Cannot find arb %s\n", tmpName);
					continue;
				}
				//if we get here, it means it is just the name...
				ef.leftStanceARBIndex = character.getARBIndexByName(ef.bName);
				//if (ef.leftStanceARB==null)
					//throwError("Cannot find joint %s\n", ef->bName);
				ef.rightStanceARBIndex = ef.leftStanceARBIndex;
			}
		
			for (int i=0;i<state.getTrajectoryCount();i++){
				Trajectory jt = state.getTrajectory(i);
				//deal with the 'root' special case
				if (jt.jName == "root"){
					jt.leftStanceIndex = jt.rightStanceIndex = -1;
				}else if(jt.jName.StartsWith("SWING_")){
					tmpName = jt.jName.Substring(6);
					tmpName = tmpName.Insert(0,"r");
					jt.leftStanceIndex = character.getJointIndex(tmpName);
					//if (jt.leftStanceIndex<0)
						//throwError("Cannot find joint %s\n", tmpName);
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"l");
					jt.rightStanceIndex = character.getJointIndex(tmpName);
					//if (jt.rightStanceIndex<0)
						//throwError("Cannot find joint %s\n", tmpName);
				}else if(jt.jName.StartsWith("STANCE_")){
					tmpName = jt.jName.Substring(7);
					tmpName = tmpName.Insert(0,"l");
					jt.leftStanceIndex = character.getJointIndex(tmpName);
					//if (jt.leftStanceIndex<0)
						//throwError("Cannot find joint %s\n", tmpName);
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"r");
					jt.rightStanceIndex = character.getJointIndex(tmpName);
					//if (jt.rightStanceIndex<0)
						//throwError("Cannot find joint %s\n", tmpName);
				}else{
					//if we get here, it means it is just the name...
					jt.leftStanceIndex = character.getJointIndex(jt.jName);
					//if (jt.leftStanceIndex<0)
						//throwError("Cannot find joint %s\n", jt->jName);
					jt.rightStanceIndex = jt.leftStanceIndex;
				}
				if(jt.hitFeedback != null){
					jt.hitFeedback.hitARBIndex = getRBIndexBySymbolicName(jt.jName);
				}
			}
		}
		
		/**
			This method is used to return a pointer to a rigid body, based on its name and the current stance of the character
		*/
		protected int getRBIndexBySymbolicName(string sName){
			int result = -1;
			string resolvedName;
			//deal with the SWING/STANCE_XXX' case
			if(sName.StartsWith("SWING_")){
				resolvedName = sName.Substring(6);
				if (stance == SimGlobals.LEFT_STANCE)
					resolvedName = resolvedName.Insert(0,"r");
				else
					resolvedName = resolvedName.Insert(0,"l");
			}else if (sName.StartsWith("STANCE_")){
					resolvedName = sName.Substring(7);
					if (stance == SimGlobals.LEFT_STANCE)
						resolvedName = resolvedName.Insert(0,"l");
					else
						resolvedName = resolvedName.Insert(0,"r");
			}else{
				resolvedName = sName;
			}
		
			result = character.getJointIndex(resolvedName) + 1;
		
			return result;
		}
		
		//keep a copy of the initial character state, starting stance and file that contains the character's initial state
		public int startingState;
		public int startingStance;
		
		public SimBiController(Character b):base(b){
			//if (b == null)
				//throwError("Cannot create a SIMBICON controller if there is no associated biped!!");
			//characters controlled by a simbicon controller are assumed to have: 2 feet
			lLowerLeg = b.getARBByName("lLowerLeg");
			rLowerLeg = b.getARBByName("rLowerLeg");

			lUpperLeg = b.getARBByName ("lUpperLeg");
			rUpperLeg = b.getARBByName ("rUpperLeg");
			
			lFoot = lLowerLeg;
			rFoot = rLowerLeg;
			
			lHand = b.getARBByName("lLowerArm");
			rHand = b.getARBByName("rLowerArm");
			
			//and two hips connected to the root
			//Joint lHip = b.getJointByName("lHip");
			//Joint rHip = b.getJointByName("rHip");
		
			lHipIndex = b.getJointIndex("lHip");
			rHipIndex = b.getJointIndex("rHip");
			
			root = b.getRoot();
			torso = b.getARBByName("torso");
			head = b.getARBByName("head");
			
			statesData = null;
			comVelocity = new Vector3();
			comPosition = new Vector3();
			d = new Vector3();
			v = new Vector3();
			doubleStanceCOMError = new Vector3();
			comOffsetSagittal = 0;
			comOffsetCoronal = 0;
			
			qRootD = Quaternion.identity;
			
			isFreeHeading = true;
			isAddFootJoint = false;

			setStance(SimGlobals.LEFT_STANCE);
			phi = 0;
		
			setFSMStateTo(-1);
			prevFSMState = -1;
			nextFSMState = -1;

			isLeftReverseStance = true;
			
			bodyTouchedTheGround = false;
			lFootTouchedTheGround = false;
			rFootTouchedTheGround = false;
			twoFootTouchedGround = false;
			
			startingState = 1;
			startingStance = SimGlobals.LEFT_STANCE;
		}
		
		public void setIsLeftReverseStance(bool isLeft){
			isLeftReverseStance = isLeft;
		}
		
		public float getComOffsetSagittal(){
			return comOffsetSagittal;
		}
		public void setComOffsetSagittal(float offset){
			comOffsetSagittal = offset;
		}
		public float getComOffsetCoronal(){
			return comOffsetCoronal;
		}
		public void setComOffsetCoronal(float offset){
			comOffsetCoronal = offset;
		}
		
		public Vector3 getDoubleStanceCOMError() { return doubleStanceCOMError; }
		
		public CWRigidBody getStanceFoot() { return stanceFoot; }
		public CWRigidBody getSwingFoot() { return swingFoot; }	
		public CWRigidBody getStanceHand() { return stanceHand; }
		public CWRigidBody getSwingHand() { return swingHand; }
		
		/**
			This method is used to set the stance 
		*/
		public void setStance(int newStance){
			stance = newStance;
			if (stance == SimGlobals.LEFT_STANCE){
				stanceFoot = lFoot;
				swingFoot = rFoot;
				stanceHand = lHand;
				swingHand = rHand;
				swingHipIndex = rHipIndex;
				stanceHipIndex = lHipIndex;
			}else{
				stanceFoot = rFoot;
				swingFoot = lFoot;
				stanceHand = rHand;
				swingHand = lHand;
				swingHipIndex = lHipIndex;
				stanceHipIndex = rHipIndex;
			}
		}
		
		public void setStatesData(SimBiData data){
			statesData = data;
		}
		public SimBiData getStatesData(){
			return statesData;
		}
		
		public SimBiConState getCurrentState() {
			return currentState;
		}
		
		/**
			This method is used to compute the PD torques, given the target trajectories
		*/
		public void computePDTorques(){
			//compute the torques now, using the desired pose information - the hip torques will get overwritten below
			base.computeTorques();
		}
		
		/**
			This method is used to compute the torques that are to be applied at the next step.
		*/
		override public void computeTorques(){
			if( FSMStateIndex < 0 )
				setFSMStateTo( startingState );
		
			if (FSMStateIndex >= statesData.getStateCount()){
				//tprintf("Warning: no FSM state was selected in the controller!\n");
				return;
			}
			evaluateJointTargets();
			computePDTorques();
			//blendOutTorques();
		}
		
		public void evaluateJointTargets(){
			ReducedCharacterState poseRS = new ReducedCharacterState(desiredPose);
		
			//d and v are specified in the rotation (heading) invariant coordinate frame
			updateDAndV();
		
			//there are two stages here. First we will compute the pose (i.e. relative orientations), using the joint trajectories for the current state
			//and then we will compute the PD torques that are needed to drive the links towards their orientations - here the special case of the
			//swing and stance hips will need to be considered
		
			//always start from a neutral desired pose, and build from there...
			root.setExternalForce(Vector3.zero);
			root.setExternalTorque(Vector3.zero);
			CWJoint j;
			for (int i=0;i<jointCount;i++){
				j = character.getJoint(i);
				j.getChild().setExternalForce(Vector3.zero);
				j.getChild().setExternalTorque(Vector3.zero);
				poseRS.setJointRelativeOrientation(Quaternion.identity, i);
				poseRS.setJointRelativeAngVelocity(Vector3.zero, i);
			}

			float phiToUse = phi;
			//make sure that we only evaluate trajectories for phi values between 0 and 1
			if (phiToUse>1)
				phiToUse = 1;
			
			Vector3 force = Vector3.zero;
			Vector3 torque = Vector3.zero;
		
			// First compute external forces
			for (int i=0;i<currentState.getExternalForceCount();i++){
				ExternalForce ef = currentState.getExternalForce(i);
				CWRigidBody arb = character.getArticulatedRigidBody(ef.getARBIndex(stance));
				
				force = Vector3.zero;
				force += character.getLocalFrontAxis() * ef.forceX.evaluate_catmull_rom( phiToUse );
				force += character.getLocalUpAxis() * ef.forceY.evaluate_catmull_rom( phiToUse );
				force += character.getLocalLeftAxis() * ef.forceZ.evaluate_catmull_rom( phiToUse );
				force = characterFrame*force;
				arb.addExternalForce(force);
		
				torque = Vector3.zero;
				torque += character.getLocalFrontAxis() * ef.torqueX.evaluate_catmull_rom( phiToUse );
				torque += character.getLocalUpAxis() * ef.torqueY.evaluate_catmull_rom( phiToUse );
				torque += character.getLocalLeftAxis() * ef.torqueZ.evaluate_catmull_rom( phiToUse );
				torque = characterFrame*torque;
				arb.addExternalTorque(torque);
			}
			//Debug.Log(Random.value + "ffffffffffff  "+curState.getTrajectoryCount());
			for (int i=0;i<currentState.getTrajectoryCount();i++){
				Trajectory tra = currentState.getTrajectory(i);
				//now we have the desired rotation angle and axis, so we need to see which joint this is intended for
				int jIndex = tra.getJointIndex(stance, isLeftReverseStance);
		//Debug.Log(Random.value + "00000000000  "+(curState.sTraj[i] as Trajectory).getJointName());
				j = character.getJoint(jIndex);
		
				//get the desired joint orientation to track - include the feedback if necessary/applicable
				Vector3 d0 = new Vector3();
				Vector3 v0 = new Vector3();
				computeD0( phiToUse, ref d0 );
				computeV0( phiToUse, ref v0 );
				
				//if the index is -1, it must mean it's the root's trajectory. Otherwise we should give an error
				if (jIndex == -1){
					qRootD = tra.evaluateTrajectory(this, j, stance, phiToUse, d - d0, v - v0);
				}else{
					Quaternion newOrientation = tra.evaluateTrajectory(this, j, stance, phiToUse, d - d0, v - v0);
					j.setRelToRootFrame(tra.relToCharFrame);
					poseRS.setJointRelativeOrientation(newOrientation, jIndex);
				}
				
				if(tra.hitFeedback != null){
					CWRigidBody arb = character.getArticulatedRigidBody(tra.hitFeedback.hitARBIndex);
					if(phiToUse > tra.hitFeedback.minTime && arb.contactPoints.Count>0){
						ContactPointInfo cp = (ContactPointInfo)arb.contactPoints[0];
						onHitOtherSBCharacter(tra.hitFeedback, cp);
					}
				}
			}
		}
		
		/**
			This method is used to lower the torques as the character gets closer to the ground (so that it doesn't try to keep walking).
		*/
		virtual public void blendOutTorques(){
		}
		
		/**
			This method is used to obtain the d and v parameters, using the current postural information of the biped
		*/
		public void updateDAndV(){
			characterFrame = root.getOrientation();
			currentHeading = character.getHeading();
			
			Vector3 comOffset = Vector3.zero;
			comOffset[localFrontAxisID] = comOffsetSagittal;
			comOffset[localLeftAxisID] = comOffsetCoronal;
			comOffset = characterFrame * comOffset;
			
			comPosition = character.getCOM();
			comPosition += comOffset;
		
			comVelocity = character.getCOMVelocity();
		
		//	d = comPosition - stanceFoot.getWorldBottomPosition();
			//d is now in world coord frame, so we'll represent it in the 'character frame'
		//	d = Quaternion.Inverse(characterFrame)*d;
			//compute v in the 'character frame' as well
			v = Quaternion.Inverse(characterFrame)*comVelocity;
			
			//and now compute the vector from the COM to the center of midpoint between the feet, again expressed in world coordinates
			Vector3 feetMidpoint = (stanceFoot.getWorldBottomPosition() + swingFoot.getWorldBottomPosition());
			feetMidpoint /= 2.0f;
			
			Vector3 comToFeetMiddle = comPosition - feetMidpoint;
			d = Quaternion.Inverse(characterFrame) * comToFeetMiddle;
			
			fallDownAngle = Mathf.Acos(Vector3.Dot(comToFeetMiddle.normalized, SimGlobals.upAxis))*Mathf.Rad2Deg;
			if(Vector3.Dot(characterFrame * character.getLocalFrontAxis(), SimGlobals.upAxis) < 0 && fallDownAngle > 30){
				Vector3 dirAxis = head.getCMPosition() - root.getCMPosition();
				Vector3 axis = (Vector3.Project(dirAxis, Vector3.right) + Vector3.Project(dirAxis, Vector3.forward)).normalized;
				float ang = Vector3.Angle(axis, character.getLocalFrontAxis());
				if(Vector3.Dot(Vector3.Cross(character.getLocalFrontAxis(), axis), SimGlobals.upAxis) < 0){
					ang = -ang;
				}
				currentHeading = Quaternion.AngleAxis(ang, SimGlobals.upAxis);
			}
			
			//Debug.DrawRay(feetMidpoint, comToFeetMiddle, Color.red);
			//now we have to compute the difference between the current COM and the desired COMPosition, in world coordinates
			doubleStanceCOMError = feetMidpoint - comPosition;
			doubleStanceCOMError[worldUpAxisID] = 0;
			//and add the user specified offset
			
			//Debug.DrawRay(root.getCMPosition(), 10*(currentHeading*character.getLocalFrontAxis()), Color.black);
			
			//Debug.Log("hhhhhhhhhh  "+fallDownAngle + " aaaaaaaaaaaa "+doubleStanceCOMError);
		}
		
		/**
			This method is used to return the current state number
		*/
		public int getFSMState(){
			//if( FSMStateIndex < 0 )
				//setFSMStateTo( startingState );
			return this.FSMStateIndex;
		}
		
		public int getPrevFSMState(){
			return prevFSMState;
		}
		
		/**
			This method returns the character frame orientation
		*/
		public Quaternion getCharacterFrame(){
			return characterFrame;
		}
		
		public Quaternion getCurrentHeading(){
			return currentHeading;
		}
		
		public Quaternion getDesiredHeading(){
			return desiredHeading;
		}
		public void setDesiredHeading(Quaternion heading){
			//Debug.Log("aaaasdd ==== "+ Quaternion.Angle (currentHeading, heading));
			float v = -10.0f / 170.0f * Quaternion.Angle (currentHeading, heading) + 15.6f;
			desiredHeading = Quaternion.Lerp(currentHeading, heading, v * Time.deltaTime);
		}
		public void setDesiredHeading(float heading){
			setDesiredHeading(Quaternion.AngleAxis(heading, SimGlobals.upAxis));
		}
		public void setDesiredHeading(Vector3 targetPos){
			Vector3 dirAxis = targetPos - root.getCMPosition();
			Vector3 axis = (Vector3.Project(dirAxis, Vector3.right) + Vector3.Project(dirAxis, Vector3.forward)).normalized;
			float ang = Vector3.Angle(axis, character.getLocalFrontAxis());
			if(Vector3.Dot(Vector3.Cross(character.getLocalFrontAxis(), axis), SimGlobals.upAxis) < 0){
				ang = -ang;
			}
			setDesiredHeading(ang);
		}
		
		public bool getIsFreeHeading(){
			return isFreeHeading;
		}
		public void setIsFreeHeading(bool isFree){
			isFreeHeading = isFree;
		}

		public bool getIsAddFootJoint(){
			return isAddFootJoint;
		}
		public void setisAddFootJoint(bool isAdd){
			isAddFootJoint = isAdd;
			
			if (!isAdd) {
				lFoot.removeFixedPositionJoint();
				rFoot.removeFixedPositionJoint();
			}
		}
		
		public Vector3 getComVelocity(){
			return comVelocity;
		}
		
		public Vector3 getComPosition(){
			return comPosition;
		}
		
		/**
			this method returns the stance of the character
		*/
		public int getStance(){
			return stance;
		}
		
		public void setStartingState( int state ) {
			startingState = state;
			//notifyObservers();
		}
	
		public int getStartingState() { return startingState; } 
		
		// Evaluate the D trajectory
		public void computeD0( float phi, ref Vector3 d0 ) {
			computeDorV( phi, currentState.getDTrajX(), currentState.getDTrajZ(), stance, ref d0 );
		}
	
		// Evaluate the V trajectory 
		public void computeV0( float phi, ref Vector3 v0 ) {
			computeDorV( phi, currentState.getVTrajX(), currentState.getVTrajZ(), stance, ref v0 );
		}
		
		// Evaluate the V trajectory 
		public static void computeDorV( float phi, GenericTrajectory<float> trajX, GenericTrajectory<float> trajZ, int stance, ref Vector3 result ) {
			result.y = 0;
			float signReverse = (stance == SimGlobals.RIGHT_STANCE)?-1:1;
			if( trajX == null )
				result.x = 0;
			else
				result.x = trajX.evaluate_catmull_rom( phi ) * signReverse;
			if( trajZ == null )
				result.z = 0;
			else
				result.z = trajZ.evaluate_catmull_rom( phi );
		}
		
		/**
			This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
			used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
			or the index of the state that it transitions to otherwise.
		*/
		virtual public int advanceInTime(float dt){
			if( FSMStateIndex < 0 )
				setFSMStateTo( startingState );
		
			if( dt <= 0 )
				return -1;
		
			if (FSMStateIndex >= statesData.getStateCount()){
				//tprintf("Warning: no FSM state was selected in the controller!\n");
				return -1;
			}
			
			//advance the phase of the controller
			this.phi += dt/duration;
			//see if we have to transition to the next state in the FSM, and do it if so...
			if (currentState.needTransition(phi, swingFoot)){
				int newStateIndex;
				if(nextFSMState < 0){
					newStateIndex = currentState.getNextStateIndex();
					transitionToState(newStateIndex);
				}else{
					newStateIndex = nextFSMState;
					transitionToState(newStateIndex, startingStance);
				}
				return newStateIndex;
			}
		
			//if we didn't transition to a new state...
			return -1;
		}
		
		/**
			This method is used to return the value of bodyGroundContact
		*/
		public bool isBodyInContactWithTheGround(){
			return bodyTouchedTheGround;
		}
		public void setBodyTouchedTheGround(bool flag){
			bodyTouchedTheGround = flag;
		}

		public bool isLFootTouchedTheGround (){
			return lFootTouchedTheGround;
		}
		public bool isRFootTouchedTheGround(){
			return rFootTouchedTheGround;
		}
		public bool isFootInContactWithTheGround(){
			return lFootTouchedTheGround || rFootTouchedTheGround;
		}
		public bool isTwoFootTouchedGround(){
			return twoFootTouchedGround;
		}
		
		public bool isCharacterStandUp(){
			return isFootInContactWithTheGround () && fallDownAngle < 70;
		}
		
		public bool isCharacterFallDown(){
			return isFootInContactWithTheGround () && fallDownAngle > 70;
		}
		
		public float getFallDownAngle(){
			return fallDownAngle;
		}
		
		/**
			This method is used to return the value of the phase (phi) in the current FSM state.
		*/
		public float getPhase(){
			return phi;
		}
		public void setPhase( float phi ) { this.phi = phi; }
	
		public Vector3 getV() { return v; }
		public Vector3 getD() { return d; }
		
		public CWRigidBody getLHand(){
			return lHand;
		}
		
		public CWRigidBody getRHand(){
			return rHand;
		}
		
		public CWRigidBody getLFoot(){
			return lFoot;
		}
		
		public CWRigidBody getRFoot(){
			return rFoot;
		}
		
		public CWRigidBody getLLowerLeg(){
			return lLowerLeg;
		}
		
		public CWRigidBody getRLowerLeg(){
			return rLowerLeg;
		}

		public CWRigidBody getLUpperLeg(){
			return lUpperLeg;
		}

		public CWRigidBody getRUpperLeg(){
			return rUpperLeg;
		}
		
		public CWRigidBody getHead(){
			return head;
		}
		
		public CWRigidBody getTorso(){
			return torso;
		}
		
		/**
			This method returns the position of the CM of the stance foot, in world coordinates
		*/
		public Vector3 getStanceFootPos(){
			if (stanceFoot)
				return stanceFoot.getWorldBottomPosition();
			return Vector3.zero;
		}
	
		/**
			This method returns the position of the CM of the swing foot, in world coordinates
		*/
		public Vector3 getSwingFootPos(){
			if (swingFoot)
				return swingFoot.getWorldBottomPosition();
			return Vector3.zero;
		}
		/*
		override public void performPreTasks(float dt) {
			base.performPreTasks(dt);
		}
		*/
		// Returns true if it transitioned to a new state, false otherwise
		override public bool performPostTasks(float dt) {
			base.performPostTasks(dt);
			bool transition = (advanceInTime(dt) != -1);
			//updateDAndV();
			return transition;
		}
		
		/**
			This method makes it possible to evaluate the debug pose at any phase angle
		*/
		/*public void updateTrackingPose(ref ArrayList trackingPose, float phiToUse, int stanceToUse){
			if( FSMStateIndex < 0 )
				setFSMStateTo( startingState );
		
			if( phiToUse < 0 )
				phiToUse = phi;
			if( phiToUse > 1 )
				phiToUse = 1;
			if( stanceToUse < 0 )
				stanceToUse = stance;
			if( stanceToUse > 1 )
				stanceToUse = 1;
		
			trackingPose.Clear();
			trackingPose = this.character.getState();
			
			ReducedCharacterState debugRS = new ReducedCharacterState(trackingPose);
		
			//always start from a neutral desired pose, and build from there...
			for (int i=0;i<jointCount;i++){
				debugRS.setJointRelativeOrientation(0, i);
				debugRS.setJointRelativeAngVelocity(0, i);
			}
		
			//and this is the desired orientation for the root
			float qRootD = 0;
		
			for (int i=0;i<currentState.getTrajectoryCount();i++){
				//now we have the desired rotation angle and axis, so we need to see which joint this is intended for
				int jIndex = currentState.getTrajectory(i).getJointIndex(stanceToUse, isLeftReverseStance);
				
				Vector3 d0 = new Vector3();
				Vector3 v0 = new Vector3(); 
				computeD0( phiToUse, ref d0 );
				computeV0( phiToUse, ref v0 );
				
				float newOrientation = currentState.getTrajectory(i).evaluateTrajectory(this, character.getJoint(jIndex), stanceToUse, phiToUse, d - d0, v - v0);
				if (jIndex == -1){
					qRootD = newOrientation;
				}else{
					debugRS.setJointRelativeOrientation(newOrientation, jIndex);
				}
			}
		
			debugRS.setOrientation(qRootD);
			//debugRS.setOrientation(Quaternion.identity);
		
			//now, we'll make one more pass, and make sure that the orientations that are relative to the character frame are drawn that way
			/*for (int i=0;i<jointCount;i++){
				if ((controlParams[i] as ControlParams).relToFrame){
					Quaternion temp = Quaternion.identity;
					CWJoint j = character.getJoint(i);
					CWRigidBody parent = j.getParent();
					while (parent != root){
						j = parent.getParentJoint();
						parent = j.getParent();
						temp = debugRS.getJointRelativeOrientation(character.getJointIndex(j.jName)) * temp;
					}
			
					temp = qRootD * temp;
					temp = MathLib.getComplexConjugate(temp) * debugRS.getJointRelativeOrientation(i);
					debugRS.setJointRelativeOrientation(temp, i);
				}
			}*/
		//}
	}
}