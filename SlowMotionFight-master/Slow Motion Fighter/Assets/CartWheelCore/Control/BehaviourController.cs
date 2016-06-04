using UnityEngine;
using System.Collections;
using CartWheelCore.Math;

public enum ColliderLayer{
	defaultLayer = 0,
	ignoreRaycastLayer = 2,
	waterLayer = 4,
	sceneLayer = 8,
	roleLayer = 9,
	ignoreRoleLayer = 10,
	ignoreAllLayer = 11,
	guiLayer = 12,
	propsLayer = 13,
	touchPointLayer = 14,
	waterRipple = 15,
	sceneRefLayer = 16
}

/**
	This class implements an intermediate-level controller. Given a low level controller (of the type IKVMCController, for now),
	this class knows how to set a series of parameters in order to accomplish higher level behaviours.

	NOTE: We will assume a fixed character morphology (i.e. joints & links), and a fixed controller structure 
	(i.e. trajectories,	components).
*/
namespace CartWheelCore.Control{
	public class BehaviourController {
	
		protected Character bip;
		protected IKVMCController lowLCon;
	
		//we need to keep track of the position of the swing foot at the beginning of the step
		protected Vector3 swingFootStartPos;
		protected Vector3 standFootStartPos;
		
		protected Vector3 upperBodyPos;
		
		//these are attributes/properties of the motion
		protected float sagittalStepWidth;
		protected float coronalStepWidth;
		
		protected ArrayList stepRaycastHits;
		
		public bool doubleStanceMode = true;
		public float time = 0;
		public float belenceError = 0;
		public float velDSagittal = 0;
		public float velDCoronal = 0;
		public float comOffsetSagittal = 0;
		public float comOffsetCoronal = 0;
		public float stepHeight = 0;
		public float swingStepLength;
		public float standStepLength;
		public float coronalWidth = 0;
		public float rootExternalTorqueKp = 0;
		public float rootExternalTorqueKd = 0;
		public float rootExternalForceKp = 0;
		public float rootExternalForceKd = 0;
		public float fixedPositionKp = 0;
		public float fixedPositionKd = 0;
		public float velSagittalInAir = 0;
		public float bodySpringKp = 0;
		public float bodySpringKd = 0;
		public CWRigidBody impactBody = null;

		public float hipBend = 0;
		public float maxStepHeight = 0;
		public int stepLayerMask;
		
		public BehaviourController(Character b, IKVMCController llc){
			this.bip = b;
			this.lowLCon = llc;
		
			upperBodyPos = Vector3.zero;
			sagittalStepWidth = 0;
			coronalStepWidth = 0;
			stepHeight = 0;
			swingStepLength = 0;
			standStepLength = 0;
			
			maxStepHeight = llc.getLegLength() * 1.0f;
			stepLayerMask = ~((1 << (int)ColliderLayer.roleLayer) | (1 << (int)ColliderLayer.ignoreAllLayer) | (1 << (int)ColliderLayer.waterLayer) | (1 << (int)ColliderLayer.waterRipple) | (1 << (int)ColliderLayer.ignoreRaycastLayer) | (1 << (int)ColliderLayer.propsLayer) | (1 << (int)ColliderLayer.touchPointLayer));

			stepRaycastHits = new ArrayList();
		}
		
		public Vector3 getUpperBodyPos(){
			return upperBodyPos;
		}
		
		virtual public void setUpperBodyPose(int index, float rootLean, float torsoLean, float headLean){
			upperBodyPos[index] = rootLean;
			SimBiConState curState = lowLCon.getCurrentState();
			Trajectory tmpTraj = curState.getTrajectory("root");
			if (tmpTraj != null){
				tmpTraj.getTrajectoryComponent(index).offset = rootLean;
			}
			tmpTraj = curState.getTrajectory("pelvis_waist");
			if (tmpTraj != null){
				tmpTraj.getTrajectoryComponent(index).offset = torsoLean;
			}
			tmpTraj = curState.getTrajectory("torso_head");
			if (tmpTraj != null){
				tmpTraj.getTrajectoryComponent(index).offset = headLean;
			}
		}

		public void setHipBend(float stanceAngle, float swingAngle){
			SimBiConState curState = lowLCon.getCurrentState();
			
			Trajectory tmpTraj = curState.getTrajectory(1);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = swingAngle;
			
			tmpTraj = curState.getTrajectory(2);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = stanceAngle;
		}
		
		public void setKneeBend(float stanceAngle, float swingAngle){
			SimBiConState curState = lowLCon.getCurrentState();
			
			Trajectory tmpTraj = curState.getTrajectory(3);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = swingAngle;
			
			tmpTraj = curState.getTrajectory(4);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = stanceAngle;
		}

		public void setLegBend(float stanceAngle, float swingAngle){
			SimBiConState curState = lowLCon.getCurrentState();
			
			Trajectory tmpTraj = curState.getTrajectory(1);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = swingAngle;
			
			tmpTraj = curState.getTrajectory(2);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = stanceAngle;
			
			tmpTraj = curState.getTrajectory(3);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = -swingAngle;
			
			tmpTraj = curState.getTrajectory(4);
			if(tmpTraj.getTrajectoryComponentCount()>0)tmpTraj.getTrajectoryComponent(0).offset = -stanceAngle;
		}
		
		virtual public void adjustStepHeight(){
			Vector3 footPos = lowLCon.getSwingFoot().getWorldBottomPosition();
			//Vector3 swingFootV = lowLCon.getSwingFoot().getAbsoluteVelocityForLocalPoint(lowLCon.getSwingFoot().getLocalBottomPosition());
			Vector3 swingFootV = lowLCon.getCharacterFrame()*(lowLCon.getVelSagittal()*lowLCon.getCharacter ().getLocalFrontAxis () - lowLCon.getVelDCoronal()*lowLCon.getCharacter ().getLocalLeftAxis());
			swingFootV[lowLCon.getWorldUpAxisID()] = 0;
			//swingFootV.Normalize();
			Vector3 rayPos = footPos + 0.7f*swingFootV;
			float rayStartY = lowLCon.getComPosition () [lowLCon.getWorldUpAxisID ()];
			Vector3 rayStartPos = rayPos;
			rayStartPos[lowLCon.getWorldUpAxisID()] = rayStartY;
			float rayEndY = (footPos - (2 * maxStepHeight + lowLCon.additionHeight) * SimGlobals.upAxis) [lowLCon.getWorldUpAxisID ()];
			Vector3 rayEndPos = rayPos;
			rayEndPos[lowLCon.getWorldUpAxisID()] = rayEndY;
			
			RaycastHit raycastHit = new RaycastHit();
			if(Physics.Linecast(rayStartPos, rayEndPos, out raycastHit, stepLayerMask)){
				Debug.DrawLine(rayStartPos, rayEndPos, Color.blue);
				lowLCon.swingFootFrontPosition = raycastHit.point;
			}else{
				Vector3 pos = lowLCon.getSwingHip ().getChildJointWorldPosition () - lowLCon.getLegLength () * SimGlobals.upAxis;
				Debug.DrawLine (pos, pos - (2 * maxStepHeight + lowLCon.additionHeight) * SimGlobals.upAxis, Color.blue);
				lowLCon.swingFootFrontPosition = pos - lowLCon.additionHeight * SimGlobals.upAxis;
			}
			//Debug.Log("aaaaaaaaaaaaa  "+lowLCon.swingFootFrontPosition);
			
			rayPos = footPos;
			rayStartPos = rayPos;
			rayStartPos[lowLCon.getWorldUpAxisID()] = rayStartY;
			rayEndPos = rayPos;
			rayEndPos [lowLCon.getWorldUpAxisID ()] = (footPos - (2 * maxStepHeight + lowLCon.additionHeight) * SimGlobals.upAxis) [lowLCon.getWorldUpAxisID ()];
			if(Physics.Linecast(rayStartPos, rayEndPos, out raycastHit, stepLayerMask)){
				Debug.DrawLine(rayStartPos, rayEndPos, Color.green);
				lowLCon.swingFootBasePosition = raycastHit.point;
				lowLCon.swingFootBaseNormal = raycastHit.normal;
			}else{
				lowLCon.swingFootBasePosition = lowLCon.swingFootFrontPosition;
				lowLCon.swingFootBaseNormal = Vector3.zero;
			}
			//Debug.Log("bbbbbbbbbbbbb  "+lowLCon.swingFootBasePosition);
		//	Debug.Log("aaaaaaaaaaaaaaaaa  "+lowLCon.swingFootBaseNormal);
			
			footPos = lowLCon.getStanceFoot().getWorldBottomPosition();
			rayPos = footPos;
			rayEndY = (footPos - (2 * maxStepHeight + lowLCon.additionHeight) * SimGlobals.upAxis) [lowLCon.getWorldUpAxisID ()];
			rayStartPos = rayPos;
			rayStartPos[lowLCon.getWorldUpAxisID()] = rayStartY;
			rayEndPos = rayPos;
			rayEndPos[lowLCon.getWorldUpAxisID()] = rayEndY;
			if(Physics.Linecast(rayStartPos, rayEndPos, out raycastHit, stepLayerMask)){
				Debug.DrawLine(rayStartPos, rayEndPos, Color.green);
				lowLCon.standFootBasePosition = raycastHit.point;
				lowLCon.standFootBaseNormal = raycastHit.normal;
			}else{
				Vector3 pos = lowLCon.getStandHip ().getChildJointWorldPosition () - lowLCon.getLegLength () * SimGlobals.upAxis;
				Debug.DrawLine (pos, pos - (2 * maxStepHeight + lowLCon.additionHeight) * SimGlobals.upAxis, Color.green);
				lowLCon.standFootBasePosition = pos - lowLCon.additionHeight * SimGlobals.upAxis;
				lowLCon.standFootBaseNormal = Vector3.zero;
			}
		//	Debug.Log("bbbbbbbbbbbbbbbbb  "+lowLCon.standFootBaseNormal);
			
			lowLCon.swingToStandBaseVector = Quaternion.Inverse(lowLCon.getCharacterFrame())*(lowLCon.standFootBasePosition - lowLCon.swingFootBasePosition);
			//Debug.Log("bbbbbbbbbbbbb  "+lowLCon.swingToStandBaseVector);
			float h = lowLCon.getDoubleStanceCOMError ().magnitude * 1.6f;
			h = Mathf.Clamp (h, lowLCon.getStepHeight(), maxStepHeight);
			lowLCon.swingLegPanicHeight = h - lowLCon.getStepHeight();
			//Debug.Log("bbbbbbbbbbbbbbbbb "+lowLCon.swingLegPanicHeight);
		}
		
		virtual public void requestSagittalStepWidth(float sagSW){
			sagittalStepWidth = sagSW;
		}
		
		public void requestCoronalStepWidth(float corSW) {
			coronalStepWidth = corSW;
		}
		
		public float getSagittalStepWidth() {return sagittalStepWidth;}
		public float getCoronalStepWidth() { return coronalStepWidth; }
		
		/**
			modify the sagittal location of the step so that the desired step width results.
		*/
		public float adjustSagittalStepLocation(float IPPrediction){
			if (sagittalStepWidth == 0)
				return IPPrediction;
		
			float stepLength = sagittalStepWidth;
			stepLength = (lowLCon.getStance() == SimGlobals.LEFT_STANCE)?(stepLength):(-stepLength);
			IPPrediction += lowLCon.getLocalLeftAxisSign()*stepLength;
			
			return IPPrediction;
		}
		
		/**
			modify the coronal location of the step so that the desired step width results.
		*/
		public float adjustCoronalStepLocation(float IPPrediction){
			//nothing to do if it's the default value...
			if (coronalStepWidth == 0)
				return IPPrediction;
		
			float stepWidth = coronalStepWidth;
			stepWidth = (lowLCon.getStance() == SimGlobals.LEFT_STANCE)?(stepWidth):(-stepWidth);
			//if(lowLCon.getDoubleStanceMode()){
			IPPrediction+=lowLCon.getLocalLeftAxisSign()*stepWidth;
		/*	}else{
				//now for the step in the coronal direction - figure out if the character is still doing well - panic = 0 is good, panic = 1 is bad...
				float panicLevel = 1;
				if(lowLCon.getStance() == SimGlobals.LEFT_STANCE){
					panicLevel = getValueInFuzzyRange(lowLCon.getD()[lowLCon.getLocalLeftAxisID()], 1.15f * stepWidth, 0.5f * stepWidth, 0.25f * stepWidth, -0.25f * stepWidth);
					panicLevel += getValueInFuzzyRange(lowLCon.getV()[lowLCon.getLocalLeftAxisID()], 2*stepWidth, stepWidth, -stepWidth, -stepWidth*1.5f);
				} else {
					panicLevel = getValueInFuzzyRange(lowLCon.getD()[lowLCon.getLocalLeftAxisID()], -0.25f * stepWidth, 0.25f * stepWidth, 0.5f * stepWidth, 1.15f * stepWidth);
					panicLevel += getValueInFuzzyRange(lowLCon.getV()[lowLCon.getLocalLeftAxisID()], -stepWidth*1.5f, -stepWidth, stepWidth, 2*stepWidth);
				}
				panicLevel = Mathf.Clamp(panicLevel, 0.0f, 1.0f);
				GenericTrajectory<float> offsetMultiplier = new GenericTrajectory<float>();
				offsetMultiplier.addKnot(0.05f, 0); offsetMultiplier.addKnot(0.075f, 1/2.0f);
				float offset = stepWidth * offsetMultiplier.evaluate_linear(Mathf.Abs(stepWidth));
			//	if (IPPrediction * stepWidth < 0) offset = 0;
				//if it's doing well, use the desired step width...
				IPPrediction = panicLevel * (IPPrediction + offset) + (1-panicLevel) * stepWidth;
				//lowLCon.setComOffsetCoronal((1-panicLevel) * stepWidth);
			}*/
		
			return IPPrediction;
		}
		
		/**
			determine the estimate desired location of the swing foot, given the etimated position of the COM, and the phase
		*/
		virtual public Vector3 computeSwingFootLocationEstimate(Vector3 comPos, float phase){
			Vector3 step = lowLCon.computeIPStepLocation();
			
			//applying the IP prediction would make the character stop, so take a smaller step if you want it to walk faster, or larger
			//if you want it to go backwards
			//step.x += lowLCon.velDSagittal / 20;
			//and adjust the stepping in the coronal plane in order to account for desired step width...
			step[lowLCon.getLocalFrontAxisID()] = adjustSagittalStepLocation(step[lowLCon.getLocalFrontAxisID()]);
			step[lowLCon.getLocalLeftAxisID()] = adjustCoronalStepLocation(step[lowLCon.getLocalLeftAxisID()]);

			Vector3 result = Vector3.zero;
			Vector3 initialStep = swingFootStartPos - comPos;
			initialStep = Quaternion.Inverse(lowLCon.getCharacterFrame())*initialStep;
			//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
			//now interpolate between this position and initial foot position - but provide two estimates in order to provide some gradient information
			float t = (1-phase);
			t = t * t;
			t = Mathf.Clamp(t, 0.0f, 1.0f);
		
			result+=(step * (1-t));
			result+=(initialStep * t);
		
			result[lowLCon.getLocalUpAxisID()] = 0;

			return result;
		}
		
		/**
			determines the desired swing foot location
		*/
		virtual public void setDesiredSwingFootLocation(){
			Vector3 step = computeSwingFootLocationEstimate(lowLCon.getComPosition(), lowLCon.getPhase());
			//Debug.Log("aaaaaaaaaa "+step);
			lowLCon.swingFootCoronalTrajectory.setKnotValue(0, step[lowLCon.getLocalLeftAxisID()]);
			lowLCon.swingFootSagittalTrajectory.setKnotValue(0, step[lowLCon.getLocalFrontAxisID()]);
			
			float dt = SimGlobals.dt;
			step = computeSwingFootLocationEstimate(lowLCon.getComPosition() + lowLCon.getComVelocity() * dt, lowLCon.getPhase()+dt);
			lowLCon.swingFootCoronalTrajectory.setKnotValue(1, step[lowLCon.getLocalLeftAxisID()]);
			lowLCon.swingFootSagittalTrajectory.setKnotValue(1, step[lowLCon.getLocalFrontAxisID()]);
			
			
			//to give some gradient information, here's what the position will be a short time later...
			lowLCon.swingFootSagittalTrajectory.setKnotPosition(0, lowLCon.getPhase());
			lowLCon.swingFootSagittalTrajectory.setKnotPosition(1, lowLCon.getPhase()+dt);
		
			lowLCon.swingFootCoronalTrajectory.setKnotPosition(0, lowLCon.getPhase());
			lowLCon.swingFootCoronalTrajectory.setKnotPosition(1, lowLCon.getPhase()+dt);
		}
		
		/**
			sets a bunch of parameters to some default initial value
		*/
		virtual public void initializeDefaultParameters(){
			lowLCon.updateDAndV();
		}
		
		/**
			this method gets called at every simulation time step
		*/
		virtual public void simStepPlan(float dt){
			lowLCon.updateSwingAndStanceReferences();
		
			//compute desired swing foot location...
			if(lowLCon.getIsComputeIPStep()){
				setDesiredSwingFootLocation();
			}
		
			//adjust for panic mode or unplanned terrain...
			adjustStepHeight();
			//adjustGait ();
			//and see if we're really in trouble...
			//if (shouldAbort()) onAbort();
		}
		
		/**
			this method gets called every time the controller transitions to a new state
		*/
		virtual public void conTransitionPlan(){
			lowLCon.updateSwingAndStanceReferences();
			lowLCon.updateDAndV();

			standFootStartPos = lowLCon.getStanceFoot().getWorldBottomPosition();
			swingFootStartPos = lowLCon.getSwingFoot().getWorldBottomPosition();
			
			//now prepare the step information for the following step:
			lowLCon.swingFootHeightTrajectory.clear();
			//lowLCon.standFootHeightTrajectory.clear();
			lowLCon.swingFootCoronalTrajectory.clear();
			lowLCon.standFootCoronalTrajectory.clear();
			lowLCon.swingFootSagittalTrajectory.clear();
			lowLCon.standFootSagittalTrajectory.clear();
			
			stepRaycastHits.Clear();
			
			if(lowLCon.getIsComputeIPStep()){
				lowLCon.swingFootHeightTrajectory.addKnot(0, 0);
				lowLCon.swingFootHeightTrajectory.addKnot (0.5f, lowLCon.getStepHeight ());
				lowLCon.swingFootHeightTrajectory.addKnot(1, 0);
				
				lowLCon.swingFootSagittalTrajectory.addKnot(0, 0);
				lowLCon.swingFootSagittalTrajectory.addKnot(1, 0);
				
				lowLCon.swingFootCoronalTrajectory.addKnot(0, 0);
				lowLCon.swingFootCoronalTrajectory.addKnot(1, 0);
			}else{
				if (lowLCon.getIsAddFootJoint ()) {
					lowLCon.getSwingFoot().removeFixedPositionJoint();
					lowLCon.getStanceFoot().removeFixedPositionJoint();
				}
				
				lowLCon.swingFootHeightTrajectory.addKnot(0, 0);
				lowLCon.swingFootHeightTrajectory.addKnot (0.5f, lowLCon.getStepHeight ());
				lowLCon.swingFootHeightTrajectory.addKnot(1, 0);
				
				float w = lowLCon.getLocalLeftAxisSign()*sagittalStepWidth;
				w = (lowLCon.getStance() == SimGlobals.LEFT_STANCE)?w:-w;
				
				lowLCon.swingFootSagittalTrajectory.addKnot(0, 0 + w);
				lowLCon.swingFootSagittalTrajectory.addKnot(1, lowLCon.getSwingStepLength() + w);

				lowLCon.standFootSagittalTrajectory.addKnot(0, 0 - w);
				lowLCon.standFootSagittalTrajectory.addKnot(1, lowLCon.getStandStepLength() - w);
				
				w = lowLCon.getLocalLeftAxisSign()*coronalStepWidth;
				w = (lowLCon.getStance() == SimGlobals.LEFT_STANCE)?w:-w;
				
				lowLCon.swingFootCoronalTrajectory.addKnot(0, 0 + w);
				lowLCon.swingFootCoronalTrajectory.addKnot(1, 0 + w);
				
				//lowLCon.standFootCoronalTrajectory.addKnot(0, 0);
				//lowLCon.standFootCoronalTrajectory.addKnot(1, 0);
				
				//float contactPosDis = (lowLCon.getStanceFoot().getWorldBottomPosition() - lowLCon.swingFootBasePosition).magnitude;
				//if(Vector2.Dot(lowLCon.standFootBaseNormal, -1*Physics.gravity.normalized) > 0.7f/* && contactPosDis < 0.5f*/){
				//	lowLCon.getStanceFoot().addFixedPositionJoint(lowLCon.swingFootBasePosition, 800);
				//}
				
				//lowLCon.standFootHeightTrajectory.addKnot(0, lowLCon.getLegLength());
				//lowLCon.standFootHeightTrajectory.addKnot(1, lowLCon.getLegLength());
			}
		}
		
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
		
	}
}