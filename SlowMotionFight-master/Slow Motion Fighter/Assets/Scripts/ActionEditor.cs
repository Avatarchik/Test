using UnityEngine;
using System.Collections;
using CartWheelCore.Control;

	public enum StanceType
	{
		LEFT_STANCE = 0,
		RIGHT_STANCE = 1,
		STATE_REVERSE_STANCE = 100,
		STATE_KEEP_STANCE = 101
	}

	public enum TransitionState
	{
		TRANSITIONONTIMEUP = 102,
		TRANSITIONONFOOTDOWN = 103,
		TRANSITIONONPICKUP = 104
	}
	
	[System.Serializable]
	public class TrajectoryDataED{
		public float t;
		public float angle;
	}
	
	[System.Serializable]
	public class BalanceED{
		public Vector3 feedbackProjectionAxis = Vector3.right;
		public float cd = 0;
		public float cv = 0;
		public bool reversedCD = false;
		public bool reversedCV = false;
		public float vMin = -100;
		public float vMax = 100;
		public float dMin = -100;
		public float dMax = 100;
	}
	
	[System.Serializable]
	public class ComponentED
	{
		public bool reverseAngleOnLeftStance = false;
		public bool reverseAngleOnRightStance = false;
		public float offset;
		public BalanceED balance;
		public TrajectoryDataED[] baseTrajectory;
	}
	
	[System.Serializable]
	public class TrajectoryED
	{
		public bool characterFrameRelative = false;
		public bool noReverseStance = false;
		public ComponentED[] component;
	}
	
	[System.Serializable]
	public class ConStateED
	{
		public bool doubleStanceMode = true;
		public bool doIPBelence = false;
		public TransitionState transitionOnFootState = TransitionState.TRANSITIONONTIMEUP;
		public StanceType stateStance;
		public float stepTime = 0.5f;
		public float belenceError = 0.2f;
		public float stepHeight = 0.4f;
		public float swingStepLength = 0;
		public float standStepLength = 0;
		public float sagittalStepWidth = 0.5f;
		public float coronalStepWidth = 0.5f;
		public float comOffsetSagittal = 0;
		public float comOffsetCoronal = 0;
		public float velSagittal = 0;
		public float velCoronal = 0;
		public float rootExternalTorqueKp = 0;
		public float rootExternalTorqueKd = 0;
		public float rootExternalForceKp = 0;
		public float rootExternalForceKd = 0;
		public float fixedPositionKp = 0;
		public float fixedPositionKd = 0;
		public float velSagittalInAir = 0;
		public float bodySpringKp = 0;
		public float bodySpringKd = 0;
		public bool continueTrajectory = false;
		
		public TrajectoryED root;
		public TrajectoryED SWING_Hip;
		public TrajectoryED STANCE_Hip;
		public TrajectoryED SWING_Knee;
		public TrajectoryED STANCE_Knee;
		public TrajectoryED SWING_Shoulder;
		public TrajectoryED STANCE_Shoulder;
		public TrajectoryED SWING_Elbow;
		public TrajectoryED STANCE_Elbow;
		public TrajectoryED SWING_Wrist;
		public TrajectoryED STANCE_Wrist;
		public TrajectoryED pelvis_waist;
		public TrajectoryED waist_torso;
		public TrajectoryED torso_head;
	}
	
	public class ActionEditor : MonoBehaviour {
		public ConStateED conState;
		
		private IKVMCController controller = null;
	
		public void setController(IKVMCController c){
			controller = c;
		}
		
		void Start () {
		}
		
		void FixedUpdate () {
			if(controller==null) return;
		
			controller.getCurrentState().doubleStanceMode = conState.doubleStanceMode;
			controller.getCurrentState().doIPBelence = conState.doIPBelence;
			controller.getCurrentState().comOffsetSagittal = conState.comOffsetSagittal;
			controller.getCurrentState().comOffsetCoronal = conState.comOffsetCoronal;
			controller.getCurrentState().velSagittal = conState.velSagittal;
			controller.getCurrentState().velCoronal = conState.velCoronal;
			controller.getCurrentState().belenceError = conState.belenceError;
			controller.getCurrentState().stepHeight = conState.stepHeight;
			controller.getCurrentState().swingStepLength = conState.swingStepLength;
			controller.getCurrentState().standStepLength = conState.standStepLength;
			controller.getCurrentState().sagittalStepWidth = conState.sagittalStepWidth;
			controller.getCurrentState().coronalStepWidth = conState.coronalStepWidth;
			controller.getCurrentState().rootExternalTorqueKp = conState.rootExternalTorqueKp;
			controller.getCurrentState().rootExternalTorqueKd = conState.rootExternalTorqueKd;
			controller.getCurrentState().rootExternalForceKp = conState.rootExternalForceKp;
			controller.getCurrentState().rootExternalForceKd = conState.rootExternalForceKd;
			controller.getCurrentState().fixedPositionKp = conState.fixedPositionKp;
			controller.getCurrentState().fixedPositionKd = conState.fixedPositionKd;
			controller.getCurrentState().velSagittalInAir = conState.velSagittalInAir;
			controller.getCurrentState().bodySpringKp = conState.bodySpringKp;
			controller.getCurrentState().bodySpringKd = conState.bodySpringKd;
			controller.getCurrentState().continueTrajectory = conState.continueTrajectory;
			controller.getCurrentState().setDuration(conState.stepTime);
			controller.getCurrentState().setTransitionOnFootState((int)conState.transitionOnFootState);
			controller.getCurrentState().setStance((int)conState.stateStance);
			
			//root
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("root"), conState.root);
			
			//SWING_Hip
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("SWING_Hip"), conState.SWING_Hip);
			
			//STANCE_Hip
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("STANCE_Hip"), conState.STANCE_Hip);
			
			//SWING_Knee
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("SWING_Knee"), conState.SWING_Knee);
			
			//STANCE_Knee
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("STANCE_Knee"), conState.STANCE_Knee);
			
			//SWING_Shoulder
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("SWING_Shoulder"), conState.SWING_Shoulder);
			
			//STANCE_Shoulder
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("STANCE_Shoulder"), conState.STANCE_Shoulder);
			
			//SWING_Elbow
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("SWING_Elbow"), conState.SWING_Elbow);
			
			//STANCE_Elbow
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("STANCE_Elbow"), conState.STANCE_Elbow);

			//SWING_Wrist
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("SWING_Wrist"), conState.SWING_Wrist);

			//STANCE_Wrist
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("STANCE_Wrist"), conState.STANCE_Wrist);
			
			//pelvis_torso
		setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("pelvis_waist"), conState.pelvis_waist);
		setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("waist_torso"), conState.waist_torso);
			
			//torso_head
			setEnginTrajectoryFromEditor(controller.getCurrentState().getTrajectory("torso_head"), conState.torso_head);
		}
		
		private void setEnginTrajectoryFromEditor(Trajectory tra, TrajectoryED traED){
			if(tra!=null){
				tra.relToCharFrame = traED.characterFrameRelative;
				tra.noReverseStance = traED.noReverseStance;
				for(int j = 0; j<tra.getTrajectoryComponentCount(); j++){
					TrajectoryComponent com = tra.getTrajectoryComponent(j);
					com.offset = traED.component[j].offset;
					com.reverseAngleOnLeftStance = traED.component[j].reverseAngleOnLeftStance;
					com.reverseAngleOnRightStance = traED.component[j].reverseAngleOnRightStance;
					if(Mathf.Abs(traED.component[j].balance.cd)>0 || Mathf.Abs(traED.component[j].balance.cv)>0){
						com.bFeedback = new LinearBalanceFeedback();
						com.bFeedback.feedbackProjectionAxis = traED.component[j].balance.feedbackProjectionAxis;
						com.bFeedback.cd = traED.component[j].balance.cd;
						com.bFeedback.cv = traED.component[j].balance.cv;
						com.bFeedback.reversedCD = traED.component[j].balance.reversedCD;
						com.bFeedback.reversedCV = traED.component[j].balance.reversedCV;
						com.bFeedback.dMin = traED.component[j].balance.dMin;
						com.bFeedback.dMax = traED.component[j].balance.dMax;
						com.bFeedback.vMin = traED.component[j].balance.vMin;
						com.bFeedback.vMax = traED.component[j].balance.vMax;
					}else{
						com.bFeedback = null;
					}
					com.baseTraj.clear();
					for(int i=0;i<traED.component[j].baseTrajectory.Length;i++){
						com.baseTraj.addKnot(traED.component[j].baseTrajectory[i].t, traED.component[j].baseTrajectory[i].angle);
					}
				}
			}
		}
		
		private void setEditorStateFromEngine(TrajectoryED traED, Trajectory tra){
			if(tra!=null){
				traED.characterFrameRelative = tra.relToCharFrame;
				traED.noReverseStance = tra.noReverseStance;
				traED.component = new ComponentED[tra.getTrajectoryComponentCount()];
				for(int j = 0; j<tra.getTrajectoryComponentCount(); j++){
					TrajectoryComponent com = tra.getTrajectoryComponent(j);
					traED.component[j] = new ComponentED();
					traED.component[j].offset = com.offset;
					traED.component[j].reverseAngleOnLeftStance = com.reverseAngleOnLeftStance;
					traED.component[j].reverseAngleOnRightStance = com.reverseAngleOnRightStance;
					if(com.bFeedback!=null){
						traED.component[j].balance = new BalanceED();
						traED.component[j].balance.feedbackProjectionAxis = com.bFeedback.feedbackProjectionAxis;
						traED.component[j].balance.cd = com.bFeedback.cd;
						traED.component[j].balance.cv = com.bFeedback.cv;
						traED.component[j].balance.reversedCD = com.bFeedback.reversedCD;
						traED.component[j].balance.reversedCV = com.bFeedback.reversedCV;
						traED.component[j].balance.dMin = com.bFeedback.dMin;
						traED.component[j].balance.dMax = com.bFeedback.dMax;
						traED.component[j].balance.vMin = com.bFeedback.vMin;
						traED.component[j].balance.vMax = com.bFeedback.vMax;
					}else{
						traED.component[j].balance = new BalanceED();
						traED.component[j].balance.feedbackProjectionAxis = Vector3.right;
						traED.component[j].balance.cd = 0;
						traED.component[j].balance.cv = 0;
						traED.component[j].balance.reversedCD = false;
						traED.component[j].balance.reversedCV = false;
						traED.component[j].balance.dMin = -100;
						traED.component[j].balance.dMax = 100;
						traED.component[j].balance.vMin = -100;
						traED.component[j].balance.vMax = 100;
					}
					traED.component[j].baseTrajectory = new TrajectoryDataED[com.baseTraj.getKnotCount()];
					for(int i=0;i<com.baseTraj.getKnotCount();i++){
						traED.component[j].baseTrajectory[i] = new TrajectoryDataED();
						traED.component[j].baseTrajectory[i].t = com.baseTraj.getKnotPosition(i);
						traED.component[j].baseTrajectory[i].angle = com.baseTraj.getKnotValue(i);
					}
				}
			}else{
				traED.characterFrameRelative = false;
				traED.noReverseStance = false;
				traED.component = new ComponentED[1];
				traED.component[0] = new ComponentED();
				traED.component[0].offset = 0;
				traED.component[0].reverseAngleOnLeftStance = false;
				traED.component[0].reverseAngleOnRightStance = false;
				
				traED.component[0].balance = new BalanceED();
				traED.component[0].balance.feedbackProjectionAxis = Vector3.right;
				traED.component[0].balance.cd = 0;
				traED.component[0].balance.cv = 0;
				traED.component[0].balance.reversedCD = false;
				traED.component[0].balance.reversedCV = false;
				traED.component[0].balance.dMin = -100;
				traED.component[0].balance.dMax = 100;
				traED.component[0].balance.vMin = -100;
				traED.component[0].balance.vMax = 100;
				traED.component[0].baseTrajectory = new TrajectoryDataED[0];
			}
		}
		
		public void updateState(){
			if(controller == null) return;
			conState.doubleStanceMode = controller.getCurrentState().doubleStanceMode;
			conState.doIPBelence = controller.getCurrentState().doIPBelence;
			conState.comOffsetSagittal = controller.getCurrentState().comOffsetSagittal;
			conState.comOffsetCoronal = controller.getCurrentState().comOffsetCoronal;
			conState.velSagittal = controller.getCurrentState().velSagittal;
			conState.velCoronal = controller.getCurrentState().velCoronal;
			conState.belenceError = controller.getCurrentState().belenceError;
			conState.stepHeight = controller.getCurrentState().stepHeight;
			conState.swingStepLength = controller.getCurrentState().swingStepLength;
			conState.standStepLength = controller.getCurrentState().standStepLength;
			conState.sagittalStepWidth = controller.getCurrentState().sagittalStepWidth;
			conState.coronalStepWidth = controller.getCurrentState().coronalStepWidth;
			conState.rootExternalTorqueKp = controller.getCurrentState().rootExternalTorqueKp;
			conState.rootExternalTorqueKd = controller.getCurrentState().rootExternalTorqueKd;
			conState.rootExternalForceKp = controller.getCurrentState().rootExternalForceKp;
			conState.rootExternalForceKd = controller.getCurrentState().rootExternalForceKd;
			conState.fixedPositionKp = controller.getCurrentState().fixedPositionKp;
			conState.fixedPositionKd = controller.getCurrentState().fixedPositionKd;
			conState.velSagittalInAir = controller.getCurrentState().velSagittalInAir;
			conState.bodySpringKp = controller.getCurrentState().bodySpringKp;
			conState.bodySpringKd = controller.getCurrentState().bodySpringKd;
			conState.continueTrajectory = controller.getCurrentState().continueTrajectory;
			conState.stepTime = controller.getCurrentState().getDuration();
			conState.transitionOnFootState = (TransitionState)controller.getCurrentState().getTransitionOnFootState();
			conState.stateStance = (StanceType)controller.getCurrentState().getStance();
			
			//root
			setEditorStateFromEngine(conState.root, controller.getCurrentState().getTrajectory("root"));
				
			//SWING_Hip
			setEditorStateFromEngine(conState.SWING_Hip, controller.getCurrentState().getTrajectory("SWING_Hip"));
				
			//STANCE_Hip
			setEditorStateFromEngine(conState.STANCE_Hip, controller.getCurrentState().getTrajectory("STANCE_Hip"));
			
			//SWING_Knee
			setEditorStateFromEngine(conState.SWING_Knee, controller.getCurrentState().getTrajectory("SWING_Knee"));
				
			//STANCE_Knee
			setEditorStateFromEngine(conState.STANCE_Knee, controller.getCurrentState().getTrajectory("STANCE_Knee"));
				
			//SWING_Shoulder
			setEditorStateFromEngine(conState.SWING_Shoulder, controller.getCurrentState().getTrajectory("SWING_Shoulder"));
				
			//STANCE_Shoulder
			setEditorStateFromEngine(conState.STANCE_Shoulder, controller.getCurrentState().getTrajectory("STANCE_Shoulder"));
				
			//SWING_Elbow
			setEditorStateFromEngine(conState.SWING_Elbow, controller.getCurrentState().getTrajectory("SWING_Elbow"));
				
			//STANCE_Elbow
			setEditorStateFromEngine(conState.STANCE_Elbow, controller.getCurrentState().getTrajectory("STANCE_Elbow"));

			//SWING_Wrist
			setEditorStateFromEngine(conState.SWING_Wrist, controller.getCurrentState().getTrajectory("SWING_Wrist"));

			//STANCE_Wrist
			setEditorStateFromEngine(conState.STANCE_Wrist, controller.getCurrentState().getTrajectory("STANCE_Wrist"));
				
			//pelvis_torso
		setEditorStateFromEngine(conState.pelvis_waist, controller.getCurrentState().getTrajectory("pelvis_waist"));
		setEditorStateFromEngine(conState.waist_torso, controller.getCurrentState().getTrajectory("waist_torso"));
				
			//torso_head
			setEditorStateFromEngine(conState.torso_head, controller.getCurrentState().getTrajectory("torso_head"));
		}
	}