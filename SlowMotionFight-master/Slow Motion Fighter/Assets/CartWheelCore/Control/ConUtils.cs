using UnityEngine;
using System.Collections;

namespace CartWheelCore.Control{
	
	public struct ConKeyWord
	{
	    public string keyWord;
		public int retVal;
		
		public ConKeyWord(string kw, int rv){
			this.keyWord = kw;
			this.retVal = rv;
		}
	}
	
	public class ConUtils  {
		
		public const int CON_NOT_IMPORTANT		=		1;
		public const int CON_COMMENT			=			2;
		public const int CON_STATE_START		=			5;
		public const int CON_STATE_END			=		6;
		public const int CON_NEXT_STATE			=		7;
		public const int CON_STATE_DESCRIPTION	=		8;
		public const int CON_TRANSITION_ON		=		9;
		public const int CON_STATE_STANCE		=		10;
		public const int CON_STARTING_STANCE	=			11;
		public const int CON_START_AT_STATE		=		12;
		public const int CON_CHARACTER_STATE	=			13;
		public const int CON_STATE_TIME			=		14;
		public const int CON_TRAJECTORY_START	=		15;
		public const int CON_TRAJECTORY_END		=		16;
		public const int CON_REVERSE_ANGLE_ON_STANCE	=	17;
		public const int CON_ROTATION_AXIS		=		18;
		public const int CON_BASE_TRAJECTORY_START	=	19;
		public const int CON_BASE_TRAJECTORY_END	=		20;
		public const int CON_FEEDBACK_START			=	21;
		public const int CON_FEEDBACK_END			=	22;
		public const int CON_CD					=		23;
		public const int CON_CV					=		24;
		public const int CON_FEEDBACK_PROJECTION_AXIS  =  25;
		public const int LOAD_RB_FILE			=		26;
		public const int LOAD_CON_FILE			=		27;
		public const int CON_TRAJ_COMPONENT		=		28;
		public const int CON_TRAJ_COMPONENT_END		=	29;
		
		public const int CON_SYNCHRONIZE_CONTROLLERS	=	30;
		public const int CON_LOAD_COMPOSITE_CONTROLLER =	31;
		
		public const int CON_STRENGTH_TRAJECTORY_START	= 32;
		public const int CON_STRENGTH_TRAJECTORY_END	=	33;
		public const int CON_CHAR_FRAME_RELATIVE     =    34;
		
		public const int CON_EXTERNALFORCE     =    35;
		public const int CON_EXTERNALFORCE_END     =    36;
		
		public const int CON_D_MIN			=			37;
		public const int CON_D_MAX			=			38;
		public const int CON_V_MIN			=			39;
		public const int CON_V_MAX			=			40;
		
		public const int CON_D_TRAJX_START		=		41;
		public const int CON_D_TRAJX_END		=			42;
		public const int CON_D_TRAJZ_START		=		43;
		public const int CON_D_TRAJZ_END		=			44;
		public const int CON_V_TRAJX_START		=		45;
		public const int CON_V_TRAJX_END		=			46;
		public const int CON_V_TRAJZ_START		=		47;
		public const int CON_V_TRAJZ_END		=			48;
		
		public const int CON_CD_NEG			=			50;
		public const int CON_CD_POS			=			51;
		public const int CON_CV_NEG_POS		=			52;
		public const int CON_CV_NEG_NEG		=			53;
		public const int CON_CV_POS_NEG		=			54;
		public const int CON_CV_POS_POS		=			55;
		public const int CON_MAX_FEEDBACK		=		56;
		public const int CON_MIN_FEEDBACK		=		57;
		
		public const int CON_D_SCALE_TRAJECTORY_START =	58;
		public const int CON_D_SCALE_TRAJECTORY_END	=	59;
		
		public const int CON_V_SCALE_TRAJECTORY_START	= 60;
		public const int CON_V_SCALE_TRAJECTORY_END =	61;
		public const int CON_OFFSET = 62;
		public const int CON_NOT_REVERSE_STANCE = 63;
		public const int CON_DOUBLESTANCEMODE = 64;
		public const int CON_DOIPBELENCE = 65;
		public const int CON_VELSAGITTAL = 66;
		public const int CON_VELCORONAL = 67;

		public const int CON_STEPHEIGHT = 69;
		public const int CON_SAGITTALSTEPWIDTH = 70;
		public const int CON_CORONALSTEPWIDTH = 71;
		public const int CON_REVERSEDCD = 72;
		public const int CON_REVERSEDCV = 73;
		public const int CON_ROOTEXTERNALTORQUEKP = 74;
		public const int CON_ROOTEXTERNALTORQUEKD = 75;
		public const int CON_ROOTEXTERNALFORCEKP = 76;
		public const int CON_ROOTEXTERNALFORCEKD = 77;
		public const int CON_VELSAGITTAL_INAIR = 78;
		public const int CON_FIXEDPOSITIONKP = 79;
		public const int CON_FIXEDPOSITIONKD = 80;
		public const int CON_BODYSPRINGKP = 81;
		public const int CON_BODYSPRINGKD = 82;
		public const int CON_COMOFFSETSAGITTAL = 83;
		public const int CON_COMOFFSETCORONAL = 84;
		
		public const int CON_EXTERNALFORCE_X     =    85;
		public const int CON_EXTERNALFORCE_Y     =    86;
		public const int CON_EXTERNALFORCE_Z     =    87;
		public const int CON_EXTERNALTORQUE_X     =    88;
		public const int CON_EXTERNALTORQUE_Y     =    89;
		public const int CON_EXTERNALTORQUE_Z     =    90;
		public const int CON_EXTERNALFORCE_X_END     =    91;
		public const int CON_EXTERNALFORCE_Y_END     =    92;
		public const int CON_EXTERNALFORCE_Z_END     =    93;
		public const int CON_EXTERNALTORQUE_X_END     =    94;
		public const int CON_EXTERNALTORQUE_Y_END     =    95;
		public const int CON_EXTERNALTORQUE_Z_END     =    96;
		
		public const int CON_HITFEEDBACK     =    97;
		public const int CON_HITFEEDBACK_END     =    98;
		public const int CON_HITFORCE     =    99;
		public const int CON_HITMINTIME     =    100;
		public const int CON_HITBODYSPRINGKP = 101;
		public const int CON_HITBODYSPRINGKD = 102;
		public const int CON_HITROOTEXTERNALFORCEKP = 103;
		public const int CON_HITROOTEXTERNALFORCEKD = 104;
		public const int CON_HITROOTEXTERNALTORQUEKP = 105;
		public const int CON_HITROOTEXTERNALTORQUEKD = 106;
		public const int CON_HITPOWER = 107;
		
		public const int CON_SWINGSTEPLENGTH = 108;
		public const int CON_STANDSTEPLENGTH = 109;
		
		public const int CON_CONTINUETRAJECTORY = 110;
		public const int CON_BELENCEERROR = 111;
		
		public static ConKeyWord[] keywords = new ConKeyWord[] {
										new ConKeyWord("#", CON_COMMENT),
										new ConKeyWord("ConState", CON_STATE_START),
										new ConKeyWord("/ConState", CON_STATE_END),
										new ConKeyWord("nextState", CON_NEXT_STATE),
										new ConKeyWord("description", CON_STATE_DESCRIPTION),
										new ConKeyWord("transitionOn", CON_TRANSITION_ON),
										new ConKeyWord("stateStance", CON_STATE_STANCE),
										new ConKeyWord("startingStance", CON_STARTING_STANCE),
										new ConKeyWord("startAtState", CON_START_AT_STATE),
										new ConKeyWord("loadCharacterState", CON_CHARACTER_STATE),
										new ConKeyWord("time", CON_STATE_TIME),
										new ConKeyWord("trajectory", CON_TRAJECTORY_START),
										new ConKeyWord("/trajectory", CON_TRAJECTORY_END),
										new ConKeyWord("baseTrajectory", CON_BASE_TRAJECTORY_START),
										new ConKeyWord("/baseTrajectory", CON_BASE_TRAJECTORY_END),
										new ConKeyWord("rotationAxis", CON_ROTATION_AXIS),
										new ConKeyWord("reverseTargetAngleOnStance",CON_REVERSE_ANGLE_ON_STANCE),
										new ConKeyWord("feedbackProjectionAxis",CON_FEEDBACK_PROJECTION_AXIS),
										new ConKeyWord("feedback", CON_FEEDBACK_START),
										new ConKeyWord("/feedback", CON_FEEDBACK_END),
										new ConKeyWord("cdNeg", CON_CD_NEG),
										new ConKeyWord("cdPos", CON_CD_POS),
										new ConKeyWord("cd", CON_CD),
										new ConKeyWord("cvNegNeg", CON_CV_NEG_NEG),
										new ConKeyWord("cvNegPos", CON_CV_NEG_POS),
										new ConKeyWord("cvPosNeg", CON_CV_POS_NEG),
										new ConKeyWord("cvPosPos", CON_CV_POS_POS),
										new ConKeyWord("cv", CON_CV),
										new ConKeyWord("loadRBFile", LOAD_RB_FILE),
										new ConKeyWord("loadController", LOAD_CON_FILE),
										new ConKeyWord("component", CON_TRAJ_COMPONENT),
										new ConKeyWord("/component", CON_TRAJ_COMPONENT_END),
										new ConKeyWord("synchronizeControllers", CON_SYNCHRONIZE_CONTROLLERS),
										new ConKeyWord("loadCompositeController", CON_LOAD_COMPOSITE_CONTROLLER),
										new ConKeyWord("strengthTrajectory", CON_STRENGTH_TRAJECTORY_START),
										new ConKeyWord("/strengthTrajectory", CON_STRENGTH_TRAJECTORY_END),
										new ConKeyWord("characterFrameRelative", CON_CHAR_FRAME_RELATIVE),
										new ConKeyWord("externalforce", CON_EXTERNALFORCE),
										new ConKeyWord("/externalforce", CON_EXTERNALFORCE_END),
										new ConKeyWord("forceX", CON_EXTERNALFORCE_X),
										new ConKeyWord("forceY", CON_EXTERNALFORCE_Y),
										new ConKeyWord("forceZ", CON_EXTERNALFORCE_Z),
										new ConKeyWord("torqueX", CON_EXTERNALTORQUE_X),
										new ConKeyWord("torqueY", CON_EXTERNALTORQUE_Y),
										new ConKeyWord("torqueZ", CON_EXTERNALTORQUE_Z),
										new ConKeyWord("/forceX", CON_EXTERNALFORCE_X_END),
										new ConKeyWord("/forceY", CON_EXTERNALFORCE_Y_END),
										new ConKeyWord("/forceZ", CON_EXTERNALFORCE_Z_END),
										new ConKeyWord("/torqueX", CON_EXTERNALTORQUE_X_END),
										new ConKeyWord("/torqueY", CON_EXTERNALTORQUE_Y_END),
										new ConKeyWord("/torqueZ", CON_EXTERNALTORQUE_Z_END),
			
										new ConKeyWord("dMin", CON_D_MIN),
										new ConKeyWord("dMax", CON_D_MAX),
										new ConKeyWord("vMin", CON_V_MIN),
										new ConKeyWord("vMax", CON_V_MAX),
										new ConKeyWord("dTrajX", CON_D_TRAJX_START),
										new ConKeyWord("/dTrajX", CON_D_TRAJX_END),
										new ConKeyWord("dTrajZ", CON_D_TRAJZ_START),
										new ConKeyWord("/dTrajZ", CON_D_TRAJZ_END),
										new ConKeyWord("vTrajX", CON_V_TRAJX_START),
										new ConKeyWord("/vTrajX", CON_V_TRAJX_END),
										new ConKeyWord("vTrajZ", CON_V_TRAJZ_START),
										new ConKeyWord("/vTrajZ", CON_V_TRAJZ_END),
										new ConKeyWord("minFeedback", CON_MIN_FEEDBACK),
										new ConKeyWord("maxFeedback", CON_MAX_FEEDBACK),
										new ConKeyWord("dScaleTraj", CON_D_SCALE_TRAJECTORY_START),
										new ConKeyWord("/dScaleTraj", CON_D_SCALE_TRAJECTORY_END),
										new ConKeyWord("vScaleTraj", CON_V_SCALE_TRAJECTORY_START),
										new ConKeyWord("/vScaleTraj", CON_V_SCALE_TRAJECTORY_END),
										new ConKeyWord("offset", CON_OFFSET),
										new ConKeyWord("noReverseStance", CON_NOT_REVERSE_STANCE),
										new ConKeyWord("doubleStanceMode" ,CON_DOUBLESTANCEMODE),
										new ConKeyWord("doIPBelence" ,CON_DOIPBELENCE),
										new ConKeyWord("velSagittal" ,CON_VELSAGITTAL),
										new ConKeyWord("velCoronal", CON_VELCORONAL),
										new ConKeyWord("stepHeight" ,CON_STEPHEIGHT),
										new ConKeyWord("sagittalStepWidth" ,CON_SAGITTALSTEPWIDTH),
										new ConKeyWord("coronalStepWidth", CON_CORONALSTEPWIDTH),
										new ConKeyWord("reversedCD" ,CON_REVERSEDCD),
										new ConKeyWord("reversedCV" ,CON_REVERSEDCV),
										new ConKeyWord("rootExternalTorqueKp" ,CON_ROOTEXTERNALTORQUEKP),
										new ConKeyWord("rootExternalTorqueKd" ,CON_ROOTEXTERNALTORQUEKD),
										new ConKeyWord("rootExternalForceKp" ,CON_ROOTEXTERNALFORCEKP),
										new ConKeyWord("rootExternalForceKd" ,CON_ROOTEXTERNALFORCEKD),
										new ConKeyWord("velInAir", CON_VELSAGITTAL_INAIR),
										new ConKeyWord("fixedPositionKp", CON_FIXEDPOSITIONKP),
										new ConKeyWord("fixedPositionKd", CON_FIXEDPOSITIONKD),
										new ConKeyWord("bodySpringKp", CON_BODYSPRINGKP),
										new ConKeyWord("bodySpringKd", CON_BODYSPRINGKD),
										new ConKeyWord("comOffsetSagittal", CON_COMOFFSETSAGITTAL),
										new ConKeyWord("comOffsetCoronal", CON_COMOFFSETCORONAL),
										new ConKeyWord("hitFeedback", CON_HITFEEDBACK),
										new ConKeyWord("/hitFeedback", CON_HITFEEDBACK_END),
										new ConKeyWord("hitForce", CON_HITFORCE),
										new ConKeyWord("hitMinTime", CON_HITMINTIME),
										new ConKeyWord("hitBodySpringKp", CON_HITBODYSPRINGKP),
										new ConKeyWord("hitBodySpringKd", CON_HITBODYSPRINGKD),
										new ConKeyWord("hitRootExternalForceKp", CON_HITROOTEXTERNALFORCEKP),
										new ConKeyWord("hitRootExternalForceKd", CON_HITROOTEXTERNALFORCEKD),
										new ConKeyWord("hitRootExternalTorqueKp", CON_HITROOTEXTERNALTORQUEKP),
										new ConKeyWord("hitRootExternalTorqueKd", CON_HITROOTEXTERNALTORQUEKD),
										new ConKeyWord("hitPower", CON_HITPOWER),
										new ConKeyWord("swingStepLength", CON_SWINGSTEPLENGTH),
										new ConKeyWord("standStepLength", CON_STANDSTEPLENGTH),
										new ConKeyWord("continueTrajectory", CON_CONTINUETRAJECTORY),
										new ConKeyWord("belenceError", CON_BELENCEERROR)
									};
		
		/**
			This method is used to determine the type of a line that was used in the input file for a rigid body.
			It is assumed that there are no white spaces at the beginning of the string that is passed in. the pointer buffer
			will be updated to point at the first character after the keyword.
		*/
		public static int getConLineType(string buffer){
		
			if (buffer == null)
				return CON_COMMENT;
		
			for (int i=0;i<keywords.Length;i++){
				if(buffer.StartsWith(((ConKeyWord)keywords[i]).keyWord)){
				//if(buffer.Substring(0,((ConKeyWord)keywords[i]).keyWord.Length) == ((ConKeyWord)keywords[i]).keyWord){
					//buffer += strlen(keywords[i].keyWord);
					return ((ConKeyWord)keywords[i]).retVal;
				}
			}
		
			return CON_NOT_IMPORTANT;
		}
		
		/**
			This method is used to determine the string corresponding to a specific line keyword
		*/
		public static string getConLineString(int lineType){
		
			for (int i=0;i<keywords.Length;i++){
				if ( ((ConKeyWord)keywords[i]).retVal == lineType ){
					return ((ConKeyWord)keywords[i]).keyWord;
				}
			}
		
			return "ERROR! Unknown lineType";
		}

	}
}