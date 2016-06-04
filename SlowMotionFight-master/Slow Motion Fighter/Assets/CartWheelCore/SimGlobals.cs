using UnityEngine;
using System.Collections;

namespace CartWheelCore{
	public class SimGlobals {
		
		public const int LEFT_STANCE = 0;
		public const int RIGHT_STANCE = 1;
		
		//if this is set to true, then the heading of the character is controlled, otherwise it is free to do whatever it wants
		public static int forceHeadingControl = 0;
		//this variable is used to specify the desired heading of the character
		public static float desiredHeading = 0;
		//and this is the desired time interval for each simulation timestep (does not apply to animations that are played back).
		public static float dt = 0.02f;
		
		public static Vector3 frontAxis = Vector3.right;
		public static Vector3 upAxis = Vector3.up;
		
		public static int getLeftStance(bool isRight){
			if(isRight){
				return LEFT_STANCE;
			}else{
				return RIGHT_STANCE;
			}
		}
		
		public static int getRightStance(bool isRight){
			if(isRight){
				return RIGHT_STANCE;
			}else{
				return LEFT_STANCE;
			}
		}
	
		//temp..
		public static float targetPos = 0;
		public static float targetPosX = 0;
		public static float targetPosZ = 0;
	
		public static float conInterpolationValue;
		public static float bipDesiredVelocity;
	
		public static int constraintSoftness = 1;
	
		public static int CGIterCount = 0;
		public static int linearizationCount = 1;
	
		public static float rootSagittal = 0;
		public static float stanceKnee = 0;
	}
}