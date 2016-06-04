using UnityEngine;
using System.Collections;
using CartWheelCore.Math;

using System.IO;

namespace CartWheelCore.Control{
	
	/**
		A pose controller is used to have the character track a given pose.
		Each pose is given as a series of relative orientations, one for
		each parent-child pair (i.e. joint). Classes extending this one 
		have to worry about setting the desired relative orientation properly.
	*/
	public class PoseController : Controller {
	
		//this is the pose that the character is aiming at achieving
		protected ArrayList desiredPose;

		//this quaternion gives the current heading of the character. The complex conjugate of this orientation is used
		//to transform quantities from world coordinates into a rotation/heading-independent coordinate frame (called the character frame).
		//I will make the asumption that the character frame is a coordinate frame that is aligned with the vertical axis, but has 0 heading, and
		//the characterFrame quaternion is used to rotate vectors from the character frame to the real world frame
		protected Quaternion characterFrame;
		protected Quaternion currentHeading;
		protected Quaternion desiredHeading;
		
		public PoseController(Character ch):base(ch){
			//copy the current state of the character into the desired pose - makes sure that it's the correct size
			//Debug.Log("PoseController===================================================");
			desiredPose = ch.getState();
			characterFrame = Quaternion.identity;
			currentHeading = Quaternion.identity;
			desiredHeading = Quaternion.identity;
		
			//set initial values
			ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
			rs.setPosition(Vector3.zero);
			rs.setVelocity(Vector3.zero);
			rs.setOrientation(Quaternion.identity);
			rs.setAngularVelocity(Vector3.zero);
		
			for (int i=0;i<jointCount;i++){
				//Debug.Log(ch.getJoint(i).name + "+++++++++++++++++++++++++++++++ "+i);
				rs.setJointRelativeAngVelocity(Vector3.zero, i);
				rs.setJointRelativeOrientation(Quaternion.identity, i);
			}
		}
		
		/**
			This method is used to compute the torques that are to be applied at the next step.
		*/
		override public void computeTorques(){
			ReducedCharacterState rs = new ReducedCharacterState(desiredPose);
			for (int i=0;i<jointCount;i++){
				CWJoint joint = character.getJoint(i);
				if(!joint.getRelToRootFrame()){
					targetOrientation[i] = rs.getJointRelativeOrientation(i);
				}else{
					targetOrientation[i] = Quaternion.Inverse(joint.getParent().getOrientation()) * (currentHeading * rs.getJointRelativeOrientation(i));
				}
			}
		}
	}
}