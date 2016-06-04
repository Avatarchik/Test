using UnityEngine;
using System.Collections;
using CartWheelCore.Math;

/**
	This class is used to provide a generic interface to a controller. A controller acts on a character - it computes torques that are
	applied to the joints of the character. The details of how the torques are computed are left up to the classes that extend this one.
*/
namespace CartWheelCore.Control{
	public class Controller {
	
		//this is the character that the controller is acting on
		protected Character character;
		//and this is the array of torques that will be computed in order for the character to match the desired pose - stored in world coordinates
		public ArrayList torques;
		//these are the last torques that were applied - used to place limits on how fast the torques are allowed to change
		protected ArrayList oldTorques;
				
		public ArrayList targetOrientation;
		//this is the number of joints of the character - stored here for easy access
		public int jointCount;
		
		//This is a name for the controller
		public string name;

		protected int localLeftAxisSign;
		protected int localLeftAxisID;
		protected int localUpAxisID;
		protected int localFrontAxisID;
		protected int worldUpAxisID;
		
		public Controller(Character ch) {
			ch.controller = this;
			this.character = ch;
			jointCount = ch.getJointCount();
			torques = new ArrayList();
			oldTorques = new ArrayList();
			targetOrientation = new ArrayList();
			
			for (int i=0;i<jointCount;i++){
				torques.Add(Vector3.zero);
				targetOrientation.Add(Quaternion.identity);
				character.getJoint(i).setId(i);
			}
			
			localLeftAxisID = MathLib.getAxisIndex(character.getLocalLeftAxis());
			localLeftAxisSign = (character.getLocalLeftAxis()[localLeftAxisID]>0)?1:-1;
			localUpAxisID = MathLib.getAxisIndex(character.getLocalUpAxis());
			localFrontAxisID = MathLib.getAxisIndex(character.getLocalFrontAxis());
			worldUpAxisID = MathLib.getAxisIndex(SimGlobals.upAxis);
		}
		
		public Character getCharacter() {
			return character;
		}
		
		virtual public void performPreTasks(float dt) {
			computeTorques();
			applyRotations();
		}
		
		// Returns true if it transitioned to a new state, false otherwise
		virtual public bool performPostTasks(float dt) { return false; }
		
		/**
			This method is used to compute the torques, based on the current and desired poses
		*/
		virtual public void computeTorques(){
		}
		
		public void applyRotations(){
			for (int i=0;i<jointCount;i++){
				CWJoint j = character.getJoint (i);
				character.getJoint (i).setTargetOrientation ((Quaternion)targetOrientation [i]);
			}
		}

		public int getLocalLeftAxisSign(){
			return localLeftAxisSign;
		}
		public int getLocalLeftAxisID(){
			return localLeftAxisID;
		}
		public int getLocalUpAxisID(){
			return localUpAxisID;
		}
		public int getLocalFrontAxisID(){
			return localFrontAxisID;
		}
		public int getWorldUpAxisID(){
			return worldUpAxisID;
		}
		
		/**
			This method is used to reset the torques that are to be applied.
		*/
		public void resetTorques(){
			oldTorques.Clear();
			for (int i=0;i<jointCount;i++)
				torques[i] = Vector3.zero;
		}
	}
}