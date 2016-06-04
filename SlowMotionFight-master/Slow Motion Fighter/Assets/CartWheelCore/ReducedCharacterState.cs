using UnityEngine;
using System.Collections;

/**
	This class is used to map the array of doubles that is used to define the state of the character to an
	easier to use and understand meaning of the state. This should be the only place in the code where we need to know
	that the first 13 numbers in the array represent the position, orientation, velocity and angular velocity, etc.
*/
namespace CartWheelCore{
	public class ReducedCharacterState {
		private ArrayList state;
		private int startIndex;
		
		/**
			Constructor. Allows the index in the array of doubles to be specified as well.
		*/
		public ReducedCharacterState(ArrayList s, int index = 0){
			this.state = s;
			this.startIndex = index;
		}
		
		/**
			gets the root position.
		*/
		public Vector3 getPosition(){
			return new Vector3((float)this.state[0+startIndex], (float)this.state[1+startIndex], (float)this.state[2+startIndex]);
		}
		
		/**
			sets the root position.
		*/
		public void setPosition(Vector3 p){
			this.state[0 + startIndex] = p.x;
			this.state[1 + startIndex] = p.y;
			this.state[2 + startIndex] = p.z;
		}
		
		/**
			gets the root orientation.
		*/
		public Quaternion getOrientation(){
			return new Quaternion((float)this.state[3+startIndex], (float)this.state[4+startIndex], (float)this.state[5+startIndex],  (float)this.state[6+startIndex]);
		}
		
		/**
			sets the root orientation.
		*/
		public void setOrientation(Quaternion q){
			this.state[3 + startIndex] = q.x;
			this.state[4 + startIndex] = q.y;
			this.state[5 + startIndex] = q.z;
			this.state[6 + startIndex] = q.w;
		}
		
		/**
			gets the root velocity.
		*/
		public Vector3 getVelocity(){
			return new Vector3((float)this.state[7+startIndex], (float)this.state[8+startIndex], (float)this.state[9+startIndex]);
		}
		
		/**
			sets the root velocity.
		*/
		public void setVelocity(Vector3 v){
			this.state[7 + startIndex] = v.x;
			this.state[8 + startIndex] = v.y;
			this.state[9 + startIndex] = v.z;
		}
		
		/**
			gets the root angular velocity.
		*/
		public Vector3 getAngularVelocity(){
			return new Vector3((float)this.state[10+startIndex], (float)this.state[11+startIndex], (float)this.state[12+startIndex]);
		}
		
		/**
			sets the root angular velocity.
		*/
		public void setAngularVelocity(Vector3 v){
			this.state[10 + startIndex] = v.x;
			this.state[11 + startIndex] = v.y;
			this.state[12 + startIndex] = v.z;
		}
		
		/**
			gets the relative orientation for joint jIndex
		*/
		public Quaternion getJointRelativeOrientation(int jIndex){
			int offset = startIndex + 13 + 7 * jIndex;
			return new Quaternion((float)this.state[0 + offset], (float)this.state[1 + offset], (float)this.state[2 + offset],  (float)this.state[3 + offset]);
		}
		
		/**
			sets the orientation for joint jIndex
		*/
		public void setJointRelativeOrientation(Quaternion q, int jIndex){
			int offset = startIndex + 13 + 7 * jIndex;
			this.state[0 + offset] = q.x;
			this.state[1 + offset] = q.y;
			this.state[2 + offset] = q.z;
			this.state[3 + offset] = q.w;
		}
		
		/**
			gets the relative angular velocity for joint jIndex
		*/
		public Vector3 getJointRelativeAngVelocity(int jIndex){
			int offset = startIndex + 13 + 7 * jIndex;
			return new Vector3((float)this.state[4 + offset], (float)this.state[5 + offset], (float)this.state[6 + offset]);
		}
		
		/**
			sets the orientation for joint jIndex
		*/
		public void setJointRelativeAngVelocity(Vector3 w, int jIndex){
			int offset = startIndex + 13 + 7 * jIndex;
			this.state[4 + offset] = w.x;
			this.state[5 + offset] = w.y;
			this.state[6 + offset] = w.z;
		}
	}
}