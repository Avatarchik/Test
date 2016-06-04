using UnityEngine;
using System.Collections;
using CartWheelCore.Math;
using CartWheelCore.Control;

using System.IO;

/**
	A character is an articulated figure - This class implements methods that allow it to easily save and restore its state, etc.
*/
namespace CartWheelCore{
	public class Character : ArticulatedFigure {
		
		public Controller controller;
		public GameObject ownerObject;
		
		protected Vector3 localLeftAxis;
		protected Vector3 localUpAxis;
		protected Vector3 localFrontAxis;
				
		public Character():base(){
		}
		
		/**
			This method is used to populate the relative orientation of the parent and child bodies of joint i.
		*/
		public Quaternion getRelativeOrientation(int i){
			CWJoint joint = this.joints[i] as CWJoint;
			Quaternion parentOri = joint.getParent().getOrientation();
			Quaternion childOri = joint.getChild().getOrientation();
			return Quaternion.Inverse(parentOri)*childOri;
		}
		
		/**
			This method is used to get the relative angular velocities of the parent and child bodies of joint i,
			expressed in parent's local coordinates. 
			We'll assume that i is in the range 0 - joints.size()-1!!!
		*/
		public Vector3 getRelativeAngularVelocity(int i){
			CWJoint joint = this.joints[i] as CWJoint;
			Quaternion parentOri = joint.getParent().getOrientation();
			return Quaternion.Inverse(parentOri)*(joint.getChild().getAngularVelocity() - joint.getParent().getAngularVelocity());
		}
		
		/**
			This method populates the dynamic array passed in with the state of the character.
			For the state, we consider the 13-dimensional state of the root, and then only
			the relative orientation and angular velocity (as measured from the parent) for
			every other link. The velocities of the CM are derived from this information,
			using the velocity propagation technique (if that's what it is called).		
			The only thing we will not be storing explicitly is the positions of the CMs of the rigid bodies. 
			The order in which the bodies appear is given by the array of joints. 
			This works under the assumption that in the joint 
			sequence, the parent of any rigid body appears before its children (assuming that for each joint
			we read the parent first and then the child). 
		*/
		public ArrayList getState(){
			//updateJointOrdering();
			
			ArrayList state = new ArrayList();
			//we'll push the root's state information - ugly code....
			Vector3 rootPos = root.getCMPosition();
			state.Add(rootPos.x);
			state.Add(rootPos.y);
			state.Add(rootPos.z);
		
			Quaternion rootRot = root.getOrientation();
			state.Add(rootRot.x);
			state.Add(rootRot.y);
			state.Add(rootRot.z);
			state.Add(rootRot.w);
		
			Vector3 rootVel = root.getCMVelocity();
			state.Add(rootVel.x);
			state.Add(rootVel.y);
			state.Add(rootVel.z);
		
			Vector3 rootAng = root.getAngularVelocity();
			state.Add(rootAng.x);
			state.Add(rootAng.y);
			state.Add(rootAng.z);
			
			//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
			//we are assuming here that each joint is revolute!!!
			Quaternion qRel;
			Vector3 wRel;
		
			for (int i=0;i<joints.Count;i++){
				//Debug.Log(i+" ---- "+"  ---------------------------  "+(joints[i] as CWJoint).name);
				qRel = getRelativeOrientation(i);
			
				state.Add(qRel.x);
				state.Add(qRel.y);
				state.Add(qRel.z);
				state.Add(qRel.w);
		
				wRel = getRelativeAngularVelocity(i);
				state.Add(wRel.x);
				state.Add(wRel.y);
				state.Add(wRel.z);
			}
			return state;
		}
		
		
		/**
			This method populates the state of the current character with the values that are passed
			in the dynamic array. The same conventions as for the getState() method are assumed.
		*/
		public void setState(ArrayList state, int start = 0, bool hackFlag = true){
			//updateJointOrdering();
			ReducedCharacterState rs = new ReducedCharacterState(state, start);
		
			//kinda ugly code....
			root.setCMPosition(rs.getPosition());
			root.setOrientation(rs.getOrientation());
			root.setCMVelocity(rs.getVelocity());
			root.setAngularVelocity(rs.getAngularVelocity());
			
			//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
			//we are assuming here that each joint is revolute!!!
			Quaternion qRel;
			Vector3 wRel;
		
			//Vector3 r;
			//Vector3 d;
			//Vector3 vRel;
		
			CWJoint joint;
		//	root->updateToWorldTransformation();
			for (int j=0;j<joints.Count;j++){
				qRel = rs.getJointRelativeOrientation(j);
				wRel = rs.getJointRelativeAngVelocity(j);
				//transform the relative angular velocity to world coordinates
				joint = joints[j] as CWJoint;
				wRel = joint.getParent().getOrientation()*wRel;
		
				//now that we have this information, we need to restore the state of the rigid body.
		
				//set the proper orientation
				//int orderedJ = j;
				//if( hackFlag )
					//orderedJ = (int)jointOrder[j];
		
			//	joint = joints[orderedJ] as CWJoint;
				joint.getChild().setOrientation(joint.getParent().getOrientation()*qRel);
				//and the proper angular velocity
				joint.getChild().setAngularVelocity(joint.getParent().getAngularVelocity()+wRel);
				
				//Debug.Log("111111 "+joint.name+" "+qRel+" "+wRel);
				//and now set the linear position and velocity
				joint.fixJointConstraints(false, true, false);
		//		joints[j]->child->updateToWorldTransformation();
			}
		}
		
		/**
			this method is used to return the current heading of the character, specified as an angle measured in radians
		*/
		public float getHeadingAngle(){
			//first we need to get the current heading of the character. Also, note that q and -q represent the same rotation
			Quaternion q = getHeading();
			if (q.w<0){
				q.w = -q.w;
				q.x = -q.x;
				q.y = -q.y;
				q.z = -q.z;
			}
			float currentHeading = 2 * Mathf.Acos(q.w) * Mathf.Rad2Deg;
			if(Vector3.Dot(new Vector3(q.x,q.y,q.z),SimGlobals.upAxis)<0)
				currentHeading = -currentHeading;
			return currentHeading;
		}
		
		/**
			This method returns the dimension of the state. Note that we will consider
			each joint as having 3-DOFs (represented by the 4 values of the quaternion)
			without taking into account what type of joint it is (i.e. a hinge joint
			has only one degree of freedom, but we will still consider the relative orientation
			of the child relative to the parent as a quaternion.
		*/
		public int getStateDimension(){
			//13 for the root, and 7 for every other body (and each body is introduced by a joint).
			return 13 + 7 * joints.Count;
		}
		
		public void setLocalAxis(Vector3 left, Vector3 up, Vector3 front){
			localLeftAxis = left;
			localUpAxis = up;
			localFrontAxis = front;
		}
		
		public Vector3 getLocalLeftAxis(){
			return localLeftAxis;
		}
		
		public Vector3 getLocalUpAxis(){
			return localUpAxis;
		}
		
		public Vector3 getLocalFrontAxis(){
			return localFrontAxis;
		}
				
		/**
			This method is used to compute the center of mass of the articulated figure.
		*/
		public Vector3 getCOM(){
			Vector3 COM = root.getCMPosition() * root.getMass();
			float curMass = root.getMass();
			float totalMass = curMass;
			CWJoint joint;
			for (int i=0; i <joints.Count; i++){
				joint = joints[i] as CWJoint;
				curMass = joint.getChild().getMass();
				totalMass += curMass;
				COM = COM+(joint.getChild().getCMPosition()*curMass);
			}
		
			COM /= totalMass;
		
			return COM;
		}
		
		/**
			This method is used to compute the velocity of the center of mass of the articulated figure.
		*/
		public Vector3 getCOMVelocity(){
			Vector3 COMVel = root.getCMVelocity() * root.getMass();
			float curMass = root.getMass();
			float totalMass = curMass;
			CWJoint joint;
			for (int i=0; i <joints.Count; i++){
				joint = joints[i] as CWJoint;
				curMass = joint.getChild().getMass();
				totalMass += curMass;
				COMVel = COMVel+(joint.getChild().getCMVelocity()*curMass);
			}
		
			COMVel /= totalMass;
		
			return COMVel;
		}
		
		/**
			this method is used to rotate the character about the vertical axis, so that it's default heading has the value that is given as a parameter
		*/
		public void recenter(){
			ArrayList state = getState();
			ReducedCharacterState chS = new ReducedCharacterState(state);
			Vector3 currPos = chS.getPosition();
			currPos.x = currPos.z = 0;
			chS.setPosition(currPos);
			setState(state);
		}
		
		public void setBodySpringStuff(float spring, float damper){
			CWJoint joint;
			for (int i=0; i <joints.Count; i++){
				joint = joints[i] as CWJoint;
				joint.setSpring(spring);
				joint.setDamper(damper);
			}
		}
		
		public void setBodySpringStuff(CWRigidBody body, float spring, float damper, bool isFirst = true){
			if(isFirst && body.getParentJoint() != null){
				//Debug.Log(" ===== body === "+body.getParentJoint().jName);
				body.getParentJoint().setSpring(spring);
				body.getParentJoint().setDamper(damper);
				if(body.name.Contains("LowerArm") || body.name.Contains("LowerLeg")){
					body.getParentJoint().getParent().getParentJoint().setSpring(spring);
					body.getParentJoint().getParent().getParentJoint().setDamper(damper);
				}
			}
			int count = body.getChildJointCount();
			for(int i = 0; i < count; i++){
				CWJoint joint = body.getChildJoint(i);
				joint.setSpring(spring);
				joint.setDamper(damper);
				//Debug.Log(" ===== child body === "+joint.jName);
				if(joint.getChild().getChildJointCount() > 0){
					setBodySpringStuff(joint.getChild(), spring, damper, false);
				}
			}
		}
		
		/**
			this method is used to read the reduced state of the character from the file
		*/
		public void loadReducedStateFromFile(string fName){
			setState(readReducedStateFromFile(fName));
		}
		
		/**
			this method is used to write the reduced state of the character to the file
		*/
		/*public void saveReducedStateToFile(string fName){
			writeReducedStateToFile(fName, getState());
		}*/
		
		
		/**
			this method is used to read the reduced state of the character from the file, into the array passed in as a parameter. The
			state of the character is not modified
		*/
		private ArrayList readReducedStateFromFile(string fName){
			StreamReader sr = new StreamReader(fName);
		    string fileContents = sr.ReadToEnd();
		    sr.Close();
			
			ArrayList state = new ArrayList();
			ArrayList lines = new ArrayList();
			string[] tempStr = fileContents.Split(new char[]{'\n'},System.StringSplitOptions.RemoveEmptyEntries);
		    for (int i=0; i<tempStr.Length; i++) {
				string str = tempStr[i];
				if(!str.StartsWith("#")){
					lines.Add(str);
				}
		    }
			
			float heading = float.Parse((string)lines[0]);
			tempStr = ((string)lines[1]).Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
			state.Add(float.Parse(tempStr[0]));
			state.Add(float.Parse(tempStr[1]));
			state.Add(float.Parse(tempStr[2]));
			
			tempStr = ((string)lines[2]).Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
			state.Add(float.Parse(tempStr[0]));
			state.Add(float.Parse(tempStr[1]));
			state.Add(float.Parse(tempStr[2]));
			state.Add(float.Parse(tempStr[3]));
			
			tempStr = ((string)lines[3]).Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
			state.Add(float.Parse(tempStr[0]));
			state.Add(float.Parse(tempStr[1]));
			state.Add(float.Parse(tempStr[2]));
			
			tempStr = ((string)lines[4]).Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
			state.Add(float.Parse(tempStr[0]));
			state.Add(float.Parse(tempStr[1]));
			state.Add(float.Parse(tempStr[2]));
			
			for (int i=5; i<lines.Count; i++) {
				tempStr = ((string)lines[i]).Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				if(tempStr.Length>3){
					state.Add(float.Parse(tempStr[0]));
					state.Add(float.Parse(tempStr[1]));
					state.Add(float.Parse(tempStr[2]));
					state.Add(float.Parse(tempStr[3]));
				}else{
					state.Add(float.Parse(tempStr[0]));
					state.Add(float.Parse(tempStr[1]));
					state.Add(float.Parse(tempStr[2]));
				}
			}
			
			//now set the heading...
			//setHeading(Quaternion.AngleAxis(Mathf.Rad2Deg*heading, Vector3.up), state, 0);
			
			return state;
		}
		
		/**
			this method is used to return the current heading of the character
		*/
		public Quaternion getHeading(){
			//get the current root orientation, that contains information regarding the current heading and retrieve the twist about the vertical axis
			return computeHeading(root.getOrientation());
		}
		
		/**
			This method decomposes the rotation that is passed in into a rotation by the vertical axis - called vertical twist or heading, and everything else:
			rot = qHeading * qOther;
			The returned orientation is qHeading.
		*/
		private Quaternion computeHeading(Quaternion rot){
			return MathLib.getComplexConjugate(MathLib.decomposeRotation(MathLib.getComplexConjugate(rot),SimGlobals.upAxis));
		}
		
		/**
			this method is used to write the reduced state of the character to a file
		*/
		/*private void writeReducedStateToFile(string fName, ArrayList state){
			//updateJointOrdering();
			
			StreamWriter sw = new StreamWriter(fName);
			
			//retrieve the current heading, write it to file, and then set a zero heading for the state
			float heading = getHeadingAngle();
			setHeading(Quaternion.AngleAxis(0, Vector3.up), state);
			
			sw.WriteLine("# order is:\n# Heading\n# Position\n# Orientation\n# Velocity\n# AngularVelocity\n# Relative Orientation\n# Relative Angular Velocity\n#----------------\n# Heading\n"+heading+"\n# Root("+root.name+")\n");
			sw.WriteLine(""+state[0]+" "+state[1]+" "+state[2]+"\n");
			sw.WriteLine(""+state[3]+" "+state[4]+" "+state[5]+" "+state[6]+"\n");
			sw.WriteLine(""+state[7]+" "+state[8]+" "+state[9]+"\n");
			sw.WriteLine(""+state[10]+" "+state[11]+" "+state[12]+"\n");
			
			for (int i=0;i<joints.Count;i++){
				sw.WriteLine("# "+(joints[i] as CWJoint).jName+"\n");
				sw.WriteLine(""+state[13+7*i+0]+" "+state[13+7*i+1]+" "+state[13+7*i+2]+" "+state[13+7*i+3]+"\n");
				sw.WriteLine(""+state[13+7*i+4]+" "+state[13+7*i+5]+" "+state[13+7*i+6]+"\n");
			}
			
			sw.Close();
		}*/
		
	}
}