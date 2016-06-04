using UnityEngine;
using System.Collections;
using CartWheelCore.Math;

namespace CartWheelCore{
	public class CWJoint : MonoBehaviour {
		//this is the parent link
		private CWRigidBody parent = null;
		//this is the location of the joint on the parent body - expressed in the parent's local coordinates
		private Vector3 pJPos;
		
		//this is the child link
		private CWRigidBody child = null;
		//this is the location of the joint on the child body - expressed in the child's local coordinates 
		//NOTE: the locations of the parent and child joint locations must overlap in world coordinates
		private Vector3 cJPos;
		
		private ConfigurableJoint joint;

		private Quaternion initialLocalRotation;
		private Quaternion initialWorldRotation;

		//this is the id of the joint...
		private int id = -1;
		
		private bool relToRootFrame = false;

		//this is the name of the joint
		public string jName;

		void Awake () {
			joint = this.gameObject.GetComponent<ConfigurableJoint>();
			
			this.setParent(joint.connectedBody.gameObject.GetComponent<CWRigidBody>());
			this.setChild(this.gameObject.GetComponent<CWRigidBody>());

			initialLocalRotation = this.transform.localRotation;
			initialWorldRotation = this.transform.rotation;

			Vector3 localScale = this.gameObject.transform.root.transform.localScale;
			Vector3 anchorPos = new Vector3(joint.anchor.x*localScale.x,joint.anchor.y*localScale.y,joint.anchor.z*localScale.z);
			Vector3 worldAnchorPos = this.gameObject.transform.position + this.gameObject.transform.rotation * anchorPos;
			this.pJPos = MathLib.getComplexConjugate(joint.connectedBody.rotation) * worldAnchorPos - MathLib.getComplexConjugate(joint.connectedBody.rotation) * joint.connectedBody.position;
			this.cJPos = anchorPos;
			
			//pJPos.z = 0;
			//cJPos.z = 0;
			//Debug.Log(jName +"  === "+pJPos+" ==== "+cJPos);
			this.child.GetComponent<Rigidbody>().centerOfMass = anchorPos;
		}

		void Start(){
		}

		/**
			This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
			point it can be changed into a proper stabilization technique.
		*/
		public void fixJointConstraints(bool fixOrientations, bool fixVelocities, bool recursive){
			if (child == null) return;
			//if it has a parent, we will assume that the parent's position is correct, and move the children to satisfy the joint constraint
			if (parent != null){
				//fix the orientation problems here... hopefully no rigid body is locked (except for the root, which is ok)
				
				//first fix the relative orientation, if desired
				if (fixOrientations){
					//Quaternion qRel = computeRelativeOrientation();
					//fixAngularConstraint(qRel);
				}
				
				//now worry about the joint positions
		
				//compute the vector rc from the child's joint position to the child's center of mass (in world coordinates)
				Vector3 rc = child.getWorldCoordinatesVector(Vector3.zero - cJPos);
				//and the vector rp that represents the same quanity but for the parent
				Vector3 rp = parent.getWorldCoordinatesVector(Vector3.zero - pJPos);
		
				//the location of the child's CM is now: pCM - rp + rc
				child.setCMPosition(parent.getCMPosition() + (rc - rp));
		
				//fix the velocities, if need be
				/*if (fixVelocities){
					//to get the relative velocity, we note that the child rotates with wRel about the joint (axis given by wRel
					//d = vector from joint position to CM of the child),
					//but it also rotates together with the parent with the parent's angular velocity, 
					//so we need to combine these (all velocities are expressed in world coordinates already) (r is the vector
					//from the CM of the parent, to that of the child).
					float wRel = child.getAngularVelocity() - parent.getAngularVelocity();
					Vector2 r = child.getCMPosition() - parent.getCMPosition();
					Vector2 d = child.getCMPosition() - child.getWorldCoordinatesPoint(cJPos);
					Vector2 vRel = Vector3.Cross(parent.getAngularVelocity(), r) + Vector3.Cross(wRel, d);
					child.setCMVelocity(parent.getCMVelocity() + vRel);
				}*/
			}
		
			//make sure that we recursivley fix all the other joint constraints in the articulated figure
			if (recursive){
				for (int i=0;i<child.getChildJointCount();i++){
					child.getChildJoint(i).fixJointConstraints(fixOrientations, fixVelocities, recursive);
				}
			}
		}
		
		public void setSpring(float spring){
			
			JointDrive drive = joint.angularXDrive;
			drive.positionSpring = spring;
			joint.angularXDrive = drive;
			
			drive = joint.angularYZDrive;
			drive.positionSpring = spring;
			joint.angularYZDrive = drive;
		}
		
		public void setDamper(float damper){
			JointDrive drive = joint.angularXDrive;
			drive.positionDamper = damper;
			joint.angularXDrive = drive;
			
			drive = joint.angularYZDrive;
			drive.positionDamper = damper;
			joint.angularYZDrive = drive;
		}

		public void setAngularXMotion(ConfigurableJointMotion type){
			joint.angularXMotion = type;
		}
		public void setAngularYMotion(ConfigurableJointMotion type){
			joint.angularYMotion = type;
		}
		public void setAngularZMotion(ConfigurableJointMotion type){
			joint.angularZMotion = type;
		}

		public ConfigurableJointMotion getAngularXMotion(){
			return joint.angularXMotion;
		}

		public ConfigurableJointMotion getAngularYMotion(){
			return joint.angularYMotion;
		}

		public Quaternion getInitialLocalRotation(){
			return initialLocalRotation;
		}
		public Quaternion getInitialWorldRotation(){
			return initialWorldRotation;
		}

		public void setTargetOrientation(Quaternion qua){
			joint.SetTargetRotationLocal (qua, initialLocalRotation);
		}
		public Quaternion getTargetOrientation(){
			return joint.targetRotation;
		}
		
		public ConfigurableJoint getJoint(){
			return this.joint;
		}
		
		/**
			retrieves the reference to the body's parent
		*/
		public CWRigidBody getParent(){return parent;}
		
		public void setParent( CWRigidBody p ){
			this.parent = p;
			this.parent.addChildJoint(this);
		}
		
		/**
			retrieves the reference to the child's parent
		*/
		public CWRigidBody getChild(){return child;}
		
		/**
			set the chil
		*/
		public void setChild( CWRigidBody c ){
			this.child = c;
			child.setParentJoint(this);
		}
		
		/**
			returns the position of the child joint, expressed in child's coordinates
		*/
		public Vector3 getChildJointPosition(){return cJPos;}
		
		public Vector3 getChildJointWorldPosition(){
			return child.getWorldCoordinatesPoint(cJPos);
		}
		
		/**
			returns the position of the parent joint, expressed in parent's coordinates
		*/
		public Vector3 getParentJointPosition(){return pJPos;}
		
		public Vector3 getParentJointWorldPosition(){
			return parent.getWorldCoordinatesPoint(pJPos);
		}
		
		public int getId() { return id; }
	
		public void setId( int i ) {
			this.id = i;
		}
		
		public bool getRelToRootFrame(){
			return relToRootFrame;
		}
		
		public void setRelToRootFrame(bool relToRoot){
			relToRootFrame = relToRoot;
		}
	}
}