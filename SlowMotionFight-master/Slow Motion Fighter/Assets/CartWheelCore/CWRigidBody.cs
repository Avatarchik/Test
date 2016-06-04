using UnityEngine;
using System.IO;
using System.Collections;
using CartWheelCore.Math;
using CartWheelCore.Control;

namespace CartWheelCore{
	public struct CWRBState
	{
		//NOTE: all the quantities here are in world coordinates

		// the position of the center of mass of the rigid body
		public Vector3 position;
		// its orientation
		public Quaternion orientation;
		// the velocity of the center of mass
		public Vector3 velocity;
		// and finally, the angular velocity about the center of mass
		public Vector3 angularVelocity;
	}
	
	public struct ContactPointInfo
	{
		//this is the world coordinate of the origin of the contact force...
		public Vector3 cp;
		//this is the normal at the contact point
		public Vector3 n;
		public Vector3 v;
		//this is the first Collider that participated in the contact
		public Collider collider1;
		//and this is the second...
		public Collider collider2;
	}
	
	public class CWRigidBody : MonoBehaviour {
						
		private CWRBState currentState;
		private float mass;
		private float drag;
		private float angularDrag;
		private Vector3 centerOfMass;
		private Vector3 inertiaTensor;
		//An external force that can be applied to the rigid body CM
		private Vector3 externalForce = Vector3.zero;
		private Vector3 externalTorque = Vector3.zero;
		
		//this is the parent joint.
		private CWJoint pJoint = null;
		//and these are the child joints - it can have as many as it wants.
		private ArrayList cJoints = new ArrayList();
		//this is the articulated figure that the rigid body belongs to
		private ArticulatedFigure AFParent = null;
		
		private Vector3 localBottomPosition = Vector3.zero;
		
		private float fixPositionKp = 0;
		private float fixPositionKd = 0;
		private Vector3 fixPositionForce = Vector3.zero;
		private Vector3 fixTargetPosition = Vector3.zero;
		private GameObject fixedTarget = null;
		
		public ArrayList contactPoints = new ArrayList();
		
		public delegate void CollisionFeedbackHandler(HitFeedback feedback, ContactPointInfo cp);
		public event CollisionFeedbackHandler onCollisionEnterHandler;
		
		void Awake () {
			currentState = new CWRBState();
			currentState.position = this.GetComponent<Rigidbody>().position;
			currentState.orientation = this.GetComponent<Rigidbody>().rotation;
			currentState.velocity = this.GetComponent<Rigidbody>().velocity;
			currentState.angularVelocity = this.GetComponent<Rigidbody>().angularVelocity;
			mass = this.gameObject.GetComponent<Rigidbody>().mass;
			drag = this.gameObject.GetComponent<Rigidbody>().drag;
			angularDrag = this.gameObject.GetComponent<Rigidbody>().angularDrag;
			centerOfMass = this.gameObject.GetComponent<Rigidbody> ().centerOfMass;
			inertiaTensor = this.gameObject.GetComponent<Rigidbody> ().inertiaTensor;
			this.gameObject.GetComponent<Rigidbody> ().maxAngularVelocity = 60;
			//Debug.Log ("fdsfhdkshjkhfj  === "+this.gameObject.GetComponent<Rigidbody>().maxAngularVelocity);

			if(this.name=="lLowerLeg" && this.transform.childCount>0){
				localBottomPosition = getLocalCoordinatesPoint(this.transform.FindChild("lPelma").transform.position);
			}else if(this.name=="rLowerLeg" && this.transform.childCount>0){
				localBottomPosition = getLocalCoordinatesPoint(this.transform.FindChild("rPelma").transform.position);
			}else if(this.name=="lLowerArm" && this.transform.childCount>0){
				localBottomPosition = getLocalCoordinatesPoint(this.transform.FindChild("lPalm").transform.position);
			}else if(this.name=="rLowerArm" && this.transform.childCount>0){
				localBottomPosition = getLocalCoordinatesPoint(this.transform.FindChild("rPalm").transform.position);
			}
			//Debug.Log (this.name + " aaaaaaaa  " + this.gameObject.GetComponent<Rigidbody> ().centerOfMass);
		}
		
		void OnCollisionEnter(Collision collision){
		//	Debug.Log(this.name + " vvvvvvvv  "+this.rigidbody.velocity);
			ContactPointInfo cp = new ContactPointInfo();
			cp.collider1 = collision.contacts[0].thisCollider;
			cp.collider2 = collision.contacts[0].otherCollider;
			cp.cp = collision.contacts[0].point;
			cp.n = collision.contacts[0].normal;
			cp.v = collision.relativeVelocity;
			
			HitFeedback fb = new HitFeedback();
			fb.feedbackARB = this;
			fb.localPos = this.getLocalCoordinatesPoint(cp.cp);
			//if(cp.collider2.rigidbody != null && cp.collider2.rigidbody.velocity.magnitude > 0.01f){
				fb.force = collision.relativeVelocity;
			//}else{
				//fb.force = cp.collider2.rigidbody.mass * collision.relativeVelocity;
			//}
			fb.bodySpringKp = -19 * fb.force.magnitude + 580;
			fb.bodySpringKp = Mathf.Clamp(fb.bodySpringKp, 10, 200);
			fb.bodySpringKd = -0.5f * fb.force.magnitude + 20;
			fb.bodySpringKd = Mathf.Clamp(fb.bodySpringKd, 5, 10);
			fb.rootExternalTorqueKp = -2 * fb.force.magnitude + 60;
			fb.rootExternalTorqueKp = Mathf.Clamp(fb.rootExternalTorqueKp, 0.0001f, 10);
			fb.rootExternalTorqueKd = -10 * fb.force.magnitude + 300;
			fb.rootExternalTorqueKd = Mathf.Clamp(fb.rootExternalTorqueKd, 0.0001f, 100);
			fb.power = 6.7f * fb.force.magnitude - 100;
			fb.power = Mathf.Clamp(fb.power, 0, 100);
			fb.isTouchDown = true;
			
			if(onCollisionEnterHandler != null){
				onCollisionEnterHandler(fb, cp);
			}
		}
		
		void OnCollisionStay(Collision collision){
			contactPoints.Clear();
			if(collision.contacts.Length>0){
				//Debug.Log(this.name + " ================================= "+collision.contacts.Length);
				ContactPointInfo cp = new ContactPointInfo();
				cp.collider1 = collision.contacts[0].thisCollider;
				cp.collider2 = collision.contacts[0].otherCollider;
				cp.cp = collision.contacts[0].point;
				cp.n = collision.contacts[0].normal;
				cp.v = collision.relativeVelocity;
				//Debug.Log(collision.contacts[0].thisCollider.name+"<->"+collision.contacts[0].otherCollider.name+" === "+cp.cp+"  "+cp.n+"   "+cp.v);
				//Debug.DrawLine(cp.cp, cp.cp+cp.v, Color.red);
				contactPoints.Add(cp);
			}
		}
		void OnCollisionExit(Collision collision){
			contactPoints.Clear();
		}
		
		public void CWRBUpdate () {
			if(externalForce.magnitude > 0.0001f){
				if(localBottomPosition.magnitude > 0){
					this.GetComponent<Rigidbody>().AddForceAtPosition(externalForce, getWorldBottomPosition());
					Debug.DrawLine(getWorldBottomPosition(),getWorldBottomPosition()+(Vector3)externalForce,Color.yellow);
				}else{
					this.GetComponent<Rigidbody>().AddForce(externalForce);
					Debug.DrawLine(getCMPosition(),getCMPosition() + externalForce,Color.yellow);
				}
			}
			if(externalTorque.magnitude > 0.0001f) {
			//	Debug.Log(this.name + " tttttttt  "+externalTorque);
				this.GetComponent<Rigidbody>().AddTorque(externalTorque);
			//	Debug.DrawLine(getCMPosition(),getCMPosition() + externalTorque,Color.cyan);
			}
			
			if(fixPositionKp != 0){
				fixPositionForce = fixPositionKp * (fixTargetPosition - getWorldBottomPosition()) - fixPositionKd * getAbsoluteVelocityForLocalPoint(localBottomPosition);
				this.GetComponent<Rigidbody>().AddForceAtPosition(fixPositionForce, getWorldBottomPosition());
				Debug.DrawLine(getWorldBottomPosition(), getWorldBottomPosition() + fixPositionForce, Color.yellow);
			}
			/*
			if(this.name=="lLowerArm" || this.name=="rLowerArm"){
				Vector3 pos = this.getWorldBottomPosition();
				Debug.DrawLine(pos, pos+new Vector3(10, 0, 0),Color.green);
			}
			*/
			//if(pJoint!=null){
				//Vector3 pos = this.getWorldCoordinatesPoint(pJoint.getChildJointPosition());
				//Debug.DrawLine(pos, pos+new Vector3(10, 0, 0),Color.red);
				
				//Vector3 pos = pJoint.getParent().getWorldCoordinatesPoint(pJoint.getParentJointPosition());
				//Debug.DrawLine(pos, pos+new Vector3(10, 0, 0),Color.red);
			//}
		}
		
		public void setExternalForce( Vector3 ef ) { externalForce = ef; }
		public void addExternalForce( Vector3 ef ) { externalForce += ef; }

		public void setExternalTorque( Vector3 et ) { externalTorque = et; }
		public void addExternalTorque( Vector3 et ) { externalTorque += et; }
	
		public Vector3 getExternalForce() { return externalForce; }
	
		public Vector3 getExternalTorque() { return externalTorque; }
		
		/**
			This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
		*/
		public Vector3 getWorldCoordinatesPoint(Vector3 localPoint){
			return this.currentState.position + getWorldCoordinatesVector(localPoint);
		}
		
		/**
			This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
		*/
		public Vector3 getWorldCoordinatesVector(Vector3 localVector){
			//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
			Quaternion qua = this.getOrientation();
			return qua*localVector;
		}
		
		/**
			This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
		*/
		public Vector3 getLocalCoordinatesPoint(Vector3 globalPoint){
			Vector3 v = getLocalCoordinatesVector(globalPoint) - getLocalCoordinatesVector(this.currentState.position);
			return v;
		}
		
		/**
			This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
		*/
		public Vector3 getLocalCoordinatesVector(Vector3 globalVector){
			//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
			Quaternion qua = this.getOrientation();
			return MathLib.getComplexConjugate(qua)*globalVector;
		}
		
		/**
			This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
			resulting velocity will be expressed in world coordinates.
		*/
		public Vector3 getAbsoluteVelocityForLocalPoint(Vector3 localPoint){
			//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
			return Vector3.Cross(this.currentState.angularVelocity, getWorldCoordinatesVector(localPoint)) + this.currentState.velocity;
		}
		
		/**
			This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
			resulting velocity will be expressed in world coordinates.
		*/
		public Vector3 getAbsoluteVelocityForGlobalPoint(Vector3 globalPoint){
			//we need to compute the vector r, from the origin of the body to the point of interest
			Vector3 r = (globalPoint - currentState.position);
			//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
			return Vector3.Cross(this.currentState.angularVelocity, r) + this.currentState.velocity;
		}
		
		public Vector3 getWorldBottomPosition(){
			return getWorldCoordinatesPoint(localBottomPosition);
		}
		public Vector3 getLocalBottomPosition(){
			return localBottomPosition;
		}
		public void setLocalBottomPosition(Vector3 pos){
			localBottomPosition = pos;
		}
		/*
		public void setLocalRotaionAxis(Vector3 axis){
			localRotaionAxis = axis;
		}
		public Vector3 getLocalRotaionAxis(){
			return localRotaionAxis;
		}
		*/
		/**
			This method returns the world coordinates of the position of the center of mass of the object
		*/
		public Vector3 getCMPosition(){
			return this.currentState.position;
		}
	
		/**
			This method sets the world coordinate of the posision of the center of mass of the object
		*/
		public void setCMPosition(Vector3 newCMPos){
			this.currentState.position = newCMPos;
		}
		
		/**
			this method returns the body's orientation
		*/
		public Quaternion getOrientation(){
			return this.currentState.orientation;
		}
		
		public void setOrientation(Quaternion qua){
			this.currentState.orientation = qua;
		}
		
		public float getParentLocalAngle(){
			/*if(pJoint != null){
				Quaternion qua = Quaternion.Inverse(pJoint.getParent().getOrientation()) * this.currentState.orientation;
				return qua.eulerAngles[MathLib.getAxisIndex(localRotationAxis)];
			}else{
				return this.currentState.orientation.eulerAngles[MathLib.getAxisIndex(localRotationAxis)];
			}*/
			return 0;
		}
		
		/**
			This method returns the body's center of mass velocity
		*/
		public Vector3 getCMVelocity(){
			return this.currentState.velocity;
		}
		
		/**
			This method sets the velocity of the center of mass of the object
		*/
		public void setCMVelocity(Vector3 newCMVel){
			this.currentState.velocity = newCMVel;
		}
		
		/**
			this method returns the body's angular velocity
		*/
		public Vector3 getAngularVelocity(){
			return this.currentState.angularVelocity;
		}
		
		/**
			this method sets the angular velocity of the body
		*/
		public void setAngularVelocity(Vector3 newAVel){
			this.currentState.angularVelocity = newAVel;
		}
		
		
		/**
			Returns the mass of the rigid body
		*/
		public float getMass(){
			return this.mass;
		}
		public float getDrag(){
			return drag;
		}
		public float getAngularDrag(){
			return angularDrag;
		}
		public Vector3 getCenterOfMass(){
			return centerOfMass;
		}
		public Vector3 getInertiaTensor(){
			return inertiaTensor;
		}
		
		/**
			returns the parent joint for the current articulated body
		*/
		public CWJoint getParentJoint(){return pJoint;}
		public void setParentJoint(CWJoint j){
			pJoint = j;
		}
	
		public int getChildJointCount() { return cJoints.Count; }
		public CWJoint getChildJoint(int i) { return cJoints[i] as CWJoint; }
		public void addChildJoint(CWJoint j){
			cJoints.Add(j);
		}
		
		public void setAFParent(ArticulatedFigure parent){
			this.AFParent = parent;
		}
	
		public ArticulatedFigure getAFParent(){
			return AFParent;
		}
		
		public bool isFixedPosition(){return ((fixPositionKp == 0)?false:true);}
		
		public Vector3 getFixPositionForce(){
			return fixPositionForce;
		}
		
		public GameObject getFixedTarget(){
			return fixedTarget;
		}
		
		public void addFixedPositionJoint(GameObject target, Vector3 targetPosition, float kp, float kd){
			if(fixPositionKp == 0){
				fixPositionKp = kp;
				fixPositionKd = kd;
				fixedTarget = target;
				fixTargetPosition = targetPosition;
			}
		}
		
		public void updateFixedPosition(Vector3 targetPosition){
			if(fixPositionKp != 0){
				fixTargetPosition = targetPosition;
			}
		}
		
		public void removeFixedPositionJoint(){
			if(fixPositionKp != 0){
				fixPositionKp = 0;
				fixPositionKd = 0;
				fixedTarget = null;
				fixTargetPosition = Vector3.zero;
			}
		}
	}
}