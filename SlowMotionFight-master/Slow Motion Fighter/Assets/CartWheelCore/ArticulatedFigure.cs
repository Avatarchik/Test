using UnityEngine;
using System.Collections;

/*======================================================================================================================================================================*
 * An articulated figure is composed of many articulated rigid bodies that are interconnected by joints. Characters, cars, ropes, etc, can all be viewed as articulated *
 * figures. One note is that we will only allow tree structures - no loops.                                                                                             *
 *======================================================================================================================================================================*/
namespace CartWheelCore{
	public class ArticulatedFigure {
	
		//we will keep track of the root of the articulated figure. Based on the outgoing joints we can access its, children, and so on
		protected CWRigidBody root;
		
		//this is the name of the articulated figure
		protected string name;
		protected float mass;
		
		//keep a list of the character's joints, for easy access
		protected ArrayList joints;
		protected ArrayList arbs;
		
		public ArticulatedFigure(){
			this.root = null;
			this.name = "";
			this.mass = 0;
			
			this.joints = new ArrayList();
			this.arbs = new ArrayList();
		}
		
		/**
			Sets the root
		*/
		public void setRoot( CWRigidBody _root ) {
			_root.setAFParent(this);
			this.root = _root;
			arbs.Add(_root);
		}
		
		/**
			returns the root of the current articulated figure.
		*/
		public CWRigidBody getRoot(){
			return root;
		}
		
		/**
			This method adds one rigid body (articulated or not).
		*/
		public void addArticulatedRigidBody( CWRigidBody _arb ) {
			_arb.setAFParent(this);
			arbs.Add(_arb);
			if(_arb.getParentJoint() != null){
				joints.Add(_arb.getParentJoint());
			}
		}
		
		public CWRigidBody getArticulatedRigidBody( int i ) { return arbs[i] as CWRigidBody; }
		public int getArticulatedRigidBodyCount() { return arbs.Count; }
	
		public void setName( string _name ) {
			this.name = _name;
		}
	
		public string getName() {
			return name;
		}
		
		/**
			This method returns an ARB that is a child of this articulated figure
		*/
		public CWRigidBody getARBByName(string _name) {
			CWRigidBody arb;
			for (int i=0;i<arbs.Count;i++){
				arb = arbs[i] as CWRigidBody;
				if (arb.name == _name){
					return arbs[i] as CWRigidBody;
				}
			}
			return null;
		}
		
		public int getARBIndexByName(string _name){
			CWRigidBody arb;
			for (int i=0;i<arbs.Count;i++){
				arb = arbs[i] as CWRigidBody;
				if (arb.name == _name)
					return i;
			}
			return -1;
		}
		
		/**
		Adds a joint to the figure
		This is an empty function as the joints are not tracked
		by the ArticulatedFigure.
		This makes it possible to disown Python of the joint pointer
		so that it doesn't garbage collect it. 
		The real place where Python should be disowned is when
		Joint.setParent() is called since the parent is responsible
		for deleting the joint. However, I don't know how to force
		python to disown an object when a method is called.
		*/
		public void addJoint( CWJoint joint_disown ) {
			joints.Add(joint_disown);
		}
		
		/**
			This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
			point it can be changed into a proper stabilization technique.
		*/
		public void fixJointConstraints(bool fixOrientations, bool fixVelocities){
			if (!root)
				return;
		
			for (int i=0;i<root.getChildJointCount();i++)
				root.getChildJoint(i).fixJointConstraints(fixOrientations, fixVelocities, true);
		}
		
		/**
			This method is used to get all the joints in the articulated figure and add them to the list of joints that is passed in as a paramter.
		*/
	/*	public void addJointsToList(Array otherJoints){
			for( uint i=0; i<joints.length; ++i )
				otherJoints.Push(joints[i]);
		}
		*/
		/**
			This method is used to compute the total mass of the articulated figure.
		*/
		public void computeMass(){
			float curMass = root.getMass();
			float totalMass = curMass;
		
			for (int i=0; i < joints.Count; i++){
				curMass = (joints[i] as CWJoint).getChild().getMass();
				totalMass += curMass;
			}
		
			this.mass = totalMass;
		}
		
		/**
			This method is used to get the total mass of the articulated figure.
		*/
		public float getMass(){
			return this.mass;
		}
		
		/**
			Returns a pointer to the character's ith joint
		*/
		public CWJoint getJoint(int i){
			if (i < 0 || i > joints.Count-1)
				return null;
			return joints[i] as CWJoint;
		}
	
		/**
			This method is used to return the number of joints of the character.
		*/
		public int getJointCount() {
			return joints.Count;
		}
		
		/**
			this method is used to return a reference to the joint whose name is passed as a parameter, or NULL
			if it is not found.
		*/
		public CWJoint getJointByName(string jName){
			for (int i=0;i<joints.Count;i++){
				CWJoint joint = joints[i] as CWJoint;
				if (joint.jName == jName)
					return joint;
			}
			return null;
		}
		
		/**
			this method is used to return the index of the joint (whose name is passed as a parameter) in the articulated figure hierarchy.
		*/
		public int getJointIndex(string jName){
			for (int i=0;i<joints.Count;i++){
				CWJoint joint = joints[i] as CWJoint;
				if (joint.jName == jName)
					return i;
			}
			return -1;
		}
		
		/**
			this method is used to return the index of the joint (whose name is passed as a parameter) in the articulated figure hierarchy.
		*/
		public int getJointIndex(CWJoint joint){
			for (int i=0;i<joints.Count;i++)
				if (joints[i] == joint)
					return i;
			return -1;
		}
	}
}