using UnityEngine;
using System.Collections;
using CartWheelCore.Math;
using System.IO;

namespace CartWheelCore.Control{
	
	/**
	 *  This helper class is used to hold information regarding one component of a state trajectory. This includes (mainly): the base trajectory, 
	 *	a data member that specifies the feedback law to be used, and the axis about which it represents a rotation, 
	 */
	public class TrajectoryComponent {
		
		//this is the array of basis functions that specify the trajectories for the sagittal plane.
		public GenericTrajectory<float> baseTraj;
	
		//this trajectory can be used to scale the base trajectory up or down as a function of the distance from the
		//expected d-value. Used, for instance, to limit the contribution of the toe-off if not at steady state.
		public GenericTrajectory<float> dTrajScale;
	
		//this trajectory can be used to scale the base trajectory up or down as a function of the distance from the
		//expected d-value. Used, for instance, to limit the contribution of the toe-off if not at steady state.
		public GenericTrajectory<float> vTrajScale;
	
		//if this variable is set to true, then when the stance of the character is the left side, the 
		//static target provided by this trajectory should be negated
		public bool reverseAngleOnLeftStance;
		//if this variable is set to true, then when the stance of the character is the right side, the 
		//static target provided by this trajectory should be negated
		public bool reverseAngleOnRightStance;
		//this is the rotation axis that the angles obtained from the trajectory represent rotations about
		public Vector3 rotationAxis;
	
		//this is the balance feedback that is to be used with this trajectory
		public BalanceFeedback bFeedback;
	
		//this is the base value for the trajectory
		public float offset;
		
		/**
			Some public constants
		*/
		public const int ROS_DONT_REVERSE = 100;
		
		/**
			default constructor
		*/
		public TrajectoryComponent(){
			baseTraj = new GenericTrajectory<float>();
			dTrajScale = new GenericTrajectory<float>();
			vTrajScale = new GenericTrajectory<float>();
			rotationAxis = Vector3.zero;
			reverseAngleOnLeftStance = false;
			reverseAngleOnRightStance = false;
			bFeedback = null;
			offset = 0;
		}
		
		public void setRotationAxis( Vector3 axis ) {
			rotationAxis = axis;
			//notifyObservers();
		}
	
		public Vector3 getRotationAxis() {
			return rotationAxis;
		}
		
		/**
			This method sets the stance of this state
		*/
		public void setReverseOnStance( int stance ) {
			if ( stance == SimGlobals.LEFT_STANCE ) {
				reverseAngleOnLeftStance = true;
				reverseAngleOnRightStance = false;
			}
			else if ( stance == SimGlobals.RIGHT_STANCE ) {
				reverseAngleOnLeftStance = false;
				reverseAngleOnRightStance = true;
			}
			else if ( stance == ROS_DONT_REVERSE ) {
				reverseAngleOnLeftStance = false;
				reverseAngleOnRightStance = false;
			}
			//else
				//throwError( "Invalid state stance!" );
			//notifyObservers();
		}
		
		public int getReverseOnStance() {
			//if( reverseAngleOnLeftStance && reverseAngleOnRightStance )
				//throwError( "Invalid state stance!" );
			if( reverseAngleOnLeftStance )
				return SimGlobals.LEFT_STANCE;
			if( reverseAngleOnRightStance )
				return SimGlobals.RIGHT_STANCE;
	
			return ROS_DONT_REVERSE;
		}
		
		public void setFeedback( BalanceFeedback feedback_disown ) {
			bFeedback = feedback_disown;
			//notifyObservers();
		}
	
		public BalanceFeedback getFeedback() {
			return bFeedback;
		}
		
		public void setBaseTrajectory( GenericTrajectory<float> traj ) {
			baseTraj.copy( traj );
			//notifyObservers();
		}
		public GenericTrajectory<float> getBaseTrajectory() {
			return baseTraj;
		}
		
		public void setVTrajScale( GenericTrajectory<float> traj ) {
			vTrajScale.copy( traj );
			//notifyObservers();
		}
		public GenericTrajectory<float> getVTrajScale() {
			return vTrajScale;
		}
	
		public void setDTrajScale( GenericTrajectory<float> traj ) {
			dTrajScale.copy( traj );
			//notifyObservers();
		}
		public GenericTrajectory<float> getDTrajScale() {
			return dTrajScale;
		}
		
		/**
			this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
			and the d and v values used for feedback.
		*/
		public Quaternion evaluateTrajectoryComponent(SimBiController con, CWJoint j, Vector3 axis, int stance, float phi, Vector3 d, Vector3 v, bool bareTrajectory = false){
			float baseAngle = offset;

			float scale = 1;
			//this d.z should really be d dotted with some axis - probably the same as the feedback one...
			if (dTrajScale.getKnotCount() > 0)
				scale *= dTrajScale.evaluate_linear(d.x);
	
			//this v.z should really be v dotted with some axis - probably the same as the feedback one...
			if (vTrajScale.getKnotCount() > 0)
				scale *= vTrajScale.evaluate_linear(v.x);
	
			if (bareTrajectory == true)
				scale = 1;
	
			if (baseTraj.getKnotCount() > 0)
				baseAngle += baseTraj.evaluate_catmull_rom(phi) * scale;
	
			if (stance == SimGlobals.LEFT_STANCE && reverseAngleOnLeftStance)
				baseAngle = -baseAngle;
			if (stance == SimGlobals.RIGHT_STANCE && reverseAngleOnRightStance)
				baseAngle = -baseAngle;
	
			float feedbackValue = computeFeedback(con, j, phi, d, v);
			
			return Quaternion.AngleAxis(baseAngle + feedbackValue, axis);
		}
		
		/**
			this method is used to evaluate the feedback contribution, given the current phase, d and v.
		*/
		public float computeFeedback(SimBiController con, CWJoint j, float phi, Vector3 d, Vector3 v){
			if (bFeedback == null)
				return 0;
			return bFeedback.getFeedbackContribution(con, j, phi, d, v);
		}
		
		/**
			This method is used to read the knots of a base trajectory from the file, where they are specified one (knot) on a line
		*/
		public void writeBaseTrajectory(StreamWriter f){
			if (f == null)
				return;
		
			f.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_BASE_TRAJECTORY_START));
		
			for( int i=0; i < baseTraj.getKnotCount(); ++i ) {
				f.WriteLine("\t\t\t\t"+baseTraj.getKnotPosition(i)+" "+baseTraj.getKnotValue(i));
			}
		
			f.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_BASE_TRAJECTORY_END));
		}
		
		/**
			This method is used to read the knots of a base trajectory from the file, where they are specified one (knot) on a line
		*/
		public void writeScaleTraj(StreamWriter f, GenericTrajectory<float> scaleTraj, string type){
			if (f == null)
				return;
		
			f.WriteLine("\t\t\t"+type+"ScaleTraj");
		
			for( int i=0; i < scaleTraj.getKnotCount(); ++i ) {
				f.WriteLine("\t\t\t\t"+scaleTraj.getKnotPosition(i)+" "+scaleTraj.getKnotValue(i));
			}
		
			f.WriteLine("\t\t\t/"+type+"ScaleTraj");
		}
		
		/**
			This method is used to write a trajectory to a file
		*/
		public void writeTrajectoryComponent(StreamWriter f){
			if (f == null)
				return;
		
			f.WriteLine("\t\t"+ConUtils.getConLineString(ConUtils.CON_TRAJ_COMPONENT));
		
			f.WriteLine("\t\t"+ConUtils.getConLineString(ConUtils.CON_ROTATION_AXIS)+" "+rotationAxis.x+" "+rotationAxis.y+" "+rotationAxis.z);
			
			if(Mathf.Abs(offset)>0){
				f.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_OFFSET)+" "+offset);
			}
			
			if( reverseAngleOnLeftStance )
				f.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_REVERSE_ANGLE_ON_STANCE)+" left");
			else if( reverseAngleOnRightStance )
				f.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_REVERSE_ANGLE_ON_STANCE)+" right");
		
			if( bFeedback != null )
				bFeedback.writeToFile( f );
		
			writeBaseTrajectory(f);
			if (dTrajScale.getKnotCount() > 0)
				writeScaleTraj(f, dTrajScale, "d");
		
			if (vTrajScale.getKnotCount() > 0)
				writeScaleTraj(f, vTrajScale, "v");
		
			f.WriteLine("\t\t"+ConUtils.getConLineString(ConUtils.CON_TRAJ_COMPONENT_END));
		}
		
		/**
			This method is used to read a trajectory from a file
		*/
		public void readTrajectoryComponent(StringReader f){
			if (f == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = f.ReadLine();
			string tmpString;
		
			//this is where it happens.
			while (buffer!=null){
				//get a line from the file...
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				switch (lineType) {
					case ConUtils.CON_TRAJ_COMPONENT_END:
						//we're done...
						return;
					case ConUtils.CON_COMMENT:
						break;
					case ConUtils.CON_OFFSET:
						this.offset = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_ROTATION_AXIS:
						this.rotationAxis = new Vector3(float.Parse(nrParams[1]),float.Parse(nrParams[2]),float.Parse(nrParams[3]));
						this.rotationAxis.Normalize();
						break;
					case ConUtils.CON_FEEDBACK_START:
						//read the kind of feedback that is applicable to this state
						tmpString = nrParams[1];
						if (tmpString == "linear"){
							bFeedback = new LinearBalanceFeedback();
							bFeedback.loadFromFile(f);
						}else if (tmpString == "doubleStance"){
							bFeedback = new DoubleStanceFeedback();
							bFeedback.loadFromFile(f);
						}
						break;
					case ConUtils.CON_BASE_TRAJECTORY_START:
						//read in the base trajectory
						SimBiConState.readTrajectory1d(f, baseTraj, ConUtils.CON_BASE_TRAJECTORY_END);
						break;
					case ConUtils.CON_D_SCALE_TRAJECTORY_START:
						//read in the base trajectory
						SimBiConState.readTrajectory1d(f, dTrajScale, ConUtils.CON_D_SCALE_TRAJECTORY_END);
						break;
					case ConUtils.CON_V_SCALE_TRAJECTORY_START:
						//read in the base trajectory
						SimBiConState.readTrajectory1d(f, vTrajScale, ConUtils.CON_V_SCALE_TRAJECTORY_END);
						break;
					case ConUtils.CON_REVERSE_ANGLE_ON_STANCE:
						tmpString = nrParams[1];
						if (tmpString == "left")
							reverseAngleOnLeftStance = true;
						else if (tmpString == "right")
							reverseAngleOnRightStance = true;
						break;
					case ConUtils.CON_NOT_IMPORTANT:
						break;
					default:
						break;
						//throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
				}
				buffer = f.ReadLine();
			}
			//throwError("Incorrect SIMBICON input file: No \'/trajectory\' found ", buffer);
		}
	}
	
	/**
	 *  This helper class is used to hold information regarding one external force.
	 */
	public class ExternalForce {
		//if the biped that is controlled is in a left-sideed stance, then this is the pointer of the articulated rigid body that
		//the external force is applied to 
		public int leftStanceARBIndex;
		//and this is the pointer of the articulated rigid body that the trajectory is associated to if the biped is in a
		//right-side stance
		public int rightStanceARBIndex;
		//we'll keep the body name, for debugging purposes
		public string bName;
	
		//these are the trajectory to define the force
		public GenericTrajectory<float> forceX;
		public GenericTrajectory<float> forceY;
		public GenericTrajectory<float> forceZ;
	
		//these are the trajectory to define the force
		public GenericTrajectory<float> torqueX;
		public GenericTrajectory<float> torqueY;
		public GenericTrajectory<float> torqueZ;
		
		/**
			default constructor
		*/
		public ExternalForce(){
			forceX = new GenericTrajectory<float>();
			forceY = new GenericTrajectory<float>();
			forceZ = new GenericTrajectory<float>();
			torqueX = new GenericTrajectory<float>();
			torqueY = new GenericTrajectory<float>();
			torqueZ = new GenericTrajectory<float>();
			leftStanceARBIndex = rightStanceARBIndex = -1;
			bName = "NoNameBody";
		}
		
		public void setForceX( GenericTrajectory<float> traj ) {
			forceX.copy( traj );
			//notifyObservers();
		}
	
		public void setForceY( GenericTrajectory<float> traj ) {
			forceY.copy( traj );
			//notifyObservers();
		}
	
		public void setForceZ( GenericTrajectory<float> traj ) {
			forceZ.copy( traj );
			//notifyObservers();
		}
		
		public void setTorqueX( GenericTrajectory<float> traj ) {
			torqueX.copy( traj );
			//notifyObservers();
		}
	
		public void setTorqueY( GenericTrajectory<float> traj ) {
			torqueY.copy( traj );
			//notifyObservers();
		}
	
		public void setTorqueZ( GenericTrajectory<float> traj ) {
			torqueZ.copy( traj );
			//notifyObservers();
		}
			
		public GenericTrajectory<float> getForceX() {
			return forceX;
		}
		public GenericTrajectory<float> getForceY() {
			return forceY;
		}
		public GenericTrajectory<float> getForceZ() {
			return forceZ;
		}
		
		public GenericTrajectory<float> getTorqueX() {
			return torqueX;
		}
		public GenericTrajectory<float> getTorqueY() {
			return torqueY;
		}
		public GenericTrajectory<float> getTorqueZ() {
			return torqueZ;
		}
		
		/**
			this method returns the joint index that this trajectory applies to, unless this applies to the root, in which case it returns -1.
		*/
		public int getARBIndex(int stance){
			return (stance == SimGlobals.LEFT_STANCE)?(leftStanceARBIndex):(rightStanceARBIndex);
		}
		
		/**
			Sets the body name.
			"STANCE_" is replaced by "l" when the stance is left and "r" when the stance is right
			"SWING_" is replaced by "r" when the stance is left and "l" when the stance is right
		*/
		public void setBodyName(string name) {
			bName = name;
			//notifyObservers();
		}
		
		public string getBodyName() {
			return bName;
		}
		
		public void readExternalForce(StringReader f){
			if (f == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = f.ReadLine();
		
			//this is where it happens.
			while (buffer!=null){
				//get a line from the file...
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				//string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				switch (lineType) {
					case ConUtils.CON_EXTERNALFORCE_END:
						//we're done...
						return;
					case ConUtils.CON_COMMENT:
						break;
					case ConUtils.CON_EXTERNALFORCE_X:
						SimBiConState.readTrajectory1d(f, forceX, ConUtils.CON_EXTERNALFORCE_X_END);
						break;
					case ConUtils.CON_EXTERNALFORCE_Y:
						SimBiConState.readTrajectory1d(f, forceY, ConUtils.CON_EXTERNALFORCE_Y_END);
						break;
					case ConUtils.CON_EXTERNALFORCE_Z:
						SimBiConState.readTrajectory1d(f, forceZ, ConUtils.CON_EXTERNALFORCE_Z_END);
						break;
					case ConUtils.CON_EXTERNALTORQUE_X:
						SimBiConState.readTrajectory1d(f, torqueX, ConUtils.CON_EXTERNALTORQUE_X_END);
						break;
					case ConUtils.CON_EXTERNALTORQUE_Y:
						SimBiConState.readTrajectory1d(f, torqueY, ConUtils.CON_EXTERNALTORQUE_Y_END);
						break;
					case ConUtils.CON_EXTERNALTORQUE_Z:
						SimBiConState.readTrajectory1d(f, torqueZ, ConUtils.CON_EXTERNALTORQUE_Z_END);
						break;
					case ConUtils.CON_NOT_IMPORTANT:
						break;
					default:
						break;
				}
				buffer = f.ReadLine();
			}
		}
	}
	
	public class HitFeedback {
		public CWRigidBody feedbackARB;
		public Vector3 localPos;
		public Vector3 force;
		public int hitARBIndex;
		public float minTime;
		public float bodySpringKp;
		public float bodySpringKd;
		public float rootExternalForceKp;
		public float rootExternalForceKd;
		public float rootExternalTorqueKp;
		public float rootExternalTorqueKd;
		public float power;
		
		public bool isHit;
		public bool isTouchDown;
		public bool isLookHit;
		
		public HitFeedback(){
			feedbackARB = null;
			localPos = Vector3.zero;
			force = Vector3.zero;
			hitARBIndex = -1;
			minTime = 0;
			
			bodySpringKp = 0.001f;
			bodySpringKd = 0.001f;
			rootExternalForceKp = 0.001f;
			rootExternalForceKd = 0.001f;
			rootExternalTorqueKp = 0.001f;
			rootExternalTorqueKd = 0.001f;
			isHit = false;
			isTouchDown = false;
			isLookHit = false;
		}
		
		public HitFeedback copy(){
			HitFeedback hit = new HitFeedback();
			hit.localPos = this.localPos;
			hit.force = this.force;
			hit.minTime = this.minTime;
			hit.bodySpringKp = this.bodySpringKp;
			hit.bodySpringKd = this.bodySpringKd;
			hit.rootExternalForceKp = this.rootExternalForceKp;
			hit.rootExternalForceKd = this.rootExternalForceKd;
			hit.rootExternalTorqueKp = this.rootExternalTorqueKp;
			hit.rootExternalTorqueKd = this.rootExternalTorqueKd;
			hit.power = this.power;
			hit.isLookHit = this.isLookHit;
			
			return hit;
		}
		
		public void readHitFeedback(StringReader f){
			if (f == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = f.ReadLine();
		
			//this is where it happens.
			while (buffer!=null){
				//get a line from the file...
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				switch (lineType) {
					case ConUtils.CON_HITFEEDBACK_END:
						//we're done...
						return;
					case ConUtils.CON_COMMENT:
						break;
					case ConUtils.CON_HITFORCE:
						force.x = float.Parse(nrParams[1]);
						force.y = float.Parse(nrParams[2]);
						force.z = float.Parse(nrParams[3]);
						break;
					case ConUtils.CON_HITMINTIME:
						minTime = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_HITBODYSPRINGKP:
						bodySpringKp = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_HITBODYSPRINGKD:
						bodySpringKd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_HITROOTEXTERNALFORCEKP:
						rootExternalForceKp = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_HITROOTEXTERNALFORCEKD:
						rootExternalForceKd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_HITROOTEXTERNALTORQUEKP:
						rootExternalTorqueKp = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_HITROOTEXTERNALTORQUEKD:
						rootExternalTorqueKd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_HITPOWER:
						power = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_NOT_IMPORTANT:
						break;
					default:
						break;
				}
				buffer = f.ReadLine();
			}
		}
	}
	
	/**
	 *  This helper class is used to hold information regarding one state trajectory. This includes: a sequence of components, 
	 *	the index of the joint that this trajectory applies to, the coordinate frame in which the final orientation is expressed, etc.
	 */
	public class Trajectory {
		//these are the components that define the current trajectory
		public ArrayList components;
		
		//if the biped that is controlled is in a left-sideed stance, then this is the index of the joint that
		//the trajectory is used to control - it is assumed that if this is -1, then the trajectory applies
		//to the torso, and not to a joint
		public int leftStanceIndex;
		//and this is the index of the joint that the trajectory is associated to if the biped is in a
		//right-side stance
		public int rightStanceIndex;
		//we'll keep the joint name, for debugging purposes
		public string jName;
	
		//if this variable is set to true, then the desired orientation here is expressed in character coordinates, otherwise it is relative
		//to the parent
		public bool relToCharFrame;
		public bool noReverseStance;
		
		public HitFeedback hitFeedback = null;
	
		//this is the trajectory for the strength of the joing.
		public GenericTrajectory<float> strengthTraj;
		
		/**
			default constructor
		*/
		public Trajectory(){
			components = new ArrayList();
			strengthTraj = new GenericTrajectory<float>();
			leftStanceIndex = rightStanceIndex = -1;
			jName = "NoNameJoint";
			relToCharFrame = false;
			noReverseStance = false;
		}
		
		public void setStrengthTrajectory( GenericTrajectory<float> traj ) {
			strengthTraj.copy( traj );
			//notifyObservers();
		}
		
		public GenericTrajectory<float> getStrengthTrajectory() {
			return strengthTraj;
		}
		
		public void addTrajectoryComponent( TrajectoryComponent trajComp_disown ) {
			components.Add( trajComp_disown );
			//notifyObservers();
		}
	
		public void clearTrajectoryComponents() {
			components.Clear();
			//notifyObservers();
		}
		
		public TrajectoryComponent getTrajectoryComponent( int index ) {
			return components[index] as TrajectoryComponent;
		}
	
		public int getTrajectoryComponentCount() {
			return components.Count;
		}
		
		/**
			this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
			and the d and v values used for feedback.
		*/
		public Quaternion evaluateTrajectory(SimBiController con, CWJoint j, int stance, float phi, Vector3 d, Vector3 v, bool bareTrajectory = false){
	
			Quaternion q = Quaternion.identity;
	
			for (int i=0; i<components.Count; i++) {
				Vector3 axis = con.getCharacter().getLocalLeftAxis();
				if(i == 0){
					axis = con.getCharacter().getLocalLeftAxis();
				}else if(i == 1){
					axis = con.getCharacter().getLocalFrontAxis();
				}else if(i == 2){
					axis = con.getCharacter().getLocalUpAxis();
				}
				q = (components [i] as TrajectoryComponent).evaluateTrajectoryComponent (con, j, axis, stance, phi, d, v, bareTrajectory) * q;
			}
	
			Quaternion qua = (j != null) ? j.getInitialLocalRotation () : Quaternion.identity;
			return qua*q;
		}
		
		/**
			this method is used to evaluate the strength of the joint at a given value of phi.
		*/
		public float evaluateStrength(float phiToUse) {
			if( strengthTraj.getKnotCount() == 0 ) return 1.0f;
			return strengthTraj.evaluate_catmull_rom( phiToUse );
		}
		
		/**
			this method returns the joint index that this trajectory applies to, unless this applies to the root, in which case it returns -1.
		*/
		public int getJointIndex(int stance, bool isLeftReverseStance = true){
			if(noReverseStance){
				return (isLeftReverseStance)?leftStanceIndex:rightStanceIndex;
			}else{
				return (stance == SimGlobals.LEFT_STANCE)?(leftStanceIndex):(rightStanceIndex);
			}
		}
		
		/**
			Sets the joint name.
			"STANCE_" is replaced by "l" when the stance is left and "r" when the stance is right
			"SWING_" is replaced by "r" when the stance is left and "l" when the stance is right
		*/
		public void setJointName(string name) {
			jName = name;
			//notifyObservers();
		}
	
		public string getJointName() {
			return jName;
		}
		
		/** 
			Update all the components to recenter them around the new given D and V trajectories
		*/
		/*public void updateComponents(SimBiController con, CWJoint j, GenericTrajectory<float> newDTrajX, GenericTrajectory<float> newDTrajZ, GenericTrajectory<float> newVTrajX, GenericTrajectory<float> newVTrajZ,
							   GenericTrajectory<float> oldDTrajX, GenericTrajectory<float> oldDTrajZ, GenericTrajectory<float> oldVTrajX, GenericTrajectory<float> oldVTrajZ, int nbSamples ) {
			int nbComponents = components.Count;
			for (int i=0;i<nbComponents;i++)
				(components[i] as TrajectoryComponent).updateComponent(con, j, newDTrajX, newDTrajZ, newVTrajX, newVTrajZ,
											   oldDTrajX, oldDTrajZ, oldVTrajX, oldVTrajZ, nbSamples );
		}*/
		
		/**
			This method is used to write the knots of a strength trajectory to the file, where they are specified one (knot) on a line
		*/
		public void writeStrengthTrajectory(StreamWriter f){
			if (f == null)
				return;
		
			f.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_STRENGTH_TRAJECTORY_START));
		
			for( int i=0; i < strengthTraj.getKnotCount(); ++i ) {
				f.WriteLine("\t\t\t\t"+strengthTraj.getKnotPosition(i)+" "+strengthTraj.getKnotValue(i));
			}
		
			f.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_STRENGTH_TRAJECTORY_END));
		}
		
		/**
			This method is used to write a trajectory to a file
		*/
		public void writeTrajectory(StreamWriter f){
			if (f == null)
				return;
		
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_TRAJECTORY_START)+" "+jName);
		
			if (relToCharFrame)
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_CHAR_FRAME_RELATIVE));
			if(noReverseStance){
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_NOT_REVERSE_STANCE));
			}
		
			//writeStrengthTrajectory( f );
		
			for( int i=0; i < components.Count; ++i ) {
				(components[i] as TrajectoryComponent).writeTrajectoryComponent( f );
			}
		
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_TRAJECTORY_END));
		}
		
		/**
			This method is used to read a trajectory from a file
		*/
		public void readTrajectory(StringReader f){
			if (f == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = f.ReadLine();
		
			TrajectoryComponent newComponent;
		
			//this is where it happens.
			while (buffer!=null){
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				switch (lineType) {
					case ConUtils.CON_STRENGTH_TRAJECTORY_START:
						//read in the base trajectory
						SimBiConState.readTrajectory1d(f, strengthTraj, ConUtils.CON_STRENGTH_TRAJECTORY_END );
						break;
					case ConUtils.CON_TRAJECTORY_END:
						//we're done...
						return;
					case ConUtils.CON_CHAR_FRAME_RELATIVE:
						relToCharFrame = true;
						break;
					case ConUtils.CON_NOT_REVERSE_STANCE:
						noReverseStance = true;
						break;
					case ConUtils.CON_COMMENT:
						break;
					case ConUtils.CON_TRAJ_COMPONENT:
						//read in the base trajectory
						newComponent = new TrajectoryComponent();
						newComponent.readTrajectoryComponent(f);
						components.Add(newComponent);
						break;
					case ConUtils.CON_HITFEEDBACK:
						hitFeedback = new HitFeedback();
						hitFeedback.readHitFeedback(f);
						break;
					case ConUtils.CON_NOT_IMPORTANT:
						break;
					default:
						break;
						//throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
				}
				buffer = f.ReadLine();
			}
			//throwError("Incorrect SIMBICON input file: No \'/trajectory\' found ", buffer);
		}
	}
	
	/**
	 *	A simbicon controller is made up of a set of a number of states. Transition between states happen on foot contact, time out, user interaction, etc.
	 *  Each controller state holds the trajectories for all the joints that are controlled. 
	 */
	public class SimBiConState {
		
		//this is the array of external forces, one for each body that has an external force
		private ArrayList sExternalForces;
		//this is the array of trajectories, one for each joint that is controlled
		private ArrayList sTraj;
		//this is a name of this state
		private string name;
		//this is the number of the state that we should transition to in the controller's finite state machine
		private int nextStateIndex;
		//this is the ammount of time that it is expected the biped will spend in this state
		private float stateTime;
		public bool doubleStanceMode;
		public bool doIPBelence;
		public float comOffsetSagittal;
		public float comOffsetCoronal;
		public float velSagittal;
		public float velCoronal;
		public float belenceError;
		public float stepHeight;
		public float swingStepLength;
		public float standStepLength;
		public float sagittalStepWidth;
		public float coronalStepWidth;
		public float rootExternalTorqueKp;
		public float rootExternalTorqueKd;
		public float rootExternalForceKp;
		public float rootExternalForceKd;
		public float fixedPositionKp;
		public float fixedPositionKd;
		public float velSagittalInAir;
		public float bodySpringKp;
		public float bodySpringKd;
		public bool continueTrajectory;
	
		//upon a transition to a new FSM state, it is assumed that the stance of the character either will be given stance, it will be reverseed , or keept the same.
		//if a state is designed for a certain stance, it is given by this variable
		//for generic states, this variable is used to determine if the stance should be reversed (as opposed to set to left or right), or stay the same.
		private bool reverseStance;
		//and if this is the same, then upon entering this FSM state, the stance will remain the same
		private bool keepStance;
		//if both keepStance and reverseStance are set to false, then this is the state that the character is asumed to take
		private int stateStance;
		
		private bool manualTransition;
	
		//if this variable is set to true, it indicates that the transition to the new state should occur when the swing foot contacts the ground
		//if this variable is false, then it will happen when the time of the controller goes up
		private int transitionOnFootState;
		//if we are to allow a transition on foot contact, we need to take care of the possibility that it
		//will occur early. In this case, we may still want to switch. If phi is at least this, then it is assumed
		//that we can transition;
		private float minPhiBeforeTransitionOnFootContact;
	
		//this is the trajectory for the zero value of the feedback d
		private GenericTrajectory<float> dTrajX;
		private GenericTrajectory<float> dTrajZ;
	
		//this is the trajectory for the zero value of the feedback v
		private GenericTrajectory<float> vTrajX;
		private GenericTrajectory<float> vTrajZ;
		
		/**
			Some public constants
		*/
		public const int STATE_REVERSE_STANCE = 100;
		public const int STATE_KEEP_STANCE = 101;
		
		public const int TRANSITIONONTIMEUP = 102;
		public const int TRANSITIONONFOOTDOWN = 103;
		public const int TRANSITIONONMANUAL = 104;
		
		/**
			default constructor
		*/
		public SimBiConState(){
			sExternalForces = new ArrayList();
			sTraj = new ArrayList();
			name = "Uninitialized state";
			nextStateIndex = -1;
			stateTime = 0;
			doubleStanceMode = false;
			doIPBelence = false;
			comOffsetSagittal = 0;
			comOffsetCoronal = 0;
			velSagittal = 0;
			velCoronal = 0;
			belenceError = 0.2f;
			stepHeight = 0.4f;
			swingStepLength = 0;
			standStepLength = 0;
			sagittalStepWidth = 0;
			coronalStepWidth = 0;
			
			rootExternalTorqueKp = 0;
			rootExternalTorqueKd = 0;
			rootExternalForceKp = 0;
			rootExternalForceKd = 0;
			
			fixedPositionKp = 0;
			fixedPositionKd = 0;
			velSagittalInAir = 0;
			bodySpringKp = 0;
			bodySpringKd = 0;
			continueTrajectory = false;
			
			transitionOnFootState = TRANSITIONONTIMEUP;
			minPhiBeforeTransitionOnFootContact = 0.5f;
			manualTransition = false;
			reverseStance = false;
			keepStance = false;
	
			dTrajX = null;
			dTrajZ = null;
			vTrajX = null;
			vTrajZ = null;
		}
		
		public void startManualTransition(){
			manualTransition = true;
		}
		
		/**
			this method is used to determine the new stance, based on the information in this state and the old stance
		*/
		public int getStateStance(int oldStance){
			if (keepStance == true){
				return oldStance;
			}
			if (reverseStance == false){
				return stateStance;
			}
			if (oldStance == SimGlobals.LEFT_STANCE){
				return SimGlobals.RIGHT_STANCE;
			}
			return SimGlobals.LEFT_STANCE;
		}
	
		/**
			this method is used to retrieve the index of the next state 
		*/
		public int getNextStateIndex(){
			return nextStateIndex;
		}
	
		/**
			this method is used to return the number of trajectories for this state
		*/
		public int getExternalForceCount(){
			return sExternalForces.Count;
		}
		
		/**
			Access a given trajectory
		*/
		public ExternalForce getExternalForce( int idx ) {
			if( idx >= sExternalForces.Count ) return null;
			return sExternalForces[idx] as ExternalForce;
		}
	
		/**
			Access a given trajectory by name
		*/
		public ExternalForce getExternalForce( string name) {
			for (int i=0;i<sExternalForces.Count;i++)
				if ((sExternalForces[i] as ExternalForce).bName == name)
					return sExternalForces[i] as ExternalForce;
			return null;
		}
	
		/**
			this method is used to return the number of trajectories for this state
		*/
		public int getTrajectoryCount(){
			return sTraj.Count;
		}
		
		/**
			Access a given trajectory
		*/
		public Trajectory getTrajectory( int idx ) {
			if( idx >= sTraj.Count ) return null;
			return sTraj[idx] as Trajectory;
		}
	
		/**
			Access a given trajectory by name
		*/
		public Trajectory getTrajectory( string name) {
			for (int i=0;i<sTraj.Count;i++)
				if ((sTraj[i] as Trajectory).jName == name)
					return sTraj[i] as Trajectory;
			return null;
		}
		
		public void setDTrajX( GenericTrajectory<float> traj1d_disown ) {
			dTrajX = traj1d_disown;
			//notifyObservers();
		}
		
		public void setDTrajZ( GenericTrajectory<float> traj1d_disown ) {
			dTrajZ = traj1d_disown;
			//notifyObservers();
		}
		
		public void setVTrajX( GenericTrajectory<float> traj1d_disown ) {
			vTrajX = traj1d_disown;
			//notifyObservers();
		}
		
		public void setVTrajZ( GenericTrajectory<float> traj1d_disown ) {
			vTrajZ = traj1d_disown;
			//notifyObservers();
		}
		
		public GenericTrajectory<float> getDTrajX() {
			return dTrajX;
		}
	
		public GenericTrajectory<float> getDTrajZ() {
			return dTrajZ;
		}
	
		public GenericTrajectory<float> getVTrajX() {
			return vTrajX;
		}
	
		public GenericTrajectory<float> getVTrajZ() {
			return vTrajZ;
		}
		
		/**
			This method is used to determine if, based on the parameters passed in and the type of state this is,
			the current state in the controller FSM needs to be transitioned from.
		*/
		public bool needTransition(float phi, CWRigidBody swingFoot){
			//if it is a foot contact based transition
			if (transitionOnFootState == TRANSITIONONFOOTDOWN){
				Vector3 swingFootContactNormal = (swingFoot.contactPoints.Count>0)?((ContactPointInfo)swingFoot.contactPoints[0]).n:Vector3.zero;
				//transition if we have a meaningful foot contact, and if it does not happen too early on...
				//Debug.Log("aaaaaaaaaaaa  "+swingFootContactNormal);
				if ((phi > minPhiBeforeTransitionOnFootContact && Vector3.Dot(swingFootContactNormal, SimGlobals.upAxis)>0.7f) || phi >= 1)
					return true;
				return false;
			}else if(transitionOnFootState == TRANSITIONONMANUAL){
				if (manualTransition){
					manualTransition = false;
					return true;
				}
				return false;
			}
			
			//otherwise it must be a time-based transition
			if (phi >= 1)
				return true;
	
			return false;
		}
		
		/**
			This method makes it possible to set the state name
		*/
		public void setName(string name) {
			this.name = name;
			//notifyObservers();
		}
	
		public void setNextStateIndex( int nextStateIndex ) {
			this.nextStateIndex = nextStateIndex;
			//notifyObservers();
		}
	
		public void setTransitionOnFootState( int transition ) {
			this.transitionOnFootState = transition;
			//notifyObservers();
		}
	
		public int getTransitionOnFootState() {
			return this.transitionOnFootState;
		}
		
		/**
			This method sets the stance of this state
		*/
		public void setStance( int stanceType ) {
			if ( stanceType == SimGlobals.LEFT_STANCE ) {
				reverseStance = false;
				keepStance = false;
				stateStance = SimGlobals.LEFT_STANCE;
			}
			else if ( stanceType == SimGlobals.RIGHT_STANCE ) {
				reverseStance = false;
				keepStance = false;
				stateStance = SimGlobals.RIGHT_STANCE;
			}
			else if ( stanceType == STATE_REVERSE_STANCE ) {
				reverseStance = true;
				keepStance = false;
				stateStance = -1;
			}
			else if ( stanceType == STATE_KEEP_STANCE ) {
				reverseStance = false;
				keepStance = true;
				stateStance = -1;
			}
			//notifyObservers();	
		}
		
		public int getStance() {
			//if( reverseStance && keepStance )
					//throwError( "Invalid state stance!" );
			if( reverseStance )
					return STATE_REVERSE_STANCE;
			if( keepStance )
					return STATE_KEEP_STANCE;
	
			if( stateStance == SimGlobals.LEFT_STANCE )
					return SimGlobals.LEFT_STANCE;
	
			return SimGlobals.RIGHT_STANCE;
		}
		
		public void setDuration( float duration ) {
			stateTime = duration;
			//notifyObservers();
		}
	
		public float getDuration() {
			return stateTime;
		}
			
		public void addExternalForce( ExternalForce extForce_disown ) {
			sExternalForces.Add( extForce_disown );
			//notifyObservers();
		}
	
		public void clearExternalForce() {
			sExternalForces.Clear();
			//notifyObservers();
		}
	
		public void addTrajectory( Trajectory traj_disown ) {
			sTraj.Add( traj_disown );
			//notifyObservers();
		}
	
		public void clearTrajectories() {
			sTraj.Clear();
			//notifyObservers();
		}
		
		/**
			This method makes it possible to access the state name
		*/
		public string getName() {
			return name;
		}
		
		/**
			This method is used to read the state parameters from a file
		*/
		public void readState(StringReader f, int offset){
			if (f == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = f.ReadLine();
			ExternalForce tempForce;
			Trajectory tempTraj;
		
		
			//this is where it happens.
			while (buffer!=null){
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				switch (lineType) {
					case ConUtils.CON_STATE_END:
						//we're done...
						return;
					case ConUtils.CON_NEXT_STATE:
						this.nextStateIndex = int.Parse(nrParams[1]);
						this.nextStateIndex += offset;
						break;
					case ConUtils.CON_STATE_DESCRIPTION:
						this.name = line;
						break;
					case ConUtils.CON_STATE_TIME:
						this.stateTime = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_STATE_STANCE:
						reverseStance = false;
						keepStance = false;
						if (nrParams[1] == "left")
							stateStance = SimGlobals.LEFT_STANCE;
						else
							if (nrParams[1] == "right")
								stateStance = SimGlobals.RIGHT_STANCE;
							else
								if (nrParams[1] == "reverse")
									reverseStance = true;
								else if (nrParams[1] == "same")
										keepStance = true;
						break;
					case ConUtils.CON_TRANSITION_ON:
						transitionOnFootState = TRANSITIONONTIMEUP;
						if (nrParams[1] == "footDown"){
							transitionOnFootState = TRANSITIONONFOOTDOWN;
						}else if(nrParams[1] == "manual"){
							transitionOnFootState = TRANSITIONONMANUAL;
						}
						break;
					case ConUtils.CON_DOUBLESTANCEMODE:
						doubleStanceMode = false;
						if (nrParams[1] == "true")
							doubleStanceMode = true;
						break;
					case ConUtils.CON_DOIPBELENCE:
						doIPBelence = false;
						if (nrParams[1] == "true")
							doIPBelence = true;
						break;
					case ConUtils.CON_COMOFFSETSAGITTAL:
						this.comOffsetSagittal = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_COMOFFSETCORONAL:
						this.comOffsetCoronal = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_VELSAGITTAL:
						this.velSagittal = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_VELCORONAL:
						this.velCoronal = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_BELENCEERROR:
						this.belenceError = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_STEPHEIGHT:
						this.stepHeight = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_SWINGSTEPLENGTH:
						this.swingStepLength = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_STANDSTEPLENGTH:
						this.standStepLength = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_SAGITTALSTEPWIDTH:
						this.sagittalStepWidth = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_CORONALSTEPWIDTH:
						this.coronalStepWidth = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_ROOTEXTERNALTORQUEKP:
						this.rootExternalTorqueKp = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_ROOTEXTERNALTORQUEKD:
						this.rootExternalTorqueKd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_ROOTEXTERNALFORCEKP:
						this.rootExternalForceKp = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_ROOTEXTERNALFORCEKD:
						this.rootExternalForceKd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_FIXEDPOSITIONKP:
						this.fixedPositionKp = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_FIXEDPOSITIONKD:
						this.fixedPositionKd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_VELSAGITTAL_INAIR:
						this.velSagittalInAir = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_BODYSPRINGKP:
						this.bodySpringKp = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_BODYSPRINGKD:
						this.bodySpringKd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_CONTINUETRAJECTORY:
						continueTrajectory = false;
						if (nrParams[1] == "true")
							continueTrajectory = true;
						break;
					case ConUtils.CON_EXTERNALFORCE:
						tempForce = new ExternalForce();
						tempForce.bName = nrParams[1];
						tempForce.readExternalForce(f);
						this.sExternalForces.Add(tempForce);
						break;
					case ConUtils.CON_TRAJECTORY_START:
						//create a new trajectory, and read its information from the file
						tempTraj = new Trajectory();
						tempTraj.jName = nrParams[1];
						tempTraj.readTrajectory(f);
						this.sTraj.Add(tempTraj);
						break;
		
					case ConUtils.CON_D_TRAJX_START:
						dTrajX = new GenericTrajectory<float>();
						readTrajectory1d( f, dTrajX, ConUtils.CON_D_TRAJX_END );
						break;
		
					case ConUtils.CON_D_TRAJZ_START:
						dTrajZ = new GenericTrajectory<float>();
						readTrajectory1d( f, dTrajZ, ConUtils.CON_D_TRAJZ_END );
						break;
		
					case ConUtils.CON_V_TRAJX_START:
						vTrajX = new GenericTrajectory<float>();
						readTrajectory1d( f, vTrajX, ConUtils.CON_V_TRAJX_END );
						break;
		
					case ConUtils.CON_V_TRAJZ_START:
						vTrajZ = new GenericTrajectory<float>();
						readTrajectory1d( f, vTrajZ, ConUtils.CON_V_TRAJZ_END );
						break;
				
					case ConUtils.CON_COMMENT:
					case ConUtils.CON_NOT_IMPORTANT:
					default:
						break;
						//throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
				}
				buffer = f.ReadLine();
			}
			//throwError("Incorrect SIMBICON input file: No \'/State\' found", buffer);
		}
		
		
		/**
			This method is used to write the state parameters to a file
		*/
		public void writeState(StreamWriter f, int index){
			if (f == null)
				return;
		
			f.WriteLine(""+ConUtils.getConLineString(ConUtils.CON_STATE_START)+" "+index);
		
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STATE_DESCRIPTION)+" "+name);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_NEXT_STATE)+" "+nextStateIndex);
			if(transitionOnFootState == TRANSITIONONFOOTDOWN){
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_TRANSITION_ON)+" "+"footDown");
			}else if(transitionOnFootState == TRANSITIONONMANUAL){
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_TRANSITION_ON)+" "+"manual");
			}else{
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_TRANSITION_ON)+" "+"timeUp");
			}
			
			
			if( reverseStance )
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STATE_STANCE)+" reverse");
			else if( keepStance )
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STATE_STANCE)+" same");
			else if( stateStance == SimGlobals.LEFT_STANCE )
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STATE_STANCE)+" left");
			else if( stateStance == SimGlobals.RIGHT_STANCE )
				f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STATE_STANCE)+" right");
		
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STATE_TIME)+" "+stateTime);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_DOUBLESTANCEMODE)+" "+((doubleStanceMode)?"true":"false"));
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_DOIPBELENCE)+" "+((doIPBelence)?"true":"false"));
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_COMOFFSETSAGITTAL)+" "+ comOffsetSagittal);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_COMOFFSETCORONAL)+" "+ comOffsetCoronal);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_VELSAGITTAL)+" "+ velSagittal);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_VELCORONAL)+" "+ velCoronal);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_BELENCEERROR)+" "+ belenceError);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STEPHEIGHT)+" "+ stepHeight);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_SWINGSTEPLENGTH)+" "+ swingStepLength);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_STANDSTEPLENGTH)+" "+ standStepLength);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_SAGITTALSTEPWIDTH)+" "+ sagittalStepWidth);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_CORONALSTEPWIDTH)+" "+ coronalStepWidth);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_ROOTEXTERNALTORQUEKP)+" "+ rootExternalTorqueKp);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_ROOTEXTERNALTORQUEKD)+" "+ rootExternalTorqueKd);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_ROOTEXTERNALFORCEKP)+" "+ rootExternalForceKp);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_ROOTEXTERNALFORCEKD)+" "+ rootExternalForceKd);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_FIXEDPOSITIONKP)+" "+ fixedPositionKp);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_FIXEDPOSITIONKD)+" "+ fixedPositionKd);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_VELSAGITTAL_INAIR)+" "+ velSagittalInAir);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_BODYSPRINGKP)+" "+ bodySpringKp);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_BODYSPRINGKD)+" "+ bodySpringKd);
			f.WriteLine("\t"+ConUtils.getConLineString(ConUtils.CON_CONTINUETRAJECTORY)+" "+((continueTrajectory)?"true":"false"));
			
			f.WriteLine("\n");
			
			if( dTrajX != null )
				writeTrajectory1d( f, dTrajX, ConUtils.CON_D_TRAJX_START, ConUtils.CON_D_TRAJX_END );
			if( dTrajZ != null )
				writeTrajectory1d( f, dTrajZ, ConUtils.CON_D_TRAJZ_START, ConUtils.CON_D_TRAJZ_END );
			if( vTrajX != null )
				writeTrajectory1d( f, vTrajX, ConUtils.CON_V_TRAJX_START, ConUtils.CON_V_TRAJX_END );
			if( vTrajZ != null )
				writeTrajectory1d( f, vTrajZ, ConUtils.CON_V_TRAJZ_START, ConUtils.CON_V_TRAJZ_END );
			
			f.WriteLine("\n");
		
			for( int i=0; i<sTraj.Count; ++i ) {
				f.WriteLine("\n");
				(sTraj[i] as Trajectory).writeTrajectory( f );
			}
			
			f.WriteLine(""+ConUtils.getConLineString(ConUtils.CON_STATE_END));
		}
		
		public void writeStateToFile(string fileName, int index){
			StreamWriter f = new StreamWriter(fileName);
			writeState(f, index);
			f.Close();
		}
		
		/**
			This method is used to read the knots of a 1D trajectory from the file, where they are specified one (knot) on a line
			The trajectory is considered complete when a line starting with endingLineType is encountered
		*/
		public static void readTrajectory1d(StringReader f, GenericTrajectory<float> result, int endingLineType ){
			if (f == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = f.ReadLine();
			//float temp1, temp2;
		
			//this is where it happens.
			while (buffer!=null){
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				if( lineType == endingLineType )
					//we're done...
					return;
		
				switch (lineType) {
					case ConUtils.CON_COMMENT:
						break;
					case ConUtils.CON_NOT_IMPORTANT:
						result.addKnot(float.Parse(nrParams[0]), float.Parse(nrParams[1]));
						break;
					default:
						break;
						//throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
				}
				buffer = f.ReadLine();
			}
			//throwError("Incorrect SIMBICON input file: Trajectory not closed ", buffer);
		}
	
		public static void writeTrajectory1d(StreamWriter f, GenericTrajectory<float> result, int startingLineType, int endingLineType ){
			if (f == null) return;
		
			f.WriteLine("\t"+ConUtils.getConLineString(startingLineType));
		
			for( int i=0; i < result.getKnotCount(); ++i ) {
				f.WriteLine("\t\t"+result.getKnotPosition(i)+" "+result.getKnotValue(i));
			}
		
			f.WriteLine("\t"+ConUtils.getConLineString(endingLineType));
		}
	}
}