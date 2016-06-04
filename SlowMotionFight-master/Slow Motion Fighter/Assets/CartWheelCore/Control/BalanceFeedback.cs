using UnityEngine;
using System.Collections;
using CartWheelCore.Math;
using System.IO;


namespace CartWheelCore.Control{
	/**
		This generic class provides an interface for classes that provide balance feedback for controllers for physically simulated characters.
	*/
	public class BalanceFeedback {
		
		//This vector, dotted with d or v, gives the quantities that should be used in the feedback formula
		public Vector3 feedbackProjectionAxis;
		
		//these are the two feedback gains
		public float cd;
		public float cv;
		
		public float vMin, vMax, dMin, dMax;
		
		public bool reversedCD;
		public bool reversedCV;
		
		public BalanceFeedback(){
		}
		
		virtual public float getFeedbackContribution(SimBiController con, CWJoint j, float phi, Vector3 d, Vector3 v){
			return 0;
		}
		virtual public void writeToFile(StreamWriter fp){
		}
		virtual public void loadFromFile(StringReader fp){
		}
	}
	
	
	/**
		This class applies feedback that is linear in d and v - i.e. the original simbicon feedback formulation
	*/
	public class LinearBalanceFeedback : BalanceFeedback {
		
		public LinearBalanceFeedback(){
			feedbackProjectionAxis = new Vector3(1,0,0);
			cd = cv = 0;
			vMin = dMin = -100;
			vMax = dMax = 100;
			reversedCD = false;
			reversedCV = false;
		}
		
		public void setProjectionAxis( Vector3 axis ) {
			feedbackProjectionAxis = axis;
		}
		
		public Vector3 getProjectionAxis() {
			return feedbackProjectionAxis;
		}
		
		public void setCd( float cd ) {
			this.cd = cd;
		}
	
		public float getCd() { return cd; }
		
		public void setCv( float cv ) {
			this.cv = cv;
		}
	
		public float getCv() { return cv; }
		
		public void setDLimits( float dMin, float dMax ) {
			this.dMin = dMin;
			this.dMax = dMax;
		}
		
		public float getDMin() { return dMin; }
		public float getDMax() { return dMax; }
		
		public void setVLimits( float vMin, float vMax ) {
			this.vMin = vMin;
			this.vMax = vMax;
		}
		
		public float getVMin() { return vMin; }
		public float getVMax() { return vMax; }
		
		/**
			This method returns a scalar that is the ammount of feedback that needs to be added to a trajectory. It is a function of the
			phase in the controller's state (between 0 and 1), the vector between the stance foot and the center of mass, and the velocity of
			the center of mass.
		*/
		override public float getFeedbackContribution(SimBiController con, CWJoint j, float phi, Vector3 d, Vector3 v){
			Vector3 axis = Vector3.zero;
			if (MathLib.getAxisIndex (feedbackProjectionAxis) == 0) {
				axis = con.getCharacter().getLocalLeftAxis();
			} else if (MathLib.getAxisIndex (feedbackProjectionAxis) == 1) {
				axis = con.getCharacter().getLocalUpAxis();
			} else if (MathLib.getAxisIndex (feedbackProjectionAxis) == 2) {
				axis = con.getCharacter().getLocalFrontAxis();
			}
			float dToUse = Vector3.Dot(d, axis);
			if(reversedCD && dToUse<0){
				dToUse = -dToUse;
			}
			float vToUse = Vector3.Dot(v, axis);
			if(reversedCV && vToUse<0){
				vToUse = -vToUse;
			}
			if (dToUse < dMin) dToUse = dMin;
			if (vToUse < vMin) vToUse = vMin;
			if (dToUse > dMax) dToUse = dMax;
			if (vToUse > vMax) vToUse = vMax;
	
			return dToUse * cd + vToUse * cv;
		}
		
		override public void writeToFile(StreamWriter fp){
			if (fp == null) return;
		
			fp.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_FEEDBACK_START)+" linear");
		
			fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_FEEDBACK_PROJECTION_AXIS) + " " + feedbackProjectionAxis.x + " " + feedbackProjectionAxis.y + " " + feedbackProjectionAxis.z);
			fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_CD)+" "+cd);
			fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_CV)+" "+cv);
			fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_REVERSEDCD)+" "+((reversedCD)?"true":"false"));
			fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_REVERSEDCV)+" "+((reversedCV)?"true":"false"));
			if (dMin > -100) fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_D_MIN)+" "+dMin);
			if (dMax < 100) fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_D_MAX)+" "+dMax);
			if (vMin > -100) fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_V_MIN)+" "+vMin);
			if (vMax < 100) fp.WriteLine("\t\t\t\t"+ConUtils.getConLineString(ConUtils.CON_V_MAX)+" "+vMax);
		
			fp.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_FEEDBACK_END));
		}
		
		override public void loadFromFile(StringReader fp){
			if (fp == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = fp.ReadLine();
		
			//this is where it happens.
			while (buffer!=null){
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				switch (lineType) {
					case ConUtils.CON_FEEDBACK_END:
						//we're done...
						return;
					case ConUtils.CON_COMMENT:
						break;
					case ConUtils.CON_CV:
						this.cv = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_CD:
						this.cd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_REVERSEDCD:
						if (nrParams[1] == "true")
							this.reversedCD = true;
						else
							this.reversedCD = false;
						break;
					case ConUtils.CON_REVERSEDCV:
						if (nrParams[1] == "true")
							this.reversedCV = true;
						else
							this.reversedCV = false;
						break;
					case ConUtils.CON_D_MIN:
						this.dMin = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_D_MAX:
						this.dMax = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_V_MIN:
						this.vMin = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_V_MAX:
						this.vMax = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_FEEDBACK_PROJECTION_AXIS:
						feedbackProjectionAxis = new Vector3(float.Parse(nrParams[1]),float.Parse(nrParams[2]),float.Parse(nrParams[3]));
						feedbackProjectionAxis.Normalize();
						break;
					case ConUtils.CON_NOT_IMPORTANT:
						break;
					default:
						break;
						//throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
				}
				buffer = fp.ReadLine();
			}
			//throwError("Incorrect SIMBICON input file: No \'/jointTrajectory\' found ", buffer);
		}
	}
	
	public class DoubleStanceFeedback : BalanceFeedback {
	
		//this is the max value of the feedback
		public float maxFeedbackValue;
		//and this is the lowest value of the feedback
		public float minFeedbackValue;
	
		public float totalMultiplier;
		
		public DoubleStanceFeedback(){
			feedbackProjectionAxis = new Vector3(1,0,0);
			totalMultiplier = 15;
			vMin = -0.3f;
			vMax = 0.3f;
			dMin = -10;
			dMax = 10;
		
			minFeedbackValue = 0;
			maxFeedbackValue = 0;
		}
		
		/**
			This method returns a scalar that is the ammount of feedback that needs to be added to a trajectory. It is a function of the
			phase in the controller's state (between 0 and 1), the vector between the stance foot and the center of mass, and the velocity of
			the center of mass.
		*/
		override public float getFeedbackContribution(SimBiController con, CWJoint j, float phi, Vector3 d, Vector3 v){
			CWRigidBody theBody;
			//Vector3 tmpP;
			float offset = 0;
		
			if (j != null)
				theBody = j.getChild();
			else
				theBody = con.getCharacter().getRoot();
		
			//we might want to initialize it (or add to it) some default value in case we are trying to control its projection
			offset = 0; 
			float dToUse = Vector3.Dot(MathLib.getComplexConjugate(theBody.getOrientation())*con.getDoubleStanceCOMError(),feedbackProjectionAxis);
			if (dToUse < dMin) dToUse = dMin;
			if (dToUse > dMax) dToUse = dMax;
		
			float vToUse = Vector3.Dot(MathLib.getComplexConjugate(theBody.getOrientation())*con.getComVelocity(),feedbackProjectionAxis);
			
			if (vToUse < vMin) vToUse = vMin;
			if (vToUse > vMax) vToUse = vMax;
		
			float err = (dToUse * cd - vToUse * cv + offset);
		
			float result = err * totalMultiplier;

			//if (j)
			//	tprintf("%s gets: %lf\n", j->getName(), result);
		//	else
				//tprintf("root gets: %lf\n", result);
		
			if (result < minFeedbackValue) result = minFeedbackValue;
			if (result > maxFeedbackValue) result = maxFeedbackValue;
		
		//	if (j == NULL && cv > 0.4)
		//		logPrint("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", theBody->getLocalCoordinates(con->COMOffset).dotProductWith(feedbackProjectionAxis), dToUse, theBody->getLocalCoordinates(con->comVelocity).dotProductWith(feedbackProjectionAxis), vToUse, result, forceRatioWeight);
		
			return result;
		}
		
		override public void writeToFile(StreamWriter fp){
			if (fp == null) return;
			fp.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_FEEDBACK_START)+" COM_SupportCenterFeedback");
		
			fp.WriteLine("Not yet implemented...");
		
			fp.WriteLine("\t\t\t"+ConUtils.getConLineString(ConUtils.CON_FEEDBACK_END));
		}
		
		override public void loadFromFile(StringReader fp){
			if (fp == null) return;
				//throwError("File pointer is NULL - cannot read gain coefficients!!");
		
			//have a temporary buffer used to read the file line by line...
			string buffer = fp.ReadLine();
		
			//this is where it happens.
			while (buffer!=null){
				//get a line from the file...
				if (buffer.Length>195) break;
					//throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				string line = buffer.Trim();
				int lineType = ConUtils.getConLineType(line);
				string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
				switch (lineType) {
					case ConUtils.CON_FEEDBACK_END:
						//we're done...
						return;
					case ConUtils.CON_COMMENT:
						break;
					case ConUtils.CON_D_MIN:
						this.dMin = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_D_MAX:
						this.dMax = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_V_MIN:
						this.vMin = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_V_MAX:
						this.vMax = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_CV:
						this.cv = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_CD:
						this.cd = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_MIN_FEEDBACK:
						this.minFeedbackValue = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_MAX_FEEDBACK:
						this.maxFeedbackValue = float.Parse(nrParams[1]);
						break;
					case ConUtils.CON_FEEDBACK_PROJECTION_AXIS:
						feedbackProjectionAxis = new Vector3(float.Parse(nrParams[1]),float.Parse(nrParams[2]),float.Parse(nrParams[3]));
						feedbackProjectionAxis.Normalize();
						break;
					case ConUtils.CON_NOT_IMPORTANT:
						break;
					default:
						break;
						//throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
				}
				buffer = fp.ReadLine();
			}
			//throwError("Incorrect SIMBICON input file: No \'/jointTrajectory\' found ", buffer);
		}
	}
}
