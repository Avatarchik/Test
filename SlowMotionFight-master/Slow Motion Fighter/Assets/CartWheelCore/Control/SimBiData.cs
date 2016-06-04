using UnityEngine;
using System.Collections;
using CartWheelCore.Math;
using System.IO;

namespace CartWheelCore.Control
{
	public class SimBiData
	{
		public static string[] bipBodies = {"pelvis", "waist", "torso", "head", "lUpperArm", "rUpperArm", "lLowerArm", "rLowerArm", "lUpperLeg", "rUpperLeg", "lLowerLeg", "rLowerLeg", "lFoot", "rFoot", "lHand", "rHand"};
		public static string[] bipJoints = {"pelvis_waist","waist_torso", "torso_head", "lShoulder", "rShoulder", "lElbow", "rElbow", "lHip", "rHip", "lKnee", "rKnee", "lAnkle", "rAnkle", "lWrist", "rWrist"};
		
		//this is a collection of the states that are used in the controller
		private ArrayList states;
		
		public delegate void DataLoaderHandler(int index);
		public event DataLoaderHandler onDataLoadProcess;
		public event DataLoaderHandler onDataLoadFinished;
		
		public SimBiData ()
		{
			states = new ArrayList();
		}
		
		/**
			This makes it possible to externally access the states of this controller
			Returns null if the index is out of range
		 */
		public SimBiConState getState(int idx) {
			if( idx >= states.Count ) return null;
			return states[idx] as SimBiConState;
		}
		
		/**
			Return the number of states
		 */
		public int getStateCount() { return states.Count; }
		
		public void addState(SimBiConState state_disown) {	
			states.Add( state_disown );
			resolveJoints( state_disown );
		}
		
		public void clearStates() {
			states.Clear();
		}
		
		public int getBodyIndex(string bName) {
			for (int i = 0; i < bipBodies.Length; i++){
				if (bipBodies[i] == bName){
					return i;
				}
			}
			return -1;
		}
		
		public int getJointIndex(string jName){
			for (int i = 0; i < bipJoints.Length; i++){
				if (bipJoints[i] == jName)
					return i;
			}
			return -1;
		}
		
		/**
			This method is used to resolve the names (map them to their index) of the joints.
		*/
		private void resolveJoints(SimBiConState state){
			string tmpName;
			for (int i=0;i<state.getExternalForceCount();i++){
				ExternalForce ef = state.getExternalForce(i);
			
				//deal with the SWING_XXX' case
				if(ef.bName.StartsWith("SWING_")){
					tmpName = ef.bName.Substring(6);
					tmpName = tmpName.Insert(0,"r");
					ef.leftStanceARBIndex = getBodyIndex(tmpName);
					
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"l");
					ef.rightStanceARBIndex = getBodyIndex(tmpName);
					
					continue;
				}
				//deal with the STANCE_XXX' case
				if(ef.bName.StartsWith("STANCE_")){
					tmpName = ef.bName.Substring(7);
					tmpName = tmpName.Insert(0,"l");
					ef.leftStanceARBIndex = getBodyIndex(tmpName);
					
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"r");
					ef.rightStanceARBIndex = getBodyIndex(tmpName);
					
					continue;
				}
				//if we get here, it means it is just the name...
				ef.leftStanceARBIndex = getBodyIndex(ef.bName);
				ef.rightStanceARBIndex = ef.leftStanceARBIndex;
			}
		
			for (int i=0;i<state.getTrajectoryCount();i++){
				Trajectory jt = state.getTrajectory(i);
				//deal with the 'root' special case
				if (jt.jName == "root"){
					jt.leftStanceIndex = jt.rightStanceIndex = -1;
				}else if(jt.jName.StartsWith("SWING_")){
					tmpName = jt.jName.Substring(6);
					tmpName = tmpName.Insert(0,"r");
					jt.leftStanceIndex = getJointIndex(tmpName);
					
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"l");
					jt.rightStanceIndex = getJointIndex(tmpName);
					
				}else if(jt.jName.StartsWith("STANCE_")){
					tmpName = jt.jName.Substring(7);
					tmpName = tmpName.Insert(0,"l");
					jt.leftStanceIndex = getJointIndex(tmpName);
					
					tmpName = tmpName.Remove(0,1);
					tmpName = tmpName.Insert(0,"r");
					jt.rightStanceIndex = getJointIndex(tmpName);
					
				}else{
					//if we get here, it means it is just the name...
					jt.leftStanceIndex = getJointIndex(jt.jName);
					jt.rightStanceIndex = jt.leftStanceIndex;
				}
				if(jt.hitFeedback != null){
					jt.hitFeedback.hitARBIndex = getRBIndexBySymbolicName(jt.jName, state.getStance());
				}
			}
		}
		
		/**
			This method is used to return a pointer to a rigid body, based on its name and the current stance of the character
		*/
		private int getRBIndexBySymbolicName(string sName, int stance){
			int result = -1;
			string resolvedName;
			//deal with the SWING/STANCE_XXX' case
			if(sName.StartsWith("SWING_")){
				resolvedName = sName.Substring(6);
				if (stance == SimGlobals.LEFT_STANCE)
					resolvedName = resolvedName.Insert(0,"r");
				else
					resolvedName = resolvedName.Insert(0,"l");
			}else if(sName.StartsWith("STANCE_")){
				resolvedName = sName.Substring(7);
				if (stance == SimGlobals.LEFT_STANCE)
					resolvedName = resolvedName.Insert(0,"l");
				else
					resolvedName = resolvedName.Insert(0,"r");
			}else{
				resolvedName = sName;
			}
		
			result = getJointIndex(resolvedName) + 1;
		
			return result;
		}
		
		/**
			This method loads all the pertinent information regarding the simbicon controller from a file.
		*/
		public void loadFromFile(string data){
			if (data == null) return;
				//throwError("NULL file name provided.");
			StringReader f = new StringReader(data);
			if (f == null) return;
			//throwError("Could not open file: %s", data);
		
			//UnityThreadHelper.CreateThread(()=>
			//{
				//to be able to load multiple controllers from multiple files,
				//we will use this offset to make sure that the state numbers
				//mentioned in each input file are updated correctly
				int stateOffset = this.states.Count;
				SimBiConState tempState;
				int tempStateNr = -1;
			
				//have a temporary buffer used to read the file line by line...
				string buffer = f.ReadLine();
			
				while (buffer!=null){
					if (buffer.Length>195) break;
					
					string line = buffer.Trim();
					int lineType = ConUtils.getConLineType(line);
					string[] nrParams = line.Split(new char[]{' '},System.StringSplitOptions.RemoveEmptyEntries);
					switch (lineType) {
						case ConUtils.CON_STATE_START:
							tempState = new SimBiConState();
							tempStateNr = int.Parse(nrParams[1]);
							states.Add(tempState);
							tempState.readState(f, stateOffset);
							//now we have to resolve all the joint names (i.e. figure out which joints they apply to).
							resolveJoints(tempState);
							//UnityThreadHelper.Dispatcher.Dispatch(() => onDataLoadProcess(tempStateNr + 1));
							break;
						case ConUtils.CON_NOT_IMPORTANT:
							break;
						default:
							break;
					}
					buffer = f.ReadLine();
				}
				f.Close();
				
				onDataLoadFinished(states.Count);
				//UnityThreadHelper.Dispatcher.Dispatch(() => onDataLoadFinished(states.Count));
			//});
		}
		
		/**
			This method is used to write the details of the current controller to a file
		*/
		public void writeToFile(string fileName, string stateFileName){
			StreamWriter f = new StreamWriter(fileName);	
		
			for( int i=0; i<states.Count; ++i ) {
				f.WriteLine( "\n\n" );
				(states[i] as SimBiConState).writeState( f, i );
			}
			f.Close();
		}
	}
}