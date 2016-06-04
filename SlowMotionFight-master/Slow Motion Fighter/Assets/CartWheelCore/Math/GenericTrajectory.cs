using UnityEngine;
using System.Collections;


/**
	This class is used to represent generic trajectories. The class parameter T can be anything that provides basic operation such as addition and subtraction.
	We'll define a trajectory that can be parameterized by a one-d parameter (called t). Based on a set of knots ( tuples <t, T>), we can evaluate the 
	trajectory at any t, through interpolation. This is not used for extrapolation. Outside the range of the knots, the closest known value is returned instead.
*/
namespace CartWheelCore.Math{
	public class GenericTrajectory<T> {
	
		private ArrayList tValues;
		private ArrayList values;
	
		// A caching variable to optimize searching
		private int lastIndex;
		
		public GenericTrajectory(){
			tValues = new ArrayList();
			values = new ArrayList();
			lastIndex = 0;
		}
		
		public GenericTrajectory( GenericTrajectory<T> other ){
			tValues = new ArrayList();
			values = new ArrayList();
			lastIndex = 0;
			copy( other );
		}
		
		private float trajectoryAbs(float val){
			return Mathf.Abs(val);
		}
		
		private float trajectoryAbs(Vector3 val){
			return val.magnitude;
		}
		
		/**
			This method returns the index of the first knot whose value is larger than the parameter value t. If no such index exists (t is larger than any
			of the values stored), then values.size() is returned.
		*/
		private int getFirstLargerIndex(float t) {
			int size = tValues.Count;
			if( size == 0 ) 
				return 0;
			if( t < (float)tValues[(lastIndex+size-1)%size] )
				lastIndex = 0;
			for (int i = 0; i<size;i++){
				int index = (i + lastIndex) % size;
				if (t < (float)tValues[index]) {
					lastIndex = index;
					return index;
				}
			}
			return size;
		}
		
		/**
			This method performs linear interpolation to evaluate the trajectory at the point t
		*/
		public T evaluate_linear(float t) {
			int size = tValues.Count;
			if( size == 0 ) return System.Activator.CreateInstance<T>();
			if (t<=(float)tValues[0]) return (T)values[0];
			if (t>=(float)tValues[size-1])	return (T)values[size-1];
			int index = getFirstLargerIndex(t);
			
			//now linearly interpolate between inedx-1 and index
			t = (t-(float)tValues[index-1]) / ((float)tValues[index]-(float)tValues[index-1]);
			if(typeof(T)==typeof(Vector3)){
				return (T)(object)(((Vector3)values[index-1]) * (1-t) + ((Vector3)values[index]) * t);
			}else if(typeof(T)==typeof(float)){
				return (T)(object)(((float)values[index-1]) * (1-t) + ((float)values[index]) * t);
			}
			return System.Activator.CreateInstance<T>();
		}
		
		/**
			This method interprets the trajectory as a Catmul-Rom spline, and evaluates it at the point t
		*/
		public T evaluate_catmull_rom(float t) {
			int size = tValues.Count;
			if( size == 0 ) return System.Activator.CreateInstance<T>();
			if (t<=(float)tValues[0]) return (T)values[0];
			if (t>=(float)tValues[size-1])	return (T)values[size-1];
			int index = getFirstLargerIndex(t);
			
			//now that we found the interval, get a value that indicates how far we are along it
			t = (t-(float)tValues[index-1]) / ((float)tValues[index]-(float)tValues[index-1]);
	
			//approximate the derivatives at the two ends
			float t0, t1, t2, t3;
			T p0, p1, p2, p3;
			p0 = (index-2<0)?((T)values[index-1]):((T)values[index-2]);
			p1 = (T)values[index-1];
			p2 = (T)values[index];
			p3 = (index+1>=size)?((T)values[index]):((T)values[index+1]);
	
			t0 = (index-2<0)?((float)tValues[index-1]):((float)tValues[index-2]);
			t1 = (float)tValues[index-1];
			t2 = (float)tValues[index];
			t3 = (index+1>=size)?((float)tValues[index]):((float)tValues[index+1]);
	
			float d1 = (t2-t0);
			float d2 = (t3-t1);
	
			if (d1 > -0.0000001f && d1  < 0) d1 = -0.0000001f;
			if (d1 < 0.0000001f && d1  >= 0) d1 = 0.0000001f;
			if (d2 > -0.0000001f && d2  < 0) d2 = -0.0000001f;
			if (d2 < 0.0000001f && d2  >= 0) d2 = 0.0000001f;
	
	//#ifdef FANCY_SPLINES
			//T m1 = (p2 - p0) * (1-(t1-t0)/d1);
			//T m2 = (p3 - p1) * (1-(t3-t2)/d2);
	//#else
			if(typeof(T)==typeof(Vector3)){
				Vector3 m1 = ((Vector3)(object)p2 - (Vector3)(object)p0)*0.5f;
				Vector3 m2 = ((Vector3)(object)p3 - (Vector3)(object)p1)*0.5f;
		//#endif
		
				t2 = t*t;
				t3 = t2*t;
		
				//and now perform the interpolation using the four hermite basis functions from wikipedia
				return (T)(object)((Vector3)(object)p1*(2*t3-3*t2+1) + m1*(t3-2*t2+t) + (Vector3)(object)p2*(-2*t3+3*t2) + m2 * (t3 - t2));
			}else if(typeof(T)==typeof(float)){
				float m1 = ((float)(object)p2 - (float)(object)p0)*0.5f;
				float m2 = ((float)(object)p3 - (float)(object)p1)*0.5f;
		//#endif
		
				t2 = t*t;
				t3 = t2*t;
		
				//and now perform the interpolation using the four hermite basis functions from wikipedia
				return (T)(object)((float)(object)p1*(2*t3-3*t2+1) + m1*(t3-2*t2+t) + (float)(object)p2*(-2*t3+3*t2) + m2 * (t3 - t2));
			}
			return System.Activator.CreateInstance<T>();
		}
		
		/**
			Returns the value of the ith knot. It is assumed that i is within the correct range.
		*/
		public T getKnotValue(int i) {
			return (T)values[i];
		}
		
		/**
			Returns the position of the ith knot. It is assumed that i is within the correct range.
		*/
		public float getKnotPosition(int i) {
			return (float)tValues[i];
		}
		
		/**
			Sets the value of the ith knot to val. It is assumed that i is within the correct range.
		*/
		public void setKnotValue(int i, T val){
			values[i] = val;
		}
		
		/**
			Sets the position of the ith knot to pos. It is assumed that i is within the correct range.
		*/
		public void setKnotPosition(int i, float pos){
			if( i-1 >= 0 && (float)tValues[i-1] >= pos ) return;
			if( i+1 < tValues.Count-1 && (float)tValues[i+1] <= pos ) return;
			tValues[i] = pos;
		}
		
		/**
			Return the smallest tValue or infinity if none
		*/
		public float getMinPosition() {
			if( tValues.Count == 0 )
				return 1000000;
			return (float)tValues[0];
		}
		
		/**
			Return the largest tValue or -infinity if none
		*/
		public float getMaxPosition() {
			if( tValues.Count == 0 ) 
				return -1000000;
			return (float)tValues[tValues.Count-1];
		}
		
		/**
			returns the number of knots in this trajectory
		*/
		public int getKnotCount(){
			return tValues.Count;
		}
		
		/**
			This method is used to insert a new knot in the current trajectory
		*/
		public void addKnot(float t, T val){
			//first we need to know where to insert it, based on the t-values
			int index = getFirstLargerIndex(t);
	
			tValues.Insert(index, t);
			values.Insert(index, val);
		}
		
		/**
			This method is used to remove a knot from the current trajectory.
			It is assumed that i is within the correct range.
		*/
		public void removeKnot(int i){
			tValues.RemoveAt(i);
			values.RemoveAt(i);
		}
		
		/**
			This method removes everything from the trajectory.
		*/
		public void clear(){
			tValues.Clear();
			values.Clear();
		}
		
		/**
			Simplify the curve by iteratively adding knots
		*/
		public void simplify_catmull_rom( float maxError, int nbSamples = 100 ){
	
			if( getKnotCount() < 3 )
				return;
	
			float startTime = (float)tValues[0];
			float endTime = (float)tValues[tValues.Count-1];
	
			GenericTrajectory<T> result = new GenericTrajectory<T>();
			result.addKnot( startTime, (T)values[0] );
			result.addKnot( endTime, (T)values[values.Count-1] );
	
			while( true ) {
				float currError = 0;
				float currErrorTime = -1000000;
	
				for( int i=0; i < nbSamples; ++i ) {
					float interp = (float)i / (nbSamples - 1.0f);
					float time = startTime * (1 - interp) + endTime * interp;
					float error = 0;
					
					if(typeof(T)==typeof(Vector3)){
						error = trajectoryAbs((Vector3)(object)result.evaluate_catmull_rom(time) - (Vector3)(object)evaluate_catmull_rom(time));
					}else if(typeof(T)==typeof(float)){
						error = trajectoryAbs((float)(object)result.evaluate_catmull_rom(time) - (float)(object)evaluate_catmull_rom(time));
					}
					
					if( error > currError ) {
						currError = error;
						currErrorTime = time;
					}
				}
			
				if( currError <= maxError )
					break;
	
				result.addKnot( currErrorTime, evaluate_catmull_rom(currErrorTime) );
			}
	
			copy( result );
		}
		
		public void copy( GenericTrajectory<T> other ) {
			if( other == this ) 
				return;
	
			tValues.Clear();
			values.Clear();
			int size = other.getKnotCount();
	
			for( int i=0; i < size; ++i ) {
				tValues.Add( other.tValues[i] );
				values.Add( other.values[i] );
			}
		}
	}
}