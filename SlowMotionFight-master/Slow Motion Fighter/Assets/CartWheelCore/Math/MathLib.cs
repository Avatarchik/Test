using UnityEngine;
using System.Collections;

namespace CartWheelCore.Math{
	public class MathLib {
		
		public static Quaternion getComplexConjugate(Quaternion qua){
			return new Quaternion(-1*qua.x,-1*qua.y,-1*qua.z,qua.w);
		}
		
		/**
			This method is used to rotate the vector that is passed in as a parameter by the current quaternion (which is assumed to be a 
			unit quaternion).
		*/
		public static Vector3 rotate(Quaternion q, Vector3 u){
			//uRot = q * (0, u) * q' = (s, v) * (0, u) * (s, -v)
			//working it out manually, we get:
			Vector3 v = new Vector3(q.x,q.y,q.z);
			Vector3 t = u * q.w + Vector3.Cross(v, u);
			return v * Vector3.Dot(u, v) + t * q.w + Vector3.Cross(v, t);
		}
		
		/**
			this method is used to return the rotation angle represented by this quaternion - in the range -pi to pi.
			Because you can always consider a rotation to be of x degrees around axis v, or by -x degrees around axis -v,
			we need to know the base rotation axis.
		*/
		public static float getRotationAngle(Quaternion qua, Vector3 positiveRotAxis){
			float sinSign = Vector3.Dot(positiveRotAxis, new Vector3(qua.x,qua.y,qua.z));
			float result = 2 * Mathf.Acos(qua.w);
			if (sinSign < 0)
				result = -result;
			if (result > Mathf.PI) result -= 2*Mathf.PI;
			if (result < -Mathf.PI) result += 2*Mathf.PI;
			return result*Mathf.Rad2Deg;
		}
		
		/**
			Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
			This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
			parameter, and another more arbitrary rotation.
		
			AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
			that is returned by this function.
		
			In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
			from T.
		
			It is assumed that vB is a unit vector!! This method returns TqB, which represents a twist about
			the axis vB.
		*/
		public static Quaternion decomposeRotation(Quaternion qua, Vector3 vB){
			//we need to compute v in A's coordinates
			Vector3 vA = qua*vB;
			vA.Normalize();
		
			//float temp = 0;
		
			//compute the rotation that aligns the vector v in the two coordinate frames (A and T)
			Vector3 rotAxis = Vector3.Cross(vA, vB);
			rotAxis.Normalize();
			float rotAngle = -Mathf.Acos(Vector3.Dot(vA, vB))*Mathf.Rad2Deg;
			
			Quaternion TqA = Quaternion.AngleAxis(rotAngle,rotAxis*(-1));
			return TqA*qua;
		}
		
		
		/**
			Assume that the current quaternion represents the relative orientation between two coordinate frames P and C (i.e. q
			rotates vectors from the child/local frame C into the parent/global frame P).
		
			With v specified in frame C's coordinates, this method decomposes the current relative rotation, such that:
		
			PqC = qA * qB, where qB represents a rotation about axis v.
			
			This can be thought of us as a twist about axis v - qB - and a more general rotation, and swing
			- qA - decomposition. Note that qB can be thought of as a rotation from the C frame into a tmp trame T,
			and qA a rotation from T into P.
		
			In the T coordinate frame, v is the same as in C, and qA is a rotation that aligns v from P to that
			from T.
		*/
		public static void decomposeRotation(Quaternion qua, ref Quaternion qA, ref Quaternion qB, Vector3 vC){
			//we need to compute v in P's coordinates
			Vector3 vP = qua*vC;
		
			//compute the rotation that alligns the vector v in the two coordinate frames (P and T - remember that v has the same coordinates in C and in T)
			Vector3 rotAxis = Vector3.Cross(vP, vC);
			rotAxis.Normalize();
			float rotAngle = -Mathf.Acos(Vector3.Dot(vP, vC))*Mathf.Rad2Deg;
		
			qA = Quaternion.AngleAxis(rotAngle,rotAxis);
			//now qB = qAinv * PqC
			qB = MathLib.getComplexConjugate(qA)*qua;
		}
		
		/**
			This method will return a quaternion that represents a rotation of angle radians around the axis provided as a parameter.
			IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
		*/
		public static Quaternion getRotationQuaternion(float angle, Vector3 axis){
			Vector3 vec = axis * Mathf.Sin(angle/2);
			Quaternion result = new Quaternion(vec.x,vec.y,vec.z,Mathf.Cos(angle/2));
		//	result.toUnit();
			return result;
		}
		
		public static Quaternion getQuaternionFromMatrix(Matrix4x4 matrix){
	        return Quaternion.LookRotation(matrix.GetColumn(2), matrix.GetColumn(1));
	    }
		
		/**
			This method is used to mirror the given orientation. It is assumed that rotations in the sagittal plane
			(about parent frame x-axis) are to stay the same, while the rotations about the other axes need to be reversed.
		*/
		public static Quaternion mirrorOrientation(Quaternion q){
			//get the rotation about the parent's x-axis
			Quaternion qSagittal = MathLib.getComplexConjugate(MathLib.decomposeRotation(MathLib.getComplexConjugate(q),Vector3.right));
			//this is what is left, if we removed that component of the rotation
			Quaternion qOther = q * MathLib.getComplexConjugate(qSagittal);
			//and now negate the non-sagittal part of the rotation, but keep the rotation in the sagittal plane
			return MathLib.getComplexConjugate(qOther) * qSagittal;
		}
		
		public static void normalizeQuaternion (ref Quaternion q)
		{
		    float sum = 0;
		    for (int i = 0; i < 4; ++i)
		        sum += q[i] * q[i];
		
		    float magnitudeInverse = 1 / Mathf.Sqrt(sum);
		
		    for (int i = 0; i < 4; ++i)
		        q[i] *= magnitudeInverse;   
		}
		
		public static Matrix4x4 getRotateEulerXMatrix(float angle){
			Matrix4x4 mat = Matrix4x4.zero;
	        float cosq = Mathf.Cos(angle);
	        float sinq = Mathf.Sin(angle);
	        mat[0, 0] = 1.0f; 
	        mat[1, 1] = cosq; 
	        mat[1, 2] = -sinq; 
	        mat[2, 1] = sinq; 
	        mat[2, 2] = cosq; 
	        mat[3, 3] = 1.0f; 
			
			return mat;
		}
		
		public static Matrix4x4 getRotateEulerYMatrix(float angle){
			Matrix4x4 mat = Matrix4x4.zero;
	        float cosq = Mathf.Cos(angle);
	        float sinq = Mathf.Sin(angle);
	        mat[1, 1] = 1.0f; 
	        mat[0, 0] = cosq; 
	        mat[0, 2] = sinq; 
	        mat[2, 0] = -sinq; 
	        mat[2, 2] = cosq; 
	        mat[3, 3] = 1.0f; 
			
			return mat;
		}
		
		public static Matrix4x4 getRotateEulerZMatrix(float angle){
	        Matrix4x4 mat = Matrix4x4.zero;
	        float cosq = Mathf.Cos(angle);
	        float sinq = Mathf.Sin(angle);
	        mat[2, 2] = 1.0f;
	        mat[0, 0] = cosq;
	        mat[0, 1] = -sinq;
	        mat[1, 0] = sinq;
	        mat[1, 1] = cosq;
	        mat[3, 3] = 1.0f;
			
			return mat;
	    }
		
		public static int getAxisIndex(Vector3 axis){
			if(Mathf.Abs(axis.x) > 0.5f){
				return 0;
			}else if(Mathf.Abs(axis.y) > 0.5f){
				return 1;
			}else{
				return 2;
			}
		}
	}
}