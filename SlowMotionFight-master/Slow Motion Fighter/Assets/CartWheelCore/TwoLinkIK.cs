using UnityEngine;
using System.Collections;
using CartWheelCore.Control;
using CartWheelCore.Math;

/**
	This class implements an analytical solution for the inverse kinematics of a linkage composed of two links - a parent link and a child link.
	The origin of the parent link is fixed, and the desired position for the end effector of the child is given as input. The position of the origin of
	the child link is computed, and, with this information, the orientation of the parent and child links are also computed. This can be seen, generally speaking,
	as the solution of the intersection of two spheres, which gives an infinte number of solutions. To reduce this, a vector that represents the normal to the plane
	that the three points (parent origin, child origin and child end effector) should lie on is given as input. The child can only rotate relative to the parent 
	around this normal axis, which can be used as a prior. This results in two possible solutions. The direction of the normal is used to select a unique solution.
*/
namespace CartWheelCore{
	public class TwoLinkIK {
	
		public TwoLinkIK(){
		}
		
		/**
			This method determines the position of the joint between the child and parent links. 
			Input (all quantities need to be measured in the same coordinate frame):
				p1 - location of parent's origin (for example the shoulder for an arm)
				p2 - target location of end effector of child link (for example, the wrist location)
				n - normal to the plane in which the relative motion between parent and child will take place
				r1 - the length of the parent link
				r2 - the length of the child link
			Return:
				p - the position of the joint (the elbow, for instance), measured in the same coordinates as the values passed in here
		*/
		public static Vector3 solve(Vector3 p1, Vector3 p2, Vector3 n, float r1, float r2){
			//the solution for this comes from computation of the intersection of two circles of radii r1 and r2, located at
			//p1 and p2 respectively. There are, of course, two solutions to this problem. The calling application can differentiate between these
			//by passing in n or -n for the plane normal.
			
			//this is the distance between p1 and p2. If it is > r1+r2, then we have no solutions. To be nice about it,
			//we will set r to r1+r2 - the behaviour will be to reach as much as possible, even though you don't hit the target
			float r = (p2 - p1).magnitude;
			if (r > (r1+r2) * 0.993f)
				r = (r1 + r2) * 0.993f;
			//this is the length of the vector starting at p1 and going to the midpoint between p1 and p2
			float a = (r1 * r1 - r2 * r2 + r * r) / (2 * r);
			float tmp = r1*r1 - a*a;
			if (tmp < 0)
				tmp = 0;
			//and this is the distance from the midpoint of p1-p2 to the intersection point
			float h = Mathf.Sqrt(tmp);
			//now we need to get the two directions needed to reconstruct the intersection point
			Vector3 d1 = (p2 - p1).normalized;
			Vector3 d2 = Vector3.Cross(d1, n).normalized;
	
			//and now get the intersection point
			Vector3 p = p1 + d1 * a + d2 * (-h);
	
			return p;
		}
		
		/**
			This method determines the orientation for the parent link, relative to some other coordinate ("global") frame. 
			Two vectors (one that goes from the parent origin to the child origin v, as well as a normal vector n) are known,
			expressed both in the global frame and the parent frame. Using this information, we can figure out the relative orientation
			between the parent frame and the global frame.
	
			Input:
				vGlobal - parent's v expressed in grandparent coordinates
				nGlobal	- this is the rotation axis that is used for the relative rotation between the child and the parent joint, expressed in 
						  grandparent coordinates
				vLocal  - parent's v expressed in the parent's local coordinates
				nLocal  - this is the rotation axis that is used for the relative rotation between the child and the parent joint, expressed in 
						  parent's local coordinates
			Output:
				q		- the relative orientation between the parent and the grandparent (i.e. transforms vectors from parent coordinates to grandparent coordinates).
		*/
		public static Quaternion getParentArmOrientation(Controller con, CWJoint pJoint, CWRigidBody gParent, Vector3 vParent, Vector3 vChild){
			
			Quaternion q = Quaternion.identity;
			Vector3 up = Vector3.zero, left = Vector3.zero, front = Vector3.zero;
			Vector3 tPos = vParent + vChild;
			float tangle = Vector3.Angle(tPos, con.getCharacter().getLocalUpAxis());
			float vangle = Vector3.Angle(vParent, vChild);
			
			//Debug.Log(tangle+ " aaaaaaa  "+vangle + " aaaa "+tPos[con.getLocalFrontAxisID()]+" aaa "+tPos[con.getLocalLeftAxisID()]);
			
			if(tangle <= 90){
				if(tPos[con.getLocalFrontAxisID()] < 0 && vangle > 80){
					up = -1 * vParent.normalized;
					left = Vector3.Cross(vParent, vChild).normalized;
					front = Vector3.Cross(left, up).normalized;
					
					up = gParent.getOrientation() * up;
					front = gParent.getOrientation() * front;
					
					q = gParent.getOrientation() * Quaternion.LookRotation(Vector3.left, Vector3.down);
				}else{
					up = vParent.normalized;
					left = Vector3.Cross(vParent, vChild).normalized;
					front = Vector3.Cross(left, up).normalized;
					
					up = gParent.getOrientation() * up;
					front = gParent.getOrientation() * front;
					
					q = gParent.getOrientation() * Quaternion.LookRotation(Vector3.right, Vector3.up);
				}
			}else{
				if(tPos[con.getLocalFrontAxisID()] > 0 && vangle > 30){
					up = vParent.normalized;
					left = Vector3.Cross(vParent, vChild).normalized;
					front = Vector3.Cross(left, up).normalized;
					
					up = gParent.getOrientation() * up;
					front = gParent.getOrientation() * front;
					
					q = gParent.getOrientation() * Quaternion.LookRotation(Vector3.right, Vector3.up);
				}else{
					up = -1 * vParent.normalized;
					left = Vector3.Cross(vParent, vChild).normalized;
					front = Vector3.Cross(left, up).normalized;
					
					up = gParent.getOrientation() * up;
					front = gParent.getOrientation() * front;
					
					q = gParent.getOrientation() * Quaternion.LookRotation(Vector3.left, Vector3.down);
				}
			}
			
			return Quaternion.Inverse(q) * Quaternion.LookRotation(up, front);
		}
		
		public static Quaternion getParentLegOrientation(Controller con, Vector3 vParent){
			Quaternion q;
			float ang;
			Vector3 v, axis;
			
			v = vParent;
			v[con.getLocalLeftAxisID()] = 0;
			v.Normalize();
			ang = Mathf.Acos(Vector3.Dot(v, -1*con.getCharacter().getLocalUpAxis())) * Mathf.Rad2Deg;
			axis = Vector3.Cross(v, -1*con.getCharacter().getLocalUpAxis()).normalized;
			if(Vector3.Dot(con.getCharacter().getLocalLeftAxis(), axis)<0){
				ang = -ang;
			}
		//	Debug.Log("aaaaaaa "+ang);
			q = Quaternion.AngleAxis(ang, Vector3.right);
		//	Debug.Log("aaaaaaa "+q);
			
			v = vParent;
			v[con.getLocalFrontAxisID()] = 0;
			v.Normalize();
			ang = Mathf.Acos(Vector3.Dot(v, -1*con.getCharacter().getLocalUpAxis())) * Mathf.Rad2Deg;
			axis = Vector3.Cross(v, -1*con.getCharacter().getLocalUpAxis()).normalized;
			if(ang > 90){
				ang = Mathf.Acos(Vector3.Dot(v, con.getCharacter().getLocalUpAxis())) * Mathf.Rad2Deg;
				axis = Vector3.Cross(con.getCharacter().getLocalUpAxis(), v).normalized;
			}
			if(Vector3.Dot(con.getCharacter().getLocalFrontAxis(), axis)<0){
				ang = -ang;
			}
		//	Debug.Log("bbbbbbbbb "+ang);
			q = Quaternion.AngleAxis(ang, Vector3.up) * q;
			//Debug.Log("bbbbbbbbb "+q);
			
			return q;
		}
		
		/**
			This method determines the rotation angle about the axis n for the child link, relative to the orientation of the parent
			Input (all quantities need to be measured in the same coordinate frame):
			(Note: v is the vector from a link's origin to the end effector or the child link's origin).
				vParent - the v qunatity for the parent
				vChild  - the v quntity for the child (vector between the child origin and the end effector).
				n		- the axis of rotation
			Output:
				q		- the relative orientation between the child and parent frames
		*/
		public static float getChildRotationAngle(Vector3 vParent, Vector3 vChild, Vector3 n){
			//compute the angle between the vectors (p1, p) and (p, p2), and that's our result
			float v = Vector3.Dot(vParent.normalized, vChild.normalized);
			v = Mathf.Clamp(v, -1, 1);
			float angle = Mathf.Acos(v) * Mathf.Rad2Deg;
			Vector3 axis = Vector3.Cross(vChild, vParent).normalized;
			if (Vector3.Dot(axis, n)<0)
				angle = -angle;
	
			return angle;
		}
		
		/**
			All quantities that are passed in as parameters here need to be expressed in the same "global" coordinate frame, unless otherwise noted:
				- p1: this is the location of the origin of the parent link
				- p2: this is the target location of the end effector on the child link
				- n: this is the default normal to the rotation plane (it will get modified as little as possible to account for the target location)
		
				- vParent: vector from parent origin to child origin, expressed in parent coordinates
				- nParent: this is the rotation axis, expressed in parent coordinates. The relative orientation between child and parent will be about this axis
				- vChild: vector from child origin, to position of the end effector, expressed in child coordinates
		
				Output:
					- qP: relative orientation of parent, relative to "global" coordinate frame
					- qC: relative orientation of child, relative to the parent coordinate frame
		
		
			NOTE: for now, the vector vChild is pretty much ignored. Only its length is taken into account. The axis that the child is lined up around is the same as the direction
			of the vector from the parent's origin to the child's origin (when there is zero relative orientation between the two, the axis has the same coordinates in both
			child and parent frame).
		*/
		public static void getIKOrientations(Controller con, CWJoint pJoint, CWRigidBody gParent, Vector3 p1, Vector3 p2, Vector3 n, Vector3 vParent, Vector3 nParent, Vector3 vChild, ref Quaternion qP, ref Quaternion qC){
			//modify n so that it's perpendicular to the vector (p1, p2), and still as close as possible to n
			Vector3 line = p2 - p1;
			//Debug.DrawLine(gParent.getWorldCoordinatesPoint(p1), gParent.getWorldCoordinatesPoint(p2), Color.red);
			Vector3 temp = Vector3.Cross(n, line);
			//Debug.DrawLine(gParent.getWorldCoordinatesPoint(p1), gParent.getWorldCoordinatesPoint(p1) + 5*temp, Color.red);
			Vector3 nG = Vector3.Cross(line, temp);
			nG.Normalize();
			
			//Debug.DrawLine(gParent.getWorldCoordinatesPoint(p1), gParent.getWorldCoordinatesPoint(p1) + gParent.getWorldCoordinatesVector(5*nG), Color.red);
		
			//now compute the location of the child origin, in "global" coordinates
			Vector3 solvedJointPosW = TwoLinkIK.solve(p1, p2, nG, vParent.magnitude, vChild.magnitude);
			Vector3 vParentG = solvedJointPosW - p1;
			Vector3 vChildG = p2 - solvedJointPosW;
			
			//now we need to solve for the orientations of the parent and child
			//if the parent has 0 relative orientation to the grandparent (default), then the grandparent's orientation is also the parent's
			//Debug.DrawLine(gParent.getWorldCoordinatesPoint(solvedJointPosW), gParent.getWorldCoordinatesPoint(solvedJointPosW+nG),Color.red);
			//Debug.DrawLine(gParent.getWorldCoordinatesPoint(p1), gParent.getWorldCoordinatesPoint(solvedJointPosW),Color.red);
			//Debug.DrawLine(gParent.getWorldCoordinatesPoint(p2), gParent.getWorldCoordinatesPoint(solvedJointPosW),Color.red);
			if(con.getCharacter().getRoot() == gParent){
				qP = TwoLinkIK.getParentLegOrientation(con, vParentG);
			}else{
				qP = TwoLinkIK.getParentArmOrientation(con, pJoint, gParent, vParentG, vChildG);
			}
			
			float childAngle = TwoLinkIK.getChildRotationAngle(vParentG, vChildG, nG);
			if(Vector3.Dot(nG, nParent)<0){
				childAngle = -childAngle;
			}
			qC = Quaternion.AngleAxis(childAngle, Vector3.right);
			
			//Debug.Log("aaaaaa  "+qP.ToString("F4"));
			//Debug.Log("bbbbbb  "+qC.ToString("F4"));
		}

	}
}