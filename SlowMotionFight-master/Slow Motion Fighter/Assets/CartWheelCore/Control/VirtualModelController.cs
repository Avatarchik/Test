using UnityEngine;
using System.Collections;

namespace CartWheelCore.Control{
	public class VirtualModelController : Controller {
	
		public VirtualModelController(Character ch) : base(ch){
			/*for (int i=0;i<jointCount;i++){
				ch.getJoint(i).tag = ""+i;
			}*/
		}
		
		/**
			This method is used to compute the torques, based on the current and desired poses
		*/
		override public void computeTorques(){
			//don't call this method...
		}
		
		/**
			This method is used to compute the torques that mimick the effect of applying a force on
			a rigid body, at some point. It works best if the end joint is connected to something that
			is grounded, otherwise (I think) this is just an approximation.
		
			This function works by making use of the formula:
		
			t = J' * f, where J' is dp/dq, where p is the position where the force is applied, q is
			'sorta' the relative orientation between links. It makes the connection between the velocity
			of the point p and the relative angular velocities at each joint. Here's an example of how to compute it.
		
			Assume: p = pBase + R1 * v1 + R2 * v2, where R1 is the matrix from link 1 to whatever pBase is specified in,
				and R2 is the rotation matrix from link 2 to whatever pBase is specified in, v1 is the point from link 1's
				origin to link 2's origin (in link 1 coordinates), and v2 is the vector from origin of link 2 to p 
				(in link 2 coordinates).
		
				dp/dt = d(R1 * v1)/dt + d(R2 * v2)/dt = d R1/dt * v1 + d R2/dt * v2, and dR/dt = wx * R, where wx is
				the cross product matrix associated with the angular velocity w
				so dp/dt = w1x * R1 * v1 + w2x * R2 * v2, and w2 = w1 + wRel
				
				= [-(R1*v1 + R2*v2)x   -(R2*v1)x ] [w1   wRel]', so the first matrix is the Jacobian.
				The first entry is the cross product matrix of the vector (in 'global' coordinates) from the
				origin of link 1 to p, and the second entry is the vector (in 'global' coordinates) from
				the origin of link 2 to p (and therein lies the general way of writing this).
		*/
		public void computeJointTorquesEquivalentToForce(CWJoint start, Vector3 pLocal, Vector3 fGlobal, CWJoint end){
			//starting from the start joint, going towards the end joint, get the origin of each link, in world coordinates,
			//and compute the vector to the global coordinates of pLocal.
		
			CWJoint currentJoint = start;
			Vector3 tmpV;
			Vector3 pGlobal = start.getChild().getWorldCoordinatesPoint(pLocal);
			while (currentJoint != end){
				//if (currentJoint == null)
					//throwError("VirtualModelController::computeJointTorquesEquivalentToForce --> end was not a parent of start...");
				tmpV = pGlobal - currentJoint.getParent().getWorldCoordinatesPoint(currentJoint.getParentJointPosition());
				Vector3 tmpT = Vector3.Cross(tmpV, fGlobal);
				torques[currentJoint.getId()] = ((float)torques[currentJoint.getId()]) - tmpT.z;
				currentJoint = currentJoint.getParent().getParentJoint();
			}
			
			//and we just have to do it once more for the end joint, if it's not NULL
			if (end != null){
				
				tmpV = pGlobal - currentJoint.getParent().getWorldCoordinatesPoint(currentJoint.getParentJointPosition());
				//torques[currentJoint.getId()] = ((float)torques[currentJoint.getId()]) - Vector3.Cross(tmpV, fGlobal).z;
			}
		}
	}
}