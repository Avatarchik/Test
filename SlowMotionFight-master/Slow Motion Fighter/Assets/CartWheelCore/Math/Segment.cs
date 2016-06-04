using UnityEngine;
using System.Collections;

namespace CartWheelCore.Math{
	public class Segment {
	
		public Vector3 a;
		public Vector3 b;
		
		public Segment(){
			a = new Vector3();
			b = new Vector3();
		}
		
		public Segment(Vector3 a_, Vector3 b_){
			this.a = a_;
			this.b = b_;
		}
		
		/**
			Copy constructor
		*/
		public Segment(Segment other){
			this.a = other.a;
			this.b = other.b;
		}
		
		/**
			This method returns the point on the current segment that is closest to the point c that is passed in as a paremter.
		*/
		public void getClosestPointTo(Vector3 c, ref Vector3 result){
			//we'll return the point d that belongs to the segment, such that: cd . ab = 0
			//BUT, we have to make sure that this point belongs to the segment. Otherwise, we'll return the segment's end point that is closest to D.
			Vector3 v = (b - a);
		
			float len_squared = Vector3.Dot(v, v);
			//if a==b, it means either of the points can qualify as the closest point
			if (len_squared>-0.0001f && len_squared<0.0001f){
				result = a;
				return;
			}
			
			float mu = Vector3.Dot((c - a), v) / len_squared;
			if (mu<0)
				mu = 0;
			if (mu>1)
				mu = 1;
			//the point d is at: a + mu * ab
			result = (a + v * mu);
		}
		
		/**
			This method returns the segment that connects the closest pair of points - one on the current segment, and one on the segment that is passed in. The
			'a' point of the resulting segment will lie on the current segment, while the 'b' point lies on the segment that is passed in as a parameter.
		*/
		public void getShortestSegmentTo(Segment other, ref Segment result) {
			//MAIN IDEA: the resulting segment should be perpendicular to both of the original segments. Let point c belong to the current segment, and d belong to
			//the other segment. Then a1b1.cd = 0 and a2b2.cd = 0. From these two equations with two unknowns, we need to get the mu_c and mu_d parameters
			//that will let us compute the points c and d. Of course, we need to make sure that they lie on the segments, or adjust the result if they don't.
		
			//unfortunately, there are quite a few cases we need to take care of. Here it is:
			Vector3 tmp1, tmp2, tmp3;
			tmp1 = (other.a - this.a);
			tmp2 = (this.b - this.a);
			tmp3 = (other.b - other.a);
			float A = Vector3.Dot(tmp1, tmp2);
			float B = Vector3.Dot(tmp2, tmp3);
			float C = Vector3.Dot(tmp2, tmp2);
			float D = Vector3.Dot(tmp3, tmp3);
			float E = Vector3.Dot(tmp1, tmp3);
		
			//now a few special cases:
			if (C>-0.0001f && C<0.0001f){
				//current segment has 0 length
				result.a = this.a;
				other.getClosestPointTo(this.a, ref result.b);
				return;
			}
			if (D>-0.0001f && D<0.0001f){
				//other segment has 0 length
				this.getClosestPointTo(other.a, ref result.a);
				result.b = other.a;
				return;
			}
		
			float F = C*D - B*B;
			if (F>-0.0001f && F<0.0001f){
				//this means that the two segments are coplanar and parallel. In this case, there are
				//multiple segments that are perpendicular to the two segments (lines before the truncation really).
		
				//we need to get the projection of the other segment's end point on the current segment
				float mu_a2 = Vector3.Dot(tmp1, tmp2) / Vector3.Dot(tmp2, tmp2);
				float mu_b2 = Vector3.Dot((other.b - a), tmp2) / Vector3.Dot(tmp2, tmp2);
		
		
				//we are now interested in the parts of the segments that are in common between the two input segments
				if (mu_a2<0) mu_a2 = 0;
				if (mu_a2>1) mu_a2 = 1;
				if (mu_b2<0) mu_b2 = 0;
				if (mu_b2>1) mu_b2 = 1;
		
				//closest point on the current segment must lie at the midpoint of mu_a2 and mu_b2
				result.a = (this.a + tmp2 * (mu_a2 + mu_b2)/2.0f);
				other.getClosestPointTo(result.a, ref result.b);
				return;
			}
		
			//ok, now we'll find the general solution for two lines in space:
			float mu_c = (A*D - E*B) / (C*D-B*B);
			float mu_d = (mu_c*B - E) / D;
		
			//if the D point or the C point lie outside their respective segments, clamp the values
			if (mu_c<0) mu_c = 0;
			if (mu_c>1) mu_c = 1;
			if (mu_d<0) mu_d = 0;
			if (mu_d>1) mu_d = 1;
		
			result.a = (this.a + tmp2 * mu_c);
			result.b = (other.a + tmp3 * mu_d);
		}
	}
}
