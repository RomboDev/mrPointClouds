/*
 * Copyright 2010-2016 by RomboStudios
 * All rights reserved.
 ******************************************************************************

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the software's owners nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 * Created:	Augutst 15, 2011
 */

#ifndef __PCLOUD_AMBIENTOCCLUSION__
#define __PCLOUD_AMBIENTOCCLUSION__


namespace cpc
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Ambient Occlusion ///////////////////////////////////////////////////////////////////////////////
template<typename DataType, class PListType, class iOctreeNodeType, class iCache>
class AmbientOcclusion_Impl
{

	enum{ 
		NODESIZE	= PListType::NODESIZE,
		AOSLOT		= PListType::NODESIZE -DATA_RESULT_SLOT,
		AOPASSSLOT	= PListType::NODESIZE -DATA_RESULT_SLOT +1
	};


#if	__SSE2_ENHANCED__

	////////////////////////////////////////////////////////////////////////////////////////////////
	static void 
	visibleQuadAO_SSE(	const iVector3 &p, const iVector3 &n,							// point and normals
						const iVector3 &v0, const iVector3 &v1, const iVector3 &v2,		// vertices
						iVector3 &q0, iVector3 &q1, iVector3 &q2, iVector3 &q3	)		// quad output
	{
		const DataType c = n.dot(p);

		// Compute the signed distances from the vts to the plane
		iVector3 ndots( n.dot(v0), n.dot(v1), n.dot(v2) );
		iVector3 sd( ndots-c );
		
		sd = _mm_mul_ps(sd, _mm_and_ps(_mm_cmpgt_ps(_mmx_fabsf_ps(sd), M128_CF4_00006), M128_ONE));
		//if(fabs(sd[0]) <= epsilon) sd[0] = 0;
		//if(fabs(sd[1]) <= epsilon) sd[1] = 0;
		//if(fabs(sd[2]) <= epsilon) sd[2] = 0;

		iVector3 thold;
		iVector3 thold2;

		if(sd[0] > 0)
		{
			if(sd[1] > 0)
			{
				if(sd[2] > 0)			// +++
				{
					q0 = v0;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
				else if(sd[2] < 0)		// ++-
				{
					q0 = v0;
					q1 = v1;
					thold	= (v2-v1) * (sd[1]/(sd[1]-sd[2]));
					thold2	= (v2-v0) * (sd[0]/(sd[0]-sd[2]));
					q2 = v1+ thold;
					q3 = v0+ thold2;
				}
				else					// ++0
				{
					q0 = v0;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
			}
			else if(sd[1] < 0)
			{
				if(sd[2] > 0)			// +-+
				{
					q0 = v0;
					thold	= (v1-v0) * (sd[0]/(sd[0]-sd[1]));
					thold2	= (v2-v1) * (sd[1]/(sd[1]-sd[2]));
					q1 = v0+ thold;
					q2 = v1+ thold2;
					q3 = v2;
				}
				else if(sd[2] < 0)		// +--
				{
					q0 = v0;
					thold	= (v1-v0) * (sd[0]/(sd[0]-sd[1]));
					thold2	= (v2-v0) * (sd[0]/(sd[0]-sd[2]));
					q1 = v0 + thold;
					q2 = v0 + thold2;
					q3 = q2;
				}
				else					// +-0
				{
					q0 = v0;
					thold	= (v1-v0);
					thold2	= (sd[0]/(sd[0]-sd[1]));
					q1 = v0 + thold*thold2;
					q2 = v2;
					q3 = q2;
				}
			}
			else
			{
				if(sd[2] > 0)			// +0+
				{
					q0 = v0;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
				else if(sd[2] < 0)		// +0-
				{
					q0 = v0;
					q1 = v1;
					thold	= (v2-v0);
					thold2	= (sd[0]/(sd[0]-sd[2]));
					q2 = v0 + thold*thold2;
					q3 = q2;
				}
				else					// +00
				{
					q0 = v0;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
			}
		}
		else if(sd[0] < 0)
		{
			if(sd[1] > 0)
			{
				if(sd[2] > 0)			// -++
				{
					thold	= (v1-v0) * (sd[0]/(sd[0]-sd[1]));
					thold2	= (v2-v0) * (sd[0]/(sd[0]-sd[2]));
					q0 = v0 + thold;
					q1 = v1;
					q2 = v2;
					q3 = v0 + thold2;
				}
				else if(sd[2] < 0)		// -+-
				{
					thold	= (v1-v0) * (sd[0]/(sd[0]-sd[1]));
					thold2	= (v2-v1) * (sd[1]/(sd[1]-sd[2]));
					q0 = v0 + thold;
					q1 = v1;
					q2 = v1 + thold2;
					q3 = q2;
				}
				else					// -+0
				{
					thold	= (v1-v0);
					thold2	= (sd[0]/(sd[0]-sd[1]));
					q0 = v0 + thold*thold2;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
			}
			else if(sd[1] < 0)
			{
				if(sd[2] > 0)			// --+
				{
					thold = (v2-v0) * (sd[0]/(sd[0]-sd[2]));
					thold2 = (v2-v1) * (sd[1]/(sd[1]-sd[2]));
					q0 = v0 + thold;
					q1 = v1 + thold2;
					q2 = v2;
					q3 = q2;
				}
				else if(sd[2] < 0)		// ---
				q0 = q1 = q2 = q3 = p;
				else					// --0
				q0 = q1 = q2 = q3 = p;
			}
			else
			{
				if(sd[2] > 0)			// -0+
				{
					thold	= (v2-v0);
					thold2	= (sd[0]/(sd[0]-sd[2]));
					q0 = v0 + thold*thold2;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
				else if(sd[2] < 0)		// -0-
				q0 = q1 = q2 = q3 = p;
				else					// -00
				q0 = q1 = q2 = q3 = p;
			}
		}
		else
		{
			if(sd[1] > 0)
			{
				if(sd[2] > 0)			// 0++
				{
				
					q0 = v0;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
				else if(sd[2] < 0)		// 0+-
				{
					q0 = v0;
					q1 = v1;
					thold = (v2-v1) * (sd[1]/(sd[1]-sd[2]));
					q2 = v1 + thold;
					q3 = q2;
				}
				else					// 0+0
				{
					q0 = v0;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
			}
			else if(sd[1] < 0)
			{
				if(sd[2] > 0)			// 0-+
				{
					q0 = v0;
					thold	= (v2-v1);
					thold2	= (sd[1]/(sd[1]-sd[2]));
					q1 = v1 + thold*thold2;
					q2 = v2;
					q3 = q2;
				}
				else if(sd[2] < 0)		// 0--
				q0 = q1 = q2 = q3 = p;
				else					// 0-0
				q0 = q1 = q2 = q3 = p;
			}
			else
			{
				if(sd[2] > 0)			// 00+
				{
					q0 = v0;
					q1 = v1;
					q2 = v2;
					q3 = q2;
				}
				else if(sd[2] < 0)		// 00-
				q0 = q1 = q2 = q3 = p;
				else					// 000
				q0 = q1 = q2 = q3 = p;
			}
		}
	}

	// clamp( ... )

	// ComputeFormFactor ///////////////////////////////////////////////////////////////////////////
	static inline
	float computeFormFactor_SSE(	const iVector3 &p, const iVector3 &n,
									const iVector3 &q0, const iVector3 &q1,
									const iVector3 &q2, const iVector3 &q3	)
	{
		iVector3 r0 = q0 - p;
		iVector3 r1 = q1 - p;
		iVector3 r2 = q2 - p;
		iVector3 r3 = q3 - p;
		r0 = r0.normalize();
		r1 = r1.normalize();
		r2 = r2.normalize();
		r3 = r3.normalize();

		iVector3 g0 = r1.cross(r0).normalize();
		iVector3 g1 = r2.cross(r1).normalize();
		iVector3 g2 = r3.cross(r2).normalize();
		iVector3 g3 = r0.cross(r3).normalize();
		
		iVector3 rdots( r0.dot(r1), r1.dot(r2), r2.dot(r3), r3.dot(r0) );
		iVector3 ndots( n.dot(g0), n.dot(g1), n.dot(g2), n.dot(g3) );
		rdots = _mm_max_ps(M128_MINUSONE, _mm_min_ps(M128_ONE, rdots) );
		ndots = _mm_max_ps(M128_MINUSONE, _mm_min_ps(M128_ONE, ndots) );

		iVector3 a(_mm_acos_ps(rdots));
		iVector3 contrib( _mm_mul_ps(_mm_acos_ps(rdots), ndots) );

		iVector3 result ( _mm_hadd_ps(contrib, contrib) );
		result = _mm_hadd_ps(result, result);

		result = _mm_mul_ps( _mm_mul_ps(M128_HALF, result), M128_INVPI );
		return _mm_cvtss_f32( _mm_max_ps(M128_ZERO, result) );
	}


	// Form Factor /////////////////////////////////////////////////////////////////////////////////
	static inline
	float computeFormFactor_SSE( const iVector3 &p, const iVector3 &n,
							 const iVector3 &v0, const iVector3 &v1, const iVector3 &v2 )
	{
		iVector3 q0,q1,q2,q3;
		visibleQuadAO_SSE(	p,n,
						v0, v1, v2,
						q0, q1, q2, q3	);

		return computeFormFactor_SSE(p,n,q0,q1,q2,q3);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////
	// SSE version
	static INLINE float 
	ao_solidAngle_SSE(	const iVector3& iPos, const iVector3& iDir, const float d2,
						const iVector3& eDir, const float eArea	)
	{	
		iVector3 rDot	( iDir.dotX( iPos ) );
		iVector3 eDot	( eDir.dotX( -iPos ) );

		iVector3 pkgDots( eDot.swapLhCoords( rDot ) );							//eO,e1,r0,r1
		iVector3 pkgDotsSat( pkgDots.clampMin(M128_ONE).clampMax(M128_ZERO) );	//eSat0,eSat1,rSat0,rSat1

		return ( (eArea * pkgDotsSat[0] * pkgDotsSat[2]) / (d2 + eArea * inv_pi) ) * inv_pi;
	}
#endif

	static INLINE float 
	ao_solidAngle(	const iVector3& iPos, const iVector3& iDir, const float d2,
					const iVector3& eDir, const float& eArea	)
	{
		float result =	( 
							(												// first term
								eArea									*
								ao_saturate(eDir.dot(iVector3(-iPos)))	*
								ao_saturate(iDir.dot(iPos)) 
							) 

							/ (d2 + eArea * inv_pi)							// second term
						) 
						* inv_pi;											// ...

		return result;
	}


/// ////////////////////////////////////////////////////////////////////////////////////////////////
public:
	typedef PListType			PListType;
	typedef iOctreeNodeType		iOctreeNodeType;
	typedef iCache				iCache;

	static const bool HasMultiPass = true;


	////////////////////////////////////////////////////////////////////////////////////////////////
	static void 
	ComputeAOcclusion(	const iVector3&			XiPos,
						const iVector3&			XiDir,

						PListType 				* pointlist,
						iOctreeNodeType			* node,

						const int				level,

						const float				epsilon,
						const float				distAttenuation,
						const bool				trivis,
							
						float&					occlusion,
						const float				rRadius=0.0f )
	{

		float dAtt = 1.0;
		float contribution;


		// We have 3 situations:
		// - overlapping disks (rArea>dist)
		// - close to receiver disks (in leaf node)
		// - clustered disk for far from receiver

		if (node->isleaf)											// Compute point-to-point occlusion
		{
			int firstpoint= node->firstpoint;
			int lastpoint = firstpoint + node->npoints;

			for(int i = firstpoint; i < lastpoint; i++) 
			{
				iVector3 XePos		( pointlist->template GetNodeVectorAt<DATA_SLOT_POS>(i) );
				iVector3 XeNormal	( pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>(i) );

				if( XiPos.dot(XeNormal) <= XePos.dot(XeNormal) )		// Are we behind the item?
				continue;


				iVector3 Xv			( XePos - XiPos );

				float Xdist = Xv.dot(Xv) + 1e-16f;					// squared distance
				float sq_dist = sqrtf(Xdist);
				Xv /= sq_dist;										// reciprocal distance

		
				////////////////////////////////////////////////////////////////////////////////
				//if(trivis && (dist < rRadius))
				//{													// For overlapping disk ..
				//													// compute formfactor from trivis
				//	contribution	= computeFormFactor(	iPos, iDir, 
				//											pointlist->GetNodeT0(i), 
				//											pointlist->GetNodeT1(i), 
				//											pointlist->GetNodeT2(i)	);
			
				//}else 
				contribution	= ao_solidAngle(Xv, XiDir, Xdist, XeNormal, pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>(i));

				contribution *= pointlist->template GetNodeScalarAt< AOPASSSLOT >(i);			
																	// For ao passes ...
																	// we blend with previous ao
																	// to mitigate double shadowing
																	// ie. this caster points may be already
																	// shadowed by another caster point on top
																	// we know that by looking at its ao value

				if(distAttenuation) {
					dAtt = 1.0f + distAttenuation * Xdist;
					contribution /= dAtt;							// Distance attenuation
				}

				occlusion += contribution;							// Accumulate occlusion on the receiver
			}
		}
		else	// not a leaf //////////////////////////////////////////////////////////////////////////
		{

			// Compute the minimum distance between the point and the node bbox.
			// The distance is 0 if the point is inside the bbox.
			float dist2 = XiPos.sqr_dist_bbox( node->bbox_min,  node->bbox_max);

			float omega=0.0f;
			if (dist2 > 0.0f)
			omega = node->sumArea / dist2;

			// error too large
			if (dist2 == 0.0f || omega > epsilon) 
			{ 
				// recursively visit children
				for (int c = 0; c < 8; c++) 
				if(node->children[c]) 
				ComputeAOcclusion(	XiPos, XiDir,					// Traverse hierarchy
									
									pointlist,
									node->children[c],

									level+1,
									epsilon, 
									distAttenuation,
									trivis,
									occlusion,
									rRadius	);

			}
			else { // error low enough: use node as is				// Cluster node

				iVector3 XePos = node->centroid;
				iVector3 XeNormal = node->ncentroid;

				if( XiPos.dot(XeNormal) > XePos.dot(XeNormal) )		// Are we behind the item?
				{
					iVector3 Xv			( XePos - XiPos );

					float Xdist = Xv.dot(Xv) + 1e-16f;
					float sq_dist = sqrtf(Xdist);
					Xv /= sq_dist;

					// get cluster contribution
					contribution = ao_solidAngle(Xv, XiDir, Xdist, XeNormal, node->sumArea);

					contribution *= node->sumAO;					// Cluster contribution

					if(distAttenuation) {
						dAtt = 1.0f + distAttenuation * Xdist ;
						contribution /= dAtt;
					}

					occlusion += contribution;						// Accumulate cluster ao 
				}													// on the receiver point
			}
		}
	}

#if	__SSE2_ENHANCED__

	////////////////////////////////////////////////////////////////////////////////////////////////
	static void 
	ComputeAOcclusionSSE(	const iVector3&			XiPos,
							const iVector3&			XiDir,

							PListType 				* pointlist,
							iOctreeNodeType			* node,

							const int				level,

							const float				epsilon,
							const float				distAttenuation,
							const bool				trivis,
							
							float&					occlusion,
							const float				rRadius=0.0f )
	{

		float dAtt = 1.0;
		float contribution;


		// There're 3 possible situations here :
		// - overlapping disks (rArea>dist)
		// - close to receiver disks (in leaf node)
		// - clustered disk for far from receiver

		if (node->isleaf)											// Compute point-to-point occlusion
		{
			int lastpoint = node->firstpoint + node->npoints;
			for (int i = node->firstpoint; i < lastpoint; i++)
			{
				iVector3 XePos		( pointlist->template GetNodeVectorAt<DATA_SLOT_POS>(i) );
				iVector3 XeNormal	( pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>(i) );

				if(XiPos.dotX( XeNormal ) <= XePos.dotX( XeNormal ))
				continue;


				iVector3 Xv			( XePos - XiPos );
				iVector3 Xdist		( Xv.dotX(Xv) );				// Reciprocal distance
				iVector3 Xsq_dist	( Xdist.sqrtX() );
				Xv /= Xsq_dist;


				////////////////////////////////////////////////////////////////////////////////
				if(trivis && (Xdist[0] < rRadius))
				{													// Overlapping surfels ...
																	// compute formfactor from trivis
					contribution	= computeFormFactor_SSE (	XiPos, XiDir,
																pointlist->template GetNodeVectorAt<DATA_SLOT_T0>(i),
																pointlist->template GetNodeVectorAt<DATA_SLOT_T1>(i),
																pointlist->template GetNodeVectorAt<DATA_SLOT_T2>(i)
															);					
				}else
				contribution = ao_solidAngle_SSE(Xv, XiDir, Xdist[0], XeNormal, pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>(i));

				contribution *= pointlist->template GetNodeScalarAt< AOPASSSLOT >(i);
																	// Pass contribution

				if(distAttenuation) {
					dAtt = 1.0f + distAttenuation * Xdist[0];
					contribution /= dAtt;							// Distance attenuation
				}

				occlusion += contribution;							// Accumulate occlusion on the receiver
			}
		}
		else// not a leaf //////////////////////////////////////////////////////////////////////////
		{
			// Compute the minimum distance between the point and the node bbox.
			// The distance is 0 if the point is inside the bbox.
			//float dist2 = iVector3::sqr_dist_bboxSSE(XiPos, node->bbox_min,  node->bbox_max);
			float dist2 = XiPos.sqr_dist_bbox( node->bbox_min,  node->bbox_max);

			float omega=0.0f;
			if (dist2 > 0.0f)
			omega = node->sumArea / dist2;

			// /////////////////									// If ............
			if(	dist2 == 0.0f ||									// inside the node
				omega > epsilon )									// or error too large
			{ 
				// recursively visit children
				for (int c = 0; c < 8; c++) 
				if(node->children[c]) 
				ComputeAOcclusionSSE(	XiPos, XiDir,				// Traverse hierarchy

										pointlist,
										node->children[c],

										level+1,
										epsilon, 
										distAttenuation,
										trivis,
										occlusion,
										rRadius	);

			}
			else {	// error low enough: use node as is				// Cluster node

				iVector3 XePos		( __m128(node->centroid)  ); 
				iVector3 XeNormal	( __m128(node->ncentroid) ); 

				if(XiPos.dotX( XeNormal ) >= XePos.dotX( XeNormal ))
				{
					iVector3 Xv			( XePos - XiPos );
					iVector3 Xdist		( Xv.dotX(Xv) );			// Reciprocal distance
					iVector3 Xsq_dist	( Xdist.sqrtX() );
					Xv /= Xsq_dist;

					// get cluster contribution
					contribution = ao_solidAngle_SSE(Xv, XiDir, Xdist[0], XeNormal, node->sumArea);
					contribution *= node->sumAO;					// Cluster influence
				
					if(distAttenuation) {
						dAtt = 1.0f + distAttenuation * Xdist[0] ;
						contribution /= dAtt;
					}
				
					occlusion += contribution;						// Accumulate cluster ao 
				}													// on the receiver point
			}
		}

	}



	/// ////////////////////////////////////////////////////////////////////////////////////////////
	static void 
	ComputeAOcclusionSSESequentially(	const iVector3&			XiPos,
										const iVector3&			XiDir,

										PListType 				* pointlist,
										iOctreeNodeType			* root,

										const float				epsilon,
										const float				distAttenuation,
										const bool				trivis,
							
										float&					occlusion,
										const float				rRadius=0.0f )
	{

		float dAtt = 1.0;
		float contribution = 0.0f;
		
		iStackX<iOctreeNodeType*>	istack(64);
		istack.push_back(root);		//push root node to stack


		do
		{
			// stack ..............................
			iOctreeNodeType *node = istack.pop();


			// There're 3 possible situations here :
			// - overlapping disks (rArea>dist)
			// - close to receiver disks (in leaf node)
			// - clustered disk for far from receiver

			if (node->isleaf)											// Compute point-to-point occlusion
			{
				int firstpoint= node->firstpoint;
				int lastpoint = firstpoint + node->npoints;

				for (int i = firstpoint; i < lastpoint; i++)
				{
					iVector3 XePos		( pointlist->template GetNodeVectorAt<DATA_SLOT_POS>(i) );
					iVector3 XeNormal	( pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>(i) );

					if(XiPos.dotX( XeNormal ) <= XePos.dotX( XeNormal ))
					continue;


					iVector3 Xv			( XePos - XiPos );
					iVector3 Xdist		( Xv.dotX(Xv) );				// Reciprocal distance
					iVector3 Xsq_dist	( Xdist.sqrtX() );
					Xv /= Xsq_dist;


					////////////////////////////////////////////////////////////////////////////////
					if(trivis && (Xdist[0] < rRadius))
					{													// Overlapping surfels ...
																		// compute formfactor from trivis
						contribution	= computeFormFactor_SSE (	XiPos, XiDir,
																pointlist->template GetNodeVectorAt<DATA_SLOT_T0>(i),
																pointlist->template GetNodeVectorAt<DATA_SLOT_T1>(i),
																pointlist->template GetNodeVectorAt<DATA_SLOT_T2>(i)
																);					
					}else
					contribution = ao_solidAngle_SSE(Xv, XiDir, Xdist[0], XeNormal, pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>(i));

					contribution *= pointlist->template GetNodeScalarAt< AOPASSSLOT >(i);
																		// Pass contribution

					if(distAttenuation) {
						dAtt = 1.0f + distAttenuation * Xdist[0];
						contribution /= dAtt;							// Distance attenuation
					}

					occlusion += contribution;							// Accumulate occlusion on the receiver
				}
			}
			else// not a leaf //////////////////////////////////////////////////////////////////////////
			{
				// Compute the minimum distance between the point and the node bbox.
				// The distance is 0 if the point is inside the bbox.
				//float dist2 = iVector3::sqr_dist_bboxSSE(XiPos, node->bbox_min,  node->bbox_max);
				float dist2 = XiPos.sqr_dist_bbox( node->bbox_min,  node->bbox_max);

				float omega=0.0f;
				if (dist2 > 0.0f)
				omega = node->sumArea / dist2;

				// error too large ...
				if (dist2 == 0.0f || omega > epsilon) 
				{ 
					// ... stack next children nodes to visit
					for (int c = 0; c < 8; c++)
					if(node->children[c]) 
					istack.push_back(node->children[c]);				// Push Node to stack

				}
				else {	// error low enough: use node as is				// Cluster node

					iVector3 XePos		( __m128(node->centroid)  ); 
					iVector3 XeNormal	( __m128(node->ncentroid) ); 

					if(XiPos.dotX( XeNormal ) >= XePos.dotX( XeNormal ))
					{
						iVector3 Xv			( XePos - XiPos );
						iVector3 Xdist		( Xv.dotX(Xv) );			// Reciprocal distance
						iVector3 Xsq_dist	( Xdist.sqrtX() );
						Xv /= Xsq_dist;

						// get cluster contribution
						contribution = ao_solidAngle_SSE(Xv, XiDir, Xdist[0], XeNormal, node->sumArea);
						contribution *= node->sumAO;					// Cluster influence
				
						if(distAttenuation) {
							dAtt = 1.0f + distAttenuation * Xdist[0] ;
							contribution /= dAtt;
						}
				
						occlusion += contribution;						// Accumulate cluster ao 
					}													// on the receiver point
				}
			}
		}while( !istack.empty() );	////////////
		istack.clear();
	}
#endif	//SSEenhanced


	////////////////////////////////////////////////////////////////////////////////////////////////
	static bool
	doThread_CG_EffectsPass(chunk_idx		*chunk,

							PListType		* pointlist,
							iOctreeNodeType	* root,

							iCache			* cache,

							bool			isMainThread = false )
	{
		// local variables
		iVector3 p;
		iVector3 n;

		float ao;

		float trivis_threshold;
		bool do_trivis = cache->trivis;


		for (iUint o = chunk->startValue; o < chunk->endValue; o++)
		{
			if(isMainThread && ABORTCALLED)	// abortation is ctrled by main thread ..
			return true;					// breaking here, it will stop the other threads
			

			// if we don't have occlusion contribution no need for a second pass..
			// multiple passes are just to remove double shadowing when in ao mode
			if( cache->current_pass && (pointlist->template GetNodeScalarAt<AOSLOT>(o) > 0.9f) ) 
			continue;


			//  parameters
			p = pointlist->template GetNodeVectorAt<DATA_SLOT_POS>(o);
			n = pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>(o);
			ao = 0.0f;	// reset for accumulation


			// Triangle visibility check
			//do_trivis = miFALSE;			// model trivis_threshold with error_threshold

			// Compute occlusion for the point /////////////////////////////////////////////////////
#if	__SSE2_ENHANCED__
			ComputeAOcclusionSSESequentially
								(	
									p,
									n,

									pointlist,
									root,

									cache->maxsolidangle,
									cache->distAtt,
									do_trivis,

									ao,							// return with occlusion at point
									sqrt(pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>(o)/(float)f_pi)
								);
#else
			ComputeAOcclusion(	
									p,
									n,

									pointlist,
									root,

									0,
									cache->maxsolidangle,
									cache->distAtt,
									do_trivis,
									ao,							// return with occlusion at point
									sqrt(pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>(o)/(float)f_pi)
								);
#endif
			/**/

			//if( cache->current_pass == 0 )
			//_mm_stream_ps( &(pointlist->template GetNodeVectorRef<AOSLOT>(o)), iVector3(ao).minusOne().saturate());
			//else	
			pointlist->template SetNodeScalarAt<AOSLOT>(o, ao_saturate(1.f - ao));
			
			//pointlist->template SetNodeVectorAt<AOSLOT>(o, iVector3(ao).minusOne().saturate());	// add to pointlist
			//pointlist->template SetNodeScalarAt<AOSLOT>(o, ao_saturate(1.f - ao));	// add to pointlist

			// print per-point stats
			if(cache->verbose>=2 && (o % chunk->frequency == 0) && isMainThread)
			{
				PRINTME( "--- occlusion at point %u = (%f)\n",
				o, pointlist->template GetNodeScalarAt<AOSLOT>(o));
			}
		}
		return false;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	static void 
	UpdateAOPass(	PListType			* pointlist,
					iOctreeNodeType		* node,		

					const int			level,
					const bool			convergence = false,
					const int			verbose	= 0)
	{

		if(convergence && level==0)
		{
			if(verbose>=2)
				PRINTME("Occlusion Passes: Convergence");

			for(iUint p=0; p < node->npoints; p++)
			{
				float AO		= pointlist->template GetNodeScalarAt< AOSLOT >(p);
				float last_AO	= pointlist->template GetNodeScalarAt< AOPASSSLOT >(p);

				float m = ds_MIN(last_AO, AO);
				float M = ds_MAX(last_AO, AO);

				float final_ao = 0.70f * m + 0.30f * M;

				pointlist->template SetNodeScalarAt<AOSLOT>(p, final_ao);	// add  to point list

				if(verbose>=2 && p%(node->npoints/8)==0)
				PRINTME("--- last occlusion at point %u = (%f), final AO = (%f)",
				p, last_AO, AO);
			}

			return;					// final pass, we had only to converve AO, returning ...........
		}


		////////////////////////////////////////////////////////////////////////////////////////////
		if(level==0)				// for every initial pass call, plist is updated with last AO
		for(iUint p=0; p<node->npoints; p++){

		float AO = pointlist->template GetNodeScalarAt< AOSLOT >(p);
		pointlist->template SetNodeScalarAt< AOPASSSLOT >(p, AO);// add  to point list
		}

		bool leaf = node->isleaf;	////////////////////////////////////////////////////////////////
		if(leaf) return;			// leaf points contribution is done on pointlist directly,
									// we need to accumulate AO passes for octree nodes only

		iOctreeNodeType* child;
		int lastpoint;
		float nodeAO;
		
		for(int i=0; i<8; i++)									// step into hierarchy
		{
			nodeAO = 0.0f;
			child = node->children[i];							// child
			if (child)
			{
				lastpoint = child->firstpoint + child->npoints;

				for (int o = child->firstpoint; o < lastpoint; o++)
				nodeAO += pointlist->template GetNodeScalarAt< AOPASSSLOT >(o);
																// accumulate AO

				nodeAO *= 1.0f/child->npoints;					// average AO
				child->sumAO = nodeAO;							// update octree


				// Recurse /////////////////////////////////////////////////////////////////////////
				UpdateAOPass (pointlist, child, level+1, convergence, verbose);	////////////////////
			}
		}
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	static void
	LayoutPointlistData( PListType * _pointlist )
	{
		typedef PListType::_iDataFlag _iDataFlagT;
		_iDataFlagT newflag;

		newflag.name = "dir";
		newflag.type = 3;
		newflag.offset = DATA_SLOT_DIR;
		_pointlist->AddLocalField(newflag);

		newflag.name = "area";
		newflag.type = 2;
		newflag.offset = DATA_SLOT_AREA;
		_pointlist->AddLocalField(newflag);

/*		newflag.name = "t0";
		newflag.type = 3;
		newflag.offset = DATA_SLOT_T0;
		_pointlist->AddLocalField(newflag);

		newflag.name = "t1";
		newflag.type = 3;
		newflag.offset = DATA_SLOT_T1;
		_pointlist->AddLocalField(newflag);

		newflag.name = "t2";
		newflag.type = 3;
		newflag.offset = DATA_SLOT_T2;
		_pointlist->AddLocalField(newflag);
*/
		newflag.name = "ao";
		newflag.type = 2;
		newflag.offset = AOSLOT;
		_pointlist->AddLocalField(newflag);

//
		newflag.name = "maptype";
		newflag.type = 1;
		newflag.offset = -445566;			//occlusion integer identifier
		_pointlist->AddGlobalField(newflag);
	}
};

}	//end namespace
#endif __PCLOUD_AMBIENTOCCLUSION__