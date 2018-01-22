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
 
 *****************************************************************************
 */

#ifndef __PCLOUD_SUBSURFACESCATTERING__
#define __PCLOUD_SUBSURFACESCATTERING__


namespace cpc
{

static const int steps = 10000;


////////////////////////////////////////////////////////////////////////////////////////////////////
// Ambient Occlusion ///////////////////////////////////////////////////////////////////////////////
template<typename DataType, class PListType, class iOctreeNodeType, class iCache>
class SubSurfaceScattering_Impl
{

	enum{ 
		NODESIZE	= PListType::NODESIZE,
		SSSSLOT		= PListType::NODESIZE -DATA_RESULT_SLOT
	};


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Compute diffuse color (BRDF albedo) Rd
	static DataType
	ComputeRd(DataType A, DataType alpha_prime)
	{
		DataType s, a, b, c, Rd;

		s = sqrtf(3.0f * (1.0f - alpha_prime));
		a = 0.5f * alpha_prime;
		b = 1.0f + expf(- 4.0f/3.0f * A * s);
		c = expf(-s);
		Rd = a * b * c;

		return Rd;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Interpolate alpha' (from albedo Rd)
	static inline DataType
	InterpolateAlphaPrime(DataType *alphaPTable, const DataType& Rd)
	{
		int i = (int)(Rd * steps);
		DataType w = Rd * steps - i;
		DataType alpha_prime = (1.0f - w) * alphaPTable[i] + w * alphaPTable[i+1];

		return alpha_prime;
	}

#if	__SSE2_ENHANCED__

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter conversion
	static void
	ConvertParamsSSE(	const iVector3& Rd,			// diffuse color (BRDF albedo)
						const iVector3& dmfp,		// diffuse mean free path (aka. l_d)
						const DataType unitlength,	// multiplier on dmfp
						DataType *alphaPTable,

						iVector3 &alpha_prime,		// reduced scattering albedo 
						iVector3 &sigma_tr,			// effective transport extinction coeff
						iVector3 &l_u )				// mean free path length
	{
		iVector3 l_d ( dmfp );
		l_d *= unitlength;

		iVector3 mOne( M128_ONE );
		iVector3 mThree( M128_THREE );

		// Interpolate alpha' (from albedo Rd)
		alpha_prime[0] = InterpolateAlphaPrime(alphaPTable, Rd[0]);
		alpha_prime[1] = InterpolateAlphaPrime(alphaPTable, Rd[1]);
		alpha_prime[2] = InterpolateAlphaPrime(alphaPTable, Rd[2]);

		// compute effective transport extinction coeff sigma_tr (from dmfp l_d)
		sigma_tr = mOne / l_d;

		// Compute mean free path length l_u from diffuse mean free path l_d:
		// l_u = 1 / sigma'_t  with  sigma'_t = 1 / (l_d * sqrt(3 * (1 - alpha'))) 
		// gives  l_u = l_d * sqrt(3 * (1 - alpha')).
		l_u = l_d * (mThree * (mOne - alpha_prime)).sqrtX();
	}


	static inline iVector3
	ComputeBSSRDF(	iVector3 r,				// distance from illum point P_i to point P_o
					iVector3 sigma_tr,		// effective transport extinction coeff
					iVector3 l_u,			// mean free path length
					iVector3 z_r,			// depth of "real" dipole light source (= l_u)
					iVector3 z_v )			// height of virtual dipole light source
	{
		if(r[0] < l_u[0]) r[0] = l_u[0];
		if(r[1] < l_u[1]) r[1] = l_u[1];
		if(r[2] < l_u[2]) r[2] = l_u[2];

		iVector3 rr(r*r);
		iVector3 d_r( rr + z_r*z_r );		// distance to "real" dipole source
		iVector3 d_v( rr + z_v*z_v );		// distance to virtual dipole source
		d_r = d_r.sqrtX();
		d_v = d_v.sqrtX();

		iVector3 c1 ( z_r * (sigma_tr + iVector3(M128_ONE)/d_r) );
		iVector3 c2 ( z_v * (sigma_tr + iVector3(M128_ONE)/d_v) );

		iVector3 factor (  c1 * iVector3(_mmx_exp_ps(-sigma_tr * d_r)) / (d_r*d_r) );
		factor			+= c2 * iVector3(_mmx_exp_ps(-sigma_tr * d_v)) / (d_v*d_v);
		factor			*= 1.0f / four_pi;

		return factor;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Recursively traverse the octree and add up the radiosity due to all the nodes
	static void
	RecTraverseTree(const iVector3& P_o,			// surfel position

					const iVector3& sigma_tr,		// sss params
					const iVector3& alpha_prime,
					const iVector3& l_u, 
					const iVector3& z_r, 
					const iVector3& z_v,

					float maxsolidangle,			// precision

					PListType 			* pointlist,
					iOctreeNodeType		* node, int level,

					iVector3& M_o )					// return with SSS color
	{

		iOctreeNodeType *child;
		iVector3 Rd;								// diffuse BSSRDF reflection coeffs
		iVector3 rad_t;								// baked irradiance

		float omega = 0.0;							// max solid angle
		float area;
		float dist;
		float dist2;
		int lastpoint, c, i;
		bool leaf = node->isleaf;

		// If this is a leaf node: loop over the individual points in it
		if(leaf) 
		{
			lastpoint = node->firstpoint + node->npoints;

			for (i = node->firstpoint; i < lastpoint; i++) 
			{
				// Compute diffusion from data point P_i to data point P_o
				const iVector3& P_i = pointlist->template GetNodeVectorAt< DATA_SLOT_POS >(i);
				rad_t	= pointlist->template GetNodeVectorAt< DATA_SLOT_IRRAD >(i);
				area	= pointlist->template GetNodeScalarAt< DATA_SLOT_AREA >(i);

				dist = P_i.distanceX( P_o );

				Rd = ComputeBSSRDF(iVector3(dist), sigma_tr, l_u, z_r, z_v);
				Rd *= alpha_prime;
				Rd *= area;

				// Increment the radiant exitance (radiosity) at point P_o.
				// See [Jensen02] equation (13).
				// Note that rad_t can be negative, so M_o can also turn negative.
				// Also note that we omit the F_dt term here and in equation (14),
				// corresponding to diffuse radiance, as suggested right below
				// equation (14).
				// Furthermore, the alpha' term is omitted since it shouldn't be
				// there (according to Christophe Hery).  If it were, it would
				// cancel out the alpha' term in the BSSRDF.
				M_o += Rd * rad_t;
			}

		} else 
		{

			// Compute the minimum distance between the point and the node bbox.
			// The distance is 0 if the point is inside the bbox.
			dist2 = P_o.sqr_dist_bbox( node->bbox_min,  node->bbox_max);

			// Compute the maximum solid angle spanned by the points in this node.
			// ([Jensen02] p. 579 suggests estimating this using the distance
			// to the centroid of the points.  But that fails if e[1]. the points
			// are in two clumps, one of which is very close to the point P_o.) 
			if (dist2 > 0.0f)
			omega = node->sumArea / dist2;

			// To recurse or not to recurse, that is the question ...
			// We decide using the heuristic of [Jensen02] p. 579, except that
			// the solid angle is computed more conservatively: distance to
			// the bbox, not to the centroid.
			if (dist2 == 0.0f || omega > maxsolidangle) 
			{	// angle too large
				// Recursively visit children
				for (c = 0; c < 8; c++) 
				{
					child = node->children[c];
					if (child) 
					RecTraverseTree(	P_o,

										sigma_tr, 
										alpha_prime,
										l_u, 
										z_r, 
										z_v,
										maxsolidangle, 

										pointlist,
										child, level+1,

										M_o );
				}

			} else // error low enough: use node as is
			{ 
				dist = P_o.distanceX( node->centroid );

				// Compute the BSSRDF coefficients
				Rd = ComputeBSSRDF(iVector3(dist), sigma_tr, l_u, z_r, z_v);
				Rd *= alpha_prime;
				
				// Increment the radiant exitance (radiosity) at point P_o.
				// See [Jensen02] equation (13).  The F_dt and alpha' terms 
				// are omitted (as above).
				// Note that rad_t and hence sumPower can be negative, so M_o
				// can also turn negative.
				M_o += Rd * node->sumPower;
			}
		}
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Iteratively traverse the octree and add up the radiosity due to all the nodes
	static void
	TraverseTreeSSE(const iVector3& P_o,			// surfel position

					const iVector3& sigma_tr,		// sss params
					const iVector3& alpha_prime,
					const iVector3& l_u, 
					const iVector3& z_r, 
					const iVector3& z_v,

					float maxsolidangle,			// precision

					PListType 			* pointlist,
					iOctreeNodeType		* root,

					iVector3& M_o )					// return with SSS color
	{

		iOctreeNodeType *child;
		iVector3 Rd;							// diffuse BSSRDF reflection coeffs
		iVector3 rad_t;							// baked irradiance

		float omega = 0.0;						// max solid angle
		float dist, dist2;
		float area;


		iStackX< iOctreeNodeType* > istack(128);
		istack.push_back(root);					//push root node to stack


		do
		{

			// stack ..............................
			iOctreeNodeType * node = istack.pop();


			// If this is a leaf node: loop over the individual points in it
			if(node->isleaf) 
			{
				int firstpoint	= node->firstpoint;
				int lastpoint	= firstpoint + node->npoints;

				for (int i = firstpoint; i < lastpoint; i++) 
				{
					// Compute diffusion from data point P_i to data point P_o
					const iVector3& P_i = pointlist->template GetNodeVectorAt< DATA_SLOT_POS >(i);
					area				= pointlist->template GetNodeScalarAt< DATA_SLOT_AREA >(i);
					rad_t				= pointlist->template GetNodeVectorAt< DATA_SLOT_IRRAD >(i);

					dist = P_i.distanceX( P_o );

					Rd = ComputeBSSRDF(iVector3(dist), sigma_tr, l_u, z_r, z_v);
					Rd *= alpha_prime;
					Rd *= area;

					// Increment the radiant exitance (radiosity) at point P_o.
					// See [Jensen02] equation (13).
					// Note that rad_t can be negative, so M_o can also turn negative.
					// Also note that we omit the F_dt term here and in equation (14),
					// corresponding to diffuse radiance, as suggested right below
					// equation (14).
					// Furthermore, the alpha' term is omitted since it shouldn't be
					// there (according to Christophe Hery).  If it were, it would
					// cancel out the alpha' term in the BSSRDF.
					M_o += Rd * rad_t;
				}

			} else 
			{

				// Compute the minimum distance between the point and the node bbox.
				// The distance is 0 if the point is inside the bbox.
				dist2 = P_o.sqr_dist_bbox( node->bbox_min,  node->bbox_max);

				// Compute the maximum solid angle spanned by the points in this node.
				// ([Jensen02] p. 579 suggests estimating this using the distance
				// to the centroid of the points.  But that fails if e[1]. the points
				// are in two clumps, one of which is very close to the point P_o.) 
				if (dist2 > 0.0f)
				omega = node->sumArea / dist2;

				// To recurse or not to recurse, that is the question ...
				// We decide using the heuristic of [Jensen02] p. 579, except that
				// the solid angle is computed more conservatively: distance to
				// the bbox, not to the centroid.
				if (dist2 == 0.0f || omega > maxsolidangle) 
				{	// angle too large
					// Recursively visit children
					for (int c = 0; c < 8; c++)
					{
						child = node->children[c];
						if (child)
						istack.push_back( child );					// Push Node to stack
					}
				} else // error low enough: use node as is
				{ 
					dist = P_o.distanceX( node->centroid );

					// Compute the BSSRDF coefficients
					Rd = ComputeBSSRDF(iVector3(dist), sigma_tr, l_u, z_r, z_v);
					Rd *= alpha_prime;
				
					// Increment the radiant exitance (radiosity) at point P_o.
					// See [Jensen02] equation (13).  The F_dt and alpha' terms 
					// are omitted (as above).
					// Note that rad_t and hence sumPower can be negative, so M_o
					// can also turn negative.
					M_o += Rd * node->sumPower;
				}
			}
		}while( !istack.empty() );	////////////
		istack.clear();/////////////////////////
	}

#else

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter conversion
	static void
	ConvertParams(	const iVector3& Rd,			// diffuse color (BRDF albedo)
					const iVector3& dmfp,		// diffuse mean free path (aka. l_d)
					const DataType unitlength,	// multiplier on dmfp
					DataType *alphaPTable,

					iVector3 &alpha_prime,		// reduced scattering albedo 
					iVector3 &sigma_tr,			// effective transport extinction coeff
					iVector3 &l_u )				// mean free path length
	{
		iVector3 l_d ( dmfp );
		l_d *= unitlength;

		iVector3 mOne( 1.f );

		// Interpolate alpha' (from albedo Rd)
		alpha_prime[0] = InterpolateAlphaPrime(alphaPTable, Rd[0]);
		alpha_prime[1] = InterpolateAlphaPrime(alphaPTable, Rd[1]);
		alpha_prime[2] = InterpolateAlphaPrime(alphaPTable, Rd[2]);

		// compute effective transport extinction coeff sigma_tr (from dmfp l_d)
		sigma_tr[0] = 1.f / l_d[0];
		sigma_tr[1] = 1.f / l_d[1];
		sigma_tr[2] = 1.f / l_d[2];

		// Compute mean free path length l_u from diffuse mean free path l_d:
		// l_u = 1 / sigma'_t  with  sigma'_t = 1 / (l_d * sqrt(3 * (1 - alpha'))) 
		// gives  l_u = l_d * sqrt(3 * (1 - alpha')).
		l_u[0] = l_d[0] * sqrtf(3.0f * (1.0f - alpha_prime[0]));
		l_u[1] = l_d[1] * sqrtf(3.0f * (1.0f - alpha_prime[1]));
		l_u[2] = l_d[2] * sqrtf(3.0f * (1.0f - alpha_prime[2]));
	}

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Compute BSSRDF R_d using the dipole approximation: "real" point
	// light source inside volume and virtual point light source outside
	// Definition: diffuse BSSRDF R_d = dM_o / dPhi_i.
	static inline float
	ComputeBSSRDF(	float r,			// distance from illum point P_i to point P_o
					float sigma_tr,		// effective transport extinction coeff
					float l_u,			// mean free path length
					float z_r,			// depth of "real" dipole light source (= l_u)
					float z_v)			// height of virtual dipole light source
	{

		float d_r, d_v; // distance from the dipole lights to point xthe surface
		float c1, c2;
		float factor;
		//float R_d;		// the BSSRDF result

		if (r < l_u) {	// P_i and P_o are very close to each other:
		// Evaluate the BSSRDF with a minimum distance of l_u =
		// 1/sigma'_t to eliminate singularities (see Jensen et al.,
		// Proc. SIGGRAPH 01, p. 516).
		r = l_u;
		}

		d_r = sqrtf(r*r + z_r*z_r); // distance to "real" dipole source
		d_v = sqrtf(r*r + z_v*z_v); // distance to virtual dipole source

		//assert(d_r > 0.0f && d_v > 0.0f);

		c1 = z_r * (sigma_tr + 1.0f/d_r);
		c2 = z_v * (sigma_tr + 1.0f/d_v);
		factor =  c1 * expf(-sigma_tr * d_r) / (d_r*d_r);
		factor += c2 * expf(-sigma_tr * d_v) / (d_v*d_v);
		factor *= 1.0f / four_pi;

		//R_d = alpha_prime * factor;

		return factor;
	}



	////////////////////////////////////////////////////////////////////////////////////////////////
	// Recursively traverse the octree and add up the radiosity due to all the nodes
	static void
	TraverseTree(	const iVector3& P_o,			// surfel position

					const iVector3& sigma_tr,		// sss params
					const iVector3& alpha_prime,
					const iVector3& l_u, 
					const iVector3& z_r, 
					const iVector3& z_v,

					float maxsolidangle,			// precision

					PListType 			* pointlist,
					iOctreeNodeType		* root,

					iVector3& M_o )					// return with SSS color
	{

		iOctreeNodeType *child;
		iVector3 Rd;							// diffuse BSSRDF reflection coeffs
		iVector3 rad_t;							// baked irradiance

		float omega = 0.0;						// max solid angle
		float dist, dist2;
		float area;


		iStackX< iOctreeNodeType* > istack(128);
		istack.push_back(root);					//push root node to stack


		do
		{

			// stack ..............................
			iOctreeNodeType * node = istack.pop();


			// If this is a leaf node: loop over the individual points in it
			if(node->isleaf) 
			{
				int firstpoint	= node->firstpoint;
				int lastpoint	= firstpoint + node->npoints;

				for (int i = firstpoint; i < lastpoint; i++) 
				{
					// Compute diffusion from data point P_i to data point P_o
					const iVector3& P_i = pointlist->template GetNodeVectorAt< DATA_SLOT_POS >(i);
					area	= pointlist->template GetNodeScalarAt< DATA_SLOT_AREA >(i);
					rad_t	= pointlist->template GetNodeVectorAt< DATA_SLOT_IRRAD >(i);

					dist = P_i.distance( P_o );

					Rd[0] = ComputeBSSRDF( dist, sigma_tr[0], l_u[0], z_r[0], z_v[0]);
					Rd[1] = ComputeBSSRDF( dist, sigma_tr[1], l_u[1], z_r[1], z_v[1]);
					Rd[2] = ComputeBSSRDF( dist, sigma_tr[2], l_u[2], z_r[2], z_v[2]);
					Rd *= alpha_prime;
					Rd *= area;

					// Increment the radiant exitance (radiosity) at point P_o.
					M_o += Rd * rad_t;
				}

			} else 
			{

				// Compute the minimum distance between the point and the node bbox.
				// The distance is 0 if the point is inside the bbox.
				dist2 = P_o.sqr_dist_bbox( node->bbox_min,  node->bbox_max);

				// Compute the maximum solid angle spanned by the points in this node.
				if (dist2 > 0.0f)
				omega = node->sumArea / dist2;

				// To recurse or not to recurse, that is the question ...
				if (dist2 == 0.0f || omega > maxsolidangle) 
				{	// angle too large
					// Recursively visit children
					for (int c = 0; c < 8; c++)
					{
						child = node->children[c];
						if (child)
						istack.push_back( child );					// Push Node to stack
					}
				} else // error low enough: use node as is
				{ 
					dist = P_o.distance( node->centroid );

					// Compute the BSSRDF coefficients
					Rd[0] = ComputeBSSRDF( dist, sigma_tr[0], l_u[0], z_r[0], z_v[0]);
					Rd[1] = ComputeBSSRDF( dist, sigma_tr[1], l_u[1], z_r[1], z_v[1]);
					Rd[2] = ComputeBSSRDF( dist, sigma_tr[2], l_u[2], z_r[2], z_v[2]);
					Rd *= alpha_prime;
				
					// Increment the radiant exitance (radiosity) at point P_o.
					M_o += Rd * node->sumPower;
				}
			}
		}while( !istack.empty() );	////////////
		istack.clear();/////////////////////////
	}

#endif


/// ////////////////////////////////////////////////////////////////////////////////////////////////
public:

	typedef PListType			PListType;
	typedef iOctreeNodeType		iOctreeNodeType;
	typedef iCache				iCache;

	static const bool HasMultiPass = false;

	static bool
	doThread_CG_EffectsPass(chunk_idx		*chunk,

							PListType		* pointlist,
							iOctreeNodeType	* root,

							iCache			* cache,

							bool			isMainThread = false )
	{

		const DataType& unitlength		= cache->unitlength;
		const DataType& maxsolidangle	= cache->maxsolidangle;	// aka. eps
		int printProgress				= cache->verbose;


		//iVector3 Rd ( cache->albedo );
		//iVector3 l_d( cache->mfpath );

		iVector3 M_o;				// radiosity (aka. radiant exitance)
		iVector3 L_o;				// exitant radiance (ie. divide by M_PI)
		iVector3 P_o;
		iVector3 alpha_prime;		// reduced scattering albedo 
		iVector3 sigma_tr;			// effective transport extinction coeffient
		iVector3 l_u;				// mean free path length: avg dist at which light scatters
		iVector3 z_r, z_v;			// height of "real" and virtual dipole light sources



		// loop over all shading points
		for (iUint o = chunk->startValue; o < chunk->endValue; o++) 
		{ 
			if(isMainThread)		// abortation is ctrled by main thread ..
			if(ABORTCALLED)			// breaking from here, it will stop the other threads
			return true;


			// Convert parameters - albedo and meanfpath had a shader attached
			// we eval'ed and baked them while baking irradiance (lightmapping)
			//Rd =  pointlist->template GetNodeVectorAt< DATA_SLOT_ALBEDO >(o);
			//l_d = pointlist->template GetNodeVectorAt< DATA_SLOT_DMFP >(o);
			iVector3 Rd ( pointlist->template GetNodeVectorAt< DATA_SLOT_ALBEDO >(o) );
			iVector3 l_d ( pointlist->template GetNodeVectorAt< DATA_SLOT_DMFP >(o) );

#if	__SSE2_ENHANCED__
			ConvertParamsSSE(Rd, l_d, unitlength, cache->alphatable, alpha_prime, sigma_tr, l_u);
#else
			ConvertParams(Rd, l_d, unitlength, cache->alphatable, alpha_prime, sigma_tr, l_u);
#endif

			/*if(o<10){
				cout << "\no: " << o << " :: " << NODESIZE << ", " << SSSSLOT << ", "
					<< pointlist->template GetNodeVectorAt< DATA_SLOT_POS >(o)
					<< pointlist->template GetNodeVectorAt< DATA_SLOT_DIR >(o)
					<< pointlist->template GetNodeScalarAt< DATA_SLOT_AREA >(o)
					<< pointlist->template GetNodeVectorAt< DATA_SLOT_IRRAD >(o)
					<< pointlist->template GetNodeVectorAt< DATA_SLOT_ALBEDO >(o)
					<< pointlist->template GetNodeVectorAt< DATA_SLOT_DMFP >(o)
					<< pointlist->template GetNodeVectorAt< DATA_NSLOTS_SSS >(o)

					<< alpha_prime << ", "
					<< sigma_tr << ", "
					<< l_u << ", "
					<< unitlength << ", "
					<< endl;
			}*/

			if(Rd > 0.999f) 
			{
				WARNME("--- Warning: albedo is 1 (or larger) ...  clamped to 0.999.\n");
#if	__SSE2_ENHANCED__
				Rd = Rd.clampMin(M128_ALMOST_ONE);
#else
				Rd[0] =  ds_MIN(Rd[0] , .999999999f);
				Rd[1] =  ds_MIN(Rd[1] , .999999999f);
				Rd[2] =  ds_MIN(Rd[2] , .999999999f);
#endif
			}
			if(l_u == 0.0f) {
				ERRORME("--- Error: mean free path length is 0.  Aborting.\n");
				return true; // failure
			}
			/*if(printProgress && o==0) {
			PRINTME("--- Reduced scat. albedo: %f %f %f\n",alpha_prime[0], alpha_prime[1], alpha_prime[2]);
			PRINTME("--- Reduced ext. coefficient: %f %f %f\n",sigma_tr[0], sigma_tr[1], sigma_tr[2]);
			PRINTME("--- Mean free path length: %f %f %f\n", l_u[0], l_u[1], l_u[2]);	}*/


			// Dipole placements:
			// z_r = l_u is the depth of "real" dipole light (distance to surface)
			// z_v = l_u * (1.0f + 1.333333f * A) is height of virtual dipole light			
			z_r = l_u;
			z_v = l_u * cache->A;

			// Compute diffusion to data point o ///////////////////////////////////////////////////
			P_o = pointlist->template GetNodeVectorAt< DATA_SLOT_POS >(o);
#if	__SSE2_ENHANCED__
			M_o = M128_ZERO;
#else
			M_o = 0.f;
#endif

			// hierarchical gathering of light
			//RecTraverseTree(	
#if	__SSE2_ENHANCED__
			TraverseTreeSSE(	
#else
			TraverseTree(	
#endif
							P_o,

							sigma_tr, 
							alpha_prime, 
							l_u, 
							z_r, 
							z_v,

							maxsolidangle,

							pointlist,
							root, 
							//0,
							M_o	);

			// If we wanted to be physically correct, we would convert
			// radiant exitance (radiosity) to exitant radiance by
			// dividing by pi...
			L_o = M_o;
			
			// blend it with diffuse, eventually
			if(cache->diffuseblend != 1.0f)
			{
				iVector3 irrad = pointlist->template GetNodeVectorAt< DATA_SLOT_IRRAD >(o);
				L_o = irrad + ( (M_o- irrad) * cache->diffuseblend );
			}	

			// insert SSS result in pointlist //////////////////////////////////////////////////////
			pointlist->template SetNodeVectorAt< SSSSLOT >(o, L_o);
			

			// print per-point stats 
			if(isMainThread && printProgress>=2 && (o % chunk->frequency == 0))
			PRINTME("\n--- diffusion at point %u = (%f %f %f)", o, L_o[0], L_o[1], L_o[2]);
		}

	return false;	//eg. no abort called
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	static iVector3
	DoScattering(	const iVector3& P_o,			// position

					iVector3&		Rd,				// diffuse color (BRDF albedo)
					iVector3&		l_d,			// diffuse mean free path (aka. dmfp)

					float			unitlength,
					float			maxsolidangle,	// aka. eps

					float			* alphaPTable,
					float			A,

					PListType		* pointlist,
					iOctreeNodeType	* root)
	{


		iVector3 alpha_prime;			// reduced scattering albedo 
		iVector3 sigma_tr;				// effective transport extinction coeffient
		iVector3 l_u;					// mean free path length: avg dist at which light scatters
		iVector3 z_r, z_v;				// height of "real" and virtual dipole light sources

		//Ior may be a map for varying ior on surface ...
		//so A is computed for every sample .............
		//float A= GetAlphaForTable(cache->ior)
		
		
		if(Rd > 0.999f) 
		{
			WARNME("--- Warning: albedo is 1 (or larger) ..  clamped to 0.999\n");
#if	__SSE2_ENHANCED__
			Rd = Rd.clampMin(M128_ALMOST_ONE);
#else
			Rd[0] =  ds_MIN(Rd[0] , .999999999f);
			Rd[1] =  ds_MIN(Rd[1] , .999999999f);
			Rd[2] =  ds_MIN(Rd[2] , .999999999f);
#endif
		}

		// convert parameters //////////////////////////////////////////////////////////////////////
#if	__SSE2_ENHANCED__
		ConvertParamsSSE(Rd, l_d, unitlength, alphaPTable, alpha_prime, sigma_tr, l_u);
#else
		ConvertParams(Rd, l_d, unitlength, alphaPTable, alpha_prime, sigma_tr, l_u);
#endif

		if(l_u == 0.0f) {
			ERRORME("--- Error: mean free path length is 0.  Aborting.\n");
			return l_u; // failure
		}

		// Dipole placements ///////////////////////////////////////////////////////////////////////
		z_r = l_u;
		z_v = l_u * A;

		
		// Compute diffusion to data point o ///////////////////////////////////////////////////////
		iVector3 M_o;					// radiosity (aka. radiant exitance), M_o here is the same as 
										// L_o (exitant radiance) as we don't divide by 3.14

#if	__SSE2_ENHANCED__
		TraverseTreeSSE(	
#else
		TraverseTree(	
#endif
						P_o,

						sigma_tr,
						alpha_prime,
						l_u,
						z_r,
						z_v,
						
						maxsolidangle,

						pointlist,
						root,
						M_o	);

		// return with sss diffusion for the sample
		return M_o;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Alpha table
	static DataType
	GetAlphaForTable( DataType const eta)
	{
		//DataType eta = cache->ior;				// relative index of refraction
		DataType F_dr = -1.440f / (eta*eta);		// diffuse Fresnel reflection and transmission terms
		F_dr += (0.710f / eta);
		F_dr += 0.668f;
		F_dr += (0.0636f * eta);

		//assert(0.0f <= F_dr && F_dr <= 1.0);
		return DataType( (1.0f + F_dr) / (1.0f - F_dr) );
	}


	static void 
	FillAlphaPrimeTable(DataType *alphaPTable, DataType A)
	{
		DataType RdTable[steps+1];
		DataType alpha_prime, Rd;
		DataType w;
		int i, j;

		for (i = 0; i <= steps; i++) 
		{
			alpha_prime = i / (DataType)steps;
			RdTable[i] = ComputeRd(A, alpha_prime);	// compute Rd //////////////////////////////////
		}

		for (i = 0; i <= steps; i++) 
		{
			Rd = i / (DataType)steps;

			for (j = 0; RdTable[j] < Rd; j++) ;
			//assert( (j == 0) || (RdTable[j-1] <= Rd) && (Rd <= RdTable[j]) );
			w = (j > 0) ? (Rd - RdTable[j-1]) / (RdTable[j] - RdTable[j-1]) : 0.0f;

			alphaPTable[i] = (w * j + (1.0f - w) * (j-1)) / (DataType)steps;
		}
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	static void
	LayoutPointlistData( PListType * _pointlist )
	{
		typedef PListType::_iDataFlag _iDataFlagT;
		_iDataFlagT newflag;

		// local fields
		newflag.name = "dir";
		newflag.type = 3;
		newflag.offset = DATA_SLOT_DIR;
		_pointlist->AddLocalField(newflag);

		newflag.name = "area";
		newflag.type = 2;
		newflag.offset = DATA_SLOT_AREA;
		_pointlist->AddLocalField(newflag);

		newflag.name = "irrad";
		newflag.type = 4;
		newflag.offset = DATA_SLOT_IRRAD;
		_pointlist->AddLocalField(newflag);

		newflag.name = "alb";
		newflag.type = 4;
		newflag.offset = DATA_SLOT_ALBEDO;
		_pointlist->AddLocalField(newflag);

		newflag.name = "dmfp";
		newflag.type = 4;
		newflag.offset = DATA_SLOT_DMFP;
		_pointlist->AddLocalField(newflag);

		newflag.name = "sss";
		newflag.type = 4;
		newflag.offset = SSSSLOT;
		_pointlist->AddLocalField(newflag);

		// global fields
		newflag.name = "maptype";
		newflag.type = 0;
		newflag.offset = -778899;			//sss integer identifier
		_pointlist->AddGlobalField(newflag);
	}

};

}	//end namespace
#endif __PCLOUD_SUBSURFACESCATTERING__