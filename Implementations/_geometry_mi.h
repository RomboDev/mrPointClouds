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

#ifndef __PCLOUD_MI_GEOMETRY__
#define __PCLOUD_MI_GEOMETRY__


#include <geoshader.h>
using namespace mi::shader;


namespace cpc
{

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Geometric Aux functions ////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// geoobj_tri_to_vtx //////////////////////////////////////////////////////////////////////////////////
static inline void
geoobj_tri_to_vtxdata(  const miBox *box, miGeoIndex iTri,
						miVector *p0, miVector *p1, miVector *p2 )
{
	miUint* triList = (miUint*)miBOX_TRI(box, iTri);

	// Get vtxs position
	p0 = miVL_POS(box, miBOX_VERTEX_LINE(box, triList[0]));
	p1 = miVL_POS(box, miBOX_VERTEX_LINE(box, triList[1]));
	p2 = miVL_POS(box, miBOX_VERTEX_LINE(box, triList[2]));
}

// geoobj_tri_to_vtx //////////////////////////////////////////////////////////////////////////////////
static inline iVector3 
geoobj_halfedge(const iVector3& tA, const iVector3& tB)
{
	return iVector3( (tA+tB)/2.0f );
}


#if	__SSE2_ENHANCED__

// geoobj_vtx_distance ////////////////////////////////////////////////////////////////////////////////
static INLINE miScalar
geoobj_vtx_distance(const miVector *a, const miVector *b)
{
	iVector3 r (iVector3(a) - iVector3(b));
	return _mm_cvtss_f32( _mm_sqrt_ss( r.dotX(r) ) );
}

// geoobj_tri_area ////////////////////////////////////////////////////////////////////////////////////
static inline miScalar 
geoobj_tri_area(const miVector *vPosA, const miVector *vPosB, const miVector *vPosC)
{
	iVector3 sides;
	sides[0] = geoobj_vtx_distance(vPosA, vPosB);
	sides[1] = geoobj_vtx_distance(vPosA, vPosC);
	sides[2] = geoobj_vtx_distance(vPosB, vPosC);
	sides[3] = 0.f;

	iVector3 tPerim ( _mm_hadd_ps(sides, sides) );
	tPerim = _mm_div_ps( _mm_hadd_ps(tPerim, tPerim), M128_TWO);

	sides = tPerim - sides;

	return sqrtf(sides.product()*tPerim[0]);
}


// geoobj_tri_centroid ////////////////////////////////////////////////////////////////////////////////
static INLINE miVector 
geoobj_tri_centroid(const miVector *p0, const miVector *p1, const miVector *p2)
{
	return iVector3 ( iVector3(p0) + iVector3(p1) + iVector3(p2) ) / 3.f;
}

// geoobj_tri_normal //////////////////////////////////////////////////////////////////////////////////
static INLINE miVector 
geoobj_tri_normal(const miVector *p0, const miVector *p1, const miVector *p2)
{

	iVector3 v1( iVector3(p0)-iVector3(p1) );
	iVector3 v2( iVector3(p1)-iVector3(p2) );
	iVector3 n ( v1.cross(v2) );
	return n.normalize();
}
static INLINE miVector 
geoobj_tri_normal(const iVector3& p0, const iVector3& p1, const iVector3& p2)
{

	iVector3 v1( p0-p1 );
	iVector3 v2( p1-p2 );
	iVector3 n ( v1.cross(v2) );
	return n.normalize();
}
#else

// geoobj_vtx_distance ////////////////////////////////////////////////////////////////////////////////
static inline miScalar
geoobj_vtx_distance(const miVector *a, const miVector *b)
{
	miVector r;
	mi_vector_sub(&r,a,b);
	return (miScalar)sqrt( mi_vector_dot(&r,&r) );
}

// geoobj_tri_area ////////////////////////////////////////////////////////////////////////////////////
static inline miScalar 
geoobj_tri_area(const miVector *vPosA, const miVector *vPosB, const miVector *vPosC)
{
	miScalar sideAB, sideAC, sideBC;
	miScalar tPerim, tArea;

	sideAB = geoobj_vtx_distance(vPosA, vPosB);
	sideAC = geoobj_vtx_distance(vPosA, vPosC);
	sideBC = geoobj_vtx_distance(vPosB, vPosC);

	tPerim = (sideAB + sideAC + sideBC) / 2;
	sideAB = tPerim - sideAB;
	sideAC = tPerim - sideAC;
	sideBC = tPerim - sideBC;

	tArea  = (sqrtf( tPerim * sideAB * sideAC * sideBC ));	//divide by n.vtx
	return tArea;
}

// geoobj_tri_centroid ////////////////////////////////////////////////////////////////////////////////
static inline miVector 
geoobj_tri_centroid(const miVector *p0, const miVector *p1, const miVector *p2)
{
	miVector center;
	center.x = ( p0->x + p1->x + p2->x ) / 3.f;
	center.y = ( p0->y + p1->y + p2->y ) / 3.f;
	center.z = ( p0->z + p1->z + p2->z ) / 3.f;

	return center;
}

// geoobj_tri_normal //////////////////////////////////////////////////////////////////////////////////
static inline miVector 
geoobj_tri_normal(const miVector *p0, const miVector *p1, const miVector *p2)
{
    miVector v1, v2;
    miVector n;

	mi_vector_sub(&v1, p0, p1);
	mi_vector_sub(&v2, p1, p2);
	mi_vector_prod(&n, &v1, &v2);

	mi_vector_normalize(&n);
	return n;
}

#endif


// compute tex_coords for barycenter point ////////////////////////////////////////////////////////////
static inline miVector
geobj_tri_bary_texlist(miState* state)
{
	float i, j, k; const miVector *vVecs[3];
	miVector tex_list;
	if (!mi_tri_vectors(state, 't', 0,
		&vVecs[0], &vVecs[1], &vVecs[2]))
	{
		//ahiahai
	}
	
	i = state->bary[0];
	j = state->bary[1];
	k = state->bary[2];

	// fill tex_list with intep tex coord triples
	tex_list.x = i*vVecs[0]->x + j*vVecs[1]->x + k*vVecs[2]->x;
	tex_list.y = i*vVecs[0]->y + j*vVecs[1]->y + k*vVecs[2]->y;
	tex_list.z = i*vVecs[0]->z + j*vVecs[1]->z + k*vVecs[2]->z;

	return tex_list;
}

// update shadow_tol //////////////////////////////////////////////////////////////////////////////////
static inline void
geoobj_tri_shadowtol(miState* state, 
					 const miVector *vPosA, const miVector *vPosB, const miVector *vPosC)
{

	if (state->options->shadow && state->type != miRAY_SHADOW ) 
	{
		double d_shadow_tol, tmp_d;

		d_shadow_tol = mi_vector_dot_d(&state->normal, vPosA);
		tmp_d = mi_vector_dot_d(&state->normal, vPosB);
		if (d_shadow_tol < tmp_d)
			d_shadow_tol = tmp_d;
		tmp_d = mi_vector_dot_d(&state->normal, vPosC);
		if (d_shadow_tol < tmp_d)
			d_shadow_tol = tmp_d;
		state->shadow_tol = d_shadow_tol
			- mi_vector_dot_d(&state->normal, &state->point);
	}
}

#if	__SSE2_ENHANCED__
static INLINE void INTERP_FROM_BARY_IJK(miVector& r, const miVector* p0, const miVector* p1, const miVector* p2, const miVector* ijk)
{
	r = iVector3((iVector3(ijk) * iVector3(p0)) + (iVector3(ijk) * iVector3(p1)) + (iVector3(ijk) * iVector3(p2))).AsMiVector();
}
#endif
}	// end namespace



///////////////////////////////////////////////////////////////////////////////////////////////////////
// Maya style utilities ///////////////////////////////////////////////////////////////////////////////
#define MAYA_MAX(x,y)		((x) > (y) ? (x) : (y))
#define SET_SHADOW_TOLERANCE(p0, p1, p2) { \
	double dot_np0 = (double)(state->normal.x)*(double)((p0)->x) + \
					 (double)(state->normal.y)*(double)((p0)->y) + \
					 (double)(state->normal.z)*(double)((p0)->z);  \
	double dot_np1 = (double)(state->normal.x)*(double)((p1)->x) + \
					 (double)(state->normal.y)*(double)((p1)->y) + \
					 (double)(state->normal.z)*(double)((p1)->z);  \
	double dot_np2 = (double)(state->normal.x)*(double)((p2)->x) + \
					 (double)(state->normal.y)*(double)((p2)->y) + \
					 (double)(state->normal.z)*(double)((p2)->z);  \
	if (state->inv_normal) { \
		dot_np0 = -dot_np0; \
		dot_np1 = -dot_np1; \
		dot_np2 = -dot_np2; \
		} \
	state->shadow_tol = \
		MAYA_MAX(dot_np0, MAYA_MAX(dot_np1, dot_np2)) - \
			 ((double)(state->normal.x)*(double)(state->point.x) + \
	          (double)(state->normal.y)*(double)(state->point.y) + \
	          (double)(state->normal.z)*(double)(state->point.z)); \
	}


// update remaining state vars ////////////////////////////////////////////////////////////////////////
#define INTERP_FROM_BARY_IJK(r, p0, p1, p2) \
	(r).x = i*(p0)->x + j*(p1)->x + k*(p2)->x; \
	(r).y = i*(p0)->y + j*(p1)->y + k*(p2)->y; \
	(r).z = i*(p0)->z + j*(p1)->z + k*(p2)->z; \

static inline
void initIntersectionArrays(

	//miInteger	uvSpace,
	miState		*state)
{
	const miVector NullVector = { 0 };
	const float i = state->bary[0];
	const float j = state->bary[1];
	const float k = state->bary[2];
	const miVector *V[3];
	int cnt;

	// motion
	if (mi_tri_vectors(state, 'm', 0, V+0, V+1, V+2)) {
		INTERP_FROM_BARY_IJK(state->motion, V[0], V[1], V[2])
		mi_vector_from_object(state, &state->motion, &state->motion);
		}
	else
		state->motion = NullVector;

	// textures
	for (cnt=0; ; cnt++) {
		//if (cnt == uvSpace) continue;
		if (!mi_tri_vectors(state, 't', cnt, V+0, V+1, V+2)) break;
		INTERP_FROM_BARY_IJK(state->tex_list[cnt], V[0], V[1], V[2])
		}

	// bump basis
	for (cnt=0; ; cnt++) {
		if (!mi_tri_vectors(state, 'u', cnt, V+0, V+1, V+2)) break;
		INTERP_FROM_BARY_IJK(state->bump_x_list[cnt], V[0], V[1], V[2])
		}
	for (cnt=0; ; cnt++) {
		if (!mi_tri_vectors(state, 'v', cnt, V+0, V+1, V+2)) break;
		INTERP_FROM_BARY_IJK(state->bump_y_list[cnt], V[0], V[1], V[2])
		}

	// derivatives
	if (mi_tri_vectors(state, 'U', 0, V+0, V+1, V+2)) {
		INTERP_FROM_BARY_IJK(state->derivs[0], V[0], V[1], V[2])
		mi_vector_from_object(state, state->derivs+0, state->derivs+0);
		}
	else
		state->derivs[0] = NullVector;

	if (mi_tri_vectors(state, 'V', 0, V+0, V+1, V+2)) {
		INTERP_FROM_BARY_IJK(state->derivs[1], V[0], V[1], V[2])
		mi_vector_from_object(state, state->derivs+1, state->derivs+1);
		}
	else
		state->derivs[1] = NullVector;

	if (mi_tri_vectors(state, 'X', 0, V+0, V+1, V+2)) {
		INTERP_FROM_BARY_IJK(state->derivs[2], V[0], V[1], V[2])
		mi_vector_from_object(state, state->derivs+2, state->derivs+2);
		}
	else
		state->derivs[2] = NullVector;

	if (mi_tri_vectors(state, 'Y', 0, V+0, V+1, V+2)) {
		INTERP_FROM_BARY_IJK(state->derivs[3], V[0], V[1], V[2])
		mi_vector_from_object(state, state->derivs+3, state->derivs+3);
		}
	else
		state->derivs[3] = NullVector;

	if (mi_tri_vectors(state, 'Z', 0, V+0, V+1, V+2)) {
		INTERP_FROM_BARY_IJK(state->derivs[4], V[0], V[1], V[2])
		mi_vector_from_object(state, state->derivs+4, state->derivs+4);
		}
	else
		state->derivs[4] = NullVector;
}

// for state->org from camera /////////////////////////////////////////////////////////////////////////
static inline
void origin_from_camera(

	const miState	*state,
	miVector		*origin)
{
	miInstance *instance = (miInstance *) mi_db_access(state->camera_inst);

	origin->x = instance->tf.local_to_global[12];
	origin->y = instance->tf.local_to_global[13];
	origin->z = instance->tf.local_to_global[14];

	if (instance->tf.local_to_global[15] != 1.0f &&
	    instance->tf.local_to_global[15] != 0.0f)
		mi_vector_div(origin, instance->tf.local_to_global[15]);

	mi_db_unpin(state->camera_inst);
}

// point in tri check 
//TODO:USE:IVECTOR3:INSTEAD:OF:FLOAT3:!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*
	bool SameSide(const Float3& p1, const Float3& p2, const Float3& a, const Float3& b)
	{
		Float3 BminusA = b-a;
		Float3 P1minusA = p1-a;
		Float3 P2minusA = p2-a;
	
		Float3 cp1 = BminusA.GetCross( P1minusA );
		Float3 cp2 = BminusA.GetCross( P2minusA );
		float CP1dotCP2 = cp1.GetDot( cp2 );

		if( CP1dotCP2 >= 0.f)
				return true;
		else	return false;
	}
	bool PointInTriangle(const Float3& p, const Float3& a, const Float3& b, const Float3& c)
	{
		if( SameSide(p,a, b,c) && SameSide(p,b, a,c) && SameSide(p,c, a,b) )
				return true;
		else	return false;
	}
*/
#endif __PCLOUD_MI_GEOMETRY__