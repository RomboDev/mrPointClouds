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


#ifndef __PCLOUD_DATASTRUCT_ISURFEL__
#define __PCLOUD_DATASTRUCT_ISURFEL__

#include "../Implementations/_geometry_mi.h"



namespace cpc
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Init extra Plist slots //////////////////////////////////////////////////////////////////////////
template< typename DTYPE, unsigned ISLOT=0, unsigned I=0 > 
struct CopyArray3D
{
	static inline void eval( DTYPE* p, DTYPE* q )
	{
		p[ISLOT]  = q[I];
		CopyArray3D< DTYPE, ISLOT+1, I+1 >::eval( p,q );
	}
};
// Partial Template Specialization
template <typename DTYPE, unsigned ISLOT> 
struct CopyArray3D<DTYPE, ISLOT, DATA_VECTOR_END>
{
	static inline void eval( DTYPE* p, DTYPE* q )
	{
#if		__SSE2_ENHANCED__
		(void)q;
		p[ISLOT] = 0.f;
#else
		p[ISLOT] = q[DATA_VECTOR_END];
#endif
	}
};



////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
struct iSurfel_MI_bake_result
{
	miVector	point;		// point in space 
	miVector	normal;		// vertex normal 
	miVector	tex;		// texture coordinates of vertex
	miColor		irrad;		// irrad on vertex (not used)
};


struct iSurfel_MICache
{
	char dummy;
};


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename DataType, unsigned SizeBlock>
class iSurfelAllocatorBase
{
public:

	// Data node ///////////////////////////////////////////////////////////////////////////////////
	DataType* InitNodeBlock( void )
	{
		DataType* datanode = (DataType*) malloc( SizeBlock * sizeof(DataType) );
		return datanode != NULL ? datanode: NULL;
	};

	// Size of node array
	unsigned int GetNodeSize( void ){ 
		return SizeBlock; 
	};

	// Get stuff ///////////////////////////////////////////////////////////////////////////////////
	template<unsigned NODEARRAYPOS> INLINE iPoint<DataType,3> GetNodeVectorAt( DataType* Node )
	{ return iPoint<DataType,3>( &( Node[ NODEARRAYPOS ]) ); }
	template<unsigned NODEARRAYPOS> INLINE DataType GetNodeScalarAt( const iUint& NodeID )
	{ return _iData->ReadNode( NodeID )[ NODEARRAYPOS ]; }

	// Set stuff ///////////////////////////////////////////////////////////////////////////////////
	template<unsigned NODEARRAYPOS>	INLINE void SetNodeScalarAt( DataType* Node, const DataType& val )
	{ Node[NODEARRAYPOS] = val; }

	//template<unsigned NODEARRAYPOS>	INLINE void SetNodeVectorAt( DataType* Node, const iPoint<DataType, 3>& Pos )
	//{ _mm_store_ps(&(Node[ NODEARRAYPOS ]), Pos ); }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Float allocator
template<typename DataType, unsigned SizeBlock>
class iSurfelAllocator: public iSurfelAllocatorBase< DataType, SizeBlock >
{
public:
	template<unsigned NODEARRAYPOS>	INLINE void SetNodeVectorAt( DataType* Node, const iPoint<DataType, 3>& Pos )
	{ CopyArray3D<DataType, NODEARRAYPOS>::eval( Node, (DataType*)Pos.begin() ); }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// SSE allocator
template<typename DataType, unsigned SizeBlock>
class iSurfelAllocatorSSE: public iSurfelAllocatorBase< DataType, SizeBlock >
{
public:
	template<unsigned NODEARRAYPOS>	INLINE void SetNodeVectorAt( DataType* Node, const iPoint<DataType, 3>& Pos )
	{ _mm_store_ps(&(Node[ NODEARRAYPOS ]), Pos ); }
};



/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template 
<
typename DTYPE										= float, 
unsigned SIZE										= DATA_SLOT_MINIMAL, 
template<typename, unsigned> class iNodeAllocator	= iSurfelAllocator
>
class iTrianleTessellator_Reyes : public iNodeAllocator<DTYPE, SIZE>
{
public:
	typedef iNodeAllocator<DTYPE, SIZE>		iDataAllocator;
	typedef unsigned int					U32;

	#include ".\_others\RibTools\DVector.h"


	#define RI_INFINITY	1.0e38f
	#define RI_EPSILON	1.0e-10f

	//==================================================================
	#define DMT_SIMD_FLEN	4
	#define DMT_SIMD_ALIGN_SIZE	16
	#define DVECTOR_SIMD_ALIGN_ARRAYN( _X_, _N_ )	__declspec(align(DMT_SIMD_ALIGN_SIZE))	_X_ [_N_]

	#define	DMT_SIMD_PADSIZE(_SIZE_)	(((unsigned)(_SIZE_) + (DMT_SIMD_FLEN-1)) & ~(DMT_SIMD_FLEN-1))
	#define MAX_PADDED_VERTS_FOR_MakeBoundFromUVRangeN(_DIM_)	DMT_SIMD_PADSIZE( _DIM_ * _DIM_ )

	#define	DMT_SIMD_BLOCKS(_SIZE_)			(((unsigned)(_SIZE_) + (DMT_SIMD_FLEN-1)) / DMT_SIMD_FLEN)

	static const size_t	MAKE_BOUND_FROM_UV_RANGE_DIM = 4;
	static const size_t	MAX_MAKE_BOUND_OUT_SIZE =
							MAX_PADDED_VERTS_FOR_MakeBoundFromUVRangeN( MAKE_BOUND_FROM_UV_RANGE_DIM );

	static const iUint	MP_GRID_MAX_SIZE				= DMT_SIMD_PADSIZE( 48 ) * 48;
	static const iUint	MP_GRID_MAX_SIZE_SIMD_BLKS		= DMT_SIMD_BLOCKS( MP_GRID_MAX_SIZE );

	//==================================================================
	struct Bound;
	struct TriData;
	struct BiPatch;


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum CheckSplitRes
	{
		CHECKSPLITRES_CULL,
		CHECKSPLITRES_DICE,
		CHECKSPLITRES_SPLIT,
	};

	//for tridata params
	struct tParams
	{
		iUint	tIdx;
		float	shading_rate;
		int		doubleMicroTriangles;
	};

	//for initWorld
	struct camOptions
	{
		int		mXRes, mYRes;
		float	mPixelAspectRatio;
		float	mFrameAspectRatio;
		float	mLeft, mRight, mBottom, mTop;
		float	mXMin, mXMax, mYMin, mYMax;

		float mNearClip, mFarClip;
		float mFOV;
		
		Matrix44 mMtxWorldCamera;
		Matrix44 mMtxCamProj;
		Matrix44 mMtxWorldProj;
	};

	//for triangle patching
	struct triedge
	{
		Float3 mV0, mV1;
		Float3 thirdVtx;
		float width;
		
		triedge( const Float3& v0, const Float3& v1, const Float3& vx) :mV0(v0), mV1(v1), thirdVtx(vx){ evalWidth(); };
		triedge( const triedge* te) :mV0(te->mV0), mV1(te->mV1), thirdVtx(te->thirdVtx), width(te->width){ evalWidth(); };
		inline Float3& operator[](const iUint i) 
		{ 
			switch(i){
				case 0: return mV0;
				case 1: return mV1;
				case 2: return thirdVtx;
				default: return mV0;
			}
		};

		Float3 evalDelta()const{ return mV1-mV0;; };
		float evalWidth() 
		{ 
			iVector3 iV0(mV0.x(), mV0.y(), mV0.z());
			iVector3 iV1(mV1.x(), mV1.y(), mV1.z());
			width = geoobj_vtx_distance( iV1, iV0 ); 
			return width;
		};

		int hasSharedVtx(const triedge* iTe)
		{
			int ret = -1;
			Float3 mVtx[2] = {mV0, mV1};
			Float3 iVtx[2] = {iTe->mV0, iTe->mV1};
			for(int i=0; i<2; i++)
			{
				Float3 myVtx = mVtx[i];
				for(int x=0; x<2; x++)
				{
					if(myVtx == iVtx[x])
					{
						myVtx = iVtx[x];
						return x;
					}
				}
			}
			return ret;
		}
	};

	struct triedgeOrder {
	  bool operator() (triedge* i,triedge* j) { return (i->width < j->width);}
	} orderTriEdges;

	//for dicing
	struct iMicroVertex
	{
		iVector3 pos;
		iVector2 uv;

		iMicroVertex(){};
		iMicroVertex(const iVector3& iPos, const iVector2 iUV):pos(iPos), uv(iUV){};

		inline float operator[](const iUint i) { return pos[i];	};
		INLINE operator iVector3 ( void ) const	{ return pos; }
		INLINE operator iVector2 ( void ) const	{ return uv; }
		float getU (){ return uv[0]; }
		float getV (){ return uv[1]; }
	};


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	#define	DMIN(_A_,_B_)			((_A_) < (_B_) ? (_A_) : (_B_))
	#define	DMAX(_A_,_B_)			((_A_) > (_B_) ? (_A_) : (_B_))

	template < typename _TA , typename _TB >
	static inline _TA DMix( const _TA &a, const _TA &b,  const _TB &t )
	{
		return a + (b - a) * t;
	}

	static INLINE float
	geoobj_vtx_distance(const iVector3& a, const iVector3& b)
	{
		iVector3 r = a - b;
		return _mm_cvtss_f32( _mm_sqrt_ss( r.dotX(r) ) );
	}
	static INLINE float
	geoobj_vtx_distance(const Float3& a, const Float3& b)
	{
		iVector3 iA (a.x(), a.y(), a.z(), 1.0f);
		iVector3 iB (b.x(), b.y(), b.z(), 1.0f);
		iVector3 r = iA - iA;
		return _mm_cvtss_f32( _mm_sqrt_ss( r.dotX(r) ) );
	}

#if	__SSE2_ENHANCED__
	static inline float 
	geoobj_tri_area(const iVector3& vPosA, const iVector3& vPosB, const iVector3& vPosC)
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
#else
	static inline float 
	geoobj_tri_area(const iVector3& vPosA, const iVector3& vPosB, const iVector3& vPosC)
	{
		float sideAB, sideAC, sideBC;
		float tPerim, tArea;

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
#endif

	//==================================================================
	static void MakeCube( const Bound &b, Float3 out_box[8] )
	{
		out_box[0] = Float3( b.mBox[0][0], b.mBox[0][1], b.mBox[0][2] );
		out_box[1] = Float3( b.mBox[1][0], b.mBox[0][1], b.mBox[0][2] );
		out_box[2] = Float3( b.mBox[0][0], b.mBox[1][1], b.mBox[0][2] );
		out_box[3] = Float3( b.mBox[1][0], b.mBox[1][1], b.mBox[0][2] );
		out_box[4] = Float3( b.mBox[0][0], b.mBox[0][1], b.mBox[1][2] );
		out_box[5] = Float3( b.mBox[1][0], b.mBox[0][1], b.mBox[1][2] );
		out_box[6] = Float3( b.mBox[0][0], b.mBox[1][1], b.mBox[1][2] );
		out_box[7] = Float3( b.mBox[1][0], b.mBox[1][1], b.mBox[1][2] );
	}


	//==================================================================
	template <class _T>
	static inline Vec4<_T> V4__V3W1_Mul_M44( const Vec3<_T> &v, const Matrix44 &a )
	{
		const _T	&x = v.v3[0];
		const _T	&y = v.v3[1];
		const _T	&z = v.v3[2];

		return Vec4<_T>(
			x * a.mij(0,0) + y * a.mij(1,0) + z * a.mij(2,0) + a.mij(3,0),
			x * a.mij(0,1) + y * a.mij(1,1) + z * a.mij(2,1) + a.mij(3,1),
			x * a.mij(0,2) + y * a.mij(1,2) + z * a.mij(2,2) + a.mij(3,2),
			x * a.mij(0,3) + y * a.mij(1,3) + z * a.mij(2,3) + a.mij(3,3)
		);
	}

	//==================================================================
	static inline void bilinearFill(	float out_x[], float out_y[],
										float x1, float y1, 
										float x2, float y2, 
										iUint xN, iUint yN, 
										size_t paddedN )
	{
		float	dv = (y2 - y1) / (yN ? (yN - 1) : 1);
		float	du = (x2 - x1) / (xN ? (xN - 1) : 1);

		float	v = y1;
		size_t	idx = 0;
		for (u_int y=yN; y > 0; --y)
		{
			float	u = x1;
			for (u_int x=xN; x > 0; --x, ++idx)
			{
				out_x[idx] = u;
				out_y[idx] = v;
				u += du;
			}
			v += dv;
		}

		for (; idx < paddedN; ++idx)
		{
			out_x[idx] = out_x[idx-1];
			out_y[idx] = out_y[idx-1];
		}
	}


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Bound
	{
		Float3	mBox[2];

		Bound()
		{
		}

		Bound( float val )
		{
			mBox[0].Set( val, val, val );
			mBox[1].Set( val, val, val );
		}
	
		Bound( float x1, float y1, float z1,
			   float x2, float y2, float z2 )
		{
			mBox[0].Set( x1, y1, z1 );
			mBox[1].Set( x2, y2, z2 );
		}
	
		void SetMin( const float *p )			{ mBox[0].Set( p[0], p[1], p[2] );	}
		void SetMax( const float *p )			{ mBox[1].Set( p[0], p[1], p[2] );	}
		void SetMin( float x, float y, float z ){ mBox[0].Set( x, y, z );	}
		void SetMax( float x, float y, float z ){ mBox[1].Set( x, y, z );	}
	
		void Reset()
		{
			mBox[0].Set( FLT_MAX, FLT_MAX, FLT_MAX );
			mBox[1].Set( -FLT_MAX, -FLT_MAX, -FLT_MAX );
		}

		void Expand( const Float3 &p )
		{
			mBox[0].x() = DMIN( mBox[0].x(), p.x() );
			mBox[0].y() = DMIN( mBox[0].y(), p.y() );
			mBox[0].z() = DMIN( mBox[0].z(), p.z() );

			mBox[1].x() = DMAX( mBox[1].x(), p.x() );
			mBox[1].y() = DMAX( mBox[1].y(), p.y() );
			mBox[1].z() = DMAX( mBox[1].z(), p.z() );
		}
	
		bool IsValid() const
		{
			return
				mBox[0].x() <= mBox[1].x() &&
				mBox[0].y() <= mBox[1].y() &&
				mBox[0].z() <= mBox[1].z();
		}
	};


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class BiPatch
	{
	public:
		Float3		mHullPos[4];

		float		mURange[2];
		float		mVRange[2];

		iUint		mSplitCnt;
		int			mDiceGridWd;
		int			mDiceGridHe;

		int			mRefCnt;

		bool		mHasPhantomVertex;


		//Constructor ///////////////////////////////////////////////////////////////////
		BiPatch( Float3 patchVerts[] ) :
			mSplitCnt(0),
			mDiceGridWd(-1),
			mDiceGridHe(-1),
			mRefCnt(1),
			mHasPhantomVertex(false)
		{
			mURange[0] = 0.f;					//initial unit range
			mURange[1] = 1.f;
			mVRange[0] = 0.f;
			mVRange[1] = 1.f;

			for (int i=0; i < 4; ++i)		//register patch hull
			mHullPos[i] = patchVerts[i];
		}

		//Copy constructor //////////////////////////////////////////////////////////////
		BiPatch( const BiPatch* _biCopy ) :
			mSplitCnt	( _biCopy->mSplitCnt ),
			mDiceGridWd ( _biCopy->mDiceGridWd ),
			mDiceGridHe ( _biCopy->mDiceGridHe ),
			mHasPhantomVertex ( _biCopy->mHasPhantomVertex ),
			mRefCnt		( 1 )	///mmhh ... 0 or 1 ?!
		{
			mURange[0] = _biCopy->mURange[0];
			mURange[1] = _biCopy->mURange[1];
			mVRange[0] = _biCopy->mVRange[0];
			mVRange[1] = _biCopy->mVRange[1];

			for (int i=0; i < 4; ++i)
			mHullPos[i] = _biCopy->mHullPos[i];
		}


		//Get hull vectors
		iVector3 getHullPos( int i ) { return iVector3(mHullPos[i].x(), mHullPos[i].y(), mHullPos[i].z()); }

		//Get patch vertex
		Float3_ getPatchVtxs()
		{
			Float3 P0,P1,P2,P3;
			float U0 = mURange[0];
			float U1 = mURange[1];
			float V0 = mVRange[0];
			float V1 = mVRange[1];

			P0 = DMix( DMix(mHullPos[0], mHullPos[1], U0), DMix(mHullPos[2], mHullPos[3], U0), V0 );
			P1 = DMix( DMix(mHullPos[0], mHullPos[1], U1), DMix(mHullPos[2], mHullPos[3], U1), V0 );
			P2 = DMix( DMix(mHullPos[0], mHullPos[1], U0), DMix(mHullPos[2], mHullPos[3], U0), V1 );
			P3 = DMix( DMix(mHullPos[0], mHullPos[1], U1), DMix(mHullPos[2], mHullPos[3], U1), V1 );

			Float3_ pVtxs;
			pVtxs[0][0] = P0[0];
			pVtxs[0][1] = P0[1];
			pVtxs[0][2] = P0[2];

			pVtxs[1][0] = P1[0];
			pVtxs[1][1] = P1[1];
			pVtxs[1][2] = P1[2];

			pVtxs[2][0] = P2[0];
			pVtxs[2][1] = P2[1];
			pVtxs[2][2] = P2[2];

			pVtxs[3][0] = P3[0];
			pVtxs[3][1] = P3[1];
			pVtxs[3][2] = P3[2];

			return pVtxs;
		}

		float getHeWdRatio()
		{
			Float3 P0,P1,P2;

			float U0 = mURange[0];
			float U1 = mURange[1];
			float V0 = mVRange[0];
			float V1 = mVRange[1];

			P0 = DMix( DMix(mHullPos[0], mHullPos[1], U0), DMix(mHullPos[2], mHullPos[3], U0), V0 );
			P1 = DMix( DMix(mHullPos[0], mHullPos[1], U1), DMix(mHullPos[2], mHullPos[3], U1), V0 );
			P2 = DMix( DMix(mHullPos[0], mHullPos[1], U0), DMix(mHullPos[2], mHullPos[3], U0), V1 );

			float Wd = geoobj_vtx_distance( iVector3(P1.x(),P1.y(),P1.z()),iVector3(P0.x(),P0.y(),P0.z()) );
			float He = geoobj_vtx_distance( iVector3(P2.x(),P2.y(),P2.z()),iVector3(P0.x(),P0.y(),P0.z()) );

			if(He>Wd) return He/Wd;
			else return Wd/He;
		}

		//Eval_dPdu_dPdv ////////////////////////////////////////////////////////////////
		void Eval_dPdu_dPdv( const Float2_ &uv,
							 Float3_ &out_pt,
							 Float3_ *out_dPdu,
							 Float3_ *out_dPdv ) const
		{
			Float3_ hullPos[4];

			for (int i=0; i < 4; ++i){
				hullPos[i] = mHullPos[i];
			}

			Float3_	left	= DMix( hullPos[0], hullPos[2], uv[1] );
			Float3_	right	= DMix( hullPos[1], hullPos[3], uv[1] );
			out_pt			= DMix( left, right, uv[0] );

			if ( out_dPdu )
			{
				Float3_	left	= hullPos[2] - hullPos[0];
				Float3_	right	= hullPos[3] - hullPos[1];
				*out_dPdv		= DMix( left, right, uv[0] );

				Float3_	bottom	= hullPos[1] - hullPos[0];
				Float3_	top		= hullPos[3] - hullPos[2];
				*out_dPdu		= DMix( bottom, top, uv[1] );
			}
		}

		//EvalP /////////////////////////////////////////////////////////////////////////
		inline void EvalP(	
							const Float_ *pUArray,
							const Float_ *pVArray,
							Float3_ *out_pPts,
							size_t	ptsN ) const
		{
			size_t	blksN =  DMT_SIMD_BLOCKS( ptsN );						//ptsN=4, blksN=1
			for (size_t i=0; i < blksN; ++i)								//ie. for 0-1 UVarray
			{
				Float2_	uv( pUArray[i], pVArray[i] );						//reversed pUVArray
				Eval_dPdu_dPdv( uv, out_pPts[i], NULL, NULL );
			}
		}

		//MakeBound /////////////////////////////////////////////////////////////////////
		static const int _DIM_LEN = 2;
		void MakeBound( Bound &out_bound, Float3_ *out_pPo )
		{
			out_bound.Reset();

			static const size_t	N_ELEMS = _DIM_LEN*_DIM_LEN;				//2*2
			static const size_t	N_ELEMS_PAD = DMT_SIMD_PADSIZE( N_ELEMS );	//4

			float	DVECTOR_SIMD_ALIGN_ARRAYN( us, N_ELEMS_PAD );			//float us[4];
			float	DVECTOR_SIMD_ALIGN_ARRAYN( vs, N_ELEMS_PAD );			//float vs[4];

			bilinearFill(	us,				vs,
							mURange[0],		mVRange[0],
							mURange[1],		mVRange[1],
							_DIM_LEN,		_DIM_LEN,
							N_ELEMS_PAD  );

			EvalP( (const Float_ *)us, (const Float_ *)vs, out_pPo, N_ELEMS );

			for (size_t i=0; i < N_ELEMS; ++i)								//i=4:0-3
			{
				size_t	blk = i / DMT_SIMD_FLEN;							//0
				size_t	sub = i & (DMT_SIMD_FLEN-1);						//sub=i:0-3

				out_bound.Expand( Float3(
										out_pPo[blk][0][sub],
										out_pPo[blk][1][sub],
										out_pPo[blk][2][sub]
									) );
			}
		}

		//CheckForSplit /////////////////////////////////////////////////////////////////
		CheckSplitRes CheckForSplit(	const	TriData& triData, 
										int		out_bound2d[4],
										bool	&out_uSplit,
										bool	&out_vSplit )
		{	
			Matrix44	mtxLocalWorld(true);

			Float3_	testDicePo[ MAX_MAKE_BOUND_OUT_SIZE ];
			Bound bound; bound.Reset();
			MakeBound( bound, testDicePo );

			float pixelArea 
				= triData.RasterEstimate( bound, mtxLocalWorld, out_bound2d );
	
			if ( pixelArea <= MP_GRID_MAX_SIZE )
			{
				if ( pixelArea < RI_EPSILON )
				return CHECKSPLITRES_CULL;

				float	dim = sqrtf( pixelArea );

				//mDiceGridHe = DMT_SIMD_PADSIZE( (int)ceilf( dim ) );
				//mDiceGridWd = (int)ceilf( dim );

				mDiceGridHe = (int)ceilf( dim );
				mDiceGridWd = (int)ceilf( dim );

				if(mDiceGridHe <= 1) mDiceGridHe += triData.dataParams.doubleMicroTriangles;
				if(mDiceGridWd <= 1) mDiceGridWd += triData.dataParams.doubleMicroTriangles;


				out_uSplit = false;
				out_vSplit = false;

				return CHECKSPLITRES_DICE;	// will dice
			}
			else
			{
				out_uSplit = true;
				out_vSplit = true;

				return CHECKSPLITRES_SPLIT;	// will split
			}
		}
		
		//Split /////////////////////////////////////////////////////////////////////////
		void Split( TriData& triData, bool uSplit, bool vSplit )
		{
			//DASSERT( IsUsed() );

			if ( mSplitCnt > 9 )
			{
				// $$$ too many splits !!!
				mSplitCnt = mSplitCnt;
				return;
			}

			if ( uSplit )
			{
				// U split
				BiPatch * pPrimsSU[2] =	{ Clone(), Clone() };

				float	uMid = (mURange[0] + mURange[1]) * 0.5f;
				pPrimsSU[0]->mURange[1] = uMid;
				pPrimsSU[1]->mURange[0] = uMid;

				//add patches from U split
				pPrimsSU[0]->mSplitCnt += 1;
				triData.mPatchProcessorData.push_back( pPrimsSU[0]->Borrow() );

				pPrimsSU[1]->mSplitCnt += 1;
				triData.mPatchProcessorData.push_back( pPrimsSU[1]->Borrow() );
				
				if ( vSplit )
				{
					// optional V split
					float	vMid = (mVRange[0] + mVRange[1]) * 0.5f;

					// check.. because we can't be sure that the primitive didn't fail
					// at insertion time
					BiPatch * pPrimsSV[2] =	{ pPrimsSU[0]->Clone(), pPrimsSU[1]->Clone() };

					for (size_t i=0; i < 2; ++i)
					{
						if ( pPrimsSU[i]->IsUsed() )
						{
							//BiPatch * pNewBiPatch = pPrimsSU[i]->Clone();

							//resize V for previously Usplitted patches
							pPrimsSU[i]->mVRange[1] = vMid;
							pPrimsSV[i]->mVRange[0] = vMid;
						}
					}
					
					//basically we end up with 3 pathces, 
					//two interested by the phantom vtx
					//one as a full valid patch
					if(this->mHasPhantomVertex)
					{
						//V patches
						//discard the upper right patch
						if(pPrimsSV[0]->mVRange[1] > pPrimsSV[1]->mVRange[1])
						{
							pPrimsSV[1]->mSplitCnt += 1;
							triData.mPatchProcessorData.push_back( pPrimsSV[1]->Borrow() );

							pPrimsSV[0]->Release();
						}else
						{
							pPrimsSV[0]->mSplitCnt += 1;
							triData.mPatchProcessorData.push_back( pPrimsSV[0]->Borrow() );

							pPrimsSV[1]->Release();
						}

						//U patches
						//flag bottom left as without phantom vtx
						if(pPrimsSU[0]->mURange[1] > pPrimsSU[1]->mURange[1])
						{
							pPrimsSU[0]->mHasPhantomVertex = true;
							pPrimsSU[1]->mHasPhantomVertex = false;
						}else
						{
							pPrimsSU[0]->mHasPhantomVertex = false;
							pPrimsSU[1]->mHasPhantomVertex = true;
						}			
						
					//add patches from V split
					}else
					{
							pPrimsSV[0]->mSplitCnt += 1;
							triData.mPatchProcessorData.push_back( pPrimsSV[0]->Borrow() );
							pPrimsSV[1]->mSplitCnt += 1;
							triData.mPatchProcessorData.push_back( pPrimsSV[1]->Borrow() );
					}

				}
			}
			else
			{
				// exclusive V split
				if ( vSplit )
				{
					BiPatch *pPrim1 = Clone();
					BiPatch *pPrim2 = Clone();
			
					float	vMid = (mVRange[0] + mVRange[1]) * 0.5f;
					pPrim1->mVRange[1] = vMid;
					pPrim2->mVRange[0] = vMid;
					
					pPrim1->mSplitCnt += 1;
					triData.mPatchProcessorData.push_back( pPrim1->Borrow() );
					pPrim2->mSplitCnt += 1;
					triData.mPatchProcessorData.push_back( pPrim2->Borrow() );
				}
			}

		}

		//CalcLocalUV ///////////////////////////////////////////////////////////////////
		INLINE Float2 CalcLocalUV( float u, float v ) const
		{
			return
				Float2(
					DMix( mURange[0], mURange[1], u ),
					DMix( mVRange[0], mVRange[1], v )
				);
		}
		INLINE iVector2 iCalcLocalUV( float u, float v ) const
		{
			return
				iVector2(
					DMix( mURange[0], mURange[1], u ),
					DMix( mVRange[0], mVRange[1], v )
				);
		}

		//fillUVsArray //////////////////////////////////////////////////////////////////
		inline void fillUVsArray(	iVector2 out_locUV[],
									float du,
									float dv,
									iUint xDim,
									iUint yDim ) const
		{
			size_t	sampleIdx = 0;

			iUint idx = 0;
			float	v = 0.0f;
			for (u_int i=0; i < yDim; ++i, v += dv)
			{
				float	u = 0.0f;
				for (u_int j=0; j < xDim; ++j, u += du, ++sampleIdx)
				{
					iVector2	tmpUV = iCalcLocalUV( u, v );

					out_locUV[idx] = tmpUV;
					idx++;
				}
			}
		}

		//fillUVsArray //////////////////////////////////////////////////////////////////
		inline void fillUVsArray(	Float2_ out_locUV[],
									float du,
									float dv,
									iUint xDim,
									iUint yDim ) const
		{
			size_t	sampleIdx = 0;

			float	v = 0.0f;
			for (u_int i=0; i < yDim; ++i, v += dv)
			{
				float	u = 0.0f;
				for (u_int j=0; j < xDim; ++j, u += du, ++sampleIdx)
				{
					Float2	tmpUV = CalcLocalUV( u, v );

					size_t	blk = sampleIdx / DMT_SIMD_FLEN;
					size_t	sub = sampleIdx & (DMT_SIMD_FLEN-1);

					Float2_	&blkLocUV = out_locUV[ blk ];

					blkLocUV[0][ sub ] = tmpUV[0];
					blkLocUV[1][ sub ] = tmpUV[1];
				}
			}
		}

		//==================================================================
		inline void fillUVsArray(	Float2_ locUV[],
									Float2_ locDUDV[],
									float du,
									float dv,
									iUint xDim,
									iUint yDim ) const
		{
			size_t	sampleIdx = 0;

			Float2	leftUV( 0, 0 );

			float	v = 0.0f;
			for (u_int i=0; i < yDim; ++i, v += dv)
			{
				float	u = 0.0f;
				for (u_int j=0; j < xDim; ++j, u += du, ++sampleIdx)
				{
					Float2	tmpUV = CalcLocalUV( u, v );

					size_t	blk = sampleIdx / DMT_SIMD_FLEN;
					size_t	sub = sampleIdx & (DMT_SIMD_FLEN-1);

					Float2_	&blkLocUV = locUV[ blk ];
					Float2_	&blkLocDUDV = locDUDV[ blk ];

					blkLocUV[0][ sub ] = tmpUV[0];
					blkLocUV[1][ sub ] = tmpUV[1];

					if ( j > 0 )
					{
						blkLocDUDV[0][ sub ] = tmpUV[0] - leftUV[0];
						//blkLocDUDV[1][ sub ] = tmpUV[1] - leftUV[1];
					}

					leftUV = tmpUV;
				}
			}

			u_int xBlkN = DMT_SIMD_BLOCKS( xDim );

			// -- calc du edge (replicates col 1 into col 0..)
			u_int	blk = 0;
			size_t	rightBlk = 1 / DMT_SIMD_FLEN;
			size_t	rightSub = 1 & (DMT_SIMD_FLEN-1);
			for (u_int i=0; i < yDim; ++i, blk += xBlkN)
			{
				locDUDV[ blk ][0][0] = locDUDV[ blk + rightBlk ][0][ rightSub ];
			}

			// -- calc dv
			size_t	blkIdx = xBlkN;
			for (u_int i=1; i < yDim; ++i)
				for (u_int j=0; j < xBlkN; ++j, ++blkIdx)
					locDUDV[ blkIdx ][1] = locUV[blkIdx][1] - locUV[blkIdx - xBlkN][1];

			// row 0 dv = row 1 dv (coarse approximation !)
			for (u_int i=0; i < xBlkN; ++i)
				locDUDV[i][1] = locDUDV[i + xBlkN][1];
		}

		//Memory management /////////////////////////////////////////////////////////////
		void	Release()		{ if(0 == mRefCnt--) delete this; }
		BiPatch *Borrow()		{ mRefCnt += 1; return this; }
		BiPatch *Clone() const	{ return new BiPatch( *this ); }
		bool	IsUsed() const	{ return mRefCnt != 0; }
	};


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct TriData
	{
		iVector3	tA;
		iVector3	tB;
		iVector3	tC;
		iVector3	tNormal;
		
		camOptions mCamOptions;

		miState*	state;
		tParams		dataParams;


		//InitWorld /////////////////////////////////////////////////////////////////////
		void InitWorld( miState*	iState )
		{
			if(iState==NULL) 
			{
				//if(0) {
				mCamOptions.mXRes = 640;
				mCamOptions.mYRes = 480;

				mCamOptions.mPixelAspectRatio = 1.0f;
				mCamOptions.mFrameAspectRatio = (float)mCamOptions.mXRes / (float)mCamOptions.mYRes;

				mCamOptions.mFOV = 54.43f;
				mCamOptions.mNearClip = RI_EPSILON;
				mCamOptions.mFarClip = RI_INFINITY;

				if ( mCamOptions.mFrameAspectRatio >= 1.0f )	
				{
					mCamOptions.mLeft	= -mCamOptions.mFrameAspectRatio;
					mCamOptions.mRight	=  mCamOptions.mFrameAspectRatio;
					mCamOptions.mBottom = -1.0f;
					mCamOptions.mTop	=  1.0f;
				}
				else
				{
					mCamOptions.mLeft	= -1.0f;
					mCamOptions.mRight	=  1.0f;
					mCamOptions.mBottom = -1.0f / mCamOptions.mFrameAspectRatio;
					mCamOptions.mTop	=  1.0f / mCamOptions.mFrameAspectRatio;
				}

				mCamOptions.mXMin = 0;
				mCamOptions.mXMax = 1;
				mCamOptions.mYMin = 0;
				mCamOptions.mYMax = 1;

				mCamOptions.mMtxWorldCamera.v16[0] =	0.702153f;
				mCamOptions.mMtxWorldCamera.v16[1] =	0.333599f;
				mCamOptions.mMtxWorldCamera.v16[2] =	-0.629041f;
				mCamOptions.mMtxWorldCamera.v16[3] =	0.f;

				mCamOptions.mMtxWorldCamera.v16[4] =	0.f;
				mCamOptions.mMtxWorldCamera.v16[5] =	0.883452f;
				mCamOptions.mMtxWorldCamera.v16[6] =	0.468521f;
				mCamOptions.mMtxWorldCamera.v16[7] =	0.f;

				mCamOptions.mMtxWorldCamera.v16[8] =	0.712026f;
				mCamOptions.mMtxWorldCamera.v16[9] =	-0.328974f; 
				mCamOptions.mMtxWorldCamera.v16[10] =	0.620319f; 
				mCamOptions.mMtxWorldCamera.v16[11] =	0.f;

				mCamOptions.mMtxWorldCamera.v16[12] =	7.02734f;
				mCamOptions.mMtxWorldCamera.v16[13] =	-3.20695f;
				mCamOptions.mMtxWorldCamera.v16[14] =	-42.455f;
				mCamOptions.mMtxWorldCamera.v16[15] =	1.f;

			}

#if !__CPC_TESTS__
			else	//get stuff from micamera
			{

				this->state = iState;
				//this->shading_rate = dataParams.shading_rate;

				mCamOptions.mXRes = state->camera->x_resolution * dataParams.shading_rate;
				mCamOptions.mYRes = state->camera->y_resolution * dataParams.shading_rate;

				mCamOptions.mPixelAspectRatio = 1.0f;
				mCamOptions.mFrameAspectRatio = state->camera->aspect;

				mCamOptions.mFOV = 2.0f*atanf( state->camera->aperture/(2.0f*state->camera->focal) ) * (180.f/M_PI);
				mCamOptions.mNearClip = state->camera->clip.min;	//RI_EPSILON;
				mCamOptions.mFarClip = state->camera->clip.max;		//RI_INFINITY;


				if ( mCamOptions.mFrameAspectRatio >= 1.0f )
				{
					mCamOptions.mLeft	= -mCamOptions.mFrameAspectRatio;
					mCamOptions.mRight	=  mCamOptions.mFrameAspectRatio;
					mCamOptions.mBottom = -1.0f;
					mCamOptions.mTop	=  1.0f;
				}
				else
				{
					mCamOptions.mLeft	= -1.0f;
					mCamOptions.mRight	=  1.0f;
					mCamOptions.mBottom = -1.0f / mCamOptions.mFrameAspectRatio;
					mCamOptions.mTop	=  1.0f / mCamOptions.mFrameAspectRatio;
				}

				mCamOptions.mXMin = 0;
				mCamOptions.mXMax = 1;
				mCamOptions.mYMin = 0;
				mCamOptions.mYMax = 1;

				//camera matrix
				miInstance *iCam = (miInstance *) mi_db_access(state->camera_inst);
				miMatrix iMatrix;
				mi_matrix_copy( iMatrix, iCam->tf.global_to_local);
				mi_db_unpin( state->camera_inst );

				mCamOptions.mMtxWorldCamera.v16[0] =	iMatrix[0];
				mCamOptions.mMtxWorldCamera.v16[1] =	iMatrix[1];
				mCamOptions.mMtxWorldCamera.v16[2] =	iMatrix[2];
				mCamOptions.mMtxWorldCamera.v16[3] =	iMatrix[3];

				mCamOptions.mMtxWorldCamera.v16[4] =	iMatrix[4];
				mCamOptions.mMtxWorldCamera.v16[5] =	iMatrix[5];
				mCamOptions.mMtxWorldCamera.v16[6] =	iMatrix[6];
				mCamOptions.mMtxWorldCamera.v16[7] =	iMatrix[7];

				mCamOptions.mMtxWorldCamera.v16[8] =	iMatrix[8];
				mCamOptions.mMtxWorldCamera.v16[9] =	iMatrix[9]; 
				mCamOptions.mMtxWorldCamera.v16[10] =	iMatrix[10]; 
				mCamOptions.mMtxWorldCamera.v16[11] =	iMatrix[11];

				mCamOptions.mMtxWorldCamera.v16[12] =	iMatrix[12];
				mCamOptions.mMtxWorldCamera.v16[13] =	iMatrix[13];
				mCamOptions.mMtxWorldCamera.v16[14] =	iMatrix[14];
				mCamOptions.mMtxWorldCamera.v16[15] =	iMatrix[15];
			}
#endif
			
			mCamOptions.mMtxCamProj =
				Matrix44::PerspectiveRH0( DEG2RAD( mCamOptions.mFOV),
										mCamOptions.mFrameAspectRatio,
										mCamOptions.mNearClip, mCamOptions.mFarClip );

			mCamOptions.mMtxWorldProj = 
				mCamOptions.mMtxWorldCamera * mCamOptions.mMtxCamProj;
		}

		//MakeRasterBound ///////////////////////////////////////////////////////////////
		bool MakeRasterBound(	const Bound &b,
								const Matrix44 &mtxLocalWorld,
								float out_bound2d[4] ) const
		{
			Float3	boxVerts[8];
			MakeCube( b, boxVerts );

			float destHalfWd	= (float)mCamOptions.mXRes * 0.5f;
			float destHalfHe	= (float)mCamOptions.mYRes * 0.5f;

			float minX =  FLT_MAX;
			float minY =  FLT_MAX;
			float maxX = -FLT_MAX;
			float maxY = -FLT_MAX;

			Matrix44	mtxLocalProj = mtxLocalWorld *mCamOptions.mMtxWorldProj;

			static const U32	CCODE_X1 = 1;
			static const U32	CCODE_X2 = 2;
			static const U32	CCODE_Y1 = 4;
			static const U32	CCODE_Y2 = 8;
			static const U32	CCODE_Z1 = 16;
			static const U32	CCODE_Z2 = 32;

			U32 orCode = 0;
			U32 andCode = 0xff;

			for (size_t i=0; i < 8; ++i)
			{
				Float4	Pproj = V4__V3W1_Mul_M44<float>( boxVerts[i], mtxLocalProj );

				float	x = Pproj.x();
				float	y = Pproj.y();
				float	z = Pproj.z();
				float	w = Pproj.w();
		
				U32 code = 0;
				if ( x < -w )	code |= CCODE_X1;
				if ( x >  w )	code |= CCODE_X2;
				if ( y < -w )	code |= CCODE_Y1;
				if ( y >  w )	code |= CCODE_Y2;
				if ( z < -w )	code |= CCODE_Z1;
				if ( z >  w )	code |= CCODE_Z2;

				orCode |= code;
				andCode &= code;

				if ( w > 0 )
				{
					float	oow = 1.0f / w;

					float	winX = destHalfWd + destHalfWd * x * oow;
					float	winY = destHalfHe - destHalfHe * y * oow;

					minX = DMIN( minX, winX );
					minY = DMIN( minY, winY );
					maxX = DMAX( maxX, winX );
					maxY = DMAX( maxY, winY );
				}
				else
				{
					// $$$ this shouldn't happen ..once proper
					// front plane clipping is implemented 8)
					return false;
				}
			}

			if ( andCode )
				return false;

			out_bound2d[0] = minX;
			out_bound2d[1] = minY;
			out_bound2d[2] = maxX;
			out_bound2d[3] = maxY;

			return minX < maxX && minY < maxY;
		}

		//RasterEstimate ////////////////////////////////////////////////////////////////
		float RasterEstimate( const Bound &b, const Matrix44 &mtxLocalWorld, int out_box2D[4] ) const
		{
			if ( !b.IsValid() )
			{
				return MP_GRID_MAX_SIZE / 4;
			}

			float	bound2df[4];

			bool valid = MakeRasterBound( b, mtxLocalWorld, bound2df );

			out_box2D[0] = (int)floorf( bound2df[0] );
			out_box2D[1] = (int)floorf( bound2df[1] );
			out_box2D[2] =  (int)ceilf( bound2df[2] );
			out_box2D[3] =  (int)ceilf( bound2df[3] );

			if ( valid )
			{
				float	squareArea = (bound2df[3] - bound2df[1]) * (bound2df[2] - bound2df[0]);
				return squareArea * 1.0f;
			}
			else
				return 0.0f;	// invalid or zero area...
		}

		//Get data
		miState* getTriState() { return state; };
		iVector3 getTriNormal() { return tNormal; };
		iUint	 getTriIdx()	{ return dataParams.tIdx; };
		iVector3 getTA() {return tA; };
		iVector3 getTB() {return tB; };
		iVector3 getTC() {return tC; };

		//Storage ///////////////////////////////////////////////////////////////////////
		vector<DTYPE*>		vpointlist;
		vector<BiPatch*>	mPatchProcessorData;
		vector<BiPatch*>	mPatchSplatterData;
	};

	TriData GetTriData(){ return _tridata; };


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Simplify triangle as three BiPatches
	void PatchTriangle()
	{
		
		Float3 v1;
		v1.x()=_tridata.tA[0];
		v1.y()=_tridata.tA[1];
		v1.z()=_tridata.tA[2];
		Float3 v2;
		v2.x()=_tridata.tB[0];
		v2.y()=_tridata.tB[1];
		v2.z()=_tridata.tB[2];
		Float3 v3;
		v3.x()=_tridata.tC[0];
		v3.y()=_tridata.tC[1];
		v3.z()=_tridata.tC[2];

		//fourth phantom vtx patching
		triedge AB( v1,v2, v3 );
		triedge BC( v2,v3, v1 );
		triedge CA( v3,v1, v2 );

		vector< triedge* > edgeList;
		edgeList.resize(3);
		edgeList[0] = &AB;
		edgeList[1] = &BC;
		edgeList[2] = &CA;

		sort( edgeList.begin(), edgeList.end(), orderTriEdges );
		
		//	C     D=(B-A)+C
		//	......
		//	|\   .
		//	|+\  .
		//	|++\ .	//empty tri is U+V > 1
		//	|+++\.
		//	------	
		//	A     B
		triedge minEdge = edgeList[0];					//shortest side AB	= U
		triedge maxEdge = edgeList[2];					//				CA  = V	-> He, always longest
														//ipotenusa		BC
		int shPt = maxEdge.hasSharedVtx( &minEdge );
		Float3 BB = minEdge[shPt];						//B is the shared vtx
		Float3 AA = minEdge[!shPt];						//A the other one on the shortest side
		Float3 CC = minEdge.thirdVtx;					//C the other one on the ipot
		Float3 DD = (BB-AA)+CC;							//D our fourth phantom vertex

		Float3 P0 = AA;
		Float3 P1 = BB;
		Float3 P2 = CC;
		Float3 P3 = DD;

		//the patch
		Float3	patchVerts[4];

		patchVerts[0] = P0;
		patchVerts[1] = P1;
		patchVerts[2] = P2;
		patchVerts[3] = P3;

		BiPatch * pBiPatch = new BiPatch(patchVerts);

		//phantom vtx flag is used only for full patches..
		//where we exclude a patch while UV splitting
		pBiPatch->mHasPhantomVertex = true;
		_tridata.mPatchProcessorData.push_back( pBiPatch );


		//check patch ratio
		if( pBiPatch->getHeWdRatio() >= 2.f )
		{
			// pre-split V to approach square patches
			pBiPatch->mHasPhantomVertex = false;
			//for patches splitted in V we use the U+V >1 to
			//exclude unwanted vertices from doubled tri patch.

			int iRatio = (int)ceil( pBiPatch->getHeWdRatio() );
			float vCut = 1.f / (float)iRatio;

			for(int i=0; i<iRatio; i++)
			{
				BiPatch *pPrim = pBiPatch->Clone();

				pPrim->mVRange[0] = vCut *i;
				pPrim->mVRange[1] = vCut *(i+1);

				pPrim->mSplitCnt += 1;
				_tridata.mPatchProcessorData.push_back( pPrim->Borrow() );
			}

			pBiPatch->Release();
			_tridata.mPatchProcessorData[0] = NULL;
		}

		edgeList.clear();
}


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void ProcessPatches()
	{
		//here we recursively split patches until they are under a certain screen size
		//threshold and add them to the mPatchSplatterData ready for dicing/splatting
		for (size_t i=0; i < _tridata.mPatchProcessorData.size(); ++i)
		{
			if ( ! _tridata.mPatchProcessorData[i] )
			continue;


			BiPatch	*pPrim = (BiPatch *)_tridata.mPatchProcessorData[i];

			int		bound2d[4];
			bool	uSplit,	vSplit;

			//check splitting
			CheckSplitRes dosRes =
			pPrim->CheckForSplit( _tridata, bound2d, uSplit, vSplit );


			if ( dosRes == CHECKSPLITRES_DICE )		//go for dicing
			{
				_tridata.mPatchSplatterData.push_back( pPrim->Borrow() );
			}
			else
			if ( dosRes == CHECKSPLITRES_SPLIT )	//split again
			{
				pPrim->Split( _tridata, uSplit, vSplit );
			}

			// otherwise it's cull..
			pPrim->Release();
			_tridata.mPatchProcessorData[i] = NULL;
		}

		//clear up the PatchProcessorData list ...
		//while we don't need the list anymore, there may be bipatches pointers from the list 
		//still pointing to valid data that have been inserted for dicing (ie. borrowed)
		_tridata.mPatchProcessorData.clear();
	}

	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DicePatches(bool fixarea=false)
	{
		for (size_t i=0; i < _tridata.mPatchSplatterData.size(); ++i)
		{
			if ( ! _tridata.mPatchSplatterData[i] )
			continue;

			BiPatch	*pPrim = (BiPatch *)_tridata.mPatchSplatterData[i];

			{
				//float rratio = pPrim->getHeWdRatio();
				int gridHe = pPrim->mDiceGridHe ;
				int gridWd = pPrim->mDiceGridWd	;//*rratio;

				float	du = 1.0f / (gridWd -1);
				float	dv = 1.0f / (gridHe -1);

				//micropoly vtxs grid
				vector< iMicroVertex > gridPoints;

				//build the uv, dudv
				//Float2_		locUV[ MP_GRID_MAX_SIZE_SIMD_BLKS *4];
				iVector2	iLocUV[ MP_GRID_MAX_SIZE_SIMD_BLKS *4];

				//pPrim->fillUVsArray( locUV, du, dv, gridWd, gridHe );
				pPrim->fillUVsArray( iLocUV, du, dv, gridWd, gridHe );

				//scan micropoly for microvtxs and uvs and put them 
				//in a linear array interleaved by linesize (gridWd) 
				//fillUVsArray already did that for needed uvs 
				size_t	blocksN = ( gridWd * gridHe );
				for (size_t blkIdx=0; blkIdx < blocksN; ++blkIdx)
				{
					iVector2 iUV = iLocUV[blkIdx];
					iVector3 iVtx = DMix( DMix(pPrim->getHullPos(0), pPrim->getHullPos(1), iUV[0]), DMix(pPrim->getHullPos(2), pPrim->getHullPos(3), iUV[0]), iUV[1] );

					gridPoints.push_back(iMicroVertex( iVtx, iUV ));
				}

				//or do the same in block of 4 at time to exploit SSE
				//we can't do that however for micro-patches as we would
				//have a fixed 4 as gridHe for all patches of dim 1px
				//ie. any minimal patch with SSE would be 4x1.

				//we may however just skip that case with the above
				//depending if we plan to remove or not the Flot3_ class

/*				size_t	blocksN = DMT_SIMD_BLOCKS( gridWd * gridHe );
				for (size_t blkIdx=0; blkIdx < blocksN; ++blkIdx)
				{
					Float3_	posLS;
					Float2_ uv = locUV[blkIdx];
					pPrim->Eval_dPdu_dPdv( uv, posLS, NULL, NULL );

					iVector3 pp[4];
					pp[0] = iVector3( posLS[0][0], posLS[1][0], posLS[2][0] );
					pp[1] = iVector3( posLS[0][1], posLS[1][1], posLS[2][1] );
					pp[2] = iVector3( posLS[0][2], posLS[1][2], posLS[2][2] );
					pp[3] = iVector3( posLS[0][3], posLS[1][3], posLS[2][3] );

					iVector2 vtxUV[4];
					vtxUV[0] = iVector2( uv[0][0], uv[1][0] );
					vtxUV[1] = iVector2( uv[0][1], uv[1][1] );
					vtxUV[2] = iVector2( uv[0][2], uv[1][2] );
					vtxUV[3] = iVector2( uv[0][3], uv[1][3] );

					gridPoints.push_back(iMicroVertex( pp[0], vtxUV[0] ));
					gridPoints.push_back(iMicroVertex( pp[1], vtxUV[1] ));
					gridPoints.push_back(iMicroVertex( pp[2], vtxUV[2] ));
					gridPoints.push_back(iMicroVertex( pp[3], vtxUV[3] ));
				}
*/
				//build centroids grid and add to point cloud
				int linesize = gridWd;
				int gridsize = (gridWd*gridHe)-linesize;

				for(int x=0; x<gridsize; x++)
				{
					iMicroVertex pp[4];

					//build micropatch 
					//we get the first 2 vtxs in U, and then the first 2 vtxs in V
					//that's our patch. we then get the centroid with uv(0.5)
					//+++++
					//+V+V+
					//+++++
					//+U+U+ .. ......................
					//+++++ .. and so on, on the right ->

					int lastlinepos = (x+1)%linesize;
					if( !lastlinepos )	//that means we stop before last line, as two uv
					continue;			//lines are involved, and none above last ;)


					//get the above from a linear array ..
					//the U line is long gridWd, after each U block
					//we skip up a V step, v and v+1 complete the patch
					pp[0] = gridPoints[ x ];
					pp[1] = gridPoints[ x+1 ];
					pp[2] = gridPoints[ x+gridWd ];
					pp[3] = gridPoints[ x+gridWd+1 ];


					//skip phantom triangle vtxs ////////////////////////////////////////
					//if(pPrim->mHasPhantomVertex)
					{
						float iU = (pp[0].getU() + pp[1].getU()) / 2.f;
						float iV = (pp[0].getV() + pp[2].getV()) / 2.f;

						if( iU+iV > 1.f +epsilon ) //ie. only left triangle
						continue;	/////////////////////////////////////////////////////
					}


					//calculate micro-patch centroid ////////////////////////////////////
					float u,v; u=v=0.5f;
					iVector3 pCentroid = DMix( DMix(iVector3(pp[0]), iVector3(pp[1]), u), DMix(iVector3(pp[2]), iVector3(pp[3]), u), v );

					//calculate radius
					//float maxRadius = geoobj_vtx_distance( pCentroid, pp[0]);
					//for(int i = 1; i < 4; ++i)
					////maxRadius = max( maxRadius, geoobj_vtx_distance( pCentroid, pp[i]) );
					//maxRadius += geoobj_vtx_distance( pCentroid, pp[i] );
					//maxRadius /= 4.f;
					//float maxArea = maxRadius*maxRadius*M_PI;

					//calculate area 
					float areaVal20 = geoobj_tri_area(pp[0], pp[1], pp[3]);
					//float areaVal21 = geoobj_tri_area(pp[3], pp[2], pp[1]);
					float trueArea = areaVal20 *2;//+ areaVal21;
					float trueRadius = sqrtf(trueArea/(float)M_PI);
					
					//float medRadius = (maxRadius+trueRadius)/2.f;


					// Add surfel to triangle pointlist	/////////////////////////////////

					// ** init data holder
					float* hdata = iDataAllocator::InitNodeBlock();

					// pos, dir
					iDataAllocator::template SetNodeVectorAt<DATA_SLOT_POS>( hdata, pCentroid );
					iDataAllocator::template SetNodeVectorAt<DATA_SLOT_DIR>( hdata, _tridata.tNormal );

					// area/radius 
					iVector3 hd(trueRadius, trueRadius, trueRadius, trueRadius);
					if(fixarea)
						 hd = iVector3 (areaVal20, 1.f/areaVal20, 1.f/(areaVal20*inv_pi), trueRadius);
					
					iDataAllocator::template SetNodeVectorAt<DATA_SLOT_AREA>( hdata, hd );

					_tridata.vpointlist.push_back( hdata );	/////////////////////////////
				}

				gridPoints.clear();	//clear tmp grid pts
			}

			//delete patch
			pPrim->Release();
			_tridata.mPatchSplatterData[i] = NULL;
		}

		//clean splatter data
		_tridata.mPatchSplatterData.clear();
	}


	//Data storage
	TriData _tridata;	/// ///////////////////////////////////////////////////////////////////////////////////////////


	void InitTessellator(	const iVector3& vPosA, const iVector3& vPosB, const iVector3& vPosC,
							miState*	state, tParams triParams, bool fixarea=false )
	{
		//register triangle: vtxpos, trinormal
		_tridata.dataParams = triParams;

		_tridata.tA = vPosA;
		_tridata.tB = vPosB;
		_tridata.tC = vPosC;
		_tridata.tNormal = geoobj_tri_normal(vPosA, vPosB, vPosC);
		
		//init tessellator world
		_tridata.InitWorld( state );

		PatchTriangle();

		//check patch raster bound, split them and eventually 
		//insert em in the PatchSplatterData list for dicing 
		ProcessPatches();

		//dice patches
		DicePatches(fixarea);
	}

	INLINE vector<DTYPE*> getTPointList() const { return _tridata.vpointlist; }	/// don't ..
};



////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Bake class for AmbienOcclusion (minimal bake ... pos, nor, area)
template<
		typename DTYPE										= float,
		unsigned SIZE										= DATA_SLOT_MINIMAL,
		class	 MICache									= iSurfel_MICache,
		template<typename, unsigned> class iNodeAllocator	= iSurfelAllocator,
		template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator	= iTrianleTessellator_Reyes
> 
class iSurfelBaker_AO : private iNodeAllocator<DTYPE, SIZE>
{
public:
	typedef iSurfel_MI_bake_result pcloud_MI_bake_result;
	typedef MICache MICache;
	typedef iNodeAllocator<DTYPE, SIZE> iSurfelData;

	DTYPE* Bake
	(
		pcloud_MI_bake_result const	* a,
		pcloud_MI_bake_result const	* b,
		pcloud_MI_bake_result const	* c
	)
	{
		miVector vPosA, vPosB, vPosC;
		vPosA = a->point;
		vPosB = b->point;
		vPosC = c->point;

		iVector3 hResult;

		// ** init data holder
		float* hdata = iSurfelData::InitNodeBlock();


		// position
		hResult	= geoobj_tri_centroid(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_POS>( hdata, hResult );

		// direction
		hResult	= geoobj_tri_normal(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_DIR>( hdata, hResult );

		// area/radius
		miScalar tArea = geoobj_tri_area(&vPosA, &vPosB, &vPosC);
		#if	__SSE2_ENHANCED__
		iVector3 hd(tArea, 1.f/tArea, 1.f/(tArea*inv_pi), sqrtf(tArea/(float)M_PI));
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_AREA>( hdata, hd );
		#else
		iSurfelData::template SetNodeScalarAt<DATA_SLOT_AREA>( hdata, tArea );
		#endif
		

		// triangle vtxs
		if( iSurfelData::GetNodeSize() > DATA_NSLOTS_AO)
		{
			iSurfelData::template SetNodeVectorAt<DATA_SLOT_T0>( hdata, vPosA );
			iSurfelData::template SetNodeVectorAt<DATA_SLOT_T1>( hdata, vPosB );
			iSurfelData::template SetNodeVectorAt<DATA_SLOT_T2>( hdata, vPosC );
		}

		return hdata;
	}
};


////////////////////////////////////////////////////////////////////////////////////////////////////
// Bake class for AmbienOcclusion (minimal bake ... pos, nor, area)
template<
		typename DTYPE										= float,
		unsigned SIZE										= DATA_SLOT_MINIMAL,
		class	 MICache									= iSurfel_MICache,
		template<typename, unsigned> class iNodeAllocator	= iSurfelAllocator,
		template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator	= iTrianleTessellator_Reyes
> 
class iSurfelSplatter : private iNodeAllocator<DTYPE, SIZE>
{
public:
	typedef iSurfel_MI_bake_result pcloud_MI_bake_result;
	typedef MICache MICache;
	typedef iNodeAllocator<DTYPE, SIZE> iSurfelData;

	struct TriData
	{
		iVector3 tA;
		iVector3 tB;
		iVector3 tC;
		iVector3 tNormal;

		int		depth_max;
		int		depth_current;
		float	length_min;

		vector<iVector3> vpointlist;
	};

	TriData _tridata;

	void SubDivideMe(const iVector3& A, const iVector3& B, const iVector3& C, int depth)
	{
		miVector vPosA, vPosB, vPosC;
		vPosA.x = A[0];		vPosA.y = A[1];		vPosA.z = A[2];
		vPosB.x = B[0];		vPosB.y = B[1];		vPosB.z = B[2];
		vPosC.x = C[0];		vPosC.y = C[1];		vPosC.z = C[2];
		float tArea = geoobj_tri_area(	&vPosA, &vPosB, &vPosC );

		if( tArea > _tridata.length_min && 
			_tridata.depth_current < _tridata.depth_max )
		{
			_tridata.depth_current++;

			iVector3 AB ( geoobj_halfedge(A,B) );
			iVector3 AC ( geoobj_halfedge(A,C) );
			iVector3 BC ( geoobj_halfedge(B,C) );

			//keep clockwise vtx order
			SubDivideMe(A,	AB, AC, _tridata.depth_current);
			SubDivideMe(B,	BC, AB, _tridata.depth_current);
			SubDivideMe(C,	AC, BC, _tridata.depth_current);
			SubDivideMe(AB, BC, AC, _tridata.depth_current);

		}else
		{
			_tridata.vpointlist.push_back( A );
			_tridata.vpointlist.push_back( B );
			_tridata.vpointlist.push_back( C );
		}
	}

	void SplatTri
	(
		pcloud_MI_bake_result const	* a,
		pcloud_MI_bake_result const	* b,
		pcloud_MI_bake_result const	* c,
		const float& splat_length,
		const int& splat_maxrec
	)
	{
		iVector3 vPosA ( a->point );
		iVector3 vPosB ( b->point );
		iVector3 vPosC ( c->point );

		_tridata.tA = vPosA;
		_tridata.tB = vPosB;
		_tridata.tC = vPosC;

		_tridata.tNormal = geoobj_tri_normal(&a->point, &b->point, &c->point);

		_tridata.depth_max		= splat_maxrec;
		_tridata.depth_current	= 0;
		_tridata.length_min		= splat_length;//0.000500f

		//
		SubDivideMe(vPosA, vPosB, vPosC, _tridata.depth_current);
	}

	DTYPE* Bake
	(
		const iVector3& a,
		const iVector3& b,
		const iVector3& c
	)
	{
		miVector vPosA, vPosB, vPosC;
		vPosA.x = a[0];
		vPosA.y = a[1];
		vPosA.z = a[2];

		vPosB.x = b[0];
		vPosB.y = b[1];
		vPosB.z = b[2];		
		
		vPosC.x = c[0];
		vPosC.y = c[1];
		vPosC.z = c[2];	

		iVector3 hResult;

		// ** init data holder
		float* hdata = iSurfelData::InitNodeBlock();


		// position
		hResult	= geoobj_tri_centroid(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_POS>( hdata, hResult );

		// direction
		//hResult	= geoobj_tri_normal(&vPosA, &vPosB, &vPosC);
		hResult = _tridata.tNormal;
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_DIR>( hdata, hResult );

		// area/radius
		miScalar tArea = geoobj_tri_area(&vPosA, &vPosB, &vPosC);
		#if	__SSE2_ENHANCED__
		iVector3 hd(tArea, 1.f/tArea, 1.f/(tArea*inv_pi), sqrtf(tArea/(float)M_PI));
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_AREA>( hdata, hd );
		#else
		iSurfelData::template SetNodeScalarAt<DATA_SLOT_AREA>( hdata, tArea );
		#endif
		

		// triangle vtxs
		if( iSurfelData::GetNodeSize() > DATA_NSLOTS_AO)
		{
			iSurfelData::template SetNodeVectorAt<DATA_SLOT_T0>( hdata, vPosA );
			iSurfelData::template SetNodeVectorAt<DATA_SLOT_T1>( hdata, vPosB );
			iSurfelData::template SetNodeVectorAt<DATA_SLOT_T2>( hdata, vPosC );
		}

		return hdata;
	}
};


////////////////////////////////////////////////////////////////////////////////////////////////////
// Bake class for SubSurfaceScattering
template<
		typename DTYPE										= float,
		unsigned SIZE										= DATA_SLOT_MINIMAL,
		class	 MICache									= iSurfel_MICache,
		template<typename, unsigned> class iNodeAllocator	= iSurfelAllocator,
		template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator	= iTrianleTessellator_Reyes
> 
class iSurfelBaker_SSS : public iNodeAllocator<DTYPE, SIZE>
{
public:
	typedef iSurfel_MI_bake_result pcloud_MI_bake_result;
	typedef MICache MICache;
	typedef iNodeAllocator<DTYPE, SIZE> iSurfelData;

	DTYPE* Bake
	(
		iUint						idx,
		pcloud_MI_bake_result const	* a,
		pcloud_MI_bake_result const	* b,
		pcloud_MI_bake_result const	* c,
		MICache						* iMICache,
		miState						* state,
		miBoolean					FGprepass = miFALSE
	)
	{

		miVector vPosA, vPosB, vPosC;
		vPosA = a->point;
		vPosB = b->point;
		vPosC = c->point;

		iVector3 hResult;

		// ** init data holder
		float* hdata = iSurfelData::InitNodeBlock();


		// position
		hResult	= geoobj_tri_centroid(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_POS>( hdata, hResult );

		// direction
		hResult	= geoobj_tri_normal(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_DIR>( hdata, hResult );

		// area/radius
		miScalar tArea = geoobj_tri_area(&vPosA, &vPosB, &vPosC);
		#if	__SSE2_ENHANCED__
		iVector3 hd(tArea, 1.f/tArea, 1.f/(tArea*inv_pi), sqrtf(tArea/(float)M_PI));
		//iVector3 hd(sqrtf(tArea/(float)M_PI), 1.f/tArea, 1.f/(tArea*inv_pi), tArea );
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_AREA>( hdata, hd );
		#else
		iSurfelData::template SetNodeScalarAt<DATA_SLOT_AREA>( hdata, sqrtf(tArea/(float)M_PI)/*tArea*/ );
		#endif


		// ** bring state to barycenter
		miState* orig_state = state;								//bake state

		// setup QMC
		state->raster_x = (float) idx;
		state->raster_y = (float) idx;
		mi_query(miQ_PIXEL_SAMPLE, state, 0, NULL);

		// set point and normal
		state->point = iSurfelData::template GetNodeVectorAt<DATA_SLOT_POS>( hdata ).AsMiVector();
		state->normal = iSurfelData::template GetNodeVectorAt<DATA_SLOT_DIR>( hdata ).AsMiVector();
		state->normal_geom = state->normal ;

		// calculate org, dir, dot_nd and dist
		origin_from_camera(state, &state->org);						//org

		mi_vector_sub(&state->dir, &state->point, &state->org);		//dir
		mi_vector_normalize(&state->dir);

		state->dist  = mi_vector_norm(&state->dir);					//dist
		state->dot_nd = mi_vector_dot(&state->normal, &state->dir);	//dot_nd
		mi_db_unpin(state->camera_inst);

		// Backfacing?
		//if (state->dot_nd > 0.0f) {
		if (iMICache->backfacing && state->dot_nd > 0.0f) {			// disable for FG

			mi_vector_neg(&state->normal_geom);
			mi_vector_neg(&state->normal);
			state->inv_normal = miTRUE;
			state->dot_nd = -state->dot_nd;
		}

		// setup shadow_tol
		if (state->options->shadow)
		SET_SHADOW_TOLERANCE(&vPosA, &vPosB, &vPosC);

		// other state stuff to interpbary
		initIntersectionArrays(state);


		// FG preprocessing ...
		if(FGprepass)
		{
			// .. place a FG point at every vertex
			miColor tempvalue; /* Never actually used */
			mi_finalgather_store(&tempvalue, state, miFG_STORE_COMPUTE);

		// Evaluate irradiance shader ...
		}else	
		{
			miColor	irrad = {1,1,1,1};								// irradiance
			if(mi_call_shader_x(&irrad, miSHADER_MATERIAL, state, iMICache->irrad_tag, 0))
			{
				hResult = irrad;
				iSurfelData::template SetNodeVectorAt<DATA_SLOT_IRRAD>( hdata, hResult );
			}


			if(iMICache->albedo_tag){
				miColor	albedo = {.5f, .5f, .5f, 1};				// albedo
				if(mi_call_shader_x(&albedo, miSHADER_MATERIAL, state, iMICache->albedo_tag, 0)){
					hResult[0] = albedo.r * iMICache->albedo.r;
					hResult[1] = albedo.g * iMICache->albedo.g;
					hResult[2] = albedo.b * iMICache->albedo.b;
					//hdata.albedo = albedo;
				}
				else hResult = iMICache->albedo;
			}	else hResult = iMICache->albedo;

			iSurfelData::template SetNodeVectorAt<DATA_SLOT_ALBEDO>( hdata, hResult );


			if(iMICache->mfpath_tag){
				miColor	mfpath = {.5f, .5f, .5f, 1};				// meanfreepath
				if(mi_call_shader_x(&mfpath, miSHADER_MATERIAL, state, iMICache->mfpath_tag, 0)){
					hResult[0] = mfpath.r * iMICache->mfpath.r;
					hResult[1] = mfpath.g * iMICache->mfpath.g;
					hResult[2] = mfpath.b * iMICache->mfpath.b;
					//hdata.meanfpath = mfpath;
				}
				else hResult = iMICache->mfpath;
			}	else hResult = iMICache->mfpath;

			iSurfelData::template SetNodeVectorAt<DATA_SLOT_DMFP>( hdata, hResult );
			//hdata.tex = state->tex_list[0]; //TODO: actually it is never used ?
		}
		state = orig_state;

		// return with data
		return hdata;
		}
};

//
template
<
		typename DTYPE																			= float,
		unsigned SIZE																			= DATA_SLOT_MINIMAL,
		class	 MICache																		= iSurfel_MICache,
		template<typename, unsigned> class iNodeAllocator										= iSurfelAllocator,
		template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator	= iTrianleTessellator_Reyes
> 
class iSurfelBaker_ReyesSSS :	public iTriTessellator<DTYPE, SIZE, iNodeAllocator>
{
public:
	typedef iSurfel_MI_bake_result							pcloud_MI_bake_result;
	typedef MICache											MICache;
	typedef iNodeAllocator<DTYPE, SIZE>						iSurfelData;
	typedef iTriTessellator<DTYPE, SIZE, iNodeAllocator>	iTessellator;


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void SplatTri
	(
		pcloud_MI_bake_result const	* a,
		pcloud_MI_bake_result const	* b,
		pcloud_MI_bake_result const	* c,
		miState*	state,
		iUint		idx,
		float		shading_rate,
		int			double_microtriangles
	)
	{
		iVector3 vPosA ( a->point );
		iVector3 vPosB ( b->point );
		iVector3 vPosC ( c->point );

		tParams triParams;
		triParams.shading_rate			= shading_rate;
		triParams.tIdx					= idx;
		triParams.doubleMicroTriangles	= double_microtriangles;

		InitTessellator( vPosA, vPosB, vPosC, state, triParams, miTRUE );
	}

	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Bake( MICache * iMICache, miState * iState, bool FGprepass = miFALSE )
	{

	for(int i=0; i< _tridata.vpointlist.size(); i++)
	{
			iUint idx			= _tridata.getTriIdx();
			miState* state		= iState;	//_tridata.getTriState();
			//miBoolean FGprepass = miFALSE;

			iVector3 hResult;

			// ** init data holder
			DTYPE* hdata = _tridata.vpointlist[i];


			// ** bring state to barycenter
			miState* orig_state = state;								//bake state

			// setup QMC
			state->raster_x = (float) i;	///was.. idx !
			state->raster_y = (float) i;
			mi_query(miQ_PIXEL_SAMPLE, state, 0, NULL);

			// set point and normal
			state->point = iSurfelData::template GetNodeVectorAt<DATA_SLOT_POS>( hdata ).AsMiVector();
			state->normal = iSurfelData::template GetNodeVectorAt<DATA_SLOT_DIR>( hdata ).AsMiVector();
			state->normal_geom = state->normal ;

			// calculate org, dir, dot_nd and dist
			origin_from_camera(state, &state->org);						//org


#if	__SSE2_ENHANCED__
			iVector3 idir ( iVector3(state->point) - iVector3(state->org) );
			state->dir = idir.normalize().AsMiVector();

			state->dist  = iVector3(&state->dir).length();				//dist
			state->dot_nd = iVector3(&state->normal).dot(&state->dir);	//dot_nd
			//mi_db_unpin(state->camera_inst);
#else
			mi_vector_sub(&state->dir, &state->point, &state->org);		//dir
			mi_vector_normalize(&state->dir);

			state->dist  = mi_vector_norm(&state->dir);					//dist
			state->dot_nd = mi_vector_dot(&state->normal, &state->dir);	//dot_nd
			//mi_db_unpin(state->camera_inst);
#endif


			// Backfacing?
			if (iMICache->backfacing && state->dot_nd > 0.0f) {			// disable for FG

				mi_vector_neg(&state->normal_geom);
				mi_vector_neg(&state->normal);
				state->inv_normal = miTRUE;
				state->dot_nd = -state->dot_nd;
			}

			// setup shadow_tol
			if (state->options->shadow)
			{
				miVector tA = _tridata.getTA().AsMiVector();
				miVector tB = _tridata.getTB().AsMiVector();
				miVector tC = _tridata.getTC().AsMiVector();

				SET_SHADOW_TOLERANCE( &tA, &tB, &tC );
			}

			// other state stuff to interpbary
			initIntersectionArrays(state);


			// FG preprocessing ...
			if(FGprepass && ( i%iMICache->fgprebake == 0 ))
			{
				// .. place a FG point at every vertex
				miColor tempvalue; // Never actually used
				mi_finalgather_store(&tempvalue, state, miFG_STORE_COMPUTE);

			// Evaluate irradiance shader ...
			}else	
			{

			
				miColor	irrad = {1,1,1,1};								// irradiance
				if(mi_call_shader_x(&irrad, miSHADER_MATERIAL, state, iMICache->irrad_tag, 0))
				{
					hResult = irrad;
					iSurfelData::template SetNodeVectorAt<DATA_SLOT_IRRAD>( hdata, hResult );
				}


				if(iMICache->albedo_tag){
					miColor	albedo = {.5f, .5f, .5f, 1};				// albedo
					if(mi_call_shader_x(&albedo, miSHADER_MATERIAL, state, iMICache->albedo_tag, 0)){
						hResult[0] = albedo.r * iMICache->albedo.r;
						hResult[1] = albedo.g * iMICache->albedo.g;
						hResult[2] = albedo.b * iMICache->albedo.b;
						//hdata.albedo = albedo;
					}
					else hResult = iMICache->albedo;
				}	else hResult = iMICache->albedo;

				iSurfelData::template SetNodeVectorAt<DATA_SLOT_ALBEDO>( hdata, hResult );


				if(iMICache->mfpath_tag){
					miColor	mfpath = {.5f, .5f, .5f, 1};				// meanfreepath
					if(mi_call_shader_x(&mfpath, miSHADER_MATERIAL, state, iMICache->mfpath_tag, 0)){
						hResult[0] = mfpath.r * iMICache->mfpath.r;
						hResult[1] = mfpath.g * iMICache->mfpath.g;
						hResult[2] = mfpath.b * iMICache->mfpath.b;
						//hdata.meanfpath = mfpath;
					}
					else hResult = iMICache->mfpath;
				}	else hResult = iMICache->mfpath;

				iSurfelData::template SetNodeVectorAt<DATA_SLOT_DMFP>( hdata, hResult );
				//hdata.tex = state->tex_list[0]; //TODO: actually it is never used ?

			}

			state = orig_state;
	}
	}
};



////////////////////////////////////////////////////////////////////////////////////////////////////
// Bake class for SubSurfaceScattering
template
<
		typename DTYPE										= float,
		unsigned SIZE										= DATA_SLOT_MINIMAL,
		class	 MICache									= iSurfel_MICache,
		template<typename, unsigned> class iNodeAllocator	= iSurfelAllocator,
		template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator	= iTrianleTessellator_Reyes
> 
class iSurfelBaker_Attr : public iNodeAllocator<DTYPE, SIZE>
{
public:
	typedef iSurfel_MI_bake_result pcloud_MI_bake_result;
	typedef MICache MICache;
	typedef iNodeAllocator<DTYPE, SIZE> iSurfelData;

	DTYPE* Bake
	(
		iUint						idx,
		pcloud_MI_bake_result const	* a,
		pcloud_MI_bake_result const	* b,
		pcloud_MI_bake_result const	* c,
		MICache						* iMICache,
		miState						* state,
		miBoolean					FGprepass = miFALSE
	)
	{

		miVector vPosA, vPosB, vPosC;
		vPosA = a->point;
		vPosB = b->point;
		vPosC = c->point;

		iVector3 hResult;

		// ** init data holder
		float* hdata = iSurfelData::InitNodeBlock();


		// position
		hResult	= geoobj_tri_centroid(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_POS>( hdata, hResult );

		// direction
		hResult	= geoobj_tri_normal(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_DIR>( hdata, hResult );

		// area/radius
		miScalar tArea = geoobj_tri_area(&vPosA, &vPosB, &vPosC);
		#if	__SSE2_ENHANCED__
		iVector3 hd(tArea, 1.f/tArea, 1.f/(tArea*inv_pi), sqrtf(tArea/(float)M_PI));
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_AREA>( hdata, hd );
		#else
		iSurfelData::template SetNodeScalarAt<DATA_SLOT_AREA>( hdata, tArea );
		#endif


		// ** bring state to barycenter
		miState* orig_state = state;								//bake state

		// setup QMC
		state->raster_x = (float) idx;
		state->raster_y = (float) idx;
		mi_query(miQ_PIXEL_SAMPLE, state, 0, NULL);

		// set point and normal
		state->point = iSurfelData::template GetNodeVectorAt<DATA_SLOT_POS>( hdata ).AsMiVector();
		state->normal = iSurfelData::template GetNodeVectorAt<DATA_SLOT_DIR>( hdata ).AsMiVector();
		state->normal_geom = state->normal ;

		// calculate org, dir, dot_nd and dist
		origin_from_camera(state, &state->org);						//org


#if	__SSE2_ENHANCED__
		iVector3 idir ( iVector3(state->point) - iVector3(state->org) );
		state->dir = idir.normalize().AsMiVector();

		state->dist  = iVector3(&state->dir).length();				//dist
		state->dot_nd = iVector3(&state->normal).dot(&state->dir);	//dot_nd
		mi_db_unpin(state->camera_inst);
#else
		mi_vector_sub(&state->dir, &state->point, &state->org);		//dir
		mi_vector_normalize(&state->dir);

		state->dist  = mi_vector_norm(&state->dir);					//dist
		state->dot_nd = mi_vector_dot(&state->normal, &state->dir);	//dot_nd
		mi_db_unpin(state->camera_inst);
#endif

		// Backfacing?
		if (iMICache->backfacing && state->dot_nd > 0.0f) {			// disable for FG

			mi_vector_neg(&state->normal_geom);
			mi_vector_neg(&state->normal);
			state->inv_normal = miTRUE;
			state->dot_nd = -state->dot_nd;
		}

		// setup shadow_tol
		if (state->options->shadow)
		SET_SHADOW_TOLERANCE(&vPosA, &vPosB, &vPosC);

		// other state stuff to interpbary
		initIntersectionArrays(state);


		// FG preprocessing ...
		if(FGprepass)
		{
			// .. place a FG point at every vertex
			miColor tempvalue; /* Never actually used */
			mi_finalgather_store(&tempvalue, state, miFG_STORE_COMPUTE);

		// Evaluate irradiance shader ...
		}else	
		{
			miColor	irrad = {1,1,1,1};								// irradiance
			if(mi_call_shader_x(&irrad, miSHADER_MATERIAL, state, iMICache->irrad_tag, 0))
			{
				hResult = irrad;
				iSurfelData::template SetNodeVectorAt<DATA_SLOT_BATTR>( hdata, hResult );
			}
		}

		state = orig_state;

		// return with data
		return hdata;
		}
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Bake class for GI
template
<
		typename DTYPE										= float,
		unsigned SIZE										= DATA_SLOT_MINIMAL,
		class	 MICache									= iSurfel_MICache,
		template<typename, unsigned> class iNodeAllocator	= iSurfelAllocator,
		template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator	= iTrianleTessellator_Reyes
> 
class iSurfelBaker_AttrSplatter : public iNodeAllocator<DTYPE, SIZE>
{
public:
	typedef iSurfel_MI_bake_result pcloud_MI_bake_result;
	typedef MICache MICache;
	typedef iNodeAllocator<DTYPE, SIZE> iSurfelData;

	struct TriData
	{
		iVector3 tA;
		iVector3 tB;
		iVector3 tC;
		iVector3 tNormal;

		int		depth_max;
		int		depth_current;
		float	length_min;

		vector<iVector3> vpointlist;
	};

	TriData _tridata;

	void SubDivideMe(const iVector3& A, const iVector3& B, const iVector3& C, int depth)
	{
		miVector vPosA, vPosB, vPosC;
		vPosA.x = A[0];		vPosA.y = A[1];		vPosA.z = A[2];
		vPosB.x = B[0];		vPosB.y = B[1];		vPosB.z = B[2];
		vPosC.x = C[0];		vPosC.y = C[1];		vPosC.z = C[2];
		float tArea = geoobj_tri_area(	&vPosA, &vPosB, &vPosC );

		if( tArea > _tridata.length_min && 
			_tridata.depth_current < _tridata.depth_max )
		{
			_tridata.depth_current++;

			iVector3 AB ( geoobj_halfedge(A,B) );
			iVector3 AC ( geoobj_halfedge(A,C) );
			iVector3 BC ( geoobj_halfedge(B,C) );

			//keep clockwise vtx order
			SubDivideMe(A,	AB, AC, _tridata.depth_current);
			SubDivideMe(B,	BC, AB, _tridata.depth_current);
			SubDivideMe(C,	AC, BC, _tridata.depth_current);
			SubDivideMe(AB, BC, AC, _tridata.depth_current);

		}else
		{
			_tridata.vpointlist.push_back( A );
			_tridata.vpointlist.push_back( B );
			_tridata.vpointlist.push_back( C );
		}
	}

	void SplatTri
	(
		pcloud_MI_bake_result const	* a,
		pcloud_MI_bake_result const	* b,
		pcloud_MI_bake_result const	* c,
		const float& splat_length,
		const int& splat_maxrec,
		miState	* state
	)
	{
		iVector3 vPosA ( a->point );
		iVector3 vPosB ( b->point );
		iVector3 vPosC ( c->point );

		_tridata.tA = vPosA;
		_tridata.tB = vPosB;
		_tridata.tC = vPosC;

		_tridata.tNormal = geoobj_tri_normal(&a->point, &b->point, &c->point);

		iVector3 surfel_pos	= geoobj_tri_centroid(&a->point, &b->point, &c->point);
		miVector cam_pos;	origin_from_camera(state, &cam_pos);

		iVector3 idir ( iVector3(surfel_pos) - iVector3(cam_pos) );
		idir = idir.normalize();

		float dot_nd = iVector3(_tridata.tNormal).dot(idir);	//dot_nd
		if(dot_nd > 0.f) 	_tridata.depth_max	= 64;
		else				_tridata.depth_max	= splat_maxrec;

		_tridata.depth_current	= 0;
		_tridata.length_min		= splat_length;//0.000500f

		//
		SubDivideMe(vPosA, vPosB, vPosC, _tridata.depth_current);
	}



	DTYPE* Bake
	(
		iUint				idx,
		const iVector3&		a,
		const iVector3&		b,
		const iVector3&		c,
		MICache				* iMICache,
		miState				* state,
		miBoolean			FGprepass = miFALSE
	)
	{

		miVector vPosA, vPosB, vPosC;
		vPosA.x = a[0];
		vPosA.y = a[1];
		vPosA.z = a[2];

		vPosB.x = b[0];
		vPosB.y = b[1];
		vPosB.z = b[2];		
		
		vPosC.x = c[0];
		vPosC.y = c[1];
		vPosC.z = c[2];	

		iVector3 hResult;

		// ** init data holder
		float* hdata = iSurfelData::InitNodeBlock();


		// position
		hResult	= geoobj_tri_centroid(&vPosA, &vPosB, &vPosC);
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_POS>( hdata, hResult );

		// direction
		//hResult	= geoobj_tri_normal(&vPosA, &vPosB, &vPosC);
		hResult = _tridata.tNormal;
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_DIR>( hdata, hResult );

		// area/radius
		miScalar tArea = geoobj_tri_area(&vPosA, &vPosB, &vPosC);
		#if	__SSE2_ENHANCED__
		iVector3 hd(tArea, 1.f/tArea, 1.f/(tArea*inv_pi), sqrtf(tArea/(float)M_PI));
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_AREA>( hdata, hd );
		#else
		iSurfelData::template SetNodeScalarAt<DATA_SLOT_AREA>( hdata, tArea );
		#endif

		// ** bring state to barycenter
		miState* orig_state = state;								//bake state

		// setup QMC
		state->raster_x = (float) idx;
		state->raster_y = (float) idx;
		mi_query(miQ_PIXEL_SAMPLE, state, 0, NULL);

		// set point and normal
		state->point = iSurfelData::template GetNodeVectorAt<DATA_SLOT_POS>( hdata ).AsMiVector();
		state->normal = iSurfelData::template GetNodeVectorAt<DATA_SLOT_DIR>( hdata ).AsMiVector();
		state->normal_geom = state->normal ;

		// calculate org, dir, dot_nd and dist
		origin_from_camera(state, &state->org);						//org


#if	__SSE2_ENHANCED__
		iVector3 idir ( iVector3(state->point) - iVector3(state->org) );
		state->dir = idir.normalize().AsMiVector();

		state->dist  = iVector3(&state->dir).length();				//dist
		state->dot_nd = iVector3(&state->normal).dot(&state->dir);	//dot_nd
		mi_db_unpin(state->camera_inst);
#else
		mi_vector_sub(&state->dir, &state->point, &state->org);		//dir
		mi_vector_normalize(&state->dir);

		state->dist  = mi_vector_norm(&state->dir);					//dist
		state->dot_nd = mi_vector_dot(&state->normal, &state->dir);	//dot_nd
		mi_db_unpin(state->camera_inst);
#endif


		// Backfacing?
		if (iMICache->backfacing && state->dot_nd > 0.0f) {			// disable for FG

			mi_vector_neg(&state->normal_geom);
			mi_vector_neg(&state->normal);
			state->inv_normal = miTRUE;
			state->dot_nd = -state->dot_nd;
		}

		// setup shadow_tol
		if (state->options->shadow)
		SET_SHADOW_TOLERANCE(&vPosA, &vPosB, &vPosC);

		// other state stuff to interpbary
		initIntersectionArrays(state);


		// FG preprocessing ...
		if(FGprepass)
		{
			// .. place a FG point at every vertex
			miColor tempvalue; // Never actually used
			mi_finalgather_store(&tempvalue, state, miFG_STORE_COMPUTE);

		// Evaluate irradiance shader ...
		}else	
		{
			miColor	irrad = {1,1,1,1};								// irradiance
			if(mi_call_shader_x(&irrad, miSHADER_MATERIAL, state, iMICache->irrad_tag, 0))
			{
				hResult = irrad;
				iSurfelData::template SetNodeVectorAt<DATA_SLOT_BATTR>( hdata, hResult );
			}
		}

		state = orig_state;

/*
		miColor	irrad = {0,0,0,0};								// irradiance
		hResult = irrad;
		iSurfelData::template SetNodeVectorAt<DATA_SLOT_BATTR>( hdata, hResult );
*/
		// return with data
		return hdata;
		}
};




/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// Bake class for GI
template
<
		typename DTYPE																			= float,
		unsigned SIZE																			= DATA_SLOT_MINIMAL,
		class	 MICache																		= iSurfel_MICache,
		template<typename, unsigned> class iNodeAllocator										= iSurfelAllocator,
		template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator	= iTrianleTessellator_Reyes
> 
class iSurfelBaker_Reyes :	public iTriTessellator<DTYPE, SIZE, iNodeAllocator>
{
public:
	typedef iSurfel_MI_bake_result							pcloud_MI_bake_result;
	typedef MICache											MICache;
	typedef iNodeAllocator<DTYPE, SIZE>						iSurfelData;
	typedef iTriTessellator<DTYPE, SIZE, iNodeAllocator>	iTessellator;


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void SplatTri
	(
		pcloud_MI_bake_result const	* a,
		pcloud_MI_bake_result const	* b,
		pcloud_MI_bake_result const	* c,
		miState*	state,
		iUint		idx,
		float		shading_rate,
		int			double_microtriangles
	)
	{
		iVector3 vPosA ( a->point );
		iVector3 vPosB ( b->point );
		iVector3 vPosC ( c->point );

		tParams triParams;
		triParams.shading_rate			= shading_rate;
		triParams.tIdx					= idx;
		triParams.doubleMicroTriangles	= double_microtriangles;

		InitTessellator( vPosA, vPosB, vPosC, state, triParams );
	}

	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Bake( MICache * iMICache, miState * iState, bool FGprepass = miFALSE )
	{

	for(int i=0; i< _tridata.vpointlist.size(); i++)
	{
			iUint idx			= _tridata.getTriIdx();
			miState* state		= iState;	//_tridata.getTriState();
			//miBoolean FGprepass = miFALSE;

			iVector3 hResult;

			// ** init data holder
			DTYPE* hdata = _tridata.vpointlist[i];


			// ** bring state to barycenter
			miState* orig_state = state;								//bake state

			// setup QMC
			state->raster_x = (float) i;	///was.. idx !
			state->raster_y = (float) i;
			mi_query(miQ_PIXEL_SAMPLE, state, 0, NULL);

			// set point and normal
			state->point = iSurfelData::template GetNodeVectorAt<DATA_SLOT_POS>( hdata ).AsMiVector();
			state->normal = iSurfelData::template GetNodeVectorAt<DATA_SLOT_DIR>( hdata ).AsMiVector();
			state->normal_geom = state->normal ;

			// calculate org, dir, dot_nd and dist
			origin_from_camera(state, &state->org);						//org


#if	__SSE2_ENHANCED__
			iVector3 idir ( iVector3(state->point) - iVector3(state->org) );
			state->dir = idir.normalize().AsMiVector();

			state->dist  = iVector3(&state->dir).length();				//dist
			state->dot_nd = iVector3(&state->normal).dot(&state->dir);	//dot_nd
			//mi_db_unpin(state->camera_inst);
#else
			mi_vector_sub(&state->dir, &state->point, &state->org);		//dir
			mi_vector_normalize(&state->dir);

			state->dist  = mi_vector_norm(&state->dir);					//dist
			state->dot_nd = mi_vector_dot(&state->normal, &state->dir);	//dot_nd
			//mi_db_unpin(state->camera_inst);
#endif


			// Backfacing?
			if (iMICache->backfacing && state->dot_nd > 0.0f) {			// disable for FG

				mi_vector_neg(&state->normal_geom);
				mi_vector_neg(&state->normal);
				state->inv_normal = miTRUE;
				state->dot_nd = -state->dot_nd;
			}

			// setup shadow_tol
			if (state->options->shadow)
			{
				miVector tA = _tridata.getTA().AsMiVector();
				miVector tB = _tridata.getTB().AsMiVector();
				miVector tC = _tridata.getTC().AsMiVector();

				SET_SHADOW_TOLERANCE( &tA, &tB, &tC );
			}

			// other state stuff to interpbary
			initIntersectionArrays(state);


			// FG preprocessing ...
			if(FGprepass && ( i%iMICache->fgprebake == 0 ))
			{
				// .. place a FG point at every vertex
				miColor tempvalue; // Never actually used
				mi_finalgather_store(&tempvalue, state, miFG_STORE_COMPUTE);

			// Evaluate irradiance shader ...
			}else	
			{
				miColor	irrad = {1,1,1,1};								// irradiance
				if(mi_call_shader_x(&irrad, miSHADER_MATERIAL, state, iMICache->irrad_tag, 0))
				{
					hResult = irrad;
					iSurfelData::template SetNodeVectorAt<DATA_SLOT_BATTR>( hdata, hResult );
				}
			}

			state = orig_state;

			// return with data
			//return hdata;
	}
	}
};





/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iSurfelContainer - Abstract base class ////////////////////////////////////////////////////////////////////////////
template<class Derived, typename DTYPE>
class iSurfelContainer
{
public:
	typedef Derived _iSurfelCache;
	typedef DTYPE _DTYPE;

	iSurfelContainer(){};
	iSurfelContainer(const _iSurfelCache* iscache){ this = iscache; }
	~iSurfelContainer(){}

	_iSurfelCache* GetSurfelCacheTPtr() { return static_cast<_iSurfelCache*>(this); }
	_iSurfelCache& GetSurfelCacheTRef() { return static_cast<_iSurfelCache&>(*this); }


	// Structs /////////////////////////////////////////////////////////////////////////////////////
	struct surfelList
	{
		miTag			obj;
		vector<DTYPE*>	surfel_list;
	};

	// Set miState
	void SetMIState( miState* mistate ){ static_cast<_iSurfelCache*>(this)->SetMIState(mistate); };


	// Virtual stuff ///////////////////////////////////////////////////////////////////////////////
	INLINE void	 Init_MI_TLS_Cache( void )	{ static_cast<_iSurfelCache*>(this)->Init_MI_TLS_Cache(); }
	INLINE iUint GetNodeSize( void )		{ return static_cast<_iSurfelCache*>(this)->GetNodeSize(); }
	INLINE iUint GetCacheChunkSize( void )	{ return static_cast<_iSurfelCache*>(this)->GetCacheChunkSize(); }

	// Data handling
	INLINE void  PushData( DTYPE* dnode )	{ static_cast<_iSurfelCache*>(this)->PushData( dnode ); }
	INLINE surfelList** GetDataCache( void ){ static_cast<_iSurfelCache*>(this)->GetDataCache(); }


	// Node data
	INLINE DTYPE* AllocateNewNode( void )	{ return static_cast<_iSurfelCache*>(this)->AllocateNewNode(); }
};



/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iSurfelCache - Specialized template class /////////////////////////////////////////////////////////////////////////
template
<
typename DTYPE										= float,
unsigned SIZE										= DATA_SLOT_MINIMAL,
typename MICache									= iSurfel_MICache,
template<typename, unsigned, class, template<typename, unsigned> class, template<typename,unsigned,template<typename, unsigned> class> class> class iSurfelBaker = iSurfelBaker_Reyes,
template<typename, unsigned> class iNodeAllocator	= iSurfelAllocator,
template<typename, unsigned, template<typename, unsigned> class> class iTriTessellator = iTrianleTessellator_Reyes
>
class iSurfelCache
	:	public iSurfelContainer< iSurfelCache<DTYPE, SIZE, iNodeAllocator<DTYPE, SIZE> >, DTYPE >
	,	public iSurfelBaker< DTYPE, SIZE, MICache, iNodeAllocator, iTriTessellator  >
{

public:
	typedef typename
	iSurfelContainer<iSurfelCache<DTYPE, SIZE, iNodeAllocator<DTYPE, SIZE> >, DTYPE>::surfelList _surfelList;
	typedef iNodeAllocator<DTYPE, SIZE>	_iNodeAllocator;
	typedef iSurfelBaker< DTYPE, SIZE, MICache, iNodeAllocator, iTriTessellator > iSBaker;
	typedef DTYPE _DTYPE;


	// Constructors /////////////////////////////////////////////////////////////////////////////////
	iSurfelCache( void )			:_local_cache(NULL){};
	
	iSurfelCache( miState *state )	:_local_cache(NULL)
	{
		_miState = state;
		Init_MI_TLS_Cache();
	};

	~iSurfelCache(){};	// destructor //////////////////////////////////////////////////////////////


	// Data handling
	void SetMIState( miState* mistate ){ _miState = mistate; };

	// Push data to TLS
	void PushData( DTYPE* dnode )
	{
		_local_cache->surfel_list.push_back( dnode );
	};

	// Whole cache data pointer
	_surfelList** GetDataCache( void )
	{	int num=0;
		struct  surfelList** tlocal_scache = NULL;
		mi_query(miQ_FUNC_TLS_GETALL, _miState, miNULLTAG, &tlocal_scache, &num);

		return tlocal_scache;
	};

	// Size of cache(chunks)
	unsigned int GetCacheChunkSize( void )
	{
		int num; struct surfelList** tlocal_scache = NULL;
		mi_query(miQ_FUNC_TLS_GETALL, _miState, miNULLTAG, &tlocal_scache, &num);
		return num;
	};

	// Size of cache(chunks)
	unsigned int GetCacheChunksPoints( size_t * totalp_in_chunk, const int verbose=0 )
	{

		iUint ntotalpoints=0;
		int cSize = this->GetCacheChunkSize();	// .. in chunks 
		_surfelList ** tlocal_scache = this->GetDataCache();

		for (int i=0; i < cSize; i++)
		{
			totalp_in_chunk[i] = tlocal_scache[i]->surfel_list.size();	// save chunk size 
			
			if(verbose>=3)
			PRINTME("--- triangles cache enlarged, from:%i, to: %i", 
			ntotalpoints,
			int( ntotalpoints + tlocal_scache[i]->surfel_list.size() )
			);

			ntotalpoints += (int) tlocal_scache[i]->surfel_list.size();
		}
		if(tlocal_scache==miNULLTAG || (*tlocal_scache)==miNULLTAG){
			PRINTME("\n\n==> Pointlist module: shader cache problem ... aborting.\n");
			return 0;	
		};

		return ntotalpoints;
	};


	// TLS mi cache ////////////////////////////////////////////////////////////////////////////////
	inline void Init_MI_TLS_Cache()
	{
		mi_query(miQ_FUNC_TLS_GET, _miState, miNULLTAG, &_local_cache);

		if (!_local_cache)
		{	
			// new cache 
			_local_cache       = new struct surfelList;
			_local_cache->obj  = _miState->material;

			mi_query(miQ_FUNC_TLS_SET, _miState, miNULLTAG, &_local_cache);
		}
		else
		{
			// new object, flush cache
			if ( _local_cache->obj != _miState->material )
			{
				_local_cache->obj = _miState->material;
				_local_cache->surfel_list.clear();
			}
		}
	};

////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	_surfelList *	_local_cache;
	miState*		_miState;
};		//end class

}

#endif	//end ifndef
