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


#ifndef __PCLOUD_DATASTRUCT_IPOINTSSE__
#define __PCLOUD_DATASTRUCT_IPOINTSSE__


/*! SSE optimized 3D Point class */


namespace cpc
{
	#define VEC16ALIGN 4
	#define VDIM 3


	/// ////////////////////////////////////////////////////////////////////////////////////////////
	/// SSE Point Class ////////////////////////////////////////////////////////////////////////////
	/// ////////////////////////////////////////////////////////////////////////////////////////////
	// float = Floating Point Type															  	  //
	// VDIM   = Dimension of Point																  //
	////////////////////////////////////////////////////////////////////////////////////////////////


	template <template<typename, unsigned> class DataAllocator>
	class iPoint<float, VDIM, DataAllocator> : private DataAllocator<float, VDIM+1>
	{

		typedef DataAllocator<float, VDIM+1> 	_mdata;	// data holder

	public:

		///////////////////////////////////////////////////////////////////////////////////////////
		/// Constructors
		///////////////////////////////////////////////////////////////////////////////////////////
		INLINE iPoint()	: _mdata(){}


		// from another point (copy-constructor)
		INLINE iPoint(const iPoint & p)	: _mdata(p){}

		// from 1/2/3 floats
		INLINE explicit iPoint(const float& x0) : _mdata(x0){}
		INLINE explicit iPoint(const float& x0,const float& x1) : _mdata(x0,x1){}
		INLINE explicit iPoint(const float& x0,const float& x1,const float& x2) : _mdata(x0,x1,x2){}

		// from floats array ptr
		INLINE explicit iPoint( const float * ax ) : _mdata(ax){}

		// from floats array entries
		INLINE explicit iPoint( const float & a, const float & b, const float & c, const float & d )
		: _mdata(a, b, c, d){}

		// from an __m128 reg
		INLINE explicit iPoint( const __m128& p) : _mdata(p){}

#if	__MENTALRAY_ENHANCED__
		// from mray vector/color
		INLINE iPoint(const miVector& p) : _mdata(p.x, p.y, p.z){}
		INLINE iPoint(const miColor& p)	 : _mdata(p.r, p.g, p.b/*, p.a*/){}
		INLINE iPoint(const miVector* p) : _mdata(p->x, p->y, p->z){}
		INLINE iPoint(const miColor* p)	 : _mdata(p->r, p->g, p->b/*, p.a*/){}
#endif

#if	__OPENEXR_ENHANCED__
		__CUDA_HOST_AND_DEVICE__
		inline explicit iPoint(const V3f& p):_mdata(p.x,p.y,p.z){};		//V3f
		inline explicit iPoint(const C3f& p):_mdata(p.r,p.g,p.b){};		//C3f
#endif

		// Destructor //////////////////////////////////////////////////////////////////////////////
		~iPoint(){_mdata::clearData();};



		////////////////////////////////////////////////////////////////////////////////////////////
		/// Utilities functions
		////////////////////////////////////////////////////////////////////////////////////////////
		// Get/Set coords in XYZ/RGBA style
		INLINE float GetX() const { return _mdata::getValue(0); };
		INLINE float GetY() const { return _mdata::getValue(1); };
		INLINE float GetZ() const { return _mdata::getValue(2); };
		INLINE float GetW() const { return _mdata::getValue(3); };

		INLINE float GetR() const { return _mdata::getValue(0); };
		INLINE float GetG() const { return _mdata::getValue(1); };
		INLINE float GetB() const { return _mdata::getValue(2); };
		INLINE float GetA() const { return _mdata::getValue(3); };

		INLINE iPoint& SetX(const float& Val) { _mdata::setValue(Val, 0); return *this; };
		INLINE iPoint& SetY(const float& Val) { _mdata::setValue(Val, 1); return *this; };
		INLINE iPoint& SetZ(const float& Val) { _mdata::setValue(Val, 2); return *this; };
		INLINE iPoint& SetW(const float& Val) { _mdata::setValue(Val, 3); return *this; };

		INLINE iPoint& SetR(const float& Val) { _mdata::setValue(Val, 0); return *this; };
		INLINE iPoint& SetG(const float& Val) { _mdata::setValue(Val, 1); return *this; };
		INLINE iPoint& SetB(const float& Val) { _mdata::setValue(Val, 2); return *this; };
		INLINE iPoint& SetA(const float& Val) { _mdata::setValue(Val, 3); return *this; };

		// Initialize all coords to zero
		INLINE void reset(){_mdata::setValue(M128_ZERO);}

#if	__MENTALRAY_ENHANCED__
		// Get coords as miVector style
		INLINE miVector AsMiVector() 
		{ 
			miVector ret = {_mdata::getValue(0),
							_mdata::getValue(1),
							_mdata::getValue(2) };
			return ret; 
		};

		// Get coords as miColor style
		INLINE miColor AsMiColor() 
		{ 
			miColor ret = { _mdata::getValue(0),
							_mdata::getValue(1),
							_mdata::getValue(2), 1.0f };
			return ret; 
		};
#endif

#if	__OPENEXR_ENHANCED__
		// Get coords as miVector style
		INLINE V3f AsV3f() 
		{ 
			V3f ret (		_mdata::getValue(0),
							_mdata::getValue(1),
							_mdata::getValue(2) );
			return ret; 
		};

		// Get coords as miColor style
		INLINE C3f AsC3f() 
		{ 
			C3f ret (		_mdata::getValue(0),
							_mdata::getValue(1),
							_mdata::getValue(2) );
			return ret; 
		};
#endif


		////////////////////////////////////////////////////////////////////////////////////////////
		/// Others
		////////////////////////////////////////////////////////////////////////////////////////////
		// Storage type for point coords
		typedef float				__NumType;

		// Dimension of points 
		static const unsigned int	__DIM = VDIM;



		////////////////////////////////////////////////////////////////////////////////////////////
		/// SSE
		////////////////////////////////////////////////////////////////////////////////////////////
		INLINE		 __m128& SSE_GetM128Coords( void )		 { return _mdata::getValue(); };
		INLINE const __m128& SSE_GetM128Coords( void ) const { return _mdata::getValue(); };
		INLINE iPoint& SSE_SetM128Coords( const __m128& p){ _mdata::setValue(p); return *this;};


		// Static SSE stuff ////////////////////////////////////////////////////////////////////////
		// Cross product
		static INLINE __m128 SSE_Cross3D( __m128 a, __m128 b)
		{
			return 
			_mm_sub_ps(
			_mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2))), 
			_mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)))
					  );
		}


		////////////////////////////////////////////////////////////////////////////////////////////
		/// Operators
		////////////////////////////////////////////////////////////////////////////////////////////

		//Addition
		INLINE iPoint	operator+ ( const iPoint& q ) const
		{
			iPoint result( _mm_add_ps(_mdata::getValue(), q) );
			return result;
		}	
		INLINE iPoint	operator+ (	const float& q )
		{
			iPoint result( _mm_add_ps(_mdata::getValue(), _mm_set1_ps(q)) );
			return result;
		}
		INLINE iPoint&	operator+= ( const iPoint& q )
		{
			_mdata::setValue(_mm_add_ps(_mdata::getValue(), q));
			return *this;
		}
		INLINE iPoint&	operator+= ( const float& q)
		{
			_mdata::setValue(_mm_add_ps(_mdata::getValue(), _mm_set1_ps(q)));
			return *this;
		}
		INLINE iPoint&	operator+= ( const __m128& q )
		{
			_mdata::setValue(_mm_add_ps( _mdata::getValue(), q ));
			return *this;
		}

		//Substract
		INLINE iPoint	operator- (	const iPoint& q) const
		{
			iPoint result( _mm_sub_ps(_mdata::getValue(), q) );
			return result;
		}
		INLINE iPoint	operator- (	const float& q )
		{
			iPoint result( _mm_sub_ps(_mdata::getValue(), _mm_set1_ps(q)) );
			return result;
		}
		INLINE iPoint	operator- (	void ) const //unary
		{
			return ( iPoint(_mm_sub_ps(M128_ZERO, _mdata::getValue())) );
		}
		INLINE iPoint&	operator-= (const iPoint& q)
		{
			_mdata::setValue( _mm_sub_ps(_mdata::getValue(), q) );
			return *this;
		}
		INLINE iPoint&	operator-= (const float& q)
		{
			_mdata::setValue( _mm_sub_ps(_mdata::getValue(), _mm_set1_ps(q)) );
			return *this;
		}

		// Multiply
		INLINE iPoint	operator* (	const iPoint& q )
		{
			iPoint result(_mm_mul_ps(_mdata::getValue(), q));
			return result;
		}
		INLINE iPoint	operator* (	const float& q )
		{
			iPoint result(_mm_mul_ps(_mdata::getValue(), _mm_set1_ps(q)));
			return result;		
		}		
		INLINE iPoint&	operator*= ( const iPoint& q )
		{
			_mdata::setValue(_mm_mul_ps(_mdata::getValue(), q));
			return *this;
		}
		INLINE iPoint&	operator*= ( const float& q )
		{
			_mdata::setValue(_mm_mul_ps(_mdata::getValue(), _mm_set1_ps(q) ));
			return *this;
		}

		// SLOW !!
		//template <template<typename, unsigned> class DataAllocator>
		//friend INLINE iPoint<float, VDIM, DataAllocator> operator* ( const float& q, const iPoint& p );


		// Divide
		INLINE iPoint	operator/ (	const iPoint& q )
		{
			iPoint result( _mm_div_ps(_mdata::getValue(), q) );
			return result;
		}
		INLINE iPoint	operator/ (	const float& q )
		{
			iPoint result( _mm_div_ps(_mdata::getValue(), _mm_set1_ps(q)) );
			return result;
		}
		INLINE iPoint&	operator/= ( const iPoint& q)
		{
			_mdata::setValue(_mm_div_ps(_mdata::getValue(), q));
			return *this;
		}
		INLINE iPoint&	operator/= (	const float& q)
		{
			_mdata::setValue(_mm_div_ps(_mdata::getValue(), _mm_set1_ps(q)));
			return *this;
		}
		INLINE iPoint&	operator/= (	const __m128& q)
		{
			_mdata::setValue(_mm_div_ps(_mdata::getValue(), q));
			return *this;
		}

		// Assignment //////////////////////////////////////////////////////////////////////////////
		INLINE iPoint&  operator= (const iPoint& q)
		{
			_mdata::setValue(q);
			return *this;
		}
		INLINE iPoint&  operator= (const float& q)
		{
			_mdata::setValue(_mm_set1_ps(q));
			return *this;
		}

#if	__MENTALRAY_ENHANCED__
		INLINE iPoint&  operator= (const miVector& q)
		{
			iPoint ttvec(q);
			Equate<float,VDIM,DataAllocator,VDIM-1>::eval(*this,ttvec);
			return *this;
		}
		INLINE iPoint&  operator= (const miColor& q)
		{
			iPoint ttvec(q);
			Equate<float,VDIM,DataAllocator,VDIM-1>::eval(*this,ttvec);
			return *this;
		}
#endif

#if	__OPENEXR_ENHANCED__
		INLINE iPoint&  operator= (const V3f& q)
		{
			iPoint ttvec(q);
			Equate<float,VDIM,DataAllocator,VDIM-1>::eval(*this,ttvec);
			return *this;
		}
		INLINE iPoint&  operator= (V3f& q)
		{
			iPoint ttvec(q);
			Equate<float,VDIM,DataAllocator,VDIM-1>::eval(*this,ttvec);
			return *this;
		}		
		INLINE iPoint&  operator= (const C3f& q)
		{
			iPoint ttvec(q);
			Equate<float,VDIM,DataAllocator,VDIM-1>::eval(*this,ttvec);
			return *this;
		}
#endif

		INLINE iPoint&  operator= (const float* q)
		{
			_mdata::setValue(q);
			return *this;
		}

		INLINE iPoint&	operator= (const __m128& q)
		{
			_mdata::setValue(q);
			return *this;
		}

		// Comparison //////////////////////////////////////////////////////////////////////////////
		INLINE bool	operator==(const iPoint& q)
		{
			return _mm_movemask_ps( _mm_cmpeq_ps(_mdata::getValue(),q) );
		}	
		INLINE bool	operator<(const iPoint& q)
		{
			return _mm_movemask_ps( _mm_cmplt_ps(_mdata::getValue(),q) );
		}	
		INLINE bool	operator<=(const iPoint& q)
		{
			return _mm_movemask_ps( _mm_cmple_ps(_mdata::getValue(),q) );
		}
		INLINE bool	operator>(const iPoint& q)
		{
			return _mm_movemask_ps( _mm_cmpgt_ps(_mdata::getValue(),q) );
		}
		//INLINE bool	operator>(const float& q)
		//{
		//	return _mm_movemask_ps( _mm_cmpgt_ps(_mdata::getValue(),_mm_set1_ps(q)) );
		//}
		INLINE bool	operator>=(const iPoint& q)
		{
			return _mm_movemask_ps( _mm_cmpge_ps(_mdata::getValue(),q) );
		}

		// Dereference data ///////////////////////////////////////////////////////////////////////
		INLINE const	float& operator[](const iUint i) const	{ return _mdata::getValue(i); };
		INLINE			float& operator[](const iUint i)		{ return _mdata::getValue(i); };

		INLINE operator __m128 ( void ) const	{ return _mdata::getValue(); }	// used as __m128
		INLINE operator __m128&( void )			{ return _mdata::getValue(); }

#if	__MENTALRAY_ENHANCED__
		INLINE operator miVector ( void ) const
		{
			miVector t = {_mdata::getValue(0), _mdata::getValue(1), _mdata::getValue(2)};
			return t;
		}
		INLINE operator miColor ( void ) const
		{
			miColor t = {_mdata::getValue(0), _mdata::getValue(1), _mdata::getValue(2), _mdata::getValue(3)};
			return t;
		}
#endif

#if	__OPENEXR_ENHANCED__
		INLINE operator Imath::V3f ( void ) const
		{
			V3f t (_mdata::getValue(0), _mdata::getValue(1), _mdata::getValue(2));
			return t;
		}
		INLINE operator Imath::C3f ( void ) const
		{
			C3f t (_mdata::getValue(0), _mdata::getValue(1), _mdata::getValue(2));
			return t;
		}
		INLINE operator Imath::V3f ( void )
		{
			V3f t (_mdata::getValue(0), _mdata::getValue(1), _mdata::getValue(2));
			return t;
		}
		INLINE operator Imath::C3f ( void )
		{
			C3f t (_mdata::getValue(0), _mdata::getValue(1), _mdata::getValue(2));
			return t;
		}
#endif

		INLINE const float* begin() { return _mdata::getDataPtr(); }
		INLINE const float* begin() const { return _mdata::getDataPtr(); }

		// Dereference point as __m128 aligned data
		//INLINE const	 __m128& operator* ( void ) const	{ return _mdata::getValue(); }
		//INLINE		 __m128& operator* ( void )			{ return _mdata::getValue(); }

		//INLINE float* operator& (){ return _mdata::getFValue(); };			// & reference
		//inline float operator()(const iUint i){ return _mdata::getValue(i);}


		////////////////////////////////////////////////////////////////////////////////////////////
		/// Euclidian geometry
		////////////////////////////////////////////////////////////////////////////////////////////

		// Length
		INLINE float length(void)  const
		{
			return _mm_cvtss_f32(_mm_sqrt_ps( _mm_dp_ps(_mdata::getValue(), _mdata::getValue(), bit3Dmask) ));
		}
		INLINE iPoint lengthX(void)  const
		{
			return iPoint( _mm_sqrt_ps(_mm_dp_ps(_mdata::getValue(), _mdata::getValue(), bit3Dmask)) );
		}

		// Squared length
		INLINE float  sqr_length(void)  const
		{
			return _mm_cvtss_f32( _mm_dp_ps(_mdata::getValue(), _mdata::getValue(), bit3Dmask) );
		}
		INLINE float  length2(void)  const
		{
			return _mm_cvtss_f32( _mm_dp_ps(_mdata::getValue(), _mdata::getValue(), bit3Dmask) );
		}
		INLINE iPoint sqr_lengthX(void)  const
		{
			return iPoint( _mm_dp_ps(_mdata::getValue(), _mdata::getValue(), bit3Dmask) );
		}

		// Reciprocal length (approx)
		INLINE float rlength(void) const
		{
			//Intel has added a fast 1/sqrt(x) function to the SSE2 instruction set.
			//The drawback is that its precision is limited, we refine it with Newton-Rhapson:
			__m128 nr = _mm_rsqrt_ps( _mm_dp_ps(_mdata::getValue(), _mdata::getValue(), bit3Dmask) );
			__m128 muls = _mm_mul_ps( _mm_mul_ps( _mdata::getValue(), nr ), nr );
			return _mm_cvtss_f32( _mm_mul_ps( _mm_mul_ps( M128_HALF, nr ), _mm_sub_ps( M128_THREE, muls )) );
		}
		INLINE iPoint rlengthX(void) const
		{
			__m128 nr = _mm_rsqrt_ps( _mm_dp_ps(_mdata::getValue(), _mdata::getValue(), bit3Dmask) );
			__m128 muls = _mm_mul_ps( _mm_mul_ps( _mdata::getValue(), nr ), nr );
			return iPoint( _mm_mul_ps( _mm_mul_ps( M128_HALF, nr ), _mm_sub_ps( M128_THREE, muls )) );
		}

		// Sqrt
		INLINE iPoint sqrtX(void) const
		{
			return iPoint( _mm_sqrt_ps(_mdata::getValue()) );
		}

		// Distance
		inline float distance(const iPoint& q) const
		{
			return sqrtf( static_cast<float>(Distance<float,VDIM,DataAllocator,VDIM-1>::eval(*this,q)) );
		}
		inline float distanceX(const iPoint& q) const
		{
			__m128 Diff = _mm_sub_ps(_mdata::getValue(), q);
			__m128 Sqr = _mm_mul_ps(Diff, Diff);
			__m128 Sum = _mm_hadd_ps(Sqr, Sqr);
			Sum = _mm_hadd_ps(Sum, Sum);

			return _mm_cvtss_f32( _mm_sqrt_ss(Sum) );
		}

		// Squared distance
		INLINE float  sqr_dist(const iPoint& q) const
		{
			__m128 Diff = _mm_sub_ps(_mdata::getValue(), q);
			__m128 Sqr = _mm_mul_ps(Diff, Diff);
			__m128 Sum = _mm_hadd_ps(Sqr, Sqr);

			return _mm_cvtss_f32( _mm_hadd_ps(Sum, Sum) );
		}

		// Dot-product
		INLINE float dot (const iPoint& q)  const
		{
			return _mm_cvtss_f32( _mm_dp_ps(_mdata::getValue(), q, bit3Dmask) );
		}
		INLINE iPoint dotX (const iPoint& q)  const
		{
			return iPoint( _mm_dp_ps(_mdata::getValue(), q, bit3Dmask) );
		}

		// Cross-product
		INLINE iPoint	cross (const iPoint& q) const
		{
			return iPoint( SSE_Cross3D( _mdata::getValue(), q) );
		}

		// Normalized length
		INLINE iPoint normalize (void)
		{
			iPoint hNorm( *this );
			const float& len = length(); //+ 0.00000001f
			
			if (len > 0.00000001)
			hNorm /= len;

			return hNorm;
		}
		INLINE float norm (void)
		{
			return length();
		}

		INLINE iPoint& getNormalized (void)
		{
			_mdata::setValue(_mm_mul_ps(_mdata::getValue(), rlengthX()));
			return *this;
		}
		INLINE iPoint normalizeApproxed (void)
		{
			return  iPoint( _mm_mul_ps( _mdata::getValue(), rlengthX ) );
		}


		////////////////////////////////////////////////////////////////////////////////////////////
		/// BoundingBox (octree) geometry fncs
		////////////////////////////////////////////////////////////////////////////////////////////

		// Squared distance to bbox
		inline float sqr_dist_bbox( const iPoint &iMin,
									const iPoint &iMax) const
		{
			__m128 const & c_mid = _mm_mul_ps( _mm_add_ps(iMax, iMin), M128_HALF);
			__m128 const & s_mid = _mm_mul_ps( _mm_sub_ps(iMax, iMin), M128_HALF);

			__m128 dist = _mm_sub_ps(_mdata::getValue(), c_mid);

		
			__m128 const & lesst		= _mm_cmple_ps( dist, _mm_sub_ps(M128_ZERO, s_mid));
			__m128 const & lesst_msk	= _mm_and_ps( lesst, M128_ONE );
			__m128 const & distL		= _mm_add_ps( dist, _mm_mul_ps(s_mid, lesst_msk) );

			__m128 const & greatt		= _mm_cmpge_ps( dist, s_mid);
			__m128 const & greatt_msk	= _mm_and_ps( greatt, M128_ONE );

			dist = _mm_sub_ps(distL, _mm_mul_ps(s_mid, _mm_andnot_ps( lesst_msk, greatt_msk )));
			dist = _mm_mul_ps( dist, _mm_or_ps( lesst_msk, _mm_andnot_ps( lesst_msk, greatt_msk ) ) );

			return _mm_cvtss_f32( _mm_dp_ps(dist, dist, bit3Dmask) );
		}

		// Find owner bbox
		inline int find_child_bbox(	const iPoint &iMin,
									const iPoint &iMax) const
		{
			__m128 const & midp = _mm_mul_ps( _mm_add_ps(iMax, iMin), M128_HALF);
			__m128 const & gec	= _mm_cmpge_ps( _mdata::getValue(), midp  );
			__m128 mnd	= _mm_and_ps(gec, M128_1_2_4 );

			mnd = _mm_hadd_ps(mnd, mnd);
			mnd = _mm_hadd_ps(mnd, mnd);
			return (int)(_mm_cvtss_f32(mnd));
		}

		// Expand bbox regard new point
		inline void expand_bbox( iPoint &iMin, iPoint& iMax ) const
		{
			__m128 const & hex_min_msk	= _mm_cmplt_ps( _mdata::getValue(), iMin );
			__m128 const & bit_min_msk	= _mm_and_ps( hex_min_msk, M128_ONE );
			__m128 const & lerp_min		= lerp(iMin, bit_min_msk);

			__m128 const & hex_max_msk	= _mm_cmpgt_ps( _mdata::getValue(), iMax );
			__m128 const & bit_max_msk	= _mm_and_ps( hex_max_msk, M128_ONE );
			__m128 const & lerp_max		= lerp(iMax, bit_max_msk);

			iMin = lerp_min;
			iMax = lerp_max;
		}


		////////////////////////////////////////////////////////////////////////////////////////////
		/// Others
		////////////////////////////////////////////////////////////////////////////////////////////

		// Returns the dimension
		inline int dim() const { return VDIM; };

		// Members sum
		inline float  sum(void)  const
		{
			return Sum<float,VDIM,DataAllocator,VDIM-1>::eval(*this);			//TODO::SSE
		}

		// Members product
		inline float  product(void)  const
		{
			return Product<float,VDIM,DataAllocator,VDIM-1>::eval(*this);		//TODO::SSE
		}

		// Members mean
		inline float  mean(void)  const
		{
			float hMean = Sum<float,VDIM,DataAllocator,VDIM-1>::eval(*this);	//TODO::SSE
			return hMean * (1.0f/VDIM);
		}

		// Lerp
		INLINE __m128 lerp(const iPoint & q, __m128 mask) const
		{
		  return _mm_add_ps( _mm_mul_ps(_mm_sub_ps(_mdata::getValue(), q), mask ), q );
		}

		// Swap
		INLINE void swap(iPoint<float, VDIM>& q)
		{
			__m128 XMM0 = q.getValue();

			q.setValue( _mdata::getValue() );
			_mdata::setValue( XMM0 );
		}

		// Swap coords, moves lower two coords of q to the upper two of p
		INLINE iPoint swapLhCoords(const iPoint & q)
		{
			return iPoint( _mm_movelh_ps( _mdata::getValue(), q ) );
		}

		// Clamp
		INLINE iPoint clampMin( __m128 q)
		{
			return iPoint( _mm_min_ps(_mdata::getValue(), q) );
		}

		INLINE iPoint clampMax( __m128 q)
		{
			return iPoint( _mm_max_ps(_mdata::getValue(), q) );
		}

		// Saturate
		INLINE iPoint saturate( void )	
		{
			return iPoint( _mm_max_ps(_mm_min_ps(_mdata::getValue(),M128_ONE), M128_ZERO) );
		}	

		// 1-p
		INLINE iPoint minusOne( void )
		{
			return iPoint( _mm_sub_ps(M128_ONE, _mdata::getValue()) );
		}

		// Absolute
		INLINE iPoint abs( void )
		{
			return iPoint( _mm_and_ps( _mdata::getValue(), _mm_castsi128_ps(M128i_CI4_RSIGN)) );
		}
};	// end iPointSSE
	
	

	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// Implementations ///////////////////////////////////////////////////////////////////////////////////////////////
	/* template <template<typename, unsigned> class DataAllocator>
	iPoint<float, VDIM, DataAllocator> operator* ( const float& q, const iPoint<float, VDIM, DataAllocator>& p )
	{
		iPoint<float, VDIM, DataAllocator> result(_mm_mul_ps(_mm_set1_ps(q), p));
		return result;	
	}	*/

	// Standalone swap for std::swap template specialization ///////////////////////////////////////
	INLINE void swap(iPoint<float, VDIM>& pFirst, iPoint<float, VDIM>& pSecond)
	{
		pFirst.swap(pSecond);
	}
	////////////////////////////////////////////////////////////////////////////////////////////////

	//Stream
#if	__MENTALRAY_ENHANCED__ == 0
	template <template<typename, unsigned> class DataAllocator>
	ostream& operator<<(ostream& os,const iPoint<float,VDIM,DataAllocator> &p)
	{
		os << "PointSSE< ";
		for (int i=0; i<(int)VDIM; ++i)
		os << p[i] << ", ";

		return os << p[VDIM] << " >";
	}
#endif

}	//namespace end	////////////////////////////////////////////////////////////////////////////////

#endif
