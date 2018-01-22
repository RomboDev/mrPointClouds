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

#ifndef __PCLOUD_DATASTRUCT_IPOINT__
#define __PCLOUD_DATASTRUCT_IPOINT__


/*! Optimized point class file for fixed small dimensional points. */

// TODO::
// - constructors : assert for D >= 1
// - constructors:array_init : unroll loop
// - operators:divide : assert for divbyzero
// - operators:assign : assert for equality
// - operators:geom:distance : check for sqrt optim
// - utilities:begin/end : check what's that ......
// - utilities:geom:normalize : check for sqrt optim
// - check for float/double stuff (like returned dist)
// - check if it's better to have pointer instead of refs ...


#include <valarray>
#include <iostream>


namespace cpc
{

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Data point allocator/storage
	template<typename DataType, unsigned D>
	class pAlloc
	{
	public:
		pAlloc(){/*_iD[0]=0;_iD[1]=0;_iD[2]=0;*/}
		pAlloc(const DataType& x0){/*_iD[0]=x0;_iD[1]=x0;_iD[2]=x0;*/}
		pAlloc(const DataType& x0, const DataType& x1){_iD[0]=x0;_iD[1]=x1;}
		pAlloc(const DataType& x0, const DataType& x1, const DataType& x2){_iD[0]=x0;_iD[1]=x1;_iD[2]=x2;}
		pAlloc(const DataType& x0, const DataType& x1, const DataType& x2, const DataType& x3){_iD[0]=x0;_iD[1]=x1;_iD[2]=x2;(void)x3;}
		pAlloc(const DataType* ax){/*_iD[0]=ax[0];_iD[1]=ax[1];_iD[2]=ax[2];*/}
		~pAlloc(){ clearData(); }

		// members functions
		inline DataType&		getValue(int i)			{ return _iD[i]; }
		inline const DataType&	getValue(int i) const	{ return _iD[i]; }
		inline void				setValue(const DataType& val, int i) { _iD[i]=val;}

		inline DataType*		getDataPtr(void){ return _iD; }
		inline const DataType*	getDataPtr(void) const { return _iD; }

		inline void clearData(void){}

	private:
		DataType  _iD[D];	// data
	};

#if	__SSE2_ENHANCED__
	////////////////////////////////////////////////////////////////////////////////////////////////
	// specialized point allocator
	template<>
	class pAlloc<float,4>
	{
	public:
		// constructors
		INLINE pAlloc():_SSE_iD(M128_0001){}
		INLINE pAlloc(const float& x0)
			//: _SSE_iD( _mm_setr_ps(x0, (float)0.0, (float)0.0, (float)1.0) ){}
			: _SSE_iD( _mm_set1_ps(x0) ){}
		INLINE pAlloc(const float& x0, const float& x1)
			: _SSE_iD( _mm_setr_ps(x0, x1, 0.0f, 1.0f) ){}
		INLINE pAlloc(const float& x0, const float& x1, const float& x2)
			: _SSE_iD( _mm_setr_ps(x0, x1, x2, 1.0f) ){}
		INLINE pAlloc(const float* ax):_SSE_iD( _mm_load_ps(ax) ){}
		INLINE ~pAlloc(){ clearData(); }

		// specialized constructors
		INLINE pAlloc(const float& x0, const float& x1, const float& x2, const float& x3)
			: _SSE_iD( _mm_setr_ps(x0, x1, x2, x3) ){}
		INLINE pAlloc(const __m128& val ):_SSE_iD( val ){}

		//pAlloc(const iPoint<float,4>& p):_SSE_iD(p){}

		// members functions
		INLINE float& getValue(int i){ return _SSE_iD.m128_f32[i]; }
		INLINE const float& getValue(int i) const { return _SSE_iD.m128_f32[i]; }
		INLINE void setValue(const float& val, int i) { _SSE_iD.m128_f32[i]=val;}
		
		INLINE void clearData(void){}

		// specialized members functions
		INLINE __m128& getValue(void){ return _SSE_iD; }
		INLINE  float* getFValue(void) { return &(_SSE_iD.m128_f32[0]); }
		INLINE const __m128& getValue(void) const { return _SSE_iD; }
		INLINE void setValue(const __m128& val) { _SSE_iD=val; }

		INLINE float*		getDataPtr(void){ return &(_SSE_iD.m128_f32[0]); }
		INLINE const float*	getDataPtr(void) const { return &(_SSE_iD.m128_f32[0]);}

	private:
		SSE_ALIGNED( __m128	_SSE_iD );		// data storage ////////////////////////////////////////
	};
#endif


	////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////////////////
	//Forward Declaration of the main Point Class Eucledian d-dimensional point 
	template<typename DataType, unsigned D, template<typename, unsigned> class DataAllocator=pAlloc>
	class iPoint;

	template<typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I=D-1>
	struct ToXMM
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval(	__m128& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			p.m128_f32[I] = q[I];
			ToXMM< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct ToXMM<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval(	__m128& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			p.m128_f32[0] = q[0];
		}
	};

	///////////////////////////////////////////////////////
	// Origin of d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct origin
	{
		__CUDA_HOST_AND_DEVICE__
		static inline void eval( iPoint<DataType,D,DataAllocator>& p )
		{
			p[I] = 0.0;
			origin< DataType, D, DataAllocator, I-1 >::eval( p );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct origin<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& p )
		{
			p[0] = 0.0;
		}
	};

	///////////////////////////////////////////////////////
	// Equate two d-dimensional points
	///////////////////////////////////////////////////////
	template<typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I=D-1>
	struct Equate
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval(	iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			p[I] = q[I];
			Equate< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Equate<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval(	iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			p[0] = q[0];
		}
	};

	///////////////////////////////////////////////////////
	// Equate a d-dimensional point with a scalar
	///////////////////////////////////////////////////////
	template<typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct EquateScalar
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval(	iPoint<DataType,D,DataAllocator>& p,
									const DataType& q )
		{
			p[I]  = q;
			EquateScalar< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct EquateScalar<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval(	iPoint<DataType,D,DataAllocator>& p,
									const DataType& q )
		{
			p[0] = q;
		}
	};

	///////////////////////////////////////////////////////
	// Equate a d-dimensional point with a scalar
	///////////////////////////////////////////////////////
	template<typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct EquateArray
	{
		__CUDA_HOST_AND_DEVICE__
		static INLINE void eval(	iPoint<DataType,D,DataAllocator >& p,
									const DataType* q )
		{
			p[I]  = q[I];
			EquateArray< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct EquateArray<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__
		static INLINE void eval(	iPoint<DataType,D, DataAllocator>& p,
									const DataType* q )
		{
			p[0] = q[0];
		}
	};

	///////////////////////////////////////////////////////
	// Equality of two d-dimensional points
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct IsEqual
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval(	const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			if( p[I]  != q[I] ) return false;
			else return IsEqual< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct IsEqual<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval(	const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			return (p[0] == q[0])? 1: 0;
		}
	};
	///////////////////////////////////////////////////////
	// Equality of a d-dimensional points and a scalar
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct IsEqualScalar
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval(	const iPoint<DataType,D,DataAllocator>& p,
									const DataType& q )
		{
			if( p[I]  != q ) return false;
			else return IsEqualScalar< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct IsEqualScalar<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval( const iPoint<DataType,D,DataAllocator>& p,
									const DataType& q )
		{
			return (p[0] == q)? 1: 0;
		}
	};
	///////////////////////////////////////////////////////
	// Equality of a d-dimensional points and a scalar
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct IsMinorScalar
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval(	const iPoint<DataType,D,DataAllocator>& p,
									const DataType& q )
		{
			if( p[I]  >= q ) return false;
			else return IsMinorScalar< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct IsMinorScalar<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval( const iPoint<DataType,D,DataAllocator>& p,
									const DataType& q )
		{
			return (p[0] >= q) ? false: true;
		}
	};
	///////////////////////////////////////////////////////
	// Equality of a d-dimensional points and a scalar
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct IsMinorEq
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval(	const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			if( p[I] > q[I] ) return false;
			else return IsMinorEq< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct IsMinorEq<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline bool eval(	const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			return (p[0] <= q[0])? 1: 0;
		}
	};

	///////////////////////////////////////////////////////
	// Squared Distance of d-dimensional points
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Distance
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline double eval(	const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			double sum = ( (double) p[I] - (double) q[I] );
			sum = sum * sum;
			return sum + Distance< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Distance<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline double eval(	const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			double sum = ((double) p[0] - (double) q[0]);
			return sum * sum;
		}
	};
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct DistanceF
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline DataType eval(	const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			DataType sum = ( p[I] - q[I] );
			sum = sum * sum;
			return sum + DistanceF< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct DistanceF<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline DataType eval(const iPoint<DataType,D,DataAllocator>& p,
									const iPoint<DataType,D,DataAllocator>& q )
		{
			DataType sum = ( p[0] - q[0]);
			return sum * sum;
		}
	};

	///////////////////////////////////////////////////////
	// Dot Product of two d-dimensional points
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct DotProd
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE DataType eval( const iPoint<DataType,D,DataAllocator>& p,
									 const iPoint<DataType,D,DataAllocator>& q )
		{
			DataType sum = ( p[I] * q[I] );
			return sum + DotProd< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct DotProd<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE DataType eval( const iPoint<DataType,D,DataAllocator>& p,
									 const iPoint<DataType,D,DataAllocator>& q )
		{
			return (p[0] * q[0]);
		}
	};


	///////////////////////////////////////////////////////
	// Add two d-dimensional points
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Add
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			result[I] = p[I]  + q[I];
			Add< DataType, D, DataAllocator, I-1 >::eval( result,p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Add<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			result[0] = p[0] + q[0];
		}
	};


	///////////////////////////////////////////////////////
	// Add a scalar quantity to a d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct AddScalar
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			result[I] = p[I]  + k;
			AddScalar< DataType, D, DataAllocator, I-1 >::eval( result,p,k );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct AddScalar<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			result[0] = p[0] + k;
		}
	};
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct AddScalarX
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			p[I] += k;
			AddScalarX< DataType, D, DataAllocator, I-1 >::eval( p,k );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct AddScalarX<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			p[0] += k;
		}
	};

	///////////////////////////////////////////////////////
	// Subtract two d-dimensional points
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Subtract
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			result[I] = p[I]  - q[I];
			Subtract< DataType, D, DataAllocator, I-1 >::eval( result,p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Subtract<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			result[0] = p[0] - q[0];
		}
	};

	///////////////////////////////////////////////////////
	// Subtract a scalar quantity to a d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct SubtractScalar
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			result[I] = p[I]  - k;
			SubtractScalar< DataType, D, DataAllocator, I-1 >::eval( result,p,k );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct SubtractScalar<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			result[0] = p[0] - k;
		}
	};
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct SubtractScalarX
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			p[I] -= k;
			SubtractScalarX< DataType, D, DataAllocator, I-1 >::eval( p,k );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct SubtractScalarX<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& p, DataType k )
		{
			p[0] -= k;
		}
	};

	///////////////////////////////////////////////////////
	// Multiply two d-dimensional points
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Multiply
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			result[I] = p[I]  * q[I];
			Multiply< DataType, D, DataAllocator, I-1 >::eval( result,p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Multiply<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			result[0] = p[0] * q[0];
		}
	};
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct MultiplyX
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void  eval( 
								 iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			p[I] *= q[I];
			MultiplyX< DataType, D, DataAllocator, I-1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct MultiplyX<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( 
								 iPoint<DataType,D,DataAllocator>& p,
								 const iPoint<DataType,D,DataAllocator>& q )
		{
			p[0] *= q[0];
		}
	};
	///////////////////////////////////////////////////////
	//  Mutiply scalar with d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct MultiplyScalar
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const DataType k)
		{
			result[I] = p[I] * k;
			MultiplyScalar< DataType, D, DataAllocator, I-1 >::eval( result,p,k );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct MultiplyScalar<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p,
								 const DataType k )
		{
			result[0] = p[0] * k;
		}
	};
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct MultiplyScalarX
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& p,
								 const DataType k)
		{
			p[I] *= k;
			MultiplyScalarX< DataType, D, DataAllocator, I-1 >::eval( p,k );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct MultiplyScalarX<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& p,
								 const DataType k )
		{
			p[0] *= k;
		}
	};

	///////////////////////////////////////////////////////
	//  Sum up point elements
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Sum
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline DataType eval( const iPoint<DataType,D,DataAllocator>& p)
		{
			DataType result = p[I];
			return result + Sum< DataType, D, DataAllocator, I-1 >::eval( p );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Sum<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline DataType eval( const iPoint<DataType,D,DataAllocator>& p )
		{
			return p[0];
		}
	};

	///////////////////////////////////////////////////////
	//  Product of point elements
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Product
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline DataType eval( const iPoint<DataType,D,DataAllocator>& p)
		{
			DataType result = p[I];
			return result * Product< DataType, D, DataAllocator, I-1 >::eval( p );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Product<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline DataType eval( const iPoint<DataType,D,DataAllocator>& p )
		{
			return p[0];
		}
	};

	///////////////////////////////////////////////////////
	// Invert d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Invert
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p )
		{
			result[I] = -p[I];
			Invert< DataType, D, DataAllocator, I-1 >::eval( result,p );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Invert<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static INLINE void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p )
		{
			result[0] = -p[0];
		}
	};

	///////////////////////////////////////////////////////
	// Invert d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Reciprocal
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p )
		{
			result[I] = (DataType) (1./p[I]);
			Reciprocal< DataType, D, DataAllocator, I-1 >::eval( result,p );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Reciprocal<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p )
		{
			result[0] = (DataType) (1./p[0]);
		}
	};

	///////////////////////////////////////////////////////
	// Square d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct SqrtX
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result )
		{
			result[I] = (DataType) sqrt(p[I]);
			SqrtX< DataType, D, DataAllocator, I-1 >::eval( result );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct SqrtX<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p )
		{
			result[0] = (DataType)  sqrt(p[0]);
		}
	};


	///////////////////////////////////////////////////////
	// Absolute d-dimensional point
	///////////////////////////////////////////////////////
	template< typename DataType, unsigned D, template<typename, unsigned> class DataAllocator, unsigned I>
	struct Abs
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p  )
		{
			result[I] = (DataType) abs(p[I]);
			Abs< DataType, D, DataAllocator, I-1 >::eval( result,p );
		}
	};
	// Partial Template Specialization
	template <typename DataType, unsigned D, template<typename, unsigned> class DataAllocator>
	struct Abs<DataType, D, DataAllocator, 0>
	{
		__CUDA_HOST_AND_DEVICE__ 
		static inline void eval( iPoint<DataType,D,DataAllocator>& result,
								 const iPoint<DataType,D,DataAllocator>& p )
		{
			result[0] = (DataType)  abs(p[0]);
		}
	};


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Main Point Class ////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// DataType 	= Floating Point Type													  	  //
	// D   			= Dimension of Point														  //
	// dAllocator	= Data structure storage													  //
	// TODO			= ......................													  //
	////////////////////////////////////////////////////////////////////////////////////////////////

	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator >
	class iPoint : private dAllocator<DataType, D>
	{

		typedef dAllocator<DataType, D>	_data;	// data holder

	public:

		////////////////////////////////////////////////////////////////////////////////////////////
		// Constructors ////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////
		__CUDA_HOST_AND_DEVICE__
		inline iPoint() : _data() { origin<DataType, D, dAllocator, D-1>::eval(*this); };

		// 1D Point
		__CUDA_HOST_AND_DEVICE__
		inline explicit iPoint(const DataType x0) : _data() { EquateScalar<DataType,D,dAllocator,D-1>::eval(*this,x0);}

		// 2D Point
		__CUDA_HOST_AND_DEVICE__
		inline explicit iPoint(const DataType& x0, const DataType& x1):_data(x0,x1){};

		// 3D Point
		__CUDA_HOST_AND_DEVICE__
		inline explicit iPoint(const DataType& x0, const DataType& x1, const DataType& x2)
		:_data(x0,x1,x2){};

		// 4D Point
		__CUDA_HOST_AND_DEVICE__
		inline explicit iPoint(const DataType& x0, const DataType& x1, const DataType& x2, const DataType& x3)
		:_data(x0,x1,x2,x3){};

		// Array Initialization
		__CUDA_HOST_AND_DEVICE__ 
		inline explicit iPoint( const DataType * ax ) : _data() { EquateArray<DataType,D,dAllocator,D-1>::eval(*this,ax);}

		// Init from another point (copy constructor)
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint(const iPoint& p)
		{	Equate<DataType,D,dAllocator,D-1>::eval((*this),p);	 };

#if	__MENTALRAY_ENHANCED__
		// mentalray support
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint(const miVector& p):_data(p.x,p.y,p.z){};				//miVector

		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint(const miColor& p):_data(p.r,p.g,p.b){};				//miColor
#endif

#if	__QT_ENHANCED__
		// QT support
		__CUDA_HOST_AND_DEVICE__ 
		inline explicit iPoint(const QVector3D& p):_data(p.x(),p.y(),p.z()){};
																			//QVector3D

		__CUDA_HOST_AND_DEVICE__ 
		inline explicit iPoint(const QVector2D& p):_data(p.x(),p.y()){};	//QVector2D
		inline explicit iPoint(const QPoint& p):_data(p.x(),p.y()){};		//QPoint
		inline explicit iPoint(const QPointF& p):_data(p.x(),p.y()){};		//QPointF
#endif

#if	__OPENEXR_ENHANCED__
		__CUDA_HOST_AND_DEVICE__
		inline explicit iPoint(const V3f& p):_data(p.x,p.y,p.z){};			//V3f
		inline explicit iPoint(const C3f& p):_data(p.r,p.g,p.b){};		//C3f
#endif


		// Destructor //////////////////////////////////////////////////////////////////////////////
		__CUDA_HOST_AND_DEVICE__
		~iPoint(){_data::clearData();};


		////////////////////////////////////////////////////////////////////////////////////////////
		// Utilities functions /////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////
		// Get pointer to coords array
		__CUDA_HOST_AND_DEVICE__
		INLINE const DataType* begin() { return _data::getDataPtr(); }
		__CUDA_HOST_AND_DEVICE__
		INLINE const DataType* begin() const { return _data::getDataPtr(); }

/*		__CUDA_HOST_AND_DEVICE__
		INLINE const DataType* end() { return _iD+((sizeof(_iD)/sizeof(DataType))-1); }
		__CUDA_HOST_AND_DEVICE__ 
		INLINE const DataType* end() const { return _iD+((sizeof(_iD)/sizeof(DataType))-1); }
*/
		// Get/Set coords in XYZ style
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetX() const { return _data::getValue(0); };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetY() const { return _data::getValue(1); };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetZ() const { return _data::getValue(2); };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetW() const { return _data::getValue(3); };

		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetX(const DataType& Val) { _data::setValue(Val,0); return *this; };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetY(const DataType& Val) { _data::setValue(Val,1); return *this; };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetZ(const DataType& Val) { _data::setValue(Val,2); return *this; };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetW(const DataType& Val) { _data::setValue(Val,3); return *this; };

		// Get/Set  coords in RGB style
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetR() const { return _data::getValue(0); };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetG() const { return _data::getValue(1); };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetB() const { return _data::getValue(2); };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE DataType GetA() const { return _data::getValue(3); };

		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetR(const DataType& Val) { _data::setValue(Val,0); return *this; };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetG(const DataType& Val) { _data::setValue(Val,1); return *this; };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetB(const DataType& Val) { _data::setValue(Val,2); return *this; };
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& SetA(const DataType& Val) { _data::setValue(Val,3); return *this; };

		// Initialize all coords to zero
		__CUDA_HOST_AND_DEVICE__ 
		INLINE void reset(){ origin<DataType, D, dAllocator, D-1>::eval(*this); };


#if	__MENTALRAY_ENHANCED__
		// Get coords as miVector style
		__CUDA_HOST_AND_DEVICE__ 
		INLINE miVector AsMiVector() 
		{ 
			miVector ret = {_data::getValue(0), _data::getValue(1), _data::getValue(2)};
			return ret; 
		};

		// Get coords as miColor style
		__CUDA_HOST_AND_DEVICE__ 
		INLINE miColor AsMiColor() 
		{ 
			miColor ret = {_data::getValue(0), _data::getValue(1), _data::getValue(2), 0};
			return ret; 
		};
#endif

#if	__QT_ENHANCED__
		// QT support
		__CUDA_HOST_AND_DEVICE__ 
		INLINE QVector3D AsQVector3D() 
		{ 
			QVector3D ret( _data::getValue(0), _data::getValue(1), _data::getValue(2) );
			return ret; 
		};
		__CUDA_HOST_AND_DEVICE__ 
		INLINE QVector3D AsQVector2D() 
		{ 
			QVector2D ret( _data::getValue(0), _data::getValue(1) );
			return ret; 
		};
#endif

#if	__OPENEXR_ENHANCED__
		INLINE V3f AsV3f() 
		{ 
			V3f ret (		_data::getValue(0),
							_data::getValue(1),
							_data::getValue(2) );
			return ret; 
		};

		INLINE C3f AsC3f() 
		{ 
			C3f ret (		_data::getValue(0),
							_data::getValue(1),
							_data::getValue(2) );
			return ret; 
		};
#endif


		////////////////////////////////////////////////////////////////////////////////////////////
		// Others //////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////
		// Storage type for external and friend stuff
		typedef DataType				__NumType;
		typedef dAllocator<DataType, D>	__dAlloc;
		// Dimension of points
		static const iUint				__DIM = D;



		////////////////////////////////////////////////////////////////////////////////////////////
		// Operators ///////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////

		// Add
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint<DataType,D,dAllocator> operator+ (	const iPoint<DataType,D,dAllocator>& q) const
		{
			iPoint<DataType,D, dAllocator> result;
			Add<DataType,D, dAllocator,D-1>::eval(result,*this,q);
			return result;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint operator+ (	const DataType& q )
		{
			iPoint<DataType,D, dAllocator> result;
			AddScalar<DataType,D, dAllocator,D-1>::eval(result,*this,q);
			return result;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint&	operator+= ( const iPoint<DataType,D,dAllocator>& q )
		{
			Add<DataType,D, dAllocator,D-1>::eval(*this,*this,q);
			return *this;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint&	operator+= (	const DataType& q)
		{
			AddScalarX<DataType,D, dAllocator,D-1>::eval(*this,q);
			return *this;
		}

		// Substract
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint<DataType,D,dAllocator> operator- (	const iPoint<DataType,D,dAllocator>& q) const
		{
			iPoint<DataType,D, dAllocator> result;
			Subtract<DataType,D, dAllocator,D-1>::eval(result,*this,q);
			return result;
		}
		__CUDA_HOST_AND_DEVICE__
		inline iPoint	operator- (	const DataType& q )
		{
			iPoint<DataType,D, dAllocator> result;
			SubtractScalar<DataType,D, dAllocator,D-1>::eval(result,*this,q);
			return result;
		}

		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend INLINE iPoint<NT,__DIM,__dAlloc>	operator- (	const NT& q ,
													const iPoint<NT,__DIM,__dAlloc>& p );

		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint	operator- (	void ) const //unary
		{
			iPoint<DataType,D, dAllocator> hInv;
			Invert<DataType,D, dAllocator,D-1>::eval(hInv,*this);
			return hInv;
		}

		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint&	operator-= (	const iPoint<DataType,D,dAllocator>& q)
		{
			Subtract<DataType,D, dAllocator,D-1>::eval(*this,*this,q);
			return *this;
		}

		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint&	operator-= (	const DataType& q)
		{
			SubtractScalarX<DataType,D, dAllocator,D-1>::eval(*this,q);
			return *this;
		}

		// Multiply
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint operator* (	const iPoint<DataType,D,dAllocator>& q )
		{
			iPoint<DataType,D, dAllocator> result;
			Multiply<DataType,D, dAllocator,D-1>::eval(result,*this,q);
			return result;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint operator* (	const DataType& q )
		{
			iPoint<DataType,D,dAllocator> result;
			MultiplyScalar<DataType,D, dAllocator,D-1>::eval(result,*this,q);
			return result;		
		}		
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint& operator*= ( const iPoint<DataType,D,dAllocator>& q )
		{
			MultiplyX<DataType,D, dAllocator,D-1>::eval(*this,q);
			return *this;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint&	operator*= ( const DataType& q )
		{
			MultiplyScalarX<DataType,D, dAllocator,D-1>::eval(*this, q);
			return *this;
		}
		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend INLINE iPoint<NT,__DIM,__dAlloc>	operator* (	const NT& q ,
													const iPoint<NT,__DIM,__dAlloc>& p );

		// Divide
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint operator/ (	const iPoint<DataType,D,dAllocator>& q )
		{
			iPoint<DataType,D, dAllocator> result;
			Reciprocal<DataType,D, dAllocator,D-1>::eval(result,q);
			Multiply<DataType,D, dAllocator,D-1>::eval(result,*this,result);
			return result;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint<DataType,D,dAllocator> operator/ ( const DataType& q )
		{
			iPoint<DataType,D, dAllocator> result;
			MultiplyScalar<DataType,D, dAllocator,D-1>::eval( result, *this, (DataType)(1.0f/q));
			return result;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint&	operator/= ( const iPoint<DataType,D,dAllocator>& q)
		{
			iPoint<DataType,D, dAllocator> invQ;
			Reciprocal<DataType,D, dAllocator,D-1>::eval(invQ,q);
			Multiply<DataType,D, dAllocator,D-1>::eval(*this,*this,invQ);
			return *this;
		}
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint& operator/= ( const DataType& q)
		{
			MultiplyScalar<DataType,D, dAllocator,D-1>::eval(*this,*this, (DataType) (1.0f/q));
			return *this;
		}

		// Assignment //////////////////////////////////////////////////////////////////////////////
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& operator= (const iPoint<DataType,D,dAllocator>& q)
		{
			//Assert((this != &q), "Error p = p");
			Equate<DataType,D,dAllocator,D-1>::eval(*this,q);
			return *this;
		}
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& operator= (const DataType& q)
		{
			EquateScalar<DataType,D,dAllocator,D-1>::eval(*this,q);
			return *this;
		}

		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& operator= (const DataType* q)
		{
			EquateArray<DataType,D,dAllocator,D-1>::eval((*this),q);
			return *this;
		}

#if	__MENTALRAY_ENHANCED__
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& operator= (const miVector& q)
		{
			iPoint<DataType,D, dAllocator> ttvec(q);
			Equate<DataType,D,dAllocator,D-1>::eval(*this,ttvec);
			return *this;
		}
		__CUDA_HOST_AND_DEVICE__ 
		INLINE iPoint& operator= (const miColor& q)
		{
			iPoint<DataType,D, dAllocator> ttvec(q);
			Equate<DataType,D,dAllocator,D-1>::eval(*this,ttvec);
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

		// Dereference iData ///////////////////////////////////////////////////////////////////////
		__CUDA_HOST_AND_DEVICE__ 
		inline DataType& operator[](const iUint i) { return _data::getValue(i); };

		__CUDA_HOST_AND_DEVICE__ 
		inline const DataType& operator[](const iUint i) const { return _data::getValue(i); };

		INLINE DataType* operator+ ( int q ) const	{ return (DataType*)_data::getDataRef()+q; } // + pointer inc
		INLINE DataType* operator- ( int q ) const	{ return (DataType*)_data::getDataRef()-q; } // - pointer dec

		INLINE operator __m128( void ) { 
			__m128 xmm; ToXMM<DataType, D,dAllocator,D-1>::eval( xmm, *this );
			return xmm; 
		}


#if __QT_ENHANCED__
		INLINE operator QVector3D ( void ) const	{ return QVector3D(_data::getValue(0), _data::getValue(1), _data::getValue(2)); }	// used as __m128
		//INLINE operator QVector3D& ( void )			{ return QVector3D(_data::getValue(0), _data::getValue(1), _data::getValue(2)); }	// used as __m128
#endif

#if	__OPENEXR_ENHANCED__
		INLINE operator V3f ( void ) const
		{
			V3f t (_data::getValue(0), _data::getValue(1), _data::getValue(2));
			return t;
		}
		INLINE operator C3f ( void ) const
		{
			C3f t (_data::getValue(0), _data::getValue(1), _data::getValue(2));
			return t;
		}
#endif
		//INLINE DataType* operator& (){ return &_iD[0]; };								// & reference
		//INLINE operator DataType* ( void ) { return &_iD[0]; }						// risky-pesky !
		//INLINE operator const DataType* ( void ) const { return &_iD[0]; }			// risky-pesky !


		// Valarray ////////////////////////////////////////////////////////////////////////////////
/*		inline iPoint& operator= (const valarray<DataType>& v);

		inline operator valarray<DataType>() const;
		template<typename DataType, unsigned D>
		operator valarray<DataType>() const
		{
			valarray<DataType> result((*this)._iD , D);
			return result;	
		}
		template<typename DataType, unsigned D>
		iPoint<DataType,D>&
		operator= (const valarray<DataType>& v)
		{
			iPoint<DataType,D> result;
			for(int i = 0; i < D; i++)							// valarray should be of size D
			(*this)._iD[i] = v[i];								// unwind for loop into template
			return (*this);	
		}
*/

		// Comparison //////////////////////////////////////////////////////////////////////////////
		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend bool   operator== (	const iPoint<NT,__DIM,__dAlloc>& p,
									const iPoint<NT,__DIM,__dAlloc>& q);

		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend bool   operator== (	const iPoint<NT,__DIM,__dAlloc>& p,
									const NT& q);

		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend bool   operator!= (	const iPoint<NT,__DIM,__dAlloc>& p,
									const iPoint<NT,__DIM,__dAlloc>& q);

		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend bool   operator!= (	const iPoint<NT,__DIM,__dAlloc>& p,
									const NT& q);

		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend bool   operator< (	const iPoint<NT,__DIM,__dAlloc>& p,
									const NT& q);

		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend bool   operator<= (	const iPoint<NT,__DIM,__dAlloc>& p,
									const iPoint<NT,__DIM,__dAlloc>& q);

		template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
		__CUDA_HOST_AND_DEVICE__ 
		friend bool   operator> (	const iPoint<NT,__DIM,__dAlloc>& p,
									const NT& q);


		////////////////////////////////////////////////////////////////////////////////////////////
		// Geometry functions //////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////

		// Returns the dimension
		__CUDA_HOST_AND_DEVICE__ 
		inline int      dim() const { return D; };

		// Members sum
		__CUDA_HOST_AND_DEVICE__ 
		inline DataType  sum(void)  const;

		// Members product
		__CUDA_HOST_AND_DEVICE__ 
		inline DataType  product(void)  const;

		// Members mean
		__CUDA_HOST_AND_DEVICE__ 
		inline DataType  mean(void)  const;
		//Sqrt
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint<DataType,D,dAllocator>  sqrtX (void) const;

		// Squared length //////////////////////////////////////////////////////////////////////////
		__CUDA_HOST_AND_DEVICE__ 
		inline DataType  length(void)  const;
		inline DataType  sqr_length(void)  const;
		inline DataType  length2(void)  const;

		// Distance to a point
		__CUDA_HOST_AND_DEVICE__ 
		inline DataType  distance(const iPoint<DataType,D,dAllocator>& q) const ;

		// Squared distance to a point 
		__CUDA_HOST_AND_DEVICE__ 
		inline double  sqr_dist(const iPoint<DataType,D,dAllocator>& q) const ;

		// Dot/Cross/Normalize /////////////////////////////////////////////////////////////////////
		__CUDA_HOST_AND_DEVICE__
		inline DataType  dot (const iPoint<DataType,D,dAllocator>& q)  const
		{
			return DotProd<DataType,D,dAllocator,D-1>::eval(*this,q);
		}
		__CUDA_HOST_AND_DEVICE__
		inline iPoint  dotX (const iPoint<DataType,D,dAllocator>& q)  const
		{
			return iPoint( DotProd<DataType,D,dAllocator,D-1>::eval(*this,q) );
		}

		// Cross-product
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint	cross (const iPoint<DataType,D,dAllocator>& q) const;

		// Normalize vector length
		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint	normalize (void);

		__CUDA_HOST_AND_DEVICE__ 
		inline iPoint&	getNormalized (void);


		// Others //////////////////////////////////////////////////////////////////////////////////
		// TODO: add 'others' from SSE impl

		__CUDA_HOST_AND_DEVICE__
		inline iPoint  abs ( void )
		{
			iPoint holder;
			Abs<DataType,D,dAllocator,D-1>::eval(holder, *this);
			return holder;
		}

		// OCtree related //////////////////////////////////////////////////////////////////////////
		__CUDA_HOST_AND_DEVICE__ 
		inline DataType sqr_dist_bbox ( const iPoint<DataType,D, dAllocator> &iMin,
										const iPoint<DataType,D, dAllocator> &iMax) const ;

		// Octree node that own the point
		__CUDA_HOST_AND_DEVICE__ 
		inline int find_child_bbox (const iPoint<DataType,D, dAllocator> &iMin,
									const iPoint<DataType,D, dAllocator> &iMax) const ;

		// Expand bbox regard new point
		__CUDA_HOST_AND_DEVICE__ 
		inline void expand_bbox ( iPoint<DataType,D, dAllocator> &iMin, iPoint<DataType,D, dAllocator>& iMax) const ;
	};




	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// Implemntations ////////////////////////////////////////////////////////////////////////////////////////////////

	//Friend Operators

	//Multiply
	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	iPoint<NT,__DIM,__dAlloc>
	operator* ( const NT& q, const iPoint<NT,__DIM,__dAlloc>& p )
	{
		iPoint<NT,__DIM,__dAlloc> result;
		MultiplyScalar<NT,__DIM,__dAlloc,__DIM-1>::eval(result,p,q);
		return result;
	}

	//Comparison
	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	bool
	operator== (const iPoint<NT,__DIM,__dAlloc>& p, const iPoint<NT,__DIM,__dAlloc>& q)
	{
		return IsEqual<NT,__DIM,__dAlloc,__DIM-1>::eval(p,q);
	}

	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	bool
	operator== (const iPoint<NT,__DIM,__dAlloc>& p, const NT& q)
	{
		return IsEqualScalar<NT,__DIM,__dAlloc,__DIM-1>::eval(p,q);
	}

	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	bool
	operator!= (const iPoint<NT,__DIM,__dAlloc>& p, const iPoint<NT,__DIM,__dAlloc>& q)
	{
		return !(IsEqual<NT,__DIM,__dAlloc,__DIM-1>::eval(p,q));
	}

	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	bool
	operator!= (const iPoint<NT,__DIM,__dAlloc>& p, const NT& q)
	{
		return !(IsEqualScalar<NT,__DIM,__dAlloc,__DIM-1>::eval(p,q));
	}

	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	bool
	operator< (const iPoint<NT,__DIM,__dAlloc>& p, const NT& q)
	{
		return IsMinorScalar<NT,__DIM,__dAlloc,__DIM-1>::eval(p,q);
	}

	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	bool
	operator<= (const iPoint<NT,__DIM,__dAlloc>& p, const iPoint<NT,__DIM,__dAlloc>& q)
	{
		return IsMinorEq<NT,__DIM,__dAlloc,__DIM-1>::eval(p,q);
	}

	template<typename NT, unsigned __DIM, template<typename, unsigned> class __dAlloc>
	__CUDA_HOST_AND_DEVICE__ 
	bool
	operator> (const iPoint<NT,__DIM,__dAlloc>& p, const NT& q)
	{
		return !(IsMinorScalar<NT,__DIM,__dAlloc,__DIM-1>::eval(p,q));
	}

	//Geometry /////////////////////////////////////////////////////////////////////////////////////

	//Sum
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::sum (void) const
	{
		return Sum<DataType,D, dAllocator,D-1>::eval(*this);
	}

	//Product
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::product (void) const
	{
		return Product<DataType,D, dAllocator,D-1>::eval(*this);
	}

	//Mean
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::mean (void) const
	{
		DataType hMean = Sum<DataType,D, dAllocator,D-1>::eval(*this);
		return hMean * (1.0f/D);	
	}

	//Sqrt
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	iPoint<DataType,D,dAllocator> 
	iPoint<DataType,D,dAllocator>::sqrtX (void) const
	{
		iPoint holder(*this);	
		//SqrtX<DataType,D,dAllocator,D-1>::eval( iPoint );
		return holder;
	}

	//Magnitudes ///////////////////////////////////////////////////////////////////////////////////
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::length (void) const
	{
		return sqrt(DotProd<DataType,D,dAllocator,D-1>::eval(*this,*this));
	}
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::sqr_length (void) const
	{
		return DotProd<DataType,D,dAllocator,D-1>::eval(*this,*this);
	}
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::length2 (void) const
	{
		return DotProd<DataType,D,dAllocator,D-1>::eval(*this,*this);
	}

	//Distance
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::distance (const iPoint<DataType,D,dAllocator>& q) const
	{
		return sqrt( static_cast<double>(Distance<DataType,D,dAllocator,D-1>::eval(*this,q)) );
	}

	//SqrDist
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	double
	iPoint<DataType,D,dAllocator>::sqr_dist (const iPoint<DataType,D,dAllocator>& q) const
	{
		return DistanceF<DataType,D,dAllocator,D-1>::eval(*this,q);
	}

	// Cross-Product
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	iPoint<DataType,D,dAllocator>
	iPoint<DataType,D,dAllocator>::cross (const iPoint<DataType,D, dAllocator>& rhs) const
	{
		iPoint<DataType,D,dAllocator> result;
		iPoint<DataType,D,dAllocator> lhs = *this;
		iPoint<DataType,D,dAllocator> subtractMe(lhs[2]*rhs[1], lhs[0]*rhs[2], lhs[1]*rhs[0]);

		result[0] = (lhs[1] * rhs[2]);
		result[0] -= subtractMe[0];
		result[1] = (lhs[2] * rhs[0]);
		result[1] -= subtractMe[1];
		result[2] = (lhs[0] * rhs[1]);
		result[2] -= subtractMe[2];
		return result;
	}

	//Normalize
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	iPoint<DataType,D,dAllocator>
	iPoint<DataType,D,dAllocator>::normalize ( void )
	{
		iPoint<DataType,D,dAllocator> hNorm( *this );
		const DataType& len = sqrtf(sqr_length()) /*+ 0.00000001f*/;

		if (len > 0.00000001)
		hNorm /= len;

		return hNorm;
	}

	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	iPoint<DataType,D,dAllocator>&
	iPoint<DataType,D,dAllocator>::getNormalized ( void )
	{
		return (*this /= sqrtf(sqr_length()));
	}


	//Octree related ///////////////////////////////////////////////////////////////////////////////
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	DataType 
	iPoint<DataType,D,dAllocator>::sqr_dist_bbox (  const iPoint<DataType,D, dAllocator> &iMin,
													const iPoint<DataType,D, dAllocator> &iMax ) const
	{
		iPoint<DataType,D,dAllocator> c,s,d;

		//Unroll everything ..............
		
		// Compute bbox center c
		c[0] = 0.5f * (iMax[0] + iMin[0]);
		c[1] = 0.5f * (iMax[1] + iMin[1]);
		c[2] = 0.5f * (iMax[2] + iMin[2]);
		//c = ( iMax+iMin ) * 0.5f;

		// Compute bbox half-side lengths s
		s[0] = 0.5f * (iMax[0] - iMin[0]);
		s[1] = 0.5f * (iMax[1] - iMin[1]);
		s[2] = 0.5f * (iMax[2] - iMin[2]);
		//c = ( iMax-iMin ) * 0.5f;

		// Initial separation vector: d = vector bbox center c to point p
		d[0] = _data::getValue(0) - c[0];
		d[1] = _data::getValue(1) - c[1];
		d[2] = _data::getValue(2) - c[2];
		//d = *this-c;

		// Update separation vector 
		if (d[0] <= -s[0])
			d[0] += s[0];   // outside min bound
		else if (d[0] >= s[0])
			d[0] -= s[0];   // outside max bound
		else
			d[0] = 0.0f;    // inside bounds

		if (d[1] <= -s[1])
			d[1] += s[1];   // outside min bound
		else if (d[1] >= s[1])
			d[1] -= s[1];   // outside max bound
		else
			d[1] = 0.0f;    // inside bounds

		if (d[2] <= -s[2])
			d[2] += s[2];   // outside min bound
		else if (d[2] >= s[2])
			d[2] -= s[2];   // outside max bound
		else
			d[2] = 0.0f;	// inside bounds

		return d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
	};

	// BBox point belongs to in octree
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	int 
	iPoint<DataType,D,dAllocator>::find_child_bbox (const iPoint<DataType,D, dAllocator> &iMin,
													const iPoint<DataType,D, dAllocator> &iMax ) const
	{
		iPoint<DataType,D,dAllocator> midpoint;

		int i, j, k, child;

		//assert(iMin[0] <= _iD[0] && _iD[0] <= iMax[0]);
		//assert(iMin[1] <= _iD[1] && _iD[1] <= iMax[1]);
		//assert(iMin[2] <= _iD[2] && _iD[2] <= iMax[2]);

		// compute midpoint of bbox brick 
		midpoint[0] = 0.5f * (iMax[0] + iMin[0]);
		midpoint[1] = 0.5f * (iMax[1] + iMin[1]);
		midpoint[2] = 0.5f * (iMax[2] + iMin[2]);
		//midpoint = 0.5f * ( iMax+iMin );

		// compute which child point p is in: i,j,k
		i = (_data::getValue(0) >= midpoint[0]) ? 1 : 0;
		j = (_data::getValue(1) >= midpoint[1]) ? 2 : 0;
		k = (_data::getValue(2) >= midpoint[2]) ? 4 : 0;

		child = i + j + k;
		//assert(0 <= child && child <= 7);

		return child;
	}

	// Expand BBox
	template<typename DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__
	void
	iPoint<DataType,D,dAllocator>::expand_bbox (iPoint<DataType,D, dAllocator> &iMin,
												iPoint<DataType,D, dAllocator> &iMax) const
	{
		if (_data::getValue(0) < iMin[0])	iMin[0] = _data::getValue(0);
		if (_data::getValue(1) < iMin[1])	iMin[1] = _data::getValue(1);
		if (_data::getValue(2) < iMin[2])	iMin[2] = _data::getValue(2);
		if (_data::getValue(0) > iMax[0])	iMax[0] = _data::getValue(0);
		if (_data::getValue(1) > iMax[1])	iMax[1] = _data::getValue(1);
		if (_data::getValue(2) > iMax[2])	iMax[2] = _data::getValue(2);
	}


	//Others ///////////////////////////////////////////////////////////////////////////////////////

	//Stream
	template < class DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	ostream&
	operator<<(ostream& os,const iPoint<DataType,D,dAllocator> &p)
	{
		os << "Point" << D << "D< ";
		for (int i=0; i<(int)D-1; ++i)
		os << p[i] << ", ";

		return os << p[D-1] << " >";
	}

	template < class DataType, unsigned D, template<typename, unsigned> class dAllocator>
	__CUDA_HOST_AND_DEVICE__ 
	istream&
	operator>>(istream& is,iPoint<DataType,D,dAllocator> &p)
	{
		for (int i=0; i<D; ++i)
		if(!(is >> p[i]))
		if(!is.eof()){
			cerr << "Error Reading Point:" << is << endl;
			exit(1);
		}
		return is;
	}

}	// Namespace Ends here
#endif
