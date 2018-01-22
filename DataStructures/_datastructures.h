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

#ifndef __PCLOUD_DATASTRUCT__
#define __PCLOUD_DATASTRUCT__

#include <stdlib.h>
#include <malloc.h>
#include <math.h>

#include <vector>
#include <bitset>
#include <algorithm>

#if	__MENTALRAY_ENHANCED__
#include "shader.h"
#endif

#if	__QT_ENHANCED__
#include <QtGui/QVector3D>
#endif

#if	__OPENEXR_ENHANCED__
#include <Imath/ImathVec.h>
#include <Imath/ImathBox.h>
#include <Imath/ImathColor.h>
using Imath::V3f;
using Imath::C3f;
using Imath::Box3f;
#endif



using namespace std;


namespace cpc
{

// TO BE MOVED? ////////////////////////////////////////////////////////////////////////////////////
#ifdef _MSC_VER
#define INLINE		__forceinline					// __forceinline 
#define NOINLINE	__declspec(noinline)
#else
#define INLINE		__attribute__((always_inline))	// GCC
#define NOINLINE	__attribute__((noinline))
#endif

// Memory allocation
#define MEMALLOC(dtype,size) malloc(sizeof(dtype)*size)
#define MEMREALLOC(mem, dtype,size) realloc(mem, sizeof(dtype)*size)
#define MEMFREE(ptr) free(ptr)

// User abortation
#define ABORTCALLED 0

// Print verbose
#define PRINTME printf


// Common stuff ////////////////////////////////////////////////////////////////////////////////////
static const float  f_pi	= 3.14159265f;
static const double d_pi	= 3.1415926535897932384626433832795;
static const float  f_pi_div2 = 1.57079632679489661923;
static const float	inv_pi  = 0.318309886183f;
static const float	four_pi = 4.0f * 3.14159265358979323846f;
static const float	epsilon = 1e-6f;
static const float	M_SQRT2 = 1.41421356237309504880168872420969808;
#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2		1.57079632679489661923
#endif
//static float aodefault[4] = {1.f,1.f,.6f};

#define ds_MIN(a,b)	((a) < (b) ? (a) : (b))
#define ds_MAX(a,b)	((a) > (b) ? (a) : (b))
#define ds_MIN3(a,b,c)	((a) < (b) ? ds_MIN(a,c) : ds_MIN(b,c))
#define ds_MAX3(a,b,c)	((a) > (b) ? ds_MAX(a,c) : ds_MAX(b,c))

#define ROUND_DOWN(x, s) ((x) & ~((s)-1))

#define MAXPTSINNODE 8
// !!!!!!!!!!!!!!!!!!!


// MentalRay ///////////////////////////////////////////////////////////////////////////////////////
#if	__MENTALRAY_ENHANCED__
	#ifdef MEMALLOC
	#undef MEMALLOC
	#endif
	#define MEMALLOC(dtype,size) mi_mem_allocate(sizeof(dtype)*size)

	#ifdef MEMREALLOC
	#undef MEMREALLOC
	#endif
	#define MEMREALLOC(mem,dtype,size) mi_mem_reallocate(mem, sizeof(dtype)*size)

	#ifdef MEMFREE
	#undef MEMFREE
	#endif
	#define MEMFREE(ptr) mi_mem_release(ptr)

	#ifdef ABORTCALLED
	#undef ABORTCALLED
	#endif
	#define ABORTCALLED (mi_par_aborted())

	#ifdef PRINTME
	#undef PRINTME
	#endif
	#define PRINTME mi_info
#endif


//TEMP !!!!!!!!!!!!!!!!
#define WARNME PRINTME
#define ERRORME PRINTME


// OpenMP //////////////////////////////////////////////////////////////////////////////////////////
//#define __OMP_ENHANCED__ 0

#if	__OMP_ENHANCED__
#ifdef __MSC_VER
	#ifndef _OPENMP
		#undef	__OMP_ENHANCED__
		#define __OMP_ENHANCED__ 0
	#else
		#include <omp.h>
	#endif
#else
#include <omp.h>
#endif
#endif


// SSE /////////////////////////////////////////////////////////////////////////////////////////////
//#include <emmintrin.h>	//see point class that uses spare __m128 ..
//#define __SSE2_ENHANCED__ 0

#if	__SSE2_ENHANCED__
/*	Header files:
		SSE: 	xmmintrin.h
		SSE2: 	emmintrin.h
		SSE3: 	pmmintrin.h
		SSSE3: 	tmmintrin.h
		SSE4: 	smmintrin.h and nmmintrin.h
*/
	//#include "nmmintrin.h"
	//#include <smmintrin.h>
	#include"_math_sse.h"


	#define ALIGNPAD 0x10
	#ifdef _MSC_VER
	#define SSE_ALIGNED(x) __declspec(align(ALIGNPAD)) x
	#else
	#define SSE_ALIGNED(x) x __attribute__ ((aligned (ALIGNPAD)))
	#endif

	#ifdef MEMALLOC
	#undef MEMALLOC
	#endif
	#ifdef _MSC_VER
	#define MEMALLOC(dtype,size) _aligned_malloc( size * sizeof(dtype), ALIGNPAD );
	#else
	#define MEMALLOC(dtype,size) memalign( ALIGNPAD, size * sizeof(dtype) );
	#endif

	#ifdef MEMREALLOC
	#undef MEMREALLOC
	#endif
	#ifdef _MSC_VER
	#define MEMREALLOC(mem,dtype,size) _aligned_realloc( mem, size * sizeof(dtype), ALIGNPAD );
	#else
	#define MEMREALLOC(dtype,size) memalign( ALIGNPAD, size * sizeof(dtype) );
	#endif

	#ifdef MEMFREE
	#undef MEMFREE
	#endif
	#ifdef _MSC_VER
	#define MEMFREE(ptr) _aligned_free(ptr)
	#else
	#define MEMFREE(ptr) free(ptr);
	#endif

    static INLINE __m128 _mm_stream_load(const float *mem) 
	{
        return _mm_castsi128_ps(_mm_stream_load_si128(reinterpret_cast<__m128i *>(const_cast<float *>(mem))));
    }

	///////////////////////////////////////////////////////////////////////////
	#define DATA_NSLOTS_AO			12
	#define DATA_NSLOTS_AO_TRIVIS	24
	#define DATA_NSLOTS_SSS			24
	#define DATA_NSLOTS_BAKE		16

	#define DATA_RESULT_SLOT		4

	#define DATA_SLOT_POS			0
	#define DATA_SLOT_DIR			4

	#define DATA_SLOT_AREA			8

	#define DATA_SLOT_T0			12
	#define DATA_SLOT_T1			16
	#define DATA_SLOT_T2			20

	#define DATA_SLOT_IRRAD			12
	#define DATA_SLOT_ALBEDO		16
	#define DATA_SLOT_DMFP			20
	
	#define DATA_SLOT_BATTR			12

	#define DATA_VECTOR_END			3

#else
static const __m128 M128_ONE			= _mm_set1_ps(1.0f);

	#define DATA_NSLOTS_AO			7
	#define DATA_NSLOTS_AO_TRIVIS	16
	#define DATA_NSLOTS_SSS			16
	#define DATA_NSLOTS_BAKE		10

	#define DATA_RESULT_SLOT		3

	#define DATA_SLOT_POS			0
	#define DATA_SLOT_DIR			3

	#define DATA_SLOT_AREA			6

	#define DATA_SLOT_T0			7
	#define DATA_SLOT_T1			10
	#define DATA_SLOT_T2			13

	#define DATA_SLOT_IRRAD			7
	#define DATA_SLOT_ALBEDO		10
	#define DATA_SLOT_DMFP			13

	#define DATA_SLOT_BATTR			7

	#define DATA_VECTOR_END			2
#endif
#define DATA_SLOT_MINIMAL			DATA_NSLOTS_AO


// cuda ////////////////////////////////////////////////////////////////////////////////////////////
#ifndef __device__
#define __CUDA_HOST_AND_DEVICE__
#else
#define __CUDA_HOST_AND_DEVICE__ __host__ __device__
#endif


// typedefs ////////////////////////////////////////////////////////////////////////////////////////
typedef unsigned int iUint;
typedef unsigned short iUshort;
typedef char Byte;
static const /*__declspec(align(0x10))*/float iZERO[4] = {0.f,0.f,0.f,0.f};

}	// namespace

// datastructures //////////////////////////////////////////////////////////////////////////////////
typedef float LIB_DTYPE;

#include "iPoint.hpp"
#if	__SSE2_ENHANCED__
#include "iPointSSE.hpp"
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
#include "_others\iMorton.hpp"
//using namespace cpc;
namespace cpc
{

typedef iPoint<LIB_DTYPE, 3>	iVector3;
typedef iPoint<LIB_DTYPE, 2>	iVector2;
typedef iPoint<LIB_DTYPE, 3>	iColor3;
#if	__SSE2_ENHANCED__
SSE_ALIGNED(
	struct iBBox { 
	SSE_ALIGNED(iVector3 min);
	SSE_ALIGNED(iVector3 max); 
	}
);
#else
struct	iBBox { iVector3 min, max; };
#endif
typedef iBBox iBBox;


}
#include "_others\iMatrix.hpp"

/// /////////////////////////////////////////////////////////////////////////////////////////////////
/*namespace std
{
	// Standalone swap like std::swap template specialization
	template<>
	INLINE void swap(float*& pFirst, float*& pSecond)
	{
		__m128 XMM0 = _mm_load_ps((pFirst)   );
		__m128 XMM1 = _mm_load_ps((pFirst)+ 4);
		__m128 XMM2 = _mm_load_ps((pFirst)+ 8);
		__m128 XMM3 = _mm_load_ps((pFirst)+12);

		__m128 XMM4 = _mm_load_ps((pSecond)   );
		__m128 XMM5 = _mm_load_ps((pSecond)+ 4);
		__m128 XMM6 = _mm_load_ps((pSecond)+ 8);
		__m128 XMM7 = _mm_load_ps((pSecond)+12);
		
		_mm_store_ps((pFirst)   , XMM4);
		_mm_store_ps((pFirst)+ 4, XMM5);
		_mm_store_ps((pFirst)+ 8, XMM6);
		_mm_store_ps((pFirst)+12, XMM7);

		_mm_store_ps((pSecond)   , XMM0);
		_mm_store_ps((pSecond)+ 4, XMM1);
		_mm_store_ps((pSecond)+ 8, XMM2);
		_mm_store_ps((pSecond)+12, XMM3);
	}
}
*/

// Data structure //////////////////////////////////////////////////////////////////////////////////
#if	__MENTALRAY_ENHANCED__
#include "iSurfel.hpp"
//typedef iSurfelContainer<LIB_DTYPE>									iSurfelCacheContainer;
//typedef iSurfelCache<LIB_DTYPE, DATA_NSLOTS_AO>						iSurfelAOCache_Base;
//typedef iSurfelCache<LIB_DTYPE, DATA_NSLOTS_AO_TRIVIS>				iSurfelAOCache_Tri;
//typedef iSurfelCache<LIB_DTYPE, DATA_NSLOTS_SSS>						iSurfelSSSCache;
#endif

#include "iPointList.hpp"
//typedef iPointListContainer<LIB_DTYPE>								iPointlistContainer;
//typedef iPointList<LIB_DTYPE, DATA_NSLOTS_AO+DATA_RESULT_SLOT>		iPointlistAO_Base;
//typedef iPointList<LIB_DTYPE, DATA_NSLOTS_AO_TRIVIS+DATA_RESULT_SLOT>	iPointlistAO_Tri;
//typedef iPointList<LIB_DTYPE, DATA_NSLOTS_SSS+DATA_RESULT_SLOT>		iPointlistSSS;

#include "iOctree.hpp"
//typedef iOctree<LIB_DTYPE/*, iPointlistAO_Base*/>						iOctreeContainer;
//typedef iOctree<LIB_DTYPE/*, iPointlistAO_Base*/>::Node				iOctreeNode;
//typedef iOctree<float,iOctreeNodeStorage_AO>::Node sborat;

#include "pointcontainer.hpp"


// utilities ///////////////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
static double __drand48__() 
{
	// We don't have drand48.  Use rand() to get the bits.  We call
	// rand() three times since RAND_MAX is at least 16 bits.
	double f = 1.0 / (RAND_MAX + 1.0);
	double x = std::rand();
	x = x * f + std::rand();
	x = x * f + std::rand();
	return x * f;
}

template<typename iPoint, typename T>
iPoint newRandomPoint(T Min, T Max)
{
	double d;
	double e;
	double f;
	iPoint a;
	double max, min;
	max = (double) Max;
	min = (double) Min;
	for(unsigned int i=0;i < iPoint::__DIM;++i)
	{
		d = __drand48__();
		if(min > 0)
		{
			e = (d*max)-(d*min);
			f = min;
		}
		else
		{
			e = (d*max)+min;
			f = -(d*min);
		}

		a[i] = (T) (e+f);
	}
	return a;
}


/////////////////////////////////////////
#ifndef _MSC_VER

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

// Win32 stuff emulation
typedef unsigned long DWORD;
typedef long LONG;
typedef long long LONGLONG;

typedef union _LARGE_INTEGER {
    struct {
        DWORD LowPart;
        LONG HighPart;
    } DUMMYSTRUCTNAME;
    struct {
        DWORD LowPart;
        LONG HighPart;
    } u;
    LONGLONG QuadPart;
} LARGE_INTEGER;

// Helpful conversion constants
static const unsigned usec_per_sec = 1000000;

// These functions are written to match the win32
// signatures and behavior as closely as possible.
bool QueryPerformanceFrequency(LARGE_INTEGER * frequency)
{
    //sanity check
    assert(frequency != NULL);

    //gettimeofday reports to microsecond accuracy
    frequency->QuadPart = usec_per_sec;

    return true;
}

bool QueryPerformanceCounter(LARGE_INTEGER * performance_count)
{
    struct timeval time;

    //sanity check
    assert(performance_count != NULL);

    //grab the current time
    gettimeofday(&time, NULL);
    performance_count->QuadPart = time.tv_usec + 		//microseconds
                         time.tv_sec * usec_per_sec; 	//seconds

    return true;
}
#endif

/*
template <typename T> 
union hack                
{
  T data;
  char bytes[sizeof(T)] ;
};
*/

/////////////////////////////////////////
static inline void PressEnterToContinue()
{
	int c;
	printf( "\nPress ENTER to continue... \n" );
	fflush( stdout );
	do c = getchar(); while ((c != '\n') && (c != EOF));
}

#endif	// #ifndef __PCLOUD_DATASTRUCT__
