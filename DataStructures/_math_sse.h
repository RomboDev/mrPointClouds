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

#ifndef __PCLOUD_MATH_SSE__
#define __PCLOUD_MATH_SSE__
//#include <iostream>


///////////////////////////////////////////////////////////////////////////
static const int bit3Dmask				= 0x7f;
static const __m128i M128i_0XFFFFFFFF	= _mm_set1_epi32(0xffffffffu);
static const __m128i M128i_bit3Dmask	= _mm_set1_epi32(0x7f);
static const __m128i M128i_CI4_SIGN		= _mm_set1_epi32(0x80000000u);
static const __m128i M128i_CI4_RSIGN	= _mm_set1_epi32(0x7FFFFFFFu);

static const __m128 M128_MINUSONE		= _mm_set1_ps(-1.f);
static const __m128 M128_FLOAT_LOW		= _mm_set1_ps(1e-16f);
static const __m128 M128_FLOAT_MIN		= _mm_set1_ps(1e30f);
static const __m128 M128_FLOAT_MAX		= _mm_set1_ps(-1e30f);
static const __m128 M128_ALMOST_ZERO	= _mm_set1_ps(0.000001f);
static const __m128 M128_ALMOST_ONE		= _mm_set1_ps(0.999999f);

static const __m128 M128_ZERO			= _mm_set1_ps(0.0f);
static const __m128 M128_HALF			= _mm_set1_ps(0.5f);
static const __m128 M128_ONE			= _mm_set1_ps(1.0f);
static const __m128 M128_TWO			= _mm_set1_ps(2.0f);
static const __m128 M128_THREE			= _mm_set1_ps(3.0f);
static const __m128 M128_1_2_4			= _mm_setr_ps(1.f, 2.f, 4.f, 0.f);
static const __m128 M128_0001			= _mm_setr_ps(0.f, 0.f, 0.f, 1.f);

static const __m128 M128_PI				= _mm_set1_ps(f_pi);
static const __m128 M128_INVPI			= _mm_set1_ps(inv_pi);

static const __m128 M128_CF4_PIF		= _mm_set1_ps(3.14159265358979323846f);
static const __m128 M128_CF4_PIO2F		= _mm_set1_ps(1.570796326794896619f);
static const __m128 M128_CF4_PIO4F		= _mm_set1_ps(0.7853981633974483096f);

static const float DP1	= 0.78515625;
static const float DP2	= 2.4187564849853515625e-4;
static const float DP3	= 3.77489497744594108e-8;
static const float FOPI = 1.27323954473516;

static const __m128 M128_CF4_00004		= _mm_set1_ps(1.0e-4f);
static const __m128 M128_CF4_00006		= _mm_set1_ps(1.0e-6f);
static const __m128 M128_CF4_SMALL		= _mm_set1_ps(1.0E-35f);

static const __m128 M128_cphLOG2EF		= _mm_set1_ps(1.44269504088896341f);
static const __m128 M128_cphC1			= _mm_set1_ps(0.693359375f);
static const __m128 M128_cphC2			= _mm_set1_ps(-2.12194440e-4f);
static const __m128 M128_MAXLOGF		= _mm_set1_ps(88.72283905206835f);
static const __m128 M128_MINLOGF		= _mm_set1_ps(-88.f);


static const __m128 M128_cphP0			= _mm_set1_ps(1.9875691500E-4f);
static const __m128 M128_cphP1			= _mm_set1_ps(1.3981999507E-3f);
static const __m128 M128_cphP2			= _mm_set1_ps(8.3334519073E-3f);
static const __m128 M128_cphP3			= _mm_set1_ps(4.1665795894E-2f);
static const __m128 M128_cphP4			= _mm_set1_ps(1.6666665459E-1f);
static const __m128 M128_cphP5			= _mm_set1_ps(5.0000001201E-1f);

static const __m128 M128_CF4_42163199048E_2		= _mm_set1_ps(4.2163199048E-2f);
static const __m128 M128_CF4_24181311049E_2		= _mm_set1_ps(2.4181311049E-2f);
static const __m128 M128_CF4_45470025998E_2		= _mm_set1_ps(4.5470025998E-2f);
static const __m128 M128_CF4_74953002686E_2		= _mm_set1_ps(7.4953002686E-2f);
static const __m128 M128_CF4_16666752422E_1		= _mm_set1_ps(1.6666752422E-1f);

static const __m128 M128_CF4_2414213562373095	= _mm_set1_ps(2.414213562373095f);
static const __m128 M128_CF4_04142135623730950	= _mm_set1_ps(0.4142135623730950f);
static const __m128 M128_CF4_805374449538e_2	= _mm_set1_ps(8.05374449538e-2f);
static const __m128 M128_CF4_138776856032E_1	= _mm_set1_ps(1.38776856032E-1f);
static const __m128 M128_CF4_199777106478E_1	= _mm_set1_ps(1.99777106478E-1f);
static const __m128 M128_CF4_333329491539E_1	= _mm_set1_ps(3.33329491539E-1f);


////////////////////////////////////////////////////////////////////////////////////////////////////
static inline __m128
_mmx_fabsf_ps( __m128 x )
{
      return _mm_and_ps( x, _mm_castsi128_ps(M128i_CI4_RSIGN));
}

static inline 
__m128i _mmx_signf4( __m128 x )
{
	__m128i a = M128i_CI4_SIGN;
	a = _mm_castps_si128( _mm_and_ps( _mm_castsi128_ps(a),x ));
	return a;
}

static inline
__m128 _mmx_vsel_ps( __m128 a, __m128 b, __m128i sel )
{
	__m128 self = _mm_castsi128_ps(sel);
	b = _mm_and_ps(b,self);
	b = _mm_andnot_ps(self,a);
	b = _mm_or_ps(b,self);

	return b;
}

inline float reduce2quadrant(float x, int & quad) 
{
	// make argument positive
	x = fabs(x);
	x =  (x > 16777215.f) ?   16777215.f : x;
    
	quad = (int)(FOPI * x); // integer part of x/PIO4
	quad = (quad+1) & (~1);
	float y = (float)quad;

	// extended precision modular arithmetic
	return ((x - y * DP1) - y * DP2) - y * DP3;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ArcTagent
static inline __m128
_mmx_atanf_ps( __m128 x )
{
	__m128 y, z,z1,z2;
	__m128i a1, a2, a3;

	// make argument positive and save the sign
	__m128i sign = _mmx_signf4( x );
	x = _mm_xor_ps(x, _mm_castsi128_ps(sign));
      
	// range reduction
	a1 = _mm_set1_epi32(_mm_movemask_ps( _mm_cmpgt_ps(x, M128_CF4_2414213562373095) ));
	a2 = _mm_set1_epi32(_mm_movemask_ps( _mm_cmpgt_ps(x, M128_CF4_04142135623730950) ));
	a3 = _mm_xor_si128(a2, M128i_0XFFFFFFFF);	//a3 = ~a2; 
	a2 = _mm_or_si128(a2,a1);					//a2 ^= a1;


	z1 = _mm_div_ps(M128_ONE, _mm_add_ps(x, M128_CF4_SMALL));
	z2 = _mm_div_ps(_mm_sub_ps(x,M128_ONE),_mm_add_ps(x,M128_ONE));

	z1 = _mm_and_ps(z1,_mm_castsi128_ps(a1));

	z2 = _mm_and_ps(z2,_mm_castsi128_ps(a2));
	x = _mm_and_ps(x,_mm_castsi128_ps(a3));
	x = _mm_or_ps(x,z1);
 	x = _mm_or_ps(x,z2);
     
	y = M128_CF4_PIO2F;
	z1 = M128_CF4_PIO4F;

	y = _mm_and_ps(y,_mm_castsi128_ps(a1));
	z1 = _mm_and_ps(z1,_mm_castsi128_ps(a2));
	y = _mm_or_ps(y,z1);

	z = _mm_mul_ps(x,x);

	y =
		_mm_add_ps( y,
		_mm_add_ps(
		_mm_mul_ps(
		_mm_mul_ps(
		_mm_sub_ps(
		_mm_mul_ps(
		_mm_add_ps(
		_mm_mul_ps(
		_mm_sub_ps(
		_mm_mul_ps(	M128_CF4_805374449538e_2,z),
					M128_CF4_138776856032E_1),z),
					M128_CF4_199777106478E_1),z),
					M128_CF4_333329491539E_1), z), x), x));

	y = _mm_xor_ps(y,_mm_castsi128_ps(sign));
	return y;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// ArcTangent2 /* BUGGY */
static inline __m128
_mmx_atan2f_ps( __m128 y, __m128 x )
{
	__m128 z, w;

	__m128 x_neg_PI = M128_CF4_PIF;
	__m128 y_negativ_2 = M128_TWO;
	//VEC_AND(x_neg_PI,		VEC_GT( M128_ZERO, x ));
	//VEC_AND(y_negativ_2,	VEC_GT( M128_ZERO, y ));
	//x_neg_PI	= _mm_and_ps(x_neg_PI,		_mm_castsi128_ps(_mm_set1_epi32(_mm_movemask_ps( _mm_cmpgt_ps(M128_ZERO, x)))));
	//y_negativ_2 = _mm_and_ps(y_negativ_2,	_mm_castsi128_ps(_mm_set1_epi32(_mm_movemask_ps( _mm_cmpgt_ps(M128_ZERO, y)))));
	x_neg_PI	= _mm_and_ps(x_neg_PI,		_mm_cmpgt_ps(M128_ZERO, x));
	y_negativ_2 = _mm_and_ps(y_negativ_2,	_mm_cmpgt_ps(M128_ZERO, y));

	//__m128i i_x_zero  = VEC_EQ ( M128_ZERO, x );
	//__m128i i_y_zero  = VEC_EQ ( M128_ZERO, y );
	//__m128i i_x_zero = _mm_castps_si128(_mm_cmpeq_ps(M128_ZERO, x));
	//__m128i i_y_zero = _mm_castps_si128(_mm_cmpeq_ps(M128_ZERO, y));
	__m128i i_x_zero = _mm_set1_epi32(_mm_movemask_ps( _mm_cmpeq_ps(M128_ZERO, x)));
	__m128i i_y_zero = _mm_set1_epi32(_mm_movemask_ps( _mm_cmpeq_ps(M128_ZERO, y)));

	//VEC_AND(x_zero_PIO2,	i_x_zero);
	//VEC_AND(y_zero,		i_y_zero);
	__m128 x_zero_PIO2	= M128_CF4_PIO2F;
	__m128 y_zero		= M128_ONE;
	x_zero_PIO2			= _mm_and_ps(x_zero_PIO2,	_mm_castsi128_ps(i_x_zero));
	y_zero				= _mm_and_ps(y_zero,		_mm_castsi128_ps(i_y_zero));


	//w = x_neg_PI *  ( CF4_1  - y_negativ_2 );
	w = _mm_mul_ps(x_neg_PI, _mm_sub_ps(M128_ONE, y_negativ_2));

	//z = _atanf4( y / (x+x_zero_PIO2));
	z = _mmx_atanf_ps( _mm_div_ps(y, _mm_add_ps(x,x_zero_PIO2)));

	//VEC_AND(z, ~(i_x_zero|i_y_zero));
	z = _mm_and_ps(z, _mm_castsi128_ps(_mm_xor_si128(( _mm_or_si128(i_x_zero, i_y_zero)), M128i_0XFFFFFFFF)));

	//return w + z + (x_zero_PIO2 * (CF4_1 - y_zero - y_negativ_2)) + (y_zero *  x_neg_PI);
	return	_mm_add_ps (w, 
			_mm_add_ps (z,
			_mm_add_ps (_mm_mul_ps(x_zero_PIO2, _mm_sub_ps(_mm_sub_ps(M128_ONE,y_zero),y_negativ_2)), 
						_mm_mul_ps(y_zero, x_neg_PI)
						) 
						)
						);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// ArcSin
static inline __m128
_mmx_asinf_ps( __m128 x)
{
	__m128 a,b,z;
	__m128i sign;  

	sign = _mmx_signf4( x );
	x = _mm_xor_ps(x, _mm_castsi128_ps(sign));

	__m128i x_smaller_1e_4 = _mm_set1_epi32(_mm_movemask_ps( _mm_cmpgt_ps(M128_CF4_00004, x) ));
	__m128i x_larger_05    = _mm_set1_epi32(_mm_movemask_ps( _mm_cmpgt_ps(x, M128_HALF) ));
	__m128i x_else         = _mm_xor_si128(( _mm_or_si128(x_smaller_1e_4, x_larger_05)), M128i_0XFFFFFFFF);

	a = x;
	a = _mm_and_ps(a, _mm_castsi128_ps(x_smaller_1e_4));
	b = _mm_mul_ps(M128_HALF, _mm_sub_ps(M128_ONE,x));
	b = _mm_and_ps(b, _mm_castsi128_ps(x_larger_05));
	z = _mm_mul_ps(x,x);
	z = _mm_and_ps(z, _mm_castsi128_ps(x_else));

	z = _mm_or_ps(z,a);
	z = _mm_or_ps(z,b);
      
	x = _mm_and_ps(x, _mm_castsi128_ps(x_else));
	a = _mm_sqrt_ps(z);
	a = _mm_and_ps(a, _mm_castsi128_ps(x_larger_05));
	x = _mm_or_ps(x,a);

	__m128 z1 =
		_mm_add_ps(
		_mm_mul_ps(
		_mm_add_ps(
		_mm_mul_ps(
		_mm_add_ps(
		_mm_mul_ps(
		_mm_add_ps(
		_mm_mul_ps(	M128_CF4_42163199048E_2, z),
					M128_CF4_24181311049E_2),z),
					M128_CF4_45470025998E_2),z),
					M128_CF4_74953002686E_2),z),
					M128_CF4_16666752422E_1);

	z1 = _mm_add_ps(_mm_mul_ps(_mm_mul_ps(z1,z),x),x);

	z = _mmx_vsel_ps(z1,z,x_smaller_1e_4);
      
	z1 = _mm_add_ps(z,z);
	z1 = _mm_sub_ps(M128_CF4_PIO2F,z1);

	z = _mmx_vsel_ps(z,z1,x_larger_05);
	z = _mm_xor_ps(z,_mm_castsi128_ps(sign));

	return z;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
static inline __m128
_mmx_acosf_ps( __m128 x )
{
	__m128 a = x, x2, z;
	__m128i sign;  

	sign = _mmx_signf4( x );
	a = _mm_xor_ps(a, _mm_castsi128_ps(sign));
 	sign = _mm_set1_epi32(-(_mm_srli_epi32(sign,31).m128i_i32[0])); // 0xffff.. if negativ    

	__m128i a_larger_05 = _mm_set1_epi32(_mm_movemask_ps( _mm_cmpgt_ps(a, M128_HALF) ));
	x = _mmx_vsel_ps( x, _mm_sqrt_ps(_mm_sub_ps(M128_HALF,_mm_mul_ps(a,M128_HALF))), a_larger_05);

	x = _mmx_asinf_ps( x );
	x2 = _mm_add_ps(x,x);
      
	z = _mmx_vsel_ps(x2, _mm_sub_ps(M128_CF4_PIF,x2), sign);
	z = _mmx_vsel_ps(_mm_sub_ps(M128_CF4_PIO2F,x), z, a_larger_05);      
	return z;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
static inline __m128 
_mmx_exp_ps( __m128 x ) 
{
	__m128 tmp = _mm_setzero_ps(), fx;
	__m128i emm0;

	x = _mm_min_ps(x, M128_MAXLOGF);
	x = _mm_max_ps(x, M128_MINLOGF);

	// express exp(x) as exp(g + n*log(2))
	fx = _mm_mul_ps(x, M128_cphLOG2EF);
	fx = _mm_add_ps(fx, M128_HALF);

	// how to perform a floorf with SSE: just below
	emm0 = _mm_cvttps_epi32(fx);
	tmp  = _mm_cvtepi32_ps(emm0);

	// if greater, substract 1
	__m128 mask = _mm_cmpgt_ps(tmp, fx);    
	mask		= _mm_and_ps(mask, M128_ONE);
	fx			= _mm_sub_ps(tmp, mask);

	tmp			= _mm_mul_ps(fx, M128_cphC1);
	__m128 z	= _mm_mul_ps(fx, M128_cphC2);
	x			= _mm_sub_ps(x, tmp);
	x			= _mm_sub_ps(x, z);

	z			= _mm_mul_ps(x,x);

	__m128 y		= M128_cphP0;
	y = _mm_mul_ps(y, x);
	y = _mm_add_ps(y, M128_cphP1);
	y = _mm_mul_ps(y, x);
	y = _mm_add_ps(y, M128_cphP2);
	y = _mm_mul_ps(y, x);
	y = _mm_add_ps(y, M128_cphP3);
	y = _mm_mul_ps(y, x);
	y = _mm_add_ps(y, M128_cphP4);
	y = _mm_mul_ps(y, x);
	y = _mm_add_ps(y, M128_cphP5);
	y = _mm_mul_ps(y, z);
	y = _mm_add_ps(y, x);
	y = _mm_add_ps(y, M128_ONE);

	// build 2^n
	emm0 = _mm_cvttps_epi32(fx);
	emm0 = _mm_add_epi32(emm0, M128i_bit3Dmask);
	emm0 = _mm_slli_epi32(emm0, 23);
	__m128 pow2n = _mm_castsi128_ps(emm0);

	return __m128( _mm_mul_ps(y, pow2n) );
}


#endif	__PCLOUD_MATH_SSE__