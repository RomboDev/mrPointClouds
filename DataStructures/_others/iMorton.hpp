/*****************************************************************************/
/*                                                                           */
/*  Header: iMorton.hpp                                                    */
/*                                                                           */
/*  Accompanies STANN Version 0.6 Beta                                       */
/*  Aug 27, 2009                                                             */
/*                                                                           */
/*  Copyright 2007, 2008                                                     */
/*  Michael Connor and Piyush Kumar                                          */
/*  Florida State University                                                 */
/*  Tallahassee FL, 32306-4532                                               */
/*                                                                           */
/*****************************************************************************/
#ifndef __DATASTRUCTURES_IMORTON__
#define __DATASTRUCTURES_IMORTON__


/*****************************************************************************/
/*                                                                           */
/*  Header: sep_float.hpp                                                    */
/*                                                                           */
/*  Accompanies STANN Version 0.74                                           */
/*  Nov 15, 2010                                                             */
/*                                                                           */
/*  Copyright 2007, 2008                                                     */
/*  Michael Connor and Piyush Kumar                                          */
/*  Florida State University                                                 */
/*  Tallahassee FL, 32306-4532                                               */
/*                                                                           */
/*****************************************************************************/

//#include <cmath>
#include <iostream>
#include <limits>


namespace cpc
{

//This class stores a float, double or long double as a 
//significand exponent pair.  It allows basic arithmetic operations as 
//well as computing the most significant differing bit (used for z-order 
//calculations)
template<typename T>
class sep_float;

//Seperated float significand
//This union stores a seperated float significand.
//It allows access to the significand as a floating 
//point type or as an unsigned long integer

template<typename T>
union sep_float_sig
{
	/*! Union accessor of floating point type */
	T d;
	/*! Union accessor as an array of integer types */
	unsigned long int i[1];
};

//XOR function
//This templated function computes a bitwise XOR of two
//sep_float_sig unions, based on the size of the underlying 
//floating point data type.

template<typename T1>
__CUDA_HOST_AND_DEVICE__ 
void xor_sep_float_sig(T1 &a, T1 &b, T1 &c, const T1 &zero)
{
	//for(unsigned int i=0; i < sizeof(float)/sizeof(unsigned long int); ++i)
	//c.i[i]=(a.i[i]^b.i[i])|zero.i[i];

	c.i[0]=(a.i[0]^b.i[0])|zero.i[0];
}

// sep_float type
//brief This class stores a float, long or double
// as an integer significand and an integer exponent
int msdb(sep_float<float> x, sep_float<float> y);

template<typename FLT>
class sep_float
{
	/*! sep_float overload of output stream function
	\brief This will put the floating point number on the
	output stream, not the seperated number
	\param os Output stream to be appended to
	\param x seperated float to be added to output stream
	\return output stream with floating point number appended
	*/
	__CUDA_HOST_AND_DEVICE__ 
	friend ostream& operator<<(ostream& os, const sep_float<FLT> &x)
	{
		os << ldexp(x.sig.d, x.exp);
		return os;
	}
	/*! msdb function
	\brief This function computes the most significant differing 
	bit of two seperated floating point numbers.  The return value is
	the exponent of the highest order bit that differs between the two numbers.
	Note: The msdb of the two floating point numbers is NOT computed based
	on the internal representation of the floating point number.  The answer
	is computed based on a theoretical integer representation of the numbers.
	\param x First value to be compared.
	\param y Second value to be compared.
	\return Most significant differing bit of x and y
	*/
	__CUDA_HOST_AND_DEVICE__ 
	friend int msdb(sep_float<FLT> x, sep_float<FLT> y)
	{
		const sep_float_sig<FLT> lzero = {0.5};

		if(x.val == y.val)		return -2147483647;
		else if(x.exp == y.exp)
		{
			xor_sep_float_sig(x.sig, y.sig, x.sig, lzero);
			frexp(x.sig.d-0.5, &y.exp);
			return x.exp+y.exp;
		}
		else if(x.exp > y.exp)	return x.exp;
		else					return y.exp;
	}

	template <typename T>
	__CUDA_HOST_AND_DEVICE__ 
	friend sep_float<FLT>& operator+=(sep_float<FLT> &y, T &x)
	{
		y = (y.val+x);
		return y;
	}

	template <typename T>
	__CUDA_HOST_AND_DEVICE__ 
	friend sep_float<FLT>& operator-=(sep_float<FLT> &y, T &x)
	{
		y = (y.val-x);
		return y;
	}
public:
	typedef FLT flt_type;

	__CUDA_HOST_AND_DEVICE__ 
	static inline int MSDB_X(sep_float<float> x, sep_float<float> y)
	{
		return msdb(x, y);
	}

	__CUDA_HOST_AND_DEVICE__ 
	sep_float(){;}

	__CUDA_HOST_AND_DEVICE__ 
	sep_float(FLT x)
	{
		sep_set_val(x);
	}

	__CUDA_HOST_AND_DEVICE__ 
	sep_float(const sep_float<FLT> &x)
	{
		sep_float_copy(x);
	}

	__CUDA_HOST_AND_DEVICE__ 
	~sep_float(){;}

	__CUDA_HOST_AND_DEVICE__ 
	sep_float<FLT>& operator=(const sep_float<FLT> &x)
	{
		sep_float_copy(x);
		return *this;
	}

	__CUDA_HOST_AND_DEVICE__ 
	sep_float<FLT>& operator=(const double &x)
	{
		sep_set_val((FLT) x);
		return *this;
	}

	template <typename T>
	__CUDA_HOST_AND_DEVICE__ 
	sep_float<FLT>& operator+=(const T& x)
	{
		sep_set_val(val+x);
		return *this;
	}

	template <typename T>
	__CUDA_HOST_AND_DEVICE__ 
	sep_float<FLT>& operator-=(const T& x)
	{
		sep_set_val(val-x);
		return *this;
	}

	template <typename T>
	__CUDA_HOST_AND_DEVICE__ 
	sep_float<FLT>& operator*=(const T& x)
	{
		sep_set_val(val*x);
		return *this;
	}

	template <typename T>
	__CUDA_HOST_AND_DEVICE__ 
	sep_float<FLT>& operator/=(const T& x)
	{
		sep_set_val(val/x);
		return *this;
	}

	__CUDA_HOST_AND_DEVICE__ 
	FLT get_flt()
	{
		return val;
	}

	__CUDA_HOST_AND_DEVICE__ 
	double get_sig()
	{
		return sig.d;
	}

	__CUDA_HOST_AND_DEVICE__ 
	int get_exp()
	{
		return exp;
	}

	__CUDA_HOST_AND_DEVICE__ 
	operator double()
	{
		return (double)val;
	}
private:
	__CUDA_HOST_AND_DEVICE__ 
	void sep_set_val(const FLT &x)
	{
		val = x;
		sig.d = (FLT) frexp(x, &exp);
	}

	__CUDA_HOST_AND_DEVICE__ 
	void sep_float_copy(const sep_float<FLT> &x)
	{
		sig.d = x.sig.d;
		exp   = x.exp;
		val   = x.val;
	}

	sep_float_sig<FLT> sig;
	int                exp;
	FLT                val;
};



/// ////////////////////////////////////////////////////////////////////////////////////////////////
template<typename Point>
class iMorton
{
public:
	iMorton(){;}
	~iMorton(){;}


	// Z-Ordering fnc, eg. SpaceFillingCurves, aka Morton codes
	bool lt_func(const Point &p, const Point &q)
	{
		if		((p[0]< 0) != (q[0] < 0))	return p[0] < q[0];
		else if((p[1] < 0) != (q[1] < 0))	return p[1] < q[1];
		else if((p[2] < 0) != (q[2] < 0))	return p[2] < q[2];


		int y; 
		int x = -2147483647;
		unsigned short j=0;
		
		/////////////////
		unsigned short k;
		//for(k=0;k < Point::__DIM;++k)
		for(k=0;k < 3;++k)
		{
			y = sep_float<float>::MSDB_X(p[k], q[k]);
			if(x < y)
			{
				j = k;
				x = y;
			}
		}

		return p[j] < q[j];
	}

	// Function Object Operator ...
	// determines whether one point
	// preceeds the other in a Z-ordering
	bool operator()(const Point &p, const Point &q)
	{
		return lt_func(p, q);
	}
};

}	//end cpc namespace

#endif
