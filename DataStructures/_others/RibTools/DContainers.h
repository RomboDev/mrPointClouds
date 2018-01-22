//==================================================================
/// DContainers.h
///
/// Created by Davide Pasca - 2008/12/17
/// See the file "license.txt" that comes with this project for
/// copyright info.
//==================================================================

#ifndef DCONTAINERS_H
#define DCONTAINERS_H

#include <memory.h>
#include "DTypes.h"
#include "DMemory.h"
#include "DUtils_Base.h"
#include "DExceptions.h"

#if defined(__APPLE__)
# include <tr1/unordered_map>
#else
# include <unordered_map>
#endif

#define DUNORD_MAP	std::tr1::unordered_map

//==================================================================
template <class T>
class DVec
{
	T		*mpData;
	size_t	mSize;
	size_t	mSizeAlloc;

public:
	typedef const T	*const_iterator;
	typedef T		*iterator;

public:
	DVec() :
		mpData(NULL),
		mSize(0),
		mSizeAlloc(0)
	{
	}

	DVec( const DVec &from ) :
		mpData(NULL),
		mSize(0),
		mSizeAlloc(0)
	{
		copyFrom( from );
	}

	virtual ~DVec()
	{
		clear();
		freeAll();
	}

	DVec &operator=(const DVec& rhs)
	{
		clear();
		freeAll();
		copyFrom( rhs );

		return *this;
	}

private:
	void copyFrom( const DVec& from )
	{
		resize( from.mSize );

		for (size_t i=0; i < mSize; ++i)
			mpData[i] = from.mpData[i];
	}

	void freeAll()
	{
		mSizeAlloc = 0;
		if ( mpData )
			DDELETE_ARRAY( (u_char *)mpData );
		mpData = NULL;
	}

public:
	size_t size() const { return mSize; }
	size_t size_bytes() const { return mSize * sizeof(T); }

	iterator		begin()			{ return mpData;	}
	const_iterator	begin()	const	{ return mpData;	}

	iterator		end()			{ return mpData + mSize;	}
	const_iterator	end()	const	{ return mpData + mSize;	}

	void clear()
	{
		resize( 0 );
	}

	void clear_free()
	{
		resize( 0 );
		freeAll();
	}

	void force_reserve( size_t reserveSize )
	{
		size_t newSizeAlloc = mSize + reserveSize;

		if ( newSizeAlloc <= mSizeAlloc )
			return;

		T *newPData = (T *)DNEW u_char [ sizeof(T) * newSizeAlloc ];

		if ( mpData )
		{
			memcpy( newPData, mpData, mSize * sizeof(T) );

			DDELETE_ARRAY( (u_char *)mpData );
		}

		mpData = newPData;
		mSizeAlloc = newSizeAlloc;
	}

	void reserve( size_t newSizeAlloc )
	{
		force_reserve( newSizeAlloc );
	}

	void resize( size_t newSize )
	{
		if ( newSize == mSize )
			return;

		if ( newSize > mSizeAlloc )
		{
			size_t newSizeAlloc = mSizeAlloc * 2;

			if ( newSizeAlloc == 0 )
			{
				newSizeAlloc = (sizeof(T) < 128 ? 128 : sizeof(T)) / sizeof(T);
			}

			if ( newSizeAlloc < newSize )
				newSizeAlloc = newSize;

			T *newPData = (T *)DNEW u_char [ sizeof(T) * newSizeAlloc ];

			if ( mpData )
			{
				//memcpy( newPData, mpData, mSize * sizeof(T) );

				for (size_t i=0; i < mSize; ++i)
				{
					new (&newPData[i]) T;
					newPData[i] = mpData[i];
					mpData[i].~T();
				}

				DDELETE_ARRAY( (u_char *)mpData );
			}

			mpData = newPData;
			mSizeAlloc = newSizeAlloc;
		}

		if ( newSize < mSize )
		{
			for (size_t i=newSize; i < mSize; ++i)
				mpData[i].~T();
		}
		else
		{
			for (size_t i=mSize; i < newSize; ++i)
				new (&mpData[i]) T;
		}

		mSize = newSize;
	}

	T *grow( size_t n=1 )
	{
		size_t	fromIdx = size();
		resize( fromIdx + n );
		return mpData + fromIdx;
	}

	void erase( iterator it )
	{
		if NOT( it >= mpData && it < (mpData+mSize) )
			DEX_OUT_OF_RANGE( "Out of bounds !" );

		size_t	idx = it - mpData;
		mpData[idx].~T();
		for (size_t i=idx; i < mSize-1; ++i)
		{
			mpData[i] = mpData[i+1];
		}
		mSize -= 1;
	}

	void insert( iterator itBefore, T &val )
	{
		if NOT( itBefore >= mpData && itBefore < (mpData+mSize) )
			DEX_OUT_OF_RANGE( "Out of bounds !" );

		size_t	idx = itBefore - mpData;

		resize( mSize + 1 );

		for (size_t i=mSize; i > (idx+1); --i)
			mpData[i-1] = mpData[i-2];

		mpData[idx] = val;
	}

	void push_front( const T &val )
	{
		grow();
		for (size_t i=mSize; i > 1; --i)
		{
			mpData[i-1] = mpData[i-2];
		}

		mpData[0] = val;
	}

	void push_back( const T &val )
	{
		// necessary for things such as push_back( back() );
		T	tmp = val;
		*grow() = tmp;
	}

	void pop_back()
	{
		DASSERT( mSize >= 1 );
		resize( mSize - 1 );
	}

	iterator find( const T &val )
	{
		for (size_t i=0; i < mSize; ++i)
		{
			if ( mpData[i] == val )
			{
				return mpData + i;
			}
		}

		return end();
	}

	size_t find_idx( const T &val, size_t from=0 )
	{
		for (size_t i=from; i < mSize; ++i)
		{
			if ( mpData[i] == val )
			{
				return i;
			}
		}

		return DNPOS;
	}

	void find_or_push_back( const T &val )
	{
		if ( find( val ) != end() )
			return;

		// necessary for things such as push_back( back() );
		T	tmp = val;
		*grow() = tmp;
	}

	void append_array( const T *pSrc, size_t cnt )
	{
		T	*pDes = grow( cnt );
		for (size_t i=0; i < cnt; ++i)
		{
			pDes[i] = pSrc[i];
		}
	}

	void append( const DVec<T> &srcVec )
	{
		append_array( &srcVec[0], srcVec.size() );
	}

	void get_ownership( DVec<T> &from )
	{
		freeAll();

		mpData		= from.mpData;
		mSize		= from.mSize;
		mSizeAlloc	= from.mSizeAlloc;

		from.mpData		= NULL;
		from.mSize		= 0;
		from.mSizeAlloc	= 0;
	}

	const	T &back() const { DASSERT( mSize >= 1 ); return mpData[mSize-1]; }
			T &back()		{ DASSERT( mSize >= 1 ); return mpData[mSize-1]; }

	const T &operator[]( size_t idx ) const { DASSERT( idx < mSize ); return mpData[ idx ]; }

#if defined(__GNUC__)	// WTF ?!
		  T &operator[]( size_t idx )		{ return mpData[ idx ]; }
#else
		  T &operator[]( size_t idx )		{ DASSERT( idx < mSize ); return mpData[ idx ]; }
#endif

};

//==================================================================

template <size_t _NUM>
class DVecBits
{
	U8	mData[_NUM];

public:
	DVecBits()
	{
		clear();
	}

	void clear()					{ memset( &mData[0], 0, sizeof(mData) ); }
	void Set( size_t i )			{ DASSERT( i < _NUM ); mData[ i >> 3 ] |= 1 << (i & 7); }
	void Reset( size_t i )			{ DASSERT( i < _NUM ); mData[ i >> 3 ] &= ~(1 << (i & 7)); }
	bool IsSet( size_t i ) const	{ DASSERT( i < _NUM ); return !!(mData[ i >> 3 ] & (1 << (i & 7))); }
};

//==================================================================
template <class T>
class Stack
{
	DVec<T>	mVec;

public:
	void push( const T &val )
	{
		mVec.push_back( val );
	}

	void pop()
	{
		mVec.pop_back();
	}

	const T &top() const
	{
		return mVec.back();
	}

	T &top()
	{
		return mVec.back();
	}

	void clear()
	{
		mVec.clear();
	}
};

//==================================================================
template <class T>
class CopyStack
{
	DVec<T>	mVec;

public:
	CopyStack()
	{
		mVec.resize(1);
	}
	void push()
	{
		mVec.push_back( top() );
	}

	void pop()
	{
		mVec.pop_back();
	}

	const T &top() const
	{
		return mVec.back();
	}

	T &top()
	{
		return mVec.back();
	}

	void clear()
	{
		mVec.clear();
	}
};

//==================================================================
template <class T, size_t MAX>
class CopyStackMax
{
	U8		mVec[ sizeof(T) * MAX ];
	size_t	mSize;

public:
	CopyStackMax()
	{
		T	*p = (T *)&mVec[0];
		new ( p ) T;
		mSize = 1;
	}

	void push()
	{
		DASSTHROW( mSize < MAX, ("Out of bounds !") );

		*(T *)&mVec[mSize * sizeof(T)] = top();

		mSize += 1;
	}

	void pop()
	{
		DASSTHROW( mSize >= 0, ("Out of bounds !") );
		mSize -= 1;
	}

	const T &top() const
	{
		return *(const T *)&mVec[(mSize-1) * sizeof(T)];
	}

	T &top()
	{
		return *(T *)&mVec[(mSize-1) * sizeof(T)];
	}

	void clear()
	{
		for (size_t i=0; i < mSize; ++i)
			mVec[i].~T();

		mSize = 0;
	}
};

#endif
