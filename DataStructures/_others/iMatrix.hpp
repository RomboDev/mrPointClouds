#ifndef __PCLOUD_DATASTRUCT_IMATRIX__
#define __PCLOUD_DATASTRUCT_IMATRIX__

#define TALIGNPAD 16

namespace cpc
{


//==================================================================
inline float det2x2(float a, float b,
					float c, float d)
{
	return a * d - b * c;
}

//==================================================================
inline float det3x3(float a1,float a2,float a3,
					float b1,float b2,float b3,
					float c1,float c2,float c3)
{
	return
		a1 * det2x2( b2, b3, c2, c3 )
		- b1 * det2x2( a2, a3, c2, c3 )
		+ c1 * det2x2( a2, a3, b2, b3 );
}

//==================================================================
inline float det4x4(float a1, float a2, float a3, float a4, 
					float b1, float b2, float b3, float b4, 
					float c1, float c2, float c3, float c4, 
					float d1, float d2, float d3, float d4)
{
	return 
		a1 * det3x3( b2, b3, b4, c2, c3, c4, d2, d3, d4)
		- b1 * det3x3( a2, a3, a4, c2, c3, c4, d2, d3, d4)
		+ c1 * det3x3( a2, a3, a4, b2, b3, b4, d2, d3, d4)
		- d1 * det3x3( a2, a3, a4, b2, b3, b4, c2, c3, c4);
}


#if defined(_MSC_VER)
__declspec(align(TALIGNPAD))
#endif
class Matrix44
{
public:
	float	v16[16];

	Matrix44()	{}
	Matrix44( bool setToIdentity )	{ if( setToIdentity ) Identity(); }
	Matrix44( const float *pSrcMtx ) { CopyRowMajor( pSrcMtx ); }

	Matrix44(
		float m00_, float m01_, float m02_, float m03_,
		float m10_, float m11_, float m12_, float m13_,
		float m20_, float m21_, float m22_, float m23_,
		float m30_, float m31_, float m32_, float m33_ )
	{
		mij(0,0) = m00_; mij(0,1) = m01_; mij(0,2) = m02_; mij(0,3) = m03_;
		mij(1,0) = m10_; mij(1,1) = m11_; mij(1,2) = m12_; mij(1,3) = m13_;
		mij(2,0) = m20_; mij(2,1) = m21_; mij(2,2) = m22_; mij(2,3) = m23_;
		mij(3,0) = m30_; mij(3,1) = m31_; mij(3,2) = m32_; mij(3,3) = m33_;
	}

	const float &mij( size_t y, size_t x ) const	{ return v16[ y * 4 + x ];	}
		  float &mij( size_t y, size_t x )			{ return v16[ y * 4 + x ];	}

	iVector3 GetV3( size_t idx ) const
	{
		return iVector3( mij(idx,0), mij(idx,1), mij(idx,2) );
	}

	void SetV3( size_t idx, const iVector3 &v )
	{
		mij(idx,0) = v[0];
		mij(idx,1) = v[1];
		mij(idx,2) = v[2];
	}

	void Identity()
	{
		memset( v16, 0, sizeof(v16) );
		mij(0,0) = mij(1,1) = mij(2,2) = mij(3,3) = 1.0f;
	}

	inline Matrix44 GetTranspose() const;
	inline Matrix44 GetAs33() const;
	inline Matrix44 GetOrthonormal() const;

	inline static Matrix44 Scale( float sx, float sy, float sz );
	inline static Matrix44 Translate( const iVector3 &tra );
	inline static Matrix44 Translate( float tx, float ty, float tz );
	inline static Matrix44 Rot( float ang, float ax, float ay, float az );
	inline static Matrix44 OrthoRH(
						float left,
						float right,
						float bottom,
						float top,
						float nearr,
						float farr );
	inline static Matrix44 Perspective( float fov, float aspect, float n, float f );
	inline static Matrix44 PerspectiveRH0( float fov, float aspect, float n, float f );

	void CopyRowMajor( const float *pSrcMtx )
	{
		memcpy( v16, pSrcMtx, sizeof(v16) );
	}

	iVector3 GetTranslation() const
	{
		return iVector3(
			mij(3,0),
			mij(3,1),
			mij(3,2)
		);
	}

	Matrix44 GetInverse( bool *out_pSuccess=0 ) const
	{
		Matrix44 out( *this );

		if ( out_pSuccess )
			*out_pSuccess = false;

		float a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4;
		a1 = out.mij(0,0); b1 = out.mij(0,1); c1 = out.mij(0,2); d1 = out.mij(0,3);
		a2 = out.mij(1,0); b2 = out.mij(1,1); c2 = out.mij(1,2); d2 = out.mij(1,3);
		a3 = out.mij(2,0); b3 = out.mij(2,1); c3 = out.mij(2,2); d3 = out.mij(2,3);
		a4 = out.mij(3,0); b4 = out.mij(3,1); c4 = out.mij(3,2); d4 = out.mij(3,3);

		float det = det4x4(a1,a2,a3,a4,b1,b2,b3,b4,c1,c2,c3,c4,d1,d2,d3,d4);

		if ( fabs(det) < 1e-11f )
		{
			//DASSERT( 0 );
			return out;
		}

		out.mij(0,0) =   det3x3( b2, b3, b4, c2, c3, c4, d2, d3, d4);
		out.mij(1,0) = - det3x3( a2, a3, a4, c2, c3, c4, d2, d3, d4);
		out.mij(2,0) =   det3x3( a2, a3, a4, b2, b3, b4, d2, d3, d4);
		out.mij(3,0) = - det3x3( a2, a3, a4, b2, b3, b4, c2, c3, c4);

		out.mij(0,1) = - det3x3( b1, b3, b4, c1, c3, c4, d1, d3, d4);
		out.mij(1,1) =   det3x3( a1, a3, a4, c1, c3, c4, d1, d3, d4);
		out.mij(2,1) = - det3x3( a1, a3, a4, b1, b3, b4, d1, d3, d4);
		out.mij(3,1) =   det3x3( a1, a3, a4, b1, b3, b4, c1, c3, c4);

		out.mij(0,2) =   det3x3( b1, b2, b4, c1, c2, c4, d1, d2, d4);
		out.mij(1,2) = - det3x3( a1, a2, a4, c1, c2, c4, d1, d2, d4);
		out.mij(2,2) =   det3x3( a1, a2, a4, b1, b2, b4, d1, d2, d4);
		out.mij(3,2) = - det3x3( a1, a2, a4, b1, b2, b4, c1, c2, c4);

		out.mij(0,3) = - det3x3( b1, b2, b3, c1, c2, c3, d1, d2, d3);
		out.mij(1,3) =   det3x3( a1, a2, a3, c1, c2, c3, d1, d2, d3);
		out.mij(2,3) = - det3x3( a1, a2, a3, b1, b2, b3, d1, d2, d3);
		out.mij(3,3) =   det3x3( a1, a2, a3, b1, b2, b3, c1, c2, c3);

		out = out * (1/det);

		if ( out_pSuccess )
			*out_pSuccess = true;

		return out;
	}

	void PrintOut() const
	{
		printf( "[" );
		for (size_t r=0; r < 4; ++r)
		{
			if ( r != 0 )
				printf( " |" );

			for (size_t c=0; c < 4; ++c)
			{
				printf( " %f", mij(r,c) );
			}
		}
		printf( "]\n" );
	}

	Matrix44 operator *  (const float rval) const	 { Matrix44 ret; for (size_t i=0; i < 16; ++i) ret.v16[i] = v16[i] * rval; return ret; }
}
#if defined(__GNUC__)
__attribute__ ((aligned(TALIGNPAD)))
#endif
;


//==================================================================
inline Matrix44 Matrix44::GetTranspose() const
{
	Matrix44	out;

	for (int i=0; i < 4; ++i)
		for (int j=0; j < 4; ++j)
			out.mij(i,j) = mij(j,i);

	return out;
}

//==================================================================
inline Matrix44 Matrix44::GetAs33() const
{
	Matrix44	out( true );

	for (int i=0; i < 3; ++i)
		for (int j=0; j < 3; ++j)
			out.mij(i,j) = mij(i,j);

	return out;
}

//==================================================================
inline Matrix44 Matrix44::GetOrthonormal() const
{
	// TODO: verify that this actually works !!
	Matrix44	out( true );

	iVector3	v0 = GetV3(0);
	iVector3	v1 = GetV3(1);
	iVector3	v2 = GetV3(2);

	v0 = v0.getNormalized();
	v1 = v2.cross( v0 );
	v1 = v1.getNormalized();
	v2 = v0.cross( v1 );

	out.SetV3( 0, v0 );
	out.SetV3( 1, v1 );
	out.SetV3( 2, v2 );
	out.SetV3( 3, GetV3(3) );

	return out;
}

//==================================================================
inline Matrix44 Matrix44::Scale( float sx, float sy, float sz )
{
	Matrix44	m;
	memset( m.v16, 0, sizeof(m.v16) );

	m.mij(0,0) = sx;
	m.mij(1,1) = sy;
	m.mij(2,2) = sz;
	m.mij(3,3) = 1.0f;

	return m;
}
//==================================================================
inline Matrix44 Matrix44::Translate( const iVector3 &tra )
{
	return Translate( tra.GetX(), tra.GetY(), tra.GetZ() );
}
//==================================================================
inline Matrix44 Matrix44::Translate( float tx, float ty, float tz )
{
	Matrix44	m( true );
	m.mij(3,0) = tx;
	m.mij(3,1) = ty;
	m.mij(3,2) = tz;
	m.mij(3,3) = 1.0f;

	return m;
}
//==================================================================
inline Matrix44 Matrix44::Rot( float ang, float ax, float ay, float az )
{
	float   xx, yy, zz, xy, yz, zx, xs, ys, zs;

	float s = sinf( ang );
	float c = cosf( ang );

	xx = ax * ax;   yy = ay * ay;   zz = az * az;
	xy = ax * ay;   yz = ay * az;   zx = az * ax;
	xs = ax * s;    ys = ay * s;    zs = az * s;
	float one_c = 1 - c;

	return Matrix44(
			(one_c * xx) + c,	(one_c * xy) + zs,	(one_c * zx) - ys,	0,
			(one_c * xy) - zs,	(one_c * yy) + c,	(one_c * yz) + xs,	0,
			(one_c * zx) + ys,	(one_c * yz) - xs,	(one_c * zz) + c,	0,
			0,					0,					0,					1 );
}

//==================================================================
inline Matrix44 Matrix44::OrthoRH(
						float left,
						float right,
						float bottom,
						float top,
						float nearr,
						float farr )
{
	//DASSERT( right != left && top != bottom && farr != nearr );

	float	rl = right - left;
	float	tb = top - bottom;
	float	fn = farr - nearr;

	float tx = -(right + left) / rl;
	float ty = -(top + bottom) / tb;
	float tz = -(farr + nearr) / fn;

	return Matrix44(
			2 / rl,	0,		0,			0,
			0,		2 / tb,	0,			0,
			0,		0,		-2 / fn,	0,
			tx,		ty,		tz,			1 );
}

//==================================================================
inline Matrix44 Matrix44::Perspective( float fov, float aspect, float n, float f )
{
	float   ootan2 = tanf( fov * 0.5f );
	//DASSERT( ootan2 != 0 );
	ootan2 = 1.0f / ootan2;

	//DASSERT( f != n );

	return Matrix44(
			ootan2/aspect,	0,		0,			0,
			0,				ootan2,	0,			0,
			0,				0,		f/(f-n),	1,
			0,				0,		n*f/(n-f),	0 );
}

//==================================================================
inline Matrix44 Matrix44::PerspectiveRH0( float fov, float aspect, float n, float f )
{
	float   ootan2 = tanf( fov * 0.5f );
	//DASSERT( ootan2 != 0 );
	ootan2 = 1.0f / ootan2;

	//DASSERT( f != n );

	return Matrix44(
			ootan2/aspect,	0,		0,			0,
			0,				ootan2,	0,			0,
			0,				0,		f/(n-f),	-1,
			0,				0,		n*f/(n-f),	0 );
}

//==================================================================
inline Matrix44 operator * (const Matrix44 &m1, const Matrix44 &m2)
{
	Matrix44        tmp;

	for (size_t r=0; r < 4; ++r)
	{
		for (size_t c=0; c < 4; ++c)
		{
			float   sum = 0;
			for (size_t i=0; i < 4; ++i)

			sum += m1.mij(r,i) * m2.mij(i,c);			
			tmp.mij(r,c) = sum;
		}
		
	}

	return tmp;
}	
}
//end namespace cpc
#endif	//ifdef iMatrix