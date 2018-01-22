//////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright 2010-2016 by RomboStudios
// All rights reserved.
// *****************************************************************************
// Created:	July 05, 2010
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the software's owners nor the names of its
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// (This is the New BSD license)
//////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CPC_MICROBUFFER_H_INCLUDED
#define CPC_MICROBUFFER_H_INCLUDED


#include <cfloat>
#include <cmath>
#include <cstring>

#include <ImathFun.h>
#include <boost/scoped_array.hpp>


namespace cpc
{
	

INLINE float Dotty(iVector3 a, iVector3 b)
{
	return a.dot(b);
}

/// An axis-aligned cube environment buffer.
///
/// Each face has a coordinate system where the centres of the boundary pixels
/// lie just _inside_ the boundary.  That is, the positions of the pixel point
/// samples are at x_i = (1/2 + i/N) for i = 0 to N-1.
///
/// For example, with 3x3 pixel faces, each
/// face looks like
///
///   +-----------+
///   | x   x   x |
///   |           |
///   | x   x   x |
///   |           |
///   | x   x   x |
///   +-----------+
///
/// where the x's represent the positions at which point sampling will occur.
///
/// The orientation of the faces is chosen so that they have a consistent
/// coordinate system when laid out in the following unfolded net of faces,
/// viewed from the inside of the cube.  (NB that for other possible nets the
/// coordinates of neighbouring faces won't be consistent.)
///
///              +---+
///   ^          |+y |
///   |  +---+---+---+---+
///  v|  |-z |-x |+z |+x |
///   |  +---+---+---+---+
///   |          |-y |
///   |          +---+
///   |     u
///   +-------->
///
class MicroBuf
{
public:
	/// Identifiers for each cube face direction
	enum Face
	{
		Face_xp, ///< x+
		Face_yp, ///< y+
		Face_zp, ///< z+
		Face_xn, ///< x-
		Face_yn, ///< y-
		Face_zn, ///< z-
		Face_end,
		Face_begin = Face_xp
	};

	/// faceRes gives face resolution.  Faces are square.  
	/// nchans gives the number of channels in each pixel, and 
	/// defaultPix gives the values of the channels which will be used when reset() is called.
	MicroBuf( int faceRes, int nchans, const float* defaultPix )
		:	m_res(faceRes)
		,	m_nchans(nchans)
		,	m_faceSize(nchans*faceRes*faceRes)
		,	m_pixels()
	{
		m_pixels.reset(new float[m_faceSize*Face_end]);
		m_defaultPixels.reset(new float[m_faceSize*Face_end]);
		m_directions.reset(new iVector3[Face_end*faceRes*faceRes]);
		m_pixelSizes.reset(new float[m_faceSize]);

		// Cache direction vectors
		for(int face = 0; face < Face_end; ++face)
		{
			for(int iv = 0; iv < m_res; ++iv){
			for(int iu = 0; iu < m_res; ++iu)
			{
				// directions of pixels go through pixel centers
				float u = (0.5f + iu)/faceRes*2.0f - 1.0f;
				float v = (0.5f + iv)/faceRes*2.0f - 1.0f;

				m_directions[(face*m_res + iv)*m_res + iu] = direction(face, u, v);
			}
			}
		}

		for(int iv = 0; iv < m_res; ++iv){
		for(int iu = 0; iu < m_res; ++iu)
		{
			float u = (0.5f + iu)/faceRes*2.0f - 1.0f;
			float v = (0.5f + iv)/faceRes*2.0f - 1.0f;

			m_pixelSizes[iv*m_res + iu] = 1.0f/iVector3(u,v,1).sqr_length();
		}
		}

		float* pix = m_defaultPixels.get();

		for(int i = 0, iend = size(); i < iend; ++i, pix += m_nchans)
		for(int c = 0; c < m_nchans; ++c)
		pix[c] = defaultPix[c];
	}

	/// Reset buffer to default (non-rendered) state.
	void reset()
	{
		memcpy(m_pixels.get(), m_defaultPixels.get(),
			sizeof(float)*size()*m_nchans);
	}

	/// Get raw data store for face
	float* face(int which)
	{
		assert(which >= Face_begin && which < Face_end);
		return &m_pixels[0] + which*m_faceSize;
	}
	const float* face(int which) const
	{
		assert(which >= Face_begin && which < Face_end);
		return &m_pixels[0] + which*m_faceSize;
	}

	/// Get index of face which direction p sits inside.
	static Face faceIndex(iVector3 p)
	{
		//iVector3 absp (fabs(p.GetX()), fabs(p.GetY()), fabs(p.GetZ()));
		iVector3 absp ( p.abs() );

		if(absp.GetX() >= absp.GetY() && absp.GetX() >= absp.GetZ())
			return (p.GetX() > 0) ? MicroBuf::Face_xp : MicroBuf::Face_xn;
		else if(absp.GetY() >= absp.GetX() && absp.GetY() >= absp.GetZ())
			return (p.GetY() > 0) ? MicroBuf::Face_yp : MicroBuf::Face_yn;
		else
		{
			assert(absp.GetZ() >= absp.GetX() && absp.GetZ() >= absp.GetY());
			return (p.GetZ() > 0) ? MicroBuf::Face_zp : MicroBuf::Face_zn;
		}
	}

	/// Get a neighbouring face in u direction
	///
	/// \param faceIdx - current face index
	/// \param side - which side to look (0 == left, 1 == right)
	///
	// +---+---+---+  +---+---+---+  +---+---+---+
	// |+z |+x |-z |  |-x |+y |+x |  |-x |+z |+x |
	// +---+---+---+  +---+---+---+  +---+---+---+
	//
	// +---+---+---+  +---+---+---+  +---+---+---+
	// |-z |-x |+z |  |-x |-y |+x |  |+x |-z |-x |
	// +---+---+---+  +---+---+---+  +---+---+---+
	//
	static Face neighbourU(int faceIdx, int side)
	{
		static Face neighbourArray[6][2] = {
			{Face_zp, Face_zn}, {Face_xn, Face_xp}, {Face_xn, Face_xp},
			{Face_zn, Face_zp}, {Face_xn, Face_xp}, {Face_xp, Face_xn}
		};
		return neighbourArray[faceIdx][side];
	}

	/// Get a neighbouring face in v direction
	///
	/// \param faceIdx - current face index
	/// \param side - which side to look (0 == bottom, 1 == top)
	///
	// +---+   +---+   +---+   +---+   +---+   +---+
	// |+y |   |-z |   |+y |   |+y |   |+z |   |+y |
	// +---+   +---+   +---+   +---+   +---+   +---+
	// |+x |   |+y |   |+z |   |-x |   |-y |   |-z |
	// +---+   +---+   +---+   +---+   +---+   +---+
	// |-y |   |+z |   |-y |   |-y |   |-z |   |-y |
	// +---+   +---+   +---+   +---+   +---+   +---+
	static Face neighbourV(int faceIdx, int side)
	{
		static Face neighbourArray[6][2] = {
			{Face_yn, Face_yp}, {Face_zp, Face_zn}, {Face_yn, Face_yp},
			{Face_yn, Face_yp}, {Face_zn, Face_zp}, {Face_yn, Face_yp}
		};
		return neighbourArray[faceIdx][side];
	}

	/// Get coordinates on face
	///
	/// The coordinates are in the range -1 <= u,v <= 1, if faceIdx is
	/// obtained using the faceIndex function.  Coordinates outside this
	/// range are legal, as long as p has nonzero component in the
	/// direction of the normal of the face.
	///
	/// \param faceIdx - index of current face
	/// \param p - position (may lie outside cone of current face)
	static void faceCoords(int faceIdx, iVector3 p, float& u, float& v)
	{
		p = canonicalFaceCoords(faceIdx, p);
		assert(p.GetZ() != 0);
		float zinv = 1.0f/p.GetZ();
		u = p.GetX()*zinv;
		v = p.GetY()*zinv;
	}

	/// Compute dot product of vec with face normal on given face
	static float dotFaceNormal(int faceIdx, iVector3 vec)
	{
		assert(faceIdx < Face_end && faceIdx >= Face_begin);
		return (faceIdx < 3) ? vec[faceIdx] : -vec[faceIdx-3];
	}

	/// Compute face normal
	static iVector3 faceNormal(int faceIdx)
	{
		static iVector3 normals[6] = {
				iVector3(1.f,0.f,0.f), iVector3(0.f,1.f,0.f), iVector3(0.f,0.f,1.f),
				iVector3(-1.f,0.f,0.f), iVector3(0,-1,0.f), iVector3(0.f,0.f,-1.f)
		};
		return normals[faceIdx];
	}

	/// Get direction vector for pixel on given face.
	iVector3 rayDirection(int faceIdx, int u, int v) const
	{
		return m_directions[(faceIdx*m_res + v)*m_res + u];
	}

	/// Return relative size of pixel.
	///
	/// Compared to a pixel in the middle of the cube face, pixels in the
	/// corners of the cube have a smaller angular size.  We must take
	/// this into account when integrating the radiosity.
	///
	/// \param u,v - face coordinates
	float pixelSize(int u, int v) const
	{
		return m_pixelSizes[m_res*v + u];
	}

	/// Reorder vector components into "canonical face coordinates".
	///
	/// The canonical coordinates correspond to the coordinates on the +z
	/// face.  If we let the returned vector be q then (q.GetX(), q.GetY())
	/// correspond to the face (u, v) coordinates, and q.GetZ() corresponds to
	/// the signed depth out from the face.
	static iVector3 canonicalFaceCoords(int faceIdx, iVector3 p)
	{
		switch(faceIdx)
		{
		case Face_xp: return iVector3(-p.GetZ(),  p.GetY(), p.GetX());
		case Face_xn: return iVector3(-p.GetZ(), -p.GetY(), p.GetX());
		case Face_yp: return iVector3( p.GetX(), -p.GetZ(), p.GetY());
		case Face_yn: return iVector3(-p.GetX(), -p.GetZ(), p.GetY());
		case Face_zp: return iVector3( p.GetX(),  p.GetY(), p.GetZ());
		case Face_zn: return iVector3( p.GetX(), -p.GetY(), p.GetZ());
		default: assert(0 && "invalid face"); return iVector3();
		}
	}

	/// Face side resolution
	int res() const { return m_res; }
	/// Number of channels per pixel
	int nchans() const { return m_nchans; }
	/// Total size of all faces in number of texels
	int size() const { return Face_end*m_res*m_res; }

private:
	/// Get direction vector for position on a given face.
	///
	/// Roughly speaking, this is the opposite of the faceCoords function
	static iVector3 direction(int faceIdx, float u, float v)
	{
		switch(faceIdx)
		{
		case Face_xp: return iVector3( 1, v,-u).normalize();
		case Face_yp: return iVector3( u, 1,-v).normalize();
		case Face_zp: return iVector3( u, v, 1).normalize();
		case Face_xn: return iVector3(-1, v, u).normalize();
		case Face_yn: return iVector3( u,-1, v).normalize();
		case Face_zn: return iVector3(-u, v,-1).normalize();
		default: assert(0 && "unknown face"); return iVector3();
		}
	}

	/// Square face resolution
	int m_res;
	/// Number of channels per pixel
	int m_nchans;
	/// Number of floats needed to store a face
	int m_faceSize;

	/// Pixel face storage
	boost::scoped_array<float> m_pixels;
	boost::scoped_array<float> m_defaultPixels;

	/// Storage for pixel ray directions
	boost::scoped_array<iVector3> m_directions;

	/// Pixels on a unit cube are not all equal in angular size
	boost::scoped_array<float> m_pixelSizes;
};


//------------------------------------------------------------------------------
/// Integrator for ambient occlusion.
///
/// The job of an integrator class is to save the microrasterized data
/// somewhere (presumably in a microbuffer) and integrate it at the end of the
/// rasterization to give the final illumination value.
class OcclusionIntegrator
{
public:
	/// Create integrator with given resolution of the environment map
	/// faces.
	OcclusionIntegrator(int faceRes)
		: m_buf(faceRes, 1, defaultPixel()),
		m_face(0)
	{
		clear();
	}

	/// Get direction of the ray
	iVector3 rayDirection(int iface, int u, int v)
	{
		return m_buf.rayDirection(iface, u, v);
	}

	/// Get at the underlying buffer
	const MicroBuf& microBuf()
	{
		return m_buf;
	}

	/// Get desired resolution of environment map faces
	int res()
	{
		return m_buf.res();
	}

	/// Reset buffer to default state
	void clear()
	{
		m_buf.reset();
	};

	/// Set extra data (eg, radiosity) associated with the current point
	/// being shaded.
	void setPointData(const float*) { }

	/// Set the face to which subsequent calls of addSample will apply
	void setFace(int iface)
	{
		m_face = m_buf.face(iface);
	};

	/// Add a rasterized sample to the current face
	///
	/// \param u,v - coordinates of face
	/// \param distance - distance to sample
	/// \param coverage - estimate of pixel coverage due to this sample
	void addSample(int u, int v, float distance, float coverage)
	{
		float* pix = m_face + (v*m_buf.res() + u) * m_buf.nchans();
		// There's more than one way to combine the coverage.
		//
		// 1) The usual method of compositing.  This assumes that
		// successive layers of geometry are uncorrellated so that each
		// attenuates the layer before, but a bunch of semi-covered layers
		// never result in full opacity.
		//
		// 1 - (1 - o1)*(1 - o2);
		//
		// 2) Add the opacities (and clamp to 1 at the end).  This is more
		// appropriate if we assume that we have adjacent non-overlapping
		// surfels.
		pix[0] += coverage;
	}

	/// Compute ambient occlusion based on previously sampled scene.
	///
	/// This is one minus the zero-bounce light coming from infinity to the
	/// point.
	///
	/// N is the shading normal; coneAngle is the angle over which to
	/// consider the occlusion in radians.  If coneAngle is not PI/2
	/// radians, the usual cos(theta) weighting is adjusted to
	/// (cos(theta) - cos(coneAngle)) so that it falls continuously to
	/// zero when theta == coneAngle
	float occlusion(iVector3 N, float coneAngle) const
	{
		// Integrate over face to get occlusion.
		float occ = 0;
		float totWeight = 0;
		float cosConeAngle = std::cos(coneAngle);
		for(int f = MicroBuf::Face_begin; f < MicroBuf::Face_end; ++f)
		{
			const float* face = m_buf.face(f);
			for(int iv = 0; iv < m_buf.res(); ++iv)
				for(int iu = 0; iu < m_buf.res(); ++iu, face += m_buf.nchans())
				{
					float d = Dotty(m_buf.rayDirection(f, iu, iv), N) -	cosConeAngle;
					if(d > 0)
					{
						d *= m_buf.pixelSize(iu, iv);
						// Accumulate light coming from infinity.
						occ += d* ds_MIN(1.0f, face[0]);
						totWeight += d;
					}
				}
		}
		if(totWeight != 0)
			occ /= totWeight;
		return occ;
	}

private:
	static float* defaultPixel()
	{
		static float def[1] = {0};
		return def;
	}

	MicroBuf m_buf;
	float* m_face;
};


//------------------------------------------------------------------------------
/// Microbuffer integrator for radiosity
class RadiosityIntegrator
{
public:
	/// Create integrator with given resolution of the environment map faces
	RadiosityIntegrator(int faceRes)
		:	m_buf(faceRes, 5, defaultPixel())
		,	m_face(0)
		,	m_currRadiosity(0.f)
	{
		clear();
	}
	RadiosityIntegrator()
		:	m_buf(12, 5, defaultPixel())
		,	m_face(0)
		,	m_currRadiosity(0.f)
	{
		clear();
	}

	/// Get direction of the ray
	iVector3 rayDirection(int iface, int u, int v)
	{
		return m_buf.rayDirection(iface, u, v);
	}

	/// Get at the underlying buffer
	const MicroBuf& microBuf()
	{
		return m_buf;
	}

	/// Get desired resolution of environment map faces
	int res()
	{
		return m_buf.res();
	}

	/// Reset buffer to default state
	void clear()
	{
		m_buf.reset();
	};

	/// Set extra data (radiosity) associated with the current point being shaded
	void setPointData(const float* radiosity)
	{
		m_currRadiosity = iVector3(radiosity[0], radiosity[1], radiosity[2]);
	}

	/// Set the face to which subsequent calls of addSample will apply
	void setFace(int iface)
	{
		m_face = m_buf.face(iface);
	};

	/// Add a rasterized sample to the current face
	///
	/// \param u,v - coordinates of face
	/// \param distance - distance to sample
	/// \param coverage - estimate of pixel coverage due to this sample
	void addSample(int u, int v, float distance, float coverage)
	{
		float* pix = m_face + (v*m_buf.res() + u) * m_buf.nchans();

		// TODO: Eventually remove dist if not needed
		float& currDist = pix[0];
		float& currCover = pix[1];
		//iVector3& radiosity ( *(iVector3*)(pix + 2) );

		if(distance < currDist)
		currDist = distance;

		if(currCover < 1)
		{
			if(currCover + coverage <= 1)
			{
				//radiosity += coverage*m_currRadiosity;
				pix[2] += coverage*m_currRadiosity[0];
				pix[3] += coverage*m_currRadiosity[1];
				pix[4] += coverage*m_currRadiosity[2];

				currCover += coverage;
			}
			else
			{
				//radiosity += (1 - currCover)*m_currRadiosity;
				pix[2] += (1 - currCover)*m_currRadiosity[0];
				pix[3] += (1 - currCover)*m_currRadiosity[1];
				pix[4] += (1 - currCover)*m_currRadiosity[2];
				currCover = 1;
			}
		}
	}

	/// Integrate radiosity based on previously sampled scene.
	///
	/// N is the shading normal; coneAngle is the angle over which to
	/// consider the occlusion in radians.  If coneAngle is not PI/2
	/// radians, the usual cos(theta) weighting is adjusted to
	/// (cos(theta) - cos(coneAngle)) so that it falls continuously to
	/// zero when theta == coneAngle
	///
	/// If occlusion is non-null, the amount of occlusion will also be
	/// computed and stored.
	iVector3 radiosity(iVector3 N, float coneAngle, float* occlusion = 0) const
	{
		// Integrate incoming light with cosine weighting to get outgoing radiosity
		iVector3 rad(0.f);
		float totWeight = 0;
		float cosConeAngle = std::cos(coneAngle);
		float occ = 0;

		for(int f = MicroBuf::Face_begin; f < MicroBuf::Face_end; ++f)
		{
			const float* face = m_buf.face(f);

			for(int iv = 0; iv < m_buf.res(); ++iv){
			for(int iu = 0; iu < m_buf.res(); ++iu, face += m_buf.nchans())
			{
				float d = Dotty(m_buf.rayDirection(f, iu, iv), N) - cosConeAngle;
				if(d > 0)
				{
					d *= m_buf.pixelSize(iu, iv);
					//iVector3& radiosity ( *(iVector3*)(face + 2) );
					iVector3 radiosity ( face[2], face[3], face[4] );
					rad += d*radiosity;
					occ += d*face[1];
					totWeight += d;
				}
			}
			}
		}

		if(totWeight != 0)
		{
			occ /= totWeight;
			rad = (1.0f/totWeight) * rad;
		}

		if(occlusion)
		*occlusion = occ;

		return rad;
	}

private:
	static float* defaultPixel()
	{
		// depth, foreground_coverage, foreground_rgb, background_rgb
		static float def[] = {FLT_MAX, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
		return def;
	}

	MicroBuf	m_buf;
	float*		m_face;
	iVector3	m_currRadiosity;
};

//------------------------------------------------------------------------------


/// Determine whether a sphere is wholly outside a cone.
///
/// This is used to cull disks - the sphere is the bounding sphere of the
/// disk, and the cone is the cone of incoming light of interest.
///
/// Getting an expression which is both correct and efficient (ie, not
/// involving special functions) is tricky.  The simple version of it is when
/// the radius is zero, in which case we just need to compute
///
///     cosConeAngle > dot(p/plen, N)
///
/// In the general case of spheres with nonzero radius the condition to check
/// is that
///
///     coneAngle + boundingSphereAngle < coneNormalToSphereCenterAngle
///
/// After some algebra, this reduces to the expression
///
///     sqrt(dot(p,p) - r*r)*cosConeAngle - r*sinConeAngle > dot(p, n);
///
/// which is valid as long as the sum of the sphere angle and cone angle are
/// less than pi:
///
///     sqrt(1 - r*r/dot(p,p)) < -cosConeAngle
///
/// in which case the sphere must intersect the cone.
///
/// \param p - center of sphere
/// \param plen2 - length of p
/// \param r - sphere radius
/// \param n - cone normal
/// \param cosConeAngle - cos(theta) where theta = cone angle from N
/// \param sinConeAngle - sin(theta)
#include <cmath>
inline bool sphereOutsideCone(	iVector3 p, float plen2, float r,
								iVector3 n, float cosConeAngle, float sinConeAngle)
{
	// The actual expressions used here are an optimized version which does
	// the same thing, but avoids calling sqrt().  This makes a difference
	// when this is the primary culling test and you're iterating over lots of
	// points - you need to reject invalid points as fast as possible.
	float x = plen2 - r*r;

	// special case - if the sphere covers the origin, it must intersect the cone
	if(x < 0)
	return false;

	// Special case - if sphere angle and cone angle add to >= 180 degrees, the
	// sphere and cone must intersect.
	if(cosConeAngle < 0 && x < plen2*cosConeAngle*cosConeAngle)
	return false;

	// General case
	 float lhs = x*cosConeAngle*cosConeAngle;
	 float rhs = Dotty(p, n) + r*sinConeAngle;
	 return _copysignf(lhs, cosConeAngle) > _copysignf(rhs*rhs, rhs);
}


/// Find real solutions of the quadratic equation a*x^2 + b*x + c = 0.
///
/// The equation is assumed to have real-valued solutions; if they are in fact
/// complex only the real parts are returned.
inline void solveQuadratic(float a, float b, float c, float& lowRoot, float& highRoot)
{
	// Avoid NaNs when determinant is negative by clamping to zero.  This is
	// the right thing to do when det would otherwise be zero to within
	// numerical precision.
	float det = ds_MAX(0.0f, b*b - 4*a*c);
	float firstTerm = -b/(2*a);
	float secondTerm = sqrtf(det)/(2*a);
	lowRoot = firstTerm - secondTerm;
	highRoot = firstTerm + secondTerm;
}


/// Render a disk into the microbuffer using exact visibility testing
///
/// When two surfaces meet at a sharp angle, the visibility of surfels on the
/// adjacent surface needs to be computed more carefully than with the
/// approximate square-based rasterization method.  If not, disturbing
/// artifacts will appear where the surfaces join.
///
/// \param microBuf - buffer to render into.
/// \param p - position of disk center relative to microbuffer
/// \param n - disk normal
/// \param r - disk radius
template<typename IntegratorT>
static void renderDiskExact( IntegratorT& integrator, iVector3 p, iVector3 n, float r )
{
	int faceRes = integrator.res();
	float plen2 = p.length2();

	if(plen2 == 0) // Sanity check
	return;

	// Angle from face normal to edge is acos(1/sqrt(3)).
	static float cosFaceAngle = 1.0f/sqrtf(3);
	static float sinFaceAngle = sqrtf(2.0f/3.0f);

	for(int iface = 0; iface < 6; ++iface)
	{
		// Avoid rendering to the current face if the disk definitely doesn't
		// touch it.  First check the cone angle
		if(sphereOutsideCone(p, plen2, r, MicroBuf::faceNormal(iface), cosFaceAngle, sinFaceAngle))
		continue;

		float dot_pFaceN = MicroBuf::dotFaceNormal(iface, p);
		float dot_nFaceN = MicroBuf::dotFaceNormal(iface, n);

		// If the disk is behind the camera and the disk normal is relatively
		// aligned with the face normal (to within the face cone angle), the
		// disk can't contribute to the face and may be culled.
		if(dot_pFaceN < 0 && fabs(dot_nFaceN) > cosFaceAngle)
		continue;

		// Check whether disk spans the perspective divide for the current
		// face.  Note: sin^2(angle(n, faceN)) = (1 - dot_nFaceN*dot_nFaceN)
		if((1 - dot_nFaceN*dot_nFaceN)*r*r >= dot_pFaceN*dot_pFaceN)
		{
			// When the disk spans the perspective divide, the shape of the
			// disk projected onto the face is a hyperbola.  Bounding a
			// hyperbola is a pain, so the easiest thing to do is trace a ray
			// for every pixel on the face and check whether it hits the disk.
			//
			// Note that all of the tricky rasterization rubbish further down
			// could probably be replaced by the following ray tracing code if
			// I knew a way to compute the tight raster bound.
			integrator.setFace(iface);
			for(int iv = 0; iv < faceRes; ++iv){
			for(int iu = 0; iu < faceRes; ++iu)
			{
				// V = ray through the pixel
				iVector3 V ( integrator.rayDirection(iface, iu, iv) );

				// Signed distance to plane containing disk
				float t = Dotty(p, n)/Dotty(V, n);
				if(t > 0 && (t*V - p).length2() < r*r)
				{
					// The ray hit the disk, record the hit
					integrator.addSample(iu, iv, t, 1.0f);
				}
			}
			}
			
			continue;
		}

		// If the disk didn't span the perspective divide and is behind the
		// camera, it may be culled.
		if(dot_pFaceN < 0) continue;

		// Having gone through all the checks above, we know that the disk
		// doesn't span the perspective divide, and that it is in front of the
		// camera.  Therefore, the disk projected onto the current face is an
		// ellipse, and we may compute a quadratic function
		//
		//   q(u,v) = a0*u*u + b0*u*v + c0*v*v + d0*u + e0*v + f0
		//
		// such that the disk lies in the region satisfying q(u,v) < 0.  To do
		// this, start with the implicit definition of the disk on the plane,
		//
		//   norm(dot(p,n)/dot(V,n) * V - p)^2 - r^2 < 0
		//
		// and compute coefficients A,B,C such that
		//
		//   A*dot(V,V) + B*dot(V,n)*dot(p,V) + C < 0
		float dot_pn = Dotty(p,n);
		float A = dot_pn*dot_pn;
		float B = -2*dot_pn;
		float C = plen2 - r*r;

		// Project onto the current face to compute the coefficients a0 through
		// to f0 for q(u,v)
		iVector3 pp ( MicroBuf::canonicalFaceCoords(iface, p) );
		iVector3 nn ( MicroBuf::canonicalFaceCoords(iface, n) );
		float a0 = A + B*nn.GetX()*pp.GetX() + C*nn.GetX()*nn.GetX();
		float b0 = B*(nn.GetX()*pp.GetY() + nn.GetY()*pp.GetX()) + 2*C*nn.GetX()*nn.GetY();
		float c0 = A + B*nn.GetY()*pp.GetY() + C*nn.GetY()*nn.GetY();
		float d0 = (B*(nn.GetX()*pp.GetZ() + nn.GetZ()*pp.GetX()) + 2*C*nn.GetX()*nn.GetZ());
		float e0 = (B*(nn.GetY()*pp.GetZ() + nn.GetZ()*pp.GetY()) + 2*C*nn.GetY()*nn.GetZ());
		float f0 = (A + B*nn.GetZ()*pp.GetZ() + C*nn.GetZ()*nn.GetZ());

		// Finally, transform the coefficients so that they define the
		// quadratic function in *raster* face coordinates, (iu, iv)
		float scale = 2.0f/faceRes;
		float scale2 = scale*scale;
		float off = 0.5f*scale - 1.0f;
		float a = scale2*a0;
		float b = scale2*b0;
		float c = scale2*c0;
		float d = ((2*a0 + b0)*off + d0)*scale;
		float e = ((2*c0 + b0)*off + e0)*scale;
		float f = (a0 + b0 + c0)*off*off + (d0 + e0)*off + f0;

		// Construct a tight bound for the ellipse in raster coordinates.
		int ubegin = 0, uend = faceRes;
		int vbegin = 0, vend = faceRes;
		float det = 4*a*c - b*b;

		// Sanity check; a valid ellipse must have det > 0
		if(det <= 0)
		{
			// If we get here, the disk is probably edge on (det == 0) or we
			// have some hopefully small floating point errors (det < 0: the
			// hyperbolic case we've already ruled out).  Cull in either case.
			continue;
		}
		float ub = 0, ue = 0;
		solveQuadratic(det, 4*d*c - 2*b*e, 4*c*f - e*e, ub, ue);
		ubegin = ds_MAX(0, Imath::ceil(ub));
		uend   = ds_MIN(faceRes, Imath::ceil(ue));
		float vb = 0, ve = 0;
		solveQuadratic(det, 4*a*e - 2*b*d, 4*a*f - d*d, vb, ve);
		vbegin = ds_MAX(0, Imath::ceil(vb));
		vend   = ds_MIN(faceRes, Imath::ceil(ve));

		// By the time we get here, we've expended perhaps 120 FLOPS + 2 sqrts
		// to set up the coefficients of q(iu,iv).  The setup is expensive, but
		// the bound is optimal so it will be worthwhile vs raytracing, unless
		// the raster faces are very small.
		integrator.setFace(iface);
		for(int iv = vbegin; iv < vend; ++iv)
		for(int iu = ubegin; iu < uend; ++iu)
		{
			float q = a*(iu*iu) + b*(iu*iv) + c*(iv*iv) + d*iu + e*iv + f;
			if(q < 0)
			{
				iVector3 V ( integrator.rayDirection(iface, iu, iv) );
				// compute distance to hit point
				float z = dot_pn/Dotty(V, n);
				integrator.addSample(iu, iv, z, 1.0f);
			}
		}
	}
}


/// Rasterize disk into the given integrator
///
/// N is the normal of the culling cone, with cone angle specified by
/// cosConeAngle and sinConeAngle.  The position of the disk with respect to
/// the centre of the microbuffer is p, n is the normal of the disk and r is
/// the disk radius.
template<typename IntegratorT>
void renderDisk(	IntegratorT& integrator, iVector3 N, iVector3 p, iVector3 n, float r,
					float cosConeAngle, float sinConeAngle, float exactRAngle =0.05f, float exactRadius =-1.f	)
{
	if(exactRadius == -1.f) exactRadius = r;

	float dot_pn = Dotty(p, n);

	// Cull back-facing points.  In conjunction with the oddball composition
	// rule below, this is very important for smoothness of the result:  If we
	// don't cull the back faces, coverage will be overestimated in every
	// microbuffer pixel which contains an edge.
	if(dot_pn > 0)
	return;

	float plen2 = p.length2();
	float plen = sqrtf(plen2);

	// If solid angle of bounding sphere is greater than exactRenderAngle,
	// resolve the visibility exactly rather than using a cheap approx.
	//
	// TODO: Adjust exactRenderAngle for best results!
	//const float exactRenderAngle = 0.05f;
	const float exactRenderAngle = exactRAngle;
	float maxArea = M_PI*r*r;
	float origArea = M_PI*exactRadius*exactRadius;

	// Solid angle of the bound
	if(exactRenderAngle > 0.f)
	if(exactRenderAngle*plen2 < maxArea)
	{
		// Multiplier for radius to make the cracks a bit smaller.  We
		// can't do this too much, or sharp convex edges will be
		// overoccluded (TODO: Adjust for best results!  Maybe the "too
		// large" problem could be worked around using a tracing offset?)
		const float radiusMultiplier = M_SQRT2;

		/// Resolve visibility of very close surfels using ray tracing.
		/// This is necessary to avoid artifacts where surfaces meet.
		renderDiskExact(integrator, p, n, radiusMultiplier*exactRadius);
		return;
	}


	// Figure out which face we're on and get u,v coordinates on that face,
	MicroBuf::Face faceIndex = MicroBuf::faceIndex(p);
	int faceRes = integrator.res();
	float u = 0, v = 0;
	MicroBuf::faceCoords(faceIndex, p, u, v);

	// Compute the area of the surfel when projected onto the env face.
	// This depends on several things:
	// 1) The area of the original disk
	// 2) The angles between the disk normal n, viewing vector p, and face
	// normal.  This is the area projected onto a plane parallel to the env
	// map face, and through the centre of the disk.
	float pDotFaceN = MicroBuf::dotFaceNormal(faceIndex, p);
	float angleFactor = fabs(dot_pn/pDotFaceN);

	// 3) Ratio of distance to the surfel vs distance to projected point on the face.
	float distFactor = 1.0f/(pDotFaceN*pDotFaceN);

	// Putting these together gives the projected area
	float projArea = origArea * angleFactor * distFactor;

	// Half-width of a square with area projArea
	float wOn2 = sqrtf(projArea)*0.5f;

	// Transform width and position to face raster coords.
	float rasterScale = 0.5f*faceRes;
	u = rasterScale*(u + 1.0f);
	v = rasterScale*(v + 1.0f);
	wOn2 *= rasterScale;

	// Construct square box with the correct area.  This shape isn't
	// anything like the true projection of a disk onto the raster, but
	// it's much cheaper!  Note that points which are proxies for clusters
	// of smaller points aren't going to be accurately resolved no matter
	// what we do.
	struct BoundData
	{
		MicroBuf::Face faceIndex;
		float ubegin, uend;
		float vbegin, vend;
	};

	// The current surfel can cross up to three faces.
	int nfaces = 1;
	BoundData boundData[3];
	BoundData& bd0 = boundData[0];
	bd0.faceIndex = faceIndex;

	bd0.ubegin = u - wOn2;   
	bd0.uend = u + wOn2;

	bd0.vbegin = v - wOn2;   
	bd0.vend = v + wOn2;

	// Detect & handle overlap onto adjacent faces
	//
	// We assume that wOn2 is the same on the adjacent face, an assumption
	// which is true when the surfel is close the the corner of the cube.
	// We also assume that a surfel touches at most three faces.  This is
	// true as long as the surfels don't have a massive solid angle; for
	// such cases the axis-aligned box isn't going to be accurate anyway and
	// the code should have branched into the renderDiskExact function instead.
	if(bd0.ubegin < 0)
	{
		// left neighbour
		BoundData& b = boundData[nfaces++];
		b.faceIndex = MicroBuf::neighbourU(faceIndex, 0);
		MicroBuf::faceCoords(b.faceIndex, p, u, v);
		u = rasterScale*(u + 1.0f);
		v = rasterScale*(v + 1.0f);
		b.ubegin = u - wOn2;  b.uend = u + wOn2;
		b.vbegin = v - wOn2;  b.vend = v + wOn2;
	}
	else if(bd0.uend > faceRes)
	{
		// right neighbour
		BoundData& b = boundData[nfaces++];
		b.faceIndex = MicroBuf::neighbourU(faceIndex, 1);
		MicroBuf::faceCoords(b.faceIndex, p, u, v);
		u = rasterScale*(u + 1.0f);
		v = rasterScale*(v + 1.0f);
		b.ubegin = u - wOn2;  b.uend = u + wOn2;
		b.vbegin = v - wOn2;  b.vend = v + wOn2;
	}
	if(bd0.vbegin < 0)
	{
		// bottom neighbour
		BoundData& b = boundData[nfaces++];
		b.faceIndex = MicroBuf::neighbourV(faceIndex, 0);
		MicroBuf::faceCoords(b.faceIndex, p, u, v);
		u = rasterScale*(u + 1.0f);
		v = rasterScale*(v + 1.0f);
		b.ubegin = u - wOn2;  b.uend = u + wOn2;
		b.vbegin = v - wOn2;  b.vend = v + wOn2;
	}
	else if(bd0.vend > faceRes)
	{
		// top neighbour
		BoundData& b = boundData[nfaces++];
		b.faceIndex = MicroBuf::neighbourV(faceIndex, 1);
		MicroBuf::faceCoords(b.faceIndex, p, u, v);
		u = rasterScale*(u + 1.0f);
		v = rasterScale*(v + 1.0f);
		b.ubegin = u - wOn2;  b.uend = u + wOn2;
		b.vbegin = v - wOn2;  b.vend = v + wOn2;
	}

	for(int iface = 0; iface < nfaces; ++iface)
	{
		BoundData& bd = boundData[iface];

		// Range of pixels which the square touches (note, exclusive end)
		int ubeginRas = Imath::clamp(int(bd.ubegin),   0, faceRes);
		int uendRas   = Imath::clamp(int(bd.uend) + 1, 0, faceRes);
		int vbeginRas = Imath::clamp(int(bd.vbegin),   0, faceRes);
		int vendRas   = Imath::clamp(int(bd.vend) + 1, 0, faceRes);

		integrator.setFace(bd.faceIndex);

		for(int iv = vbeginRas; iv < vendRas; ++iv){
		for(int iu = ubeginRas; iu < uendRas; ++iu)
		{
			// Calculate the fraction coverage of the square over the current
			// pixel for antialiasing.  This estimate is what you'd get if you
			// filtered the square representing the surfel with a 1x1 box filter.
			float urange =	ds_MIN(iu+1, bd.uend) -
							ds_MAX(iu,   bd.ubegin);
			float vrange =	ds_MIN(iv+1, bd.vend) -
							ds_MAX(iv,   bd.vbegin);

			float coverage = urange*vrange;
			integrator.addSample(iu, iv, plen, coverage);
		}
		}
	}
}


/// Render point hierarchy into microbuffer.
template<typename IntegratorT, class PListType, class iOctreeNodeType>
static void renderNode(	IntegratorT& integrator,
						const iVector3& P, const iVector3& N,
						float cosConeAngle,	float sinConeAngle, float maxSolidAngle, float exactRAngle,
						int dataSize,
						PListType 				* pointlist,
						const iOctreeNodeType	* node	)
{
	// This is an iterative traversal of the point hierarchy, since it's
	// slightly faster than a recursive traversal.
	//
	// The max required size for the explicit stack should be < 200, since
	// tree depth shouldn't be > 24, and we have a max of 8 children per node.
	//const PointOctree::Node* nodeStack[200];
	const iOctreeNodeType* nodeStack[200];
	nodeStack[0] = node;
	int stackSize = 1;


	while(stackSize > 0)
	{
		node = nodeStack[--stackSize];
		{
			// Examine node bound and cull if possible
			// TODO: Reinvestigate using (node->aggP - P) with spherical harmonics
			iVector3 node_center ( (node->bbox_min + node->bbox_max) / 2.f);
#if	__SSE2_ENHANCED__
			float node_bradius ( node->bbox_max[3] );
#else
			float node_bradius ( iVector3(node->bbox_max - node->bbox_min).length() / 2.f );
#endif
			iVector3 c ( node_center - P );
			if(sphereOutsideCone(c, c.sqr_length(), node_bradius, N, cosConeAngle, sinConeAngle))
			continue;
		}

		float		r	= node->aggR;
		float		tR	= node->bbox_min[3];
		iVector3	p	( node->aggP - P );

		float		plen2	= p.sqr_length();
		//float		plen2 = P.sqr_dist_bbox( node->bbox_min,  node->bbox_max);

		// Examine solid angle of interior node bounding sphere to see whether we
		// can render it directly or not.
		//
		// TODO: Would be nice to use dot(node->aggN, p.normalized()) in the solid
		// angle estimation.  However, we get bad artifacts if we do this naively.
		// Perhaps with spherical harmonics it'll be better.
		float solidAngle = M_PI*r*r / plen2;
		if(solidAngle < maxSolidAngle)
		{
			integrator.setPointData(reinterpret_cast<const float*>(&node->aggCol));
			renderDisk(integrator, N, p, node->aggN, r, cosConeAngle, sinConeAngle, -1.f/*, tR*/);
		}
		else
		{
			// If we get here, the solid angle of the current node was too large
			// so we must consider the children of the node.
			//
			// The render order is sorted so that points are rendered front to
			// back.  This greatly improves the correctness of the hider.
			//
			// FIXME: The sorting procedure gets things wrong sometimes!  The
			// problem is that points may stick outside the bounds of their octree
			// nodes.  Probably we need to record all the points, sort, and
			// finally render them to get this right.
			//if(node->npoints != 0)
			if(node->isleaf)
			{
				
				// Leaf node: simply render each child point.
				std::pair<float, int> childOrder[8];
				assert(node->npoints <= 8);

				int o;
				int firstpoint= node->firstpoint;
				int lastpoint = firstpoint + node->npoints;
	
				for(int i = firstpoint, o=0; i < lastpoint; i++, ++o)
				{
#if	__SSE2_ENHANCED__
					iVector3 p ( iVector3( _mm_load_ps( pointlist->GetNodePtr(i) +0 )) - P );
#else
					iVector3 p ( pointlist->template GetNodeVectorAt<DATA_SLOT_POS>(i) - P );
#endif
					childOrder[o].first = p.length2();
					childOrder[o].second = o;
				}

				// sort points by distance
				std::sort(childOrder, childOrder + node->npoints);
				
				for(int i = firstpoint, o=0; i < lastpoint; i++, ++o)
				{
					const float* data = pointlist->GetNodePtr( firstpoint +childOrder[o].second );

#if	__SSE2_ENHANCED__
					iVector3 p		( _mm_load_ps( data +0 ));
					iVector3 n		( _mm_load_ps( data +4 ));
					iVector3 r		( _mm_load_ps( data +8 ));
					iVector3 irrad	( _mm_load_ps( data +12 ));
#else
					iVector3 p		( pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( data ));
					iVector3 n		( pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>( data ));
					float r			( pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( data ));
					iVector3 irrad	( pointlist->template GetNodeVectorAt<DATA_SLOT_IRRAD>( data ));
#endif

					p -= P;

					integrator.setPointData( irrad.begin() );
					renderDisk(integrator, N, p, n, 
#if	__SSE2_ENHANCED__
						r.GetX(), 
#else
						r, 
#endif
						cosConeAngle, sinConeAngle, exactRAngle
#if	__SSE2_ENHANCED__
						//, r.GetY()	
#endif
						);
				}
				continue;
			}
			else
			{
				// Interior node: render children.
				std::pair<float, const iOctreeNodeType*> children[8];
				int nchildren = 0;
				for(int i = 0; i < 8; ++i)
				{	
					//PointOctree::Node* child = node->children[i];
					iOctreeNodeType* child = node->children[i];
					if(!child) continue;

					iVector3 child_center ( (child->bbox_min + child->bbox_max) / 2.f);
					children[nchildren].first = (child_center - P).length2();
					children[nchildren].second = child;
					++nchildren;
				}
				std::sort(children, children + nchildren);

				// Interior node: render each non-null child.  Nodes we want to
				// render first must go onto the stack last.
				for(int i = nchildren-1; i >= 0; --i)
				nodeStack[stackSize++] = children[i].second;
			}
		}
	}
}





/// //////////////////////////////////////////////////////////////////////////////////////////////////////
/// //////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef DEPREACATEDMICROSTUFF

/// Render point hierarchy into microbuffer.
template<typename IntegratorT, class PointOctreeNode>
static void renderNodeXXX(	IntegratorT& integrator, 
						iVector3 P, iVector3 N, 
						float cosConeAngle,	float sinConeAngle, 
						float maxSolidAngle, 
						int dataSize,
						const PointOctreeNode* node	)
{
	// This is an iterative traversal of the point hierarchy, since it's
	// slightly faster than a recursive traversal.
	//
	// The max required size for the explicit stack should be < 200, since
	// tree depth shouldn't be > 24, and we have a max of 8 children per node.
	const PointOctreeNode* nodeStack[200];
	nodeStack[0] = node;
	int stackSize = 1;


	while(stackSize > 0)
	{
		node = nodeStack[--stackSize];
		{
			// Examine node bound and cull if possible
			// TODO: Reinvestigate using (node->aggP - P) with spherical harmonics
			iVector3 node_center ( (node->bbox_min + node->bbox_max) / 2.f);
			float node_bradius ( iVector3(node->bbox_max - node->bbox_min).length()/2.f );

			iVector3 c ( node_center - P );
			if(sphereOutsideCone(c, c.sqr_length(), node_bradius, N, cosConeAngle, sinConeAngle))
			continue;
		}

		float		r		= node->aggR;
		iVector3	p( node->aggP - P );
		float		plen2	= p.sqr_length();

		// Examine solid angle of interior node bounding sphere to see whether we
		// can render it directly or not.
		//
		// TODO: Would be nice to use dot(node->aggN, p.normalized()) in the solid
		// angle estimation.  However, we get bad artifacts if we do this naively.
		// Perhaps with spherical harmonics it'll be better.
		float solidAngle = M_PI*r*r / plen2;
		if(solidAngle < maxSolidAngle)
		{
			integrator.setPointData(reinterpret_cast<const float*>(&node->aggCol));
			renderDisk(integrator, N, p, node->aggN, r, cosConeAngle, sinConeAngle);
		}
		else
		{
			// If we get here, the solid angle of the current node was too large
			// so we must consider the children of the node.
			//
			// The render order is sorted so that points are rendered front to
			// back.  This greatly improves the correctness of the hider.
			//
			// FIXME: The sorting procedure gets things wrong sometimes!  The
			// problem is that points may stick outside the bounds of their octree
			// nodes.  Probably we need to record all the points, sort, and
			// finally render them to get this right.
			if(node->npoints != 0)
			{
				// Leaf node: simply render each child point.
				std::pair<float, int> childOrder[8];
				assert(node->npoints <= 8);

				for(int i = 0; i < node->npoints; ++i)
				{
					const float* data = &node->data[i*dataSize];

					iVector3 p ( iVector3(data[0], data[1], data[2]) - P );
					childOrder[i].first = p.length2();
					childOrder[i].second = i;
				}
				std::sort(childOrder, childOrder + node->npoints);
				
				for(int i = 0; i < node->npoints; ++i)
				{
					const float* data = &node->data[childOrder[i].second*dataSize];
					iVector3 p ( iVector3(data[0], data[1], data[2]) - P );
					iVector3 n ( data[3], data[4], data[5] );
					float r = data[6];

					integrator.setPointData(data+7);
					renderDisk(integrator, N, p, n, r, cosConeAngle, sinConeAngle);
				}
				
				continue;
			}
			else
			{
				// Interior node: render children.
				std::pair<float, const PointOctreeNode*> children[8];
				int nchildren = 0;
				for(int i = 0; i < 8; ++i)
				{
					PointOctreeNode* child = node->children[i];
					if(!child) continue;

					iVector3 child_center ( (child->bbox_min + child->bbox_max) / 2.f);
					children[nchildren].first = (child_center - P).length2();
					children[nchildren].second = child;
					++nchildren;
				}
				std::sort(children, children + nchildren);

				// Interior node: render each non-null child.  Nodes we want to
				// render first must go onto the stack last.
				for(int i = nchildren-1; i >= 0; --i)
				nodeStack[stackSize++] = children[i].second;
			}
		}
	}
}

/// Visualize sources of light in occlusion calculation (for debugging)
void occlWeight(MicroBuf& depthBuf, const iVector3& N)
{
	for(int f = MicroBuf::Face_begin; f < MicroBuf::Face_end; ++f)
	{
		float* face = depthBuf.face(f);
		for(int iv = 0; iv < depthBuf.res(); ++iv)
			for(int iu = 0; iu < depthBuf.res(); ++iu, face += 2)
			{
				float d = Dotty(depthBuf.rayDirection(f, iu, iv), N);
				if(d > 0)
					face[1] = d*(1.0f - face[1]);
				else
					face[1] = 0;
			}
	}
}


/// Bake occlusion from point array back into point array.
void bakeOcclusion(PointArray& points, int faceRes)
{
	const float eps = 0.1;
	OcclusionIntegrator integrator(faceRes);
	for(int pIdx = 0, npoints = points.size(); pIdx < npoints; ++pIdx)
	{
		if(pIdx % 100 == 0)
			std::cout << 100.0f*pIdx/points.size() << "%    \r" << std::flush;
		float* data = &points.data[pIdx*points.stride];
		// normal of current point
		iVector3 N ( iVector3(data[3], data[4], data[5]) );
		// position of current point relative to shading point
		iVector3 P ( iVector3(data[0], data[1], data[2]) );
		float r = data[6];
		integrator.clear();
		microRasterize(integrator, P + N*r*eps, N, M_PI_2, points);
		data[7] = data[8] = data[9] = 1 - integrator.occlusion(N, M_PI_2);
	}
}


/// Bake occlusion from point hierarchy tree into point cloud.
///
/// \param points - output array of surfels
/// \param tree - hierarchical point-based representation of scene
/// \param faceRes - resolution of microbuffer to use
/// \param maxSolidAngle - Maximum solid angle allowed for points in interior
///                        tree nodes.
void bakeOcclusion(PointArray& points, const PointOctree& tree, int faceRes,
                   float maxSolidAngle)
{
	// FIXME: Code duplication with bakeOcclusion above.
	const float eps = 0.1;
	OcclusionIntegrator integrator(faceRes);
	for(int pIdx = 0, npoints = points.size(); pIdx < npoints; ++pIdx)
	{
		if(pIdx % 400 == 0)
			std::cout << 100.0f*pIdx/points.size() << "%    \r" << std::flush;
		float* data = &points.data[pIdx*points.stride];
		// normal of current point
		iVector3 N ( iVector3(data[3], data[4], data[5]) );
		// position of current point relative to shading point
		iVector3 P ( iVector3(data[0], data[1], data[2]) );
		float r = data[6];
		integrator.clear();
		microRasterize(integrator, P + N*r*eps, N, M_PI_2, maxSolidAngle, tree);
		data[7] = data[8] = data[9] = 1 - integrator.occlusion(N, M_PI_2);
	}
}


/// Bake radiosity from point hierarchy into point cloud. /////////////////////////////////////////////////////////////
///
/// \param points - output array of surfels
/// \param tree - hierarchical point-based representation of scene
/// \param faceRes - resolution of microbuffer to use
/// \param maxSolidAngle - Maximum solid angle allowed for points in interior
///                        tree nodes.
static void bakeRadiosity(	std::vector<float>& radResult,
					PointArray& points, const PointOctree& tree, int faceRes, float maxSolidAngle,
					int startP=0, int endP=0, bool isMainThread=false )
{
	int npoints = points.size();
	if(endP==0)endP = npoints;


	const float eps = 0.1;
	RadiosityIntegrator integrator(faceRes);

/*	float* data;
	iVector3 N;
	iVector3 P;
	float r;

	int n_threads = 8;
	int chunky = points.size() / n_threads;
	omp_set_num_threads( n_threads );

	#pragma omp parallel private(data, P, N, r) 
	#pragma omp for schedule(static, chunky)
*/
	for(int pIdx = startP; pIdx < endP; ++pIdx)
	{
		//if(isMainThread)
		if( pIdx % 400 == 0 )
		std::cout << 100.0f*pIdx/points.size() << "%    \r" << std::flush;
		//if(mi_par_aborted()) return;

		// data block pointer
		float* data = &points.data[ pIdx * points.stride ];

		// normal of current point
		iVector3 N = iVector3(data[3], data[4], data[5]);

		// position of current point relative to shading point
		iVector3 P (data[0], data[1], data[2]);
		float r = data[6];

		// irradiance data
		iVector3 irrad (data[7], data[8], data[9]);

		// first bounce radiosity
		integrator.clear();
		microRasterize(integrator, P + N*r*eps, N, f_pi_div2, maxSolidAngle, tree);
		/*iVector3 firstBounce = irrad + integrator.radiosity(N, M_PI_2);
		*reinterpret_cast<iVector3*>(data+7) = firstBounce;

		// second bounce radiosity	**TODO::UPDATE OCTREE TOO**
		microRasterize(integrator, P + N*r*eps, N, M_PI_2, maxSolidAngle, tree);
		iVector3 secondBounce = firstBounce + integrator.radiosity(N, M_PI_2);
		*reinterpret_cast<iVector3*>(data+7) = secondBounce;
		*/

		// result
		float* result = &radResult[ pIdx * DATA_RESULT_SLOT ];
		*reinterpret_cast<iVector3*>(result) = irrad + integrator.radiosity(N, f_pi_div2);

		//*reinterpret_cast<iVector3*>(data+7) = irrad + integrator.radiosity(N, M_PI_2);
	}
}

#if __OMP_ENHANCED__
/// OMP ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void bakeRadiosityOMP(	std::vector<float>& radResult,
						PointArray& points, const PointOctree& tree, int faceRes, float maxSolidAngle )
{
	int npoints = points.size();
	iVector3 raddummy(0.f,0.f,0.f);
	const float eps = 0.1;
	//RadiosityIntegrator integrator(faceRes);
	//RadiosityIntegrator integrator;

	omp_set_num_threads( 8 );
	#pragma omp parallel //private( integrator )
	{
	#pragma omp for
	for(int pIdx = 0; pIdx < npoints; ++pIdx)
	{
		if( pIdx % 400 == 0 )
		std::cout << 100.0f*pIdx/points.size() << "%    \r" << std::flush;

		// data block pointer
		float* data = &points.data[ pIdx * points.stride ];

		// normal of current point
		iVector3 N (data[3], data[4], data[5]);

		// position of current point relative to shading point
		iVector3 P (data[0], data[1], data[2]);
		float r = data[6];

		// irradiance data
		//iVector3 irrad =  iVector3(data[7], data[8], data[9]);

		// first bounce radiosity
		//integrator.clear();
		RadiosityIntegrator integrator(faceRes);
		//printf("DEBUG::TILLHERE?!");///
		microRasterize(integrator, P + N*r*eps, N, f_pi_div2, maxSolidAngle, tree);

		// result
		float* result = &radResult[ pIdx * DATA_RESULT_SLOT ];
		*reinterpret_cast<iVector3*>(result) = integrator.radiosity(N, f_pi_div2);
		//*reinterpret_cast<iVector3*>(result) = raddummy;
	}
	}
}
#endif	//is __OMP_ENHANCED__
#endif	//deprecated
}		//namespace end

#endif	// CPC_MICROBUFFER_H_INCLUDED

