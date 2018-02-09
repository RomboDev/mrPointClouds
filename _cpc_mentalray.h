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

#ifndef __CPC_MENTALRAY__
#define __CPC_MENTALRAY__


#define __MENTALRAY_ENHANCED__	1
#define __OMP_ENHANCED__		0
#define __SSE2_ENHANCED__		1
#define	__OPENEXR_ENHANCED__	0


#include "AuxiliaryFunctions/cpc_utilities_mi.hpp"
#include "DataStructures/_datastructures.h"
#include "Infrastructure/_infrastructure.h"
#include "Implementations/_implementations.h"

#include "shader.h"

using namespace mi::shader;
using namespace std;
using namespace cpc;


/// ///////////////////////////////////////////////////////////////////////////////////////////////////
#if	__SSE2_ENHANCED__
typedef iPointList<float, DATA_NSLOTS_AO +DATA_RESULT_SLOT, iPListDataStorage_MemPool>				_pointlistT;
typedef iPointList<float, DATA_NSLOTS_SSS +DATA_RESULT_SLOT, iPListDataStorage_MemPool>				_pointlistSSS;
typedef iPointList<float, DATA_NSLOTS_BAKE +DATA_RESULT_SLOT, iPListDataStorage_MemPool>			_pointlistBake;
typedef iPointList<float, DATA_NSLOTS_BAKE +DATA_RESULT_SLOT, iPListDataStorage_MemPool>			_pointlistGI;
#else
typedef iPointList<float, DATA_NSLOTS_AO +DATA_RESULT_SLOT, iPListDataStorage_MemPool, iPListDataAllocator>	_pointlistT;
typedef iPointList<float, DATA_NSLOTS_SSS +DATA_RESULT_SLOT, iPListDataStorage_MemPool, iPListDataAllocator>	_pointlistSSS;
typedef iPointList<float, DATA_NSLOTS_BAKE +DATA_RESULT_SLOT, iPListDataStorage_MemPool, iPListDataAllocator>	_pointlistBake;

typedef iPointList<float, DATA_NSLOTS_BAKE, iPListDataStorage_MemPool, iPListDataAllocator>			_pointlistGI;
typedef iPointList<float, DATA_NSLOTS_BAKE, iPListDataStorage_Simple, iPListDataAllocator>			iPointListSimple;
#endif


/// ///////////////////////////////////////////////////////////////////////////////////////////////////
typedef iOctree<float, _pointlistT, iOctreeNodeStorage_AO>	_octreeT;
typedef iOctree<float, _pointlistSSS, iOctreeNodeStorage_SSS>	_octreeSSS;
typedef iOctree<float, _pointlistGI, iOctreeNodeStorage_GI>	_octreeGI;

typedef _octreeT::Node		iOctreeNode;
typedef _octreeSSS::Node	iOctreeNodeSSS;
typedef _octreeGI::Node		iOctreeNodeGI;


/// ///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename IntegratorT, class PListType, class iOctreeType>
void microRasterize(	IntegratorT& integrator, 
			const iVector3& P, const iVector3& N, 
			float cosConeAngle, float sinConeAngle, float maxSolidAngle, float exactRAngle,
			PListType* pgi,  iOctreeType* ogi	)
{
	renderNode(	integrator, 
			P, N, 
			cosConeAngle, sinConeAngle,	maxSolidAngle, exactRAngle,
			DATA_NSLOTS_BAKE, 
			pgi, ogi );
}

/// ///////////////////////////////////////////////////////////////////////////////////////////////////
// Explicit instantiations //

// for Global Illumination
template void microRasterize< RadiosityIntegrator, _pointlistGI, iOctreeNodeGI >(
	RadiosityIntegrator&, 
	const iVector3&, const iVector3&, 
	float, float, float, float, 
	_pointlistGI*, iOctreeNodeGI* );

// for Ambient Occlusion
template void microRasterize< OcclusionIntegrator, _pointlistGI, iOctreeNodeGI >(
	OcclusionIntegrator&, 
	const iVector3&, const iVector3&, 
	float, float, float, float, 
	_pointlistGI*, iOctreeNodeGI* );


// Static stuff //////////////////////////////////////////////////////////////////////////////////////
static miBoolean is_GI_init = miFALSE;
static miBoolean is_GI_exit = miFALSE;

static _pointlistGI *gxPointlistGI	= NULL;
static _octreeGI *gxOctreeGI		= NULL;

#endif
