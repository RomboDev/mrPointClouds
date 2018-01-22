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

#ifndef __PCLOUD_GLOBALILLUMINATION__
#define __PCLOUD_GLOBALILLUMINATION__


namespace cpc
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Ambient Occlusion ///////////////////////////////////////////////////////////////////////////////
template<typename DataType, class PListType, class iOctreeNodeType, class iCache>
class GlobalIllumination_Impl
{

	enum{ 
		NODESIZE	= PListType::NODESIZE,
		GISLOT		= PListType::NODESIZE -DATA_RESULT_SLOT,
	};
	

	
/// ////////////////////////////////////////////////////////////////////////////////////////////////
public:
	typedef PListType			PListType;
	typedef iOctreeNodeType		iOctreeNodeType;
	typedef iCache				iCache;

	static const bool HasMultiPass = false;



	static bool
	doThread_CG_EffectsPass(chunk_idx		*chunk,

							PListType		* pointlist,
							iOctreeNodeType	* root,

							iCache			* cache,

							bool			isMainThread = false )
	{

		// radiosity integrator
		RadiosityIntegrator integrator( cache->buffer_res );
		//const float& eps = 0.1;


		// loop over all shading points
		for (iUint o = chunk->startValue; o < chunk->endValue; o++)
		{
			if(isMainThread)		// abortation is ctrled by main thread ..
			if(ABORTCALLED)			// breaking from here, it will stop the other threads
			return true;

			iVector3 P		= pointlist->template GetNodeVectorAt<DATA_SLOT_POS>(o);
			iVector3 N		= pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>(o);
			float r			= pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>(o);
			iVector3 irrad	= pointlist->template GetNodeVectorAt<DATA_SLOT_IRRAD>(o);


			// compute radiosity
			integrator.clear();
			microRasterize(		integrator, 

								P /*+ N*r*eps*/, N, 
								
								cache->cosConeAngle, 
								cache->sinConeAngle, 
								cache->maxsolidangle, 
								cache->exactRenderAngle,

								pointlist, root
							);
			
			// result
			pointlist->template SetNodeVectorAt<GISLOT>( o, irrad + integrator.radiosity(N, f_pi_div2) );

			//if(cache->verbose>=2 && (o % chunk->frequency == 0) && isMainThread)
			//PRINTME( "--- GI at point %u = (%f, %f, %f)\n", o, irradPlusGI[0],irradPlusGI[1],irradPlusGI[2]);
	
		}

		return false;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	static void
	LayoutPointlistData( PListType * _pointlist )
	{

		typedef _pointlistBake::_iDataFlag _iDataFlagT;
		_iDataFlagT newflag;	

		// local fields
		newflag.name = "normal";
		newflag.type = 3;
		newflag.offset = DATA_SLOT_DIR;
		_pointlist->AddLocalField(newflag);

#if	__SSE2_ENHANCED__
		newflag.name = "radius";
		newflag.type = 3;
		newflag.offset = DATA_SLOT_AREA;
		_pointlist->AddLocalField(newflag);
#else
		newflag.name = "radius";
		newflag.type = 2;
		newflag.offset = DATA_SLOT_AREA;
		_pointlist->AddLocalField(newflag);
#endif

		newflag.name = "_radiosity";
		newflag.type = 4;
		newflag.offset = DATA_SLOT_BATTR;
		_pointlist->AddLocalField(newflag);

		// global fields
		newflag.name = "maptype";
		newflag.type = 1;
		newflag.offset = -112233;
		_pointlist->AddGlobalField(newflag);
	}

};	//end class
}	//end namespace
#endif __PCLOUD_AMBIENTOCCLUSION__