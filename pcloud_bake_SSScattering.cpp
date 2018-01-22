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
 
 *****************************************************************************

 * Created:	13.11.00
 * Module:	pcloud_Bake_SSScattering
 * Purpose:	Point Based Baker
 *
 * Exports:
 *	pcloud_bake_SSScattering
 *	pcloud_bake_SSScattering_init
 *	pcloud_bake_SSScattering_exit
 *	pcloud_bake_SSScattering_version
 *
 */

#include "_cpc_mentalray.h"



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// parameter type for pcloud_bake_SSScattering ///////////////////////////////////////////////////////////
typedef struct pcloud_bake_SSScattering_param
{
	miBoolean		enable;
	miInteger		verbose;

	miTag			map_file;				// filepath where to save SSS pcloud map

	miTag			sample_irrad;			// sampling irradiance

	miColor			diffuse_albedo;			// SSS parameters:
	miTag			sample_albedo;			// sampling albedo
	miColor			diffuse_meanfreepath;
	miTag			sample_mfpath;			// sampling meanfpath

	miScalar		unitlength;				// multiplier on dmfp
	miScalar		ior;
	miScalar		maxsolidangle;			// tolerance

	miScalar		diffuseblend;			// diffuse blending
	miBoolean		backfacing;				// consider backfacing
	miInteger		fgprebake;				// FG prebaking quality

	miInteger		nthreads;				// number of threads
	miBoolean		skip_sss;				// bake only

	miScalar		splat_length;
	miInteger		splat_maxrec;

} pcloud_bake_SSScattering_param;

// cache structure for the lmap shader ///////////////////////////////////////////////////////////////////
struct ctrl_lightmap_cache
{
	miBoolean		enable;
	miInteger		verbose;
	miInteger		nthreads;
	miInteger		it_o;
	miBoolean		skip_sss;

	miTag			irrad_tag;
	miTag			albedo_tag;
	miTag			mfpath_tag;

	miColor			albedo;
	miColor			mfpath;

	miScalar		unitlength;
	miScalar		ior;
	miScalar		maxsolidangle;

	miScalar		diffuseblend;
	miBoolean		backfacing;
	miInteger		fgprebake;

	float			A;
	float			*alphatable;

	char			*map_file_name;
	
	miScalar		splat_length;
	miInteger		splat_maxrec;
};

#define REYESSPLATTINGXXX
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#if	__SSE2_ENHANCED__
	#ifdef REYESSPLATTING
		typedef iSurfelCache<float, DATA_NSLOTS_SSS, ctrl_lightmap_cache, iSurfelBaker_ReyesSSS, iSurfelAllocatorSSE> _iSCacheSSS;
	#else
		typedef iSurfelCache<float, DATA_NSLOTS_SSS, ctrl_lightmap_cache, iSurfelBaker_SSS, iSurfelAllocatorSSE> _iSCacheSSS;
	#endif
#else
	typedef iSurfelCache<float, DATA_NSLOTS_SSS, ctrl_lightmap_cache, iSurfelBaker_SSS, iSurfelAllocator> _iSCacheSSS;
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT int	pcloud_bake_SSScattering_version(void) {return(2);}
extern "C" DLLEXPORT void	pcloud_bake_SSScattering_init
(
	miState							* state,
	pcloud_bake_SSScattering_param	* param,
	miBoolean						* inst_init_req
)
{ 

	if(!param) *inst_init_req = miTRUE;
	else
	{
		// ** Create shader cache
		ctrl_lightmap_cache* cache =	(ctrl_lightmap_cache*)
										mi_mem_allocate(sizeof(ctrl_lightmap_cache));

		void **user;
		mi_query(miQ_FUNC_USERPTR, state, 0, &user);
		*user = cache;

		cache->enable = *mi_eval_boolean(&param->enable);
		if(!cache->enable) return;	//return if not enabled

		cache->verbose = *mi_eval_integer( &param->verbose );


		// evaluate the tag of the map file name
		miTag map_file_name_tag = *mi_eval_tag ( &param->map_file );
		cache->map_file_name = ( char * ) mi_db_access ( map_file_name_tag );
		mi_db_unpin ( map_file_name_tag );

		// evaluate irrad shader tag
		cache->irrad_tag = *mi_eval_tag(&param->sample_irrad);
		if(!cache->irrad_tag){

			mi_error("Bake SSScattering: No irradiance shader! Aborting..");
			cache->irrad_tag = miNULLTAG;
			cache->enable = miFALSE;
			return;
		}

		if (cache->verbose)
		{
			mi_info ( "-----------------------------------------------" );
			mi_info ( "--- Creating SSS point cloud" );
			mi_info ( "-----------------------------------------------" );
		}

		// evaluate albedo
		//miColor nullcol = {0,0,0,1};
		cache->albedo_tag = *mi_eval_tag(&param->sample_albedo);
		cache->albedo = *mi_eval_color(&param->diffuse_albedo);
		if(!cache->albedo_tag)
		{
			cache->albedo_tag = miNULLTAG;	// if attached a tex.. cache tag
		}

		// evaluate meanfreepath
		cache->mfpath_tag = *mi_eval_tag(&param->sample_mfpath);
		cache->mfpath = *mi_eval_color(&param->diffuse_meanfreepath);
		if(!cache->mfpath_tag)
		{
			cache->mfpath_tag = miNULLTAG;
		}

		// remaing sss parameters
		cache->unitlength = *mi_eval_scalar(&param->unitlength);
		cache->ior = *mi_eval_scalar(&param->ior);
		cache->maxsolidangle = *mi_eval_scalar(&param->maxsolidangle);

		cache->diffuseblend = 1.0f - *mi_eval_scalar(&param->diffuseblend);
		cache->backfacing = *mi_eval_boolean(&param->backfacing);
		cache->fgprebake = *mi_eval_boolean(&param->fgprebake);
		cache->skip_sss = *mi_eval_boolean(&param->skip_sss);

		cache->nthreads = *mi_eval_integer(&param->nthreads);
		if(!cache->nthreads){

			SYSTEM_INFO sysinfo; 
			GetSystemInfo( &sysinfo ); 

			cache->nthreads = sysinfo.dwNumberOfProcessors;
			if(cache->nthreads<=1){
				mi_error("Application requires multi-processor architecture ATM. Aborting ..");
				cache->enable = miFALSE;
				return;
			}

			if(cache->verbose>=3) mi_info("Detected <%i> cores.", cache->nthreads);
		}

		cache->it_o = 0;		//init iterator for progresses
		if(cache->verbose && state->options->finalgather)
		{
			mi_info("Pass 0: Pre-baking FinalGather\n");
		}

		// REYES splatting
		cache->splat_length = *mi_eval_scalar(&param->splat_length);
		cache->splat_maxrec = *mi_eval_integer(&param->splat_maxrec);
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT void	pcloud_bake_SSScattering_exit
(
	miState							* state,
	pcloud_bake_SSScattering_param	* param
)
{
	if(!param) return;

	// * Get shader cache
	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	ctrl_lightmap_cache* cache = static_cast< ctrl_lightmap_cache* >(*user);

	if(!cache->enable){
		mi_mem_release( *user ); return;	// return early !!!!
	}


	// Pointlist/Octree //////////////////////////////////////////////////////////////////////////////////
	_pointlistSSS	_pointlist;		// pointlist
	_iSCacheSSS		_surfelCache;	// surfellist


	// Layout definition for pointlist ///////////////////////////////////////////////////////////////////
	typedef SubSurfaceScattering_Impl< float, _pointlistSSS, iOctreeNodeSSS, ctrl_lightmap_cache > _SSST;
	_SSST::LayoutPointlistData(&_pointlist);


	// Transfer surfels to pointlist /////////////////////////////////////////////////////////////////////
	if (cache->verbose) mi_info("Pass 2: Cooking data from cache\n");

	_surfelCache.SetMIState( state );
	char* omap_file_name = cpc_util_mi::GetCharFromMITag( *mi_eval_tag ( &param->map_file ) );

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	if( cache->skip_sss )
	{	
		// save just the mimap and do AO computation at shading time
		_pointlist.TransferMISurfelDataToMiMap( &_surfelCache, omap_file_name, true, false, cache->verbose );
		//_pointlist.TransferMISurfelDataToMiMap( &_surfelCache, omap_file_name, false, true, cache->verbose );

	}
	else // pre-baking AO ////////////////////////////////////////////////////////////////////////////////
	{
		_pointlist.TransferMISurfelData( &_surfelCache, cache->verbose );

		unsigned int ntotalpoints = _pointlist.GetTotalNodes();
		if(cache->verbose)  mi_info("Pass 3: Creating pointlist\n");
		_pointlist.PrintDataInfo(cache->verbose);
		if(cache->verbose>=3) _pointlist.DumpData();

		// Octree building ///////////////////////////////////////////////////////////////////////////////////
		if (cache->verbose)  mi_info("Pass 3b: Creating octree\n");

		_octreeSSS		_octree;							// octree
		_octree.SplitOctree( &_pointlist );					// split octree
		_octree.PrintDataInfo(cache->verbose);
	
		iOctreeNodeSSS * _octroot = _octree.GetRoot();		// octree root


		// Init SSS stuff ////////////////////////////////////////////////////////////////////////////////////
		cache->alphatable = (float*)mi_mem_allocate(sizeof(float)*(steps+1));	
		float A = _SSST::GetAlphaForTable( cache->ior );
		_SSST::FillAlphaPrimeTable(cache->alphatable, A);	// build Alpha prime table ///////////////////////
		cache->A = 1.0f + 1.333333f * A;

		// Multithreaded SSS /////////////////////////////////////////////////////////////////////////////////
		Thread_CG_Effects_Wrapper<_SSST> ThreadManager;
		bool ABORT_CALLED = ThreadManager.StartMultiThreadComputing(&_pointlist, _octroot, cache);
		if( ABORT_CALLED )
		{
			mi_mem_release(cache->alphatable);
			mi_mem_release(*user);
			return;
		}

		// Begin freeing up stuff
		mi_mem_release(cache->alphatable);
		_octree.Clear();

		//////////////////////////////////////////////////////////////////////////////////////////////////////
		// Write out a point cloud file with ambient occlusion data //////////////////////////////////////////
		if (cache->verbose) mi_info("Pass 5: Writing pcloud with AO data ...\n", ntotalpoints);
		_pointlist.WriteMIMapDataMRay( omap_file_name, true, false, cache->verbose );	// write mimap ///////
	
		// Release lst resources /////////////////////////////////////////////////////////////////////////////
		_pointlist.FreeStorage();		// clear pointlist

	}	// end skip SSS computation /////////////////////////////////////////////////////////////////// **

	mi_mem_release(*user);			// clear user cache
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT miBoolean 
pcloud_bake_SSScattering
(
	iSurfel_MI_bake_result			*result,
	miState							*state,
	pcloud_bake_SSScattering_param	*param,
	miRclm_mesh_render const		*arg	// 4th argument
)	
{

	miBoolean enable = *mi_eval_boolean(&param->enable);
	if(!enable) return(miTRUE);	//return if not enabled


	switch (state->type)
	{
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	case miRAY_LM_VERTEX:		// Gather vertex data
		{

			// *Gathering vtx pos and dir
			result->point  = state->point;							// position
			result->normal = state->normal;
			mi_vector_normalize(&result->normal);					// normal
			result->tex.x = result->tex.y = result->tex.z = 0.0f;	// coords


			miInteger fgbrebaking = *mi_eval_integer(&param->fgprebake);
			if(!fgbrebaking)	// force FG at every vertex
			{
				// FG preprocessing.. place a FG point at every vertex
				miColor tempvalue;
				mi_finalgather_store(&tempvalue, state, miFG_STORE_COMPUTE);
			}
			break;
		}

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	case miRAY_LM_MESH:
		{
			if (!arg)
				return(miFALSE);

			// * Get shader cache
			void **user;
			mi_query(miQ_FUNC_USERPTR, state, 0, &user);
			ctrl_lightmap_cache* cache = static_cast< ctrl_lightmap_cache* >(*user);
			if(!cache) return miFALSE;


			// * Init surfel data per-thread /////////////////////////////////////////////////////////////
			typedef _iSCacheSSS::_DTYPE DataType;

			_iSCacheSSS	iTLScache;
			DataType* returnData;

			iTLScache.SetMIState( state );
			iTLScache.Init_MI_TLS_Cache();


			// * Get vertex data
			_iSCacheSSS::iSBaker::pcloud_MI_bake_result *resdata = (_iSCacheSSS::iSBaker::pcloud_MI_bake_result*)arg->vertex_data;
			if(!resdata) return miFALSE;

			// FG prepass/////////////////////////////////////////////////////////////////////////////////
			if(cache->fgprebake && state->options->finalgather)
			{
				miRay_type old_type = state->type;
				state->type = miRAY_LM_VERTEX;

				for (int i=0; i < arg->no_triangles; i++)						// scan triangles
				{					
					if (mi_par_aborted()) {

						cache->enable = miFALSE;
						break;
					}

					// FG baking every n. vertex
					if(i%cache->fgprebake == 0)
					{
						mi_state_set_pri(state, arg->pri, arg->triangles[i].pri_idx);
						
#ifdef REYESSPLATTING
						/// reyes splatting
						iTLScache.SplatTri(	&resdata[arg->triangles[i].a],
											&resdata[arg->triangles[i].b],
											&resdata[arg->triangles[i].c],
											state, 
											i,
											cache->splat_length,
											cache->splat_maxrec
											);

						iTLScache.Bake( cache, state, miTRUE );

						for(int x=0; x < iTLScache._tridata.vpointlist.size(); x++)
						iTLScache.PushData( iTLScache._tridata.vpointlist[x] );

						iTLScache._tridata.vpointlist.clear(); 
#else
						iTLScache.Bake(	i,
										&resdata[arg->triangles[i].a], 
										&resdata[arg->triangles[i].b],
										&resdata[arg->triangles[i].c],
										cache,
										state, 
										miTRUE	);								//enable fg prebaking
#endif
					}
				}
				state->type = old_type;
			}

			// Fill vmap /////////////////////////////////////////////////////////////////////////////////
			if(!cache->it_o)
				mi_info("Pass 1: Begin baking ... \n");

			for (int i=0; i < arg->no_triangles; i++)							// scan triangles ////////
			{					
				if (mi_par_aborted()) {
					cache->enable = miFALSE;
					break;
				}				
			
				if(cache->verbose>=2 && (cache->it_o % (arg->no_triangles/4) == 0) )
				mi_info("--- baking point: %i \n", cache->it_o);


				mi_state_set_pri(state, arg->pri, arg->triangles[i].pri_idx);

				
#ifdef REYESSPLATTING			
				/// reyes splatting
				iTLScache.SplatTri(	&resdata[arg->triangles[i].a],
									&resdata[arg->triangles[i].b],
									&resdata[arg->triangles[i].c],
									state, 
									i,
									cache->splat_length,
									cache->splat_maxrec
									);

				iTLScache.Bake( cache, state );

				for(int x=0; x < iTLScache._tridata.vpointlist.size(); x++)
				iTLScache.PushData( iTLScache._tridata.vpointlist[x] );

				iTLScache._tridata.vpointlist.clear();

#else
				/// no custom splat, bake the triangle				
				returnData = iTLScache.Bake(	i,
												&resdata[arg->triangles[i].a], 
												&resdata[arg->triangles[i].b], 
												&resdata[arg->triangles[i].c],
												cache, 
												state );
				iTLScache.PushData(returnData);
#endif

				cache->it_o ++;
			}

			break;
		}
	}
	return(miTRUE);
}
