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
 * Module:	pcloud_Bake_AOcclusion
 * Purpose:	Point Based Ambient OcclusionBaker
 *
 * Exports:
 *	pcloud_Bake_AOcclusion
 *	pcloud_Bake_AOcclusion_version
 *	pcloud_Bake_AOcclusion_init
 *	pcloud_Bake_AOcclusion_exit
 *
 * History:
 *	01.07.2010: Created
 *
 * Description:
 * pcloud_Bake() is a lightmap shader that gets called in two circumstances:
 *
 * When state->type == miRAY_LM_VERTEX, it samples vertex data :
 * position, normal, texture coordinates, and stores in a result buffer.
 *
 * When state->type == miRAY_LM_MESH it operates on a mesh.
 * For each triangle in the mesh :
 * - it finds the triangle barycenter
 * - and for each bary point, it samples a shader,
 *	 finally writes into a mr map.
 *
 * TODO:
 * - not so effective but maybe implement AO passes, as in pcloud_Bake_Occlusion.cpp ?
 */

#include "_cpc_mentalray.h"



/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parameter type for pcloud_bake_AOcclusion ////////////////////////////////////////////////////////////
typedef struct pcloud_bake_AOcclusion_param
{
	miBoolean		enable;
	miInteger		verbose;

	miTag			map_file;				// filepath where to save pcloud map
	miTag			sample_irrad;			// sampling irradiance

	miScalar		maxsolidangle;			// tolerance
	miScalar		distAtt;
	miBoolean		trivis;
	miInteger		npasses;

	miInteger		nthreads;				// number of threads
	miBoolean		skip_ao;

	miScalar		splat_length;
	miInteger		splat_maxrec;

} pcloud_bake_AOcclusion_param;

// Cache structure for the lmap shader ///////////////////////////////////////////////////////////////////
struct ctrl_ao_cache
{
	miInteger		verbose;

	miTag			irrad_tag;
	miScalar		maxsolidangle;
	miScalar		distAtt;
	miBoolean		trivis;

	miInteger		npasses;
	miInteger		current_pass;
	
	miInteger		nthreads;

	char			*map_file_name;
	miBoolean		enable;

	miInteger		it_o;
	miBoolean		geoonly;
	miBoolean		skip_ao;

	miScalar		splat_length;
	miInteger		splat_maxrec;
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////
#if	__SSE2_ENHANCED__
typedef iSurfelCache<float, DATA_NSLOTS_AO/*DATA_NSLOTS_AO_TRIVIS*/, ctrl_ao_cache, iSurfelBaker_Reyes/*iSurfelSplatter*/, iSurfelAllocatorSSE> _iSCacheAO;
#else
typedef iSurfelCache<float, DATA_NSLOTS_AO, ctrl_ao_cache, iSurfelBaker_AO, iSurfelAllocator> _iSCacheAO;
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Init function /////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT int	pcloud_bake_AOcclusion_version(void) {return(2);}
extern "C" DLLEXPORT void	pcloud_bake_AOcclusion_init
(
	miState							* state,
	pcloud_bake_AOcclusion_param	* param,
	miBoolean						* inst_init_req
)
{ 

	if(!param) *inst_init_req = miTRUE;
	else
	{
		// ** Create shader cache
		ctrl_ao_cache* cache =	(ctrl_ao_cache*)
								mi_mem_allocate(sizeof(ctrl_ao_cache));

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
		if(!cache->irrad_tag)
		{
			cache->irrad_tag = miNULLTAG;
			cache->geoonly = miTRUE;
		}else
		{
			cache->geoonly = miFALSE;
		}

		if (cache->verbose)
		{
			mi_info ( "-----------------------------------------------" );
			mi_info ( "--- Creating AO point cloud"						);
			mi_info ( "-----------------------------------------------" );
		}

		// threshold/tolerance
		cache->maxsolidangle	= *mi_eval_scalar(&param->maxsolidangle);
		cache->distAtt			= *mi_eval_scalar(&param->distAtt);
		cache->trivis			= *mi_eval_boolean(&param->trivis);
		cache->npasses			= *mi_eval_integer( &param->npasses);
		cache->skip_ao			= *mi_eval_integer(&param->skip_ao);


		cache->splat_length		= *mi_eval_scalar(&param->splat_length);
		cache->splat_maxrec		= *mi_eval_integer(&param->splat_maxrec);

		// number of proc threads
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
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Exit //////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT void	pcloud_bake_AOcclusion_exit
(
miState							* state,
pcloud_bake_AOcclusion_param	* param
)
{
	if(!param) return;

	// * Get shader cache
	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	ctrl_ao_cache* cache = static_cast< ctrl_ao_cache* >(*user);

	if(!cache->enable){
		mi_mem_release( *user ); return;					// return early //////////////////////////////
	}

	// Pointlist/Octree //////////////////////////////////////////////////////////////////////////////////	
	_pointlistT		_pointlist;		// pointlist
	_iSCacheAO		_surfelCache;	// surfellist


	// Layout definition for pointlist ///////////////////////////////////////////////////////////////////
	typedef AmbientOcclusion_Impl< float, _pointlistT, iOctreeNode, ctrl_ao_cache > _AOT;
	_AOT::LayoutPointlistData(&_pointlist);


	// Transfer surfels to pointlist /////////////////////////////////////////////////////////////////////
	if (cache->verbose) mi_info("Pass 2: Cooking data from cache\n");

	_surfelCache.SetMIState( state );
	char* omap_file_name = cpc_util_mi::GetCharFromMITag( *mi_eval_tag ( &param->map_file ) );

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	if( cache->skip_ao )
	{	
	// save just the mimap and do AO computation at shading time
	_pointlist.TransferMISurfelDataToMiMap( &_surfelCache, omap_file_name, true, /*true*/false, cache->verbose );
	//_pointlist.TransferMISurfelDataToPTC( &_surfelCache, omap_file_name, cache->verbose );
	}
	else // pre-baking AO ////////////////////////////////////////////////////////////////////////////////
	{
	_pointlist.TransferMISurfelData( &_surfelCache, cache->verbose );

	iUint ntotalpoints = _pointlist.GetTotalNodes();
	if(cache->verbose)  mi_info("Pass 3: Creating pointlist\n");
	_pointlist.PrintDataInfo(cache->verbose);

	
	/// test spacefillingcurves
	//vector<float*> * plVector = _pointlist.GetStorageVector();
	//iMorton<float*> lt;
	//sort(plVector->begin(), plVector->end(), lt);			// plist morton..izing


	// Octree building ///////////////////////////////////////////////////////////////////////////////////
	if (cache->verbose)  mi_info("Pass 4: Creating octree\n");

	_octreeT		_octree;		// octree
	_octree.SplitOctree( &_pointlist );						// split octree
	_octree.PrintDataInfo(cache->verbose);
	
	iOctreeNode * _octroot = _octree.GetRoot();				// octree root

	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
    // Compute AOcclusion ////////////////////////////////////////////////////////////////////////////////
	if(cache->verbose) mi_info("Pass 5: Computing AOcclusion \n");
	if(cache->verbose>=2 && cache->npasses > 1) mi_info("Occlusion pass<1> :");


	// Win32 threads /////////////////////////////////////////////////////////////////////////////////////
	Thread_CG_Effects_Wrapper<_AOT> ThreadManager;
	bool ABORT_CALLED = ThreadManager.StartMultiThreadComputing(&_pointlist, _octroot, cache);

	if( ! ABORT_CALLED ){
		ThreadManager.ThreadedMultiPass(&_pointlist, _octroot, cache, ABORT_CALLED);
	}else{
		mi_mem_release(*user);
		return;
	}

	// Begin freeing up stuff not needed anymore
	_octree.Clear();



	//////////////////////////////////////////////////////////////////////////////////////////////////////
    // Write out a point cloud file with ambient occlusion data //////////////////////////////////////////
	if (cache->verbose) mi_info("Pass 5: Writing pcloud with AO data ...\n", ntotalpoints);
	_pointlist.WriteMIMapDataMRay( omap_file_name, true, false, cache->verbose );	// write mimap ///////
	
	// Release resources /////////////////////////////////////////////////////////////////////////////////
	_pointlist.FreeStorage();					// Clear Plist

	} //end .. skip AO computation

	mi_mem_release(*user);						// Clear Shader Cache
};




//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation ////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT miBoolean 
pcloud_bake_AOcclusion
(
	iSurfel_MI_bake_result			*result,
	miState							*state,
	pcloud_bake_AOcclusion_param	*param,
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
			result->tex.x = result->tex.y = result->tex.z = 0.0f;	// coords (not used?)
			break;
		}

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	case miRAY_LM_MESH:
		{
			if (!arg) return(miFALSE);


			// * Get shader cache
			void **user;
			mi_query(miQ_FUNC_USERPTR, state, 0, &user);
			ctrl_ao_cache* cache = static_cast< ctrl_ao_cache* >(*user);
			if(!cache) return miFALSE;


			// * Init surfel data per-thread
			typedef _iSCacheAO::_DTYPE DataType;

			_iSCacheAO	iTLScache;
			DataType* returnData;

			iTLScache.SetMIState( state );
			iTLScache.Init_MI_TLS_Cache();


			// * Get vertex data
			_iSCacheAO::iSBaker::pcloud_MI_bake_result *resdata = (_iSCacheAO::iSBaker::pcloud_MI_bake_result*)arg->vertex_data;
			if(!resdata) return miFALSE;

			// Fill vmap /////////////////////////////////////////////////////////////////////////////////
			if(cache->verbose && !cache->it_o)
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


				/// reyes splatting
				iTLScache.SplatTri(	&resdata[arg->triangles[i].a],
									&resdata[arg->triangles[i].b],
									&resdata[arg->triangles[i].c],
									state,
									-1,
									cache->splat_length,
									cache->splat_maxrec
									);


				for(int x=0; x < iTLScache._tridata.vpointlist.size(); x++)
				iTLScache.PushData( iTLScache._tridata.vpointlist[x] );

				iTLScache._tridata.vpointlist.clear();


				///diadic splatting
				/*for(int x=0; x < iTLScache._tridata.vpointlist.size(); x+=3)
				{
					returnData = iTLScache.Bake( iTLScache._tridata.vpointlist[x+0], 
												 iTLScache._tridata.vpointlist[x+1], 
												 iTLScache._tridata.vpointlist[x+2]);

					iTLScache.PushData(returnData);
				}
				iTLScache._tridata.vpointlist.clear();
				*/

				/// no custom splat, bake the triangle
				/*returnData = iTLScache.Bake(	&resdata[arg->triangles[i].a].point, 
												&resdata[arg->triangles[i].b].point, 
												&resdata[arg->triangles[i].c].point	 );
				returnData = iTLScache.Bake(&resdata[arg->triangles[i].a], 
											&resdata[arg->triangles[i].b], 
											&resdata[arg->triangles[i].c]);
				*/
				//iTLScache.PushData(returnData);

				cache->it_o ++;
			}

			break;	//end case label
		}
	}
	return(miTRUE);
}

