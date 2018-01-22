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

 * Created:	13.11.10
 * Module:	pcloud_Bake
 * Purpose:	Point Based Baker
 *
 * Exports:
 *	pcloud_Bake
 *	pcloud_Bake_version
 *	pcloud_Bake_init
 *	pcloud_Bake_exit
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
 */


#include "_cpc_mentalray.h"


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// pcloud_Bake

// result data type for pcloud_Bake
typedef struct pcloud_Bake_result
{
	miVector	point;		// point in space 
	miVector	normal;		// vertex normal 
	miVector	tex;		// texture coordinates of vertex
	miColor		irrad;		// irrad on vertex (not used)

} pcloud_Bake_result;


// parameter type for pcloud_Bake
typedef struct pcloud_Bake_param
{
	miBoolean	enable;
	miInteger	verbose;

	miTag		sample_sh;	// sampling shader

	miTag		map_attr;	// map name attribute
	miTag		map_file;	// filepath for pcloud map
	miInteger	map_type;	// map type

	miBoolean	backfacing;	// consider backfacing
	miInteger	fgprebake;	// FG prebaking quality

	miScalar	splat_length;
	miInteger	splat_maxrec;

} pcloud_Bake_param;

// cache structure for the lmap shader
struct ctrl_lightmap_cache
{
	miInteger	verbose;

	miTag		irrad_tag;

	char*		map_attr_name;
	char*		map_file_name;
	miInteger	map_type;

	miBoolean	enable; 
	miBoolean	backfacing;
	miInteger	fgprebake;

	miInteger	it_o;

	miScalar	splat_length;
	miInteger	splat_maxrec;
};

/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if	__SSE2_ENHANCED__
typedef iSurfelCache<float, DATA_NSLOTS_BAKE, ctrl_lightmap_cache, iSurfelBaker_Reyes/*iSurfelBaker_AttrSplatter*/, iSurfelAllocatorSSE>	_iSCacheBake;
#else
typedef iSurfelCache<float, DATA_NSLOTS_BAKE, ctrl_lightmap_cache, iSurfelBaker_Attr, iSurfelAllocator>		_iSCacheBake;
#endif


/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT int	pcloud_Bake_version(void) {return(2);}
extern "C" DLLEXPORT void	pcloud_Bake_init
(
	miState				* state,
	pcloud_Bake_param	* param,
	miBoolean			*inst_init_req
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


		// evaluates sample shader tag
		cache->irrad_tag = *mi_eval_tag(&param->sample_sh);
		if(!cache->irrad_tag){
		
			cache->enable = miFALSE;
			return;
		}

		if (cache->verbose)
		{
			mi_info ( "-----------------------------------------------" );
			mi_info ( "ROMBO::  <Building point cloud>" );
			mi_info ( "-----------------------------------------------" );
		}

		// evaluates the tag of the attribute string
		miTag map_attr_name_tag = *mi_eval_tag ( &param->map_attr );
		cache->map_attr_name = ( char * ) mi_db_access ( map_attr_name_tag );
		mi_db_unpin ( map_attr_name_tag );

		// evaluates the tag of the map filename
		miTag map_file_name_tag = *mi_eval_tag ( &param->map_file );
		cache->map_file_name = ( char * ) mi_db_access ( map_file_name_tag );
		mi_db_unpin ( map_file_name_tag );

		cache->map_type = *mi_eval_integer(&param->map_type);	///TEMP:FOR:OPEN:CLOSED:BINARY:MIMAPS
		
		cache->backfacing = *mi_eval_boolean(&param->backfacing);
		cache->fgprebake = *mi_eval_integer(&param->fgprebake);

		cache->splat_length		= (*mi_eval_scalar(&param->splat_length));
		cache->splat_maxrec		= *mi_eval_integer(&param->splat_maxrec);


		cache->it_o = 0;		//init iterator for progresses

		if(cache->verbose && state->options->finalgather)
		{
			mi_info("ROMBO::PCLOUD::BAKE::INIT::  <Pre-baking FinalGather>\n");
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT void	pcloud_Bake_exit
(
	miState				* state,
	pcloud_Bake_param	* param
)
{
	if(!param) return;

	// * Get shader cache
	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	ctrl_lightmap_cache* cache = static_cast< ctrl_lightmap_cache* >(*user);

	if(!cache->enable)
	{
		mi_mem_release( *user );
		return;	// return early
	}


	// * Point cloud
	if (cache->verbose)
	{
		mi_info("---\n");
		mi_info("ROMBO::PCLOUD::BAKE::EXIT::  <Writing point cloud map>\n");
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	_pointlistBake	_pointlist;		// pointlist
	_iSCacheBake	_surfelCache;	// surfellist

	//add datalayout
	typedef _pointlistBake::_iDataFlag _iDataFlagT;
	_iDataFlagT newflag;	

	newflag.name = "normal";
	newflag.type = 3;
	newflag.offset = DATA_SLOT_DIR;
	_pointlist.AddLocalField(newflag);

#if	__SSE2_ENHANCED__
	newflag.name = "radius";
	newflag.type = 3;
	newflag.offset = DATA_SLOT_AREA;
	_pointlist.AddLocalField(newflag);
#else
	newflag.name = "radius";
	newflag.type = 2;
	newflag.offset = DATA_SLOT_AREA;
	_pointlist.AddLocalField(newflag);
#endif

	newflag.name = cache->map_attr_name;
	newflag.type = 4;
	newflag.offset = DATA_SLOT_BATTR;
	_pointlist.AddLocalField(newflag);

	newflag.name = "maptype";
	newflag.type = 1;
	newflag.offset = -112233;			//occlusion integer identifier
	_pointlist.AddGlobalField(newflag);


	//////////////////////////////////////////////////////////////////////////////////////////////////////
	char* omap_file_name = cpc_util_mi::GetCharFromMITag( *mi_eval_tag ( &param->map_file ) );

	_surfelCache.SetMIState( state );

	string map_ext (cpc_util_mi::GetFileExtension( string(omap_file_name) ));
	if(map_ext.compare("ptc")==0){
	// transfer to PTC
	//_pointlist.TransferMISurfelDataToPTC( &_surfelCache, omap_file_name, cache->verbose );
	}else{
	// transfer to mimap
	_pointlist.TransferMISurfelDataToMiMap( &_surfelCache, omap_file_name, true, (bool)cache->map_type, cache->verbose );
	}

	// delete user cache
	mi_mem_release( *user );
}



/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" DLLEXPORT miBoolean 
pcloud_Bake
(
	iSurfel_MI_bake_result			*result,
	miState							*state,
	pcloud_Bake_param				*param,
	miRclm_mesh_render const		*arg	// 4th argument
)	
{

	miBoolean enable = *mi_eval_boolean(&param->enable);
	if(!enable) return(miTRUE);				//return if not enabled


	switch (state->type)
	{
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	case miRAY_LM_VERTEX:					// Gather vertex data
		{

			// *Gathering vtx pos and dir
			result->point  = state->point;							// position
			result->normal = state->normal;
			mi_vector_normalize(&result->normal);					// normal
			result->tex.x = result->tex.y = result->tex.z = 0.0f;	// coords

			// eval this directly to skip cache retriving ..
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
			typedef _iSCacheBake::_DTYPE DataType;

			_iSCacheBake	iTLScache;
			DataType*		returnData;

			iTLScache.SetMIState( state );
			iTLScache.Init_MI_TLS_Cache();


			// * Get vertex data
			_iSCacheBake::iSBaker::pcloud_MI_bake_result *resdata = (_iSCacheBake::iSBaker::pcloud_MI_bake_result*)arg->vertex_data;
			if(!resdata) return miFALSE;


			// FG prepass/////////////////////////////////////////////////////////////////////////////////
			if(cache->fgprebake && state->options->finalgather)
			{
				miRay_type old_type = state->type;
				state->type = miRAY_LM_VERTEX;

				for (int i=0; i < arg->no_triangles; i++)			// scan triangles
				{					
					if (mi_par_aborted()) {

						cache->enable = miFALSE;
						break;
					}

					// FG baking every n. vertex
					if(i%cache->fgprebake == 0)
					{
						mi_state_set_pri(state, arg->pri, arg->triangles[i].pri_idx);

						iTLScache.SplatTri(	&resdata[arg->triangles[i].a],
											&resdata[arg->triangles[i].b],
											&resdata[arg->triangles[i].c],
											state, 
											i,
											cache->splat_length,
											cache->splat_maxrec
											);

						iTLScache.Bake( cache, state, miTRUE );
						iTLScache._tridata.vpointlist.clear();
					}
				}

				state->type = old_type;
			}

			// Fill vmap /////////////////////////////////////////////////////////////////////////////////
			if(!cache->it_o) mi_info("ROMBO::PCLOUD::BAKE::  <Begin baking>\n");

			for (int i=0; i < arg->no_triangles; i++)				// scan triangles ////////
			{					
				if (mi_par_aborted()) {
					cache->enable = miFALSE;
					break;
				}

				if(cache->verbose>=2 && (cache->it_o % (arg->no_triangles/4) == 0) )
				mi_info("--- point: %i \n", cache->it_o);

				mi_state_set_pri(state, arg->pri, arg->triangles[i].pri_idx);

				
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
				
				/*
				///diadic splatting
				iTLScache.SplatTri(	&resdata[arg->triangles[i].a], 
									&resdata[arg->triangles[i].b], 
									&resdata[arg->triangles[i].c],
									cache->splat_length,
									cache->splat_maxrec,
									state);

				
				for(int x=0; x < iTLScache._tridata.vpointlist.size(); x+=3)
				{
					returnData = iTLScache.Bake(	i,
													iTLScache._tridata.vpointlist[x+0], 
													iTLScache._tridata.vpointlist[x+1], 
													iTLScache._tridata.vpointlist[x+2],
													cache, 
													state );

					iTLScache.PushData(returnData);
				}
				iTLScache._tridata.vpointlist.clear();
				*/

				/// no custom splat, bake the triangle				
				/*returnData = iTLScache.Bake(	i,
												&resdata[arg->triangles[i].a], 
												&resdata[arg->triangles[i].b], 
												&resdata[arg->triangles[i].c],
												cache, 
												state );
				iTLScache.PushData(returnData);*/
				
				cache->it_o ++;
			}

			break;
		}
	}
	return(miTRUE);
}
