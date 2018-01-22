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
 *	pcloud_GI
 *	pcloud_GI_init
 *	pcloud_GI_exit
 *	pcloud_GI_version
 *
 */

#include "_cpc_mentalray.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////
// parameter struct for the shader
struct pcloud_GI_paras 
{
	miInteger		read_only;

	miColor			diffuse_albedo;
	miColor			diffuse_shader;

	miInteger		buffer_res;
	miScalar		micro_eps;
	miScalar		maxsolidangle;
	miScalar		exactRenderAngle;

	miInteger		points_in_lookup;		// points in lookup
	miScalar		multiply_factor;

	miColor			fg_bypass;				// fg rays don't work well with pclouds
	miInteger		verbose;				// verbose
	miInteger		nthreads;

	int             i_map;      
	int             n_map;
	miTag			bake_map;				// irrad map path
};

// cache structure for the shader
struct pcloud_GI_cache
{
	miTag			oMap;					//DB tag for storing mimap
	Map_field_id	omap_area_id;
	Map_field_id	omap_irrad_id;
	Map_field_id	omap_radiosity_id;
	
	miInteger		buffer_res;				// GI parameters
	miScalar		micro_eps;
	miScalar		maxsolidangle;
	miScalar		cosConeAngle;
	miScalar		sinConeAngle;
	miScalar		exactRenderAngle;

	miInteger		points_in_lookup;		// points in lookup
	miScalar		multiply_factor;


	miInteger		verbose;				// out controls
	miBoolean		isshaderball;
	miBoolean		disabled;

	miBoolean		enable;
	miInteger		read_only;				// eval at shading time
	miInteger		nthreads;

	_pointlistGI	*pointlist;
	_octreeGI		*octree;
};

typedef GlobalIllumination_Impl< float, _pointlistGI, iOctreeNodeGI, pcloud_GI_cache > _GIT;



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Shader Initialization
extern "C" DLLEXPORT	int	    pcloud_GI_version(void) {return(1);}
extern "C" DLLEXPORT	void	pcloud_GI_init (
	miState						*state,
	pcloud_GI_paras	*paras,
	miBoolean					*inst_init_req )
{
    if ( ! paras )
	{
		*inst_init_req = miTRUE; 	
		return;	// shader instance initialized
    }


	//////////////////////////////////////////////////////////////////////////////////////////////////
	// initialize cache ..
	pcloud_GI_cache* cache	= (pcloud_GI_cache*) mi_mem_allocate(sizeof(pcloud_GI_cache));

	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	*user = cache;


	// cache parameters
	cache->buffer_res		= *mi_eval_integer	( &paras->buffer_res );
	cache->micro_eps		= *mi_eval_scalar	( &paras->micro_eps );
	cache->maxsolidangle	= *mi_eval_scalar	( &paras->maxsolidangle	);
	cache->cosConeAngle		= acosf	( M_PI_2 );
	cache->sinConeAngle		= asinf	( M_PI_2 );
	cache->exactRenderAngle	= *mi_eval_scalar	( &paras->exactRenderAngle	);


	cache->points_in_lookup = *mi_eval_integer	( &paras->points_in_lookup );
	if(cache->points_in_lookup==0) cache->points_in_lookup=1;

	cache->multiply_factor	= *mi_eval_scalar	( &paras->multiply_factor );
	if(cache->multiply_factor==0) cache->multiply_factor=1;

	cache->verbose	= *mi_eval_boolean	( &paras->verbose );


	//////////////////////////////////////////////////////////////////////////////////////////////////
	// check for early exits
	cache->isshaderball = miFALSE;
	const miCamera * camera = state->camera;
	if( camera->focal == 1.0 || camera->x_resolution<320)
	{	// hypershader or shaderball 
		cache->isshaderball = miTRUE; return;
	}

	// check if in lightmap stage,  called during 2nd rays eval
	miStage stage;
	mi_query(miQ_STAGE, state, 0, &stage);
	if(stage == miSTAGE_LIGHTMAP)
	{
		cache->disabled = miTRUE; return;	//return earlier !!
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////
	if(cache->verbose){
	mi_info ( "-----------------------------------------------" );
	mi_info ( "ROMBO::  <Reading point cloud>" );
	mi_info ( "-----------------------------------------------" );
	}

	cache->disabled = miFALSE;

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// cache baked map as we have it already on disk, so return early
	cache->read_only = *mi_eval_integer(&paras->read_only);
	if(cache->read_only == 0)
	{
		/*
		// sss map tag
		miTag	omap_tag = *mi_eval_tag ( &paras->sss_map );
		char* omap_file_name = ( char * ) mi_db_access ( omap_tag );
		mi_db_unpin ( omap_tag );

		Access_map  sss_map ( omap_file_name );

		if(sss_map->is_empty()){ 
			mi_error ( "--- Map -> is empty !" );

			cache->disabled = miTRUE;
			return;
		}
		else{
			if(cache->verbose){
			mi_info ( "--- Map -> n. of elements: %u" , sss_map->size() );
			mi_info ( "---" );
			}
		}

		// store the map in scene DB
		cache->oMap = sss_map.store();

		Map_declaration				sss_declaration( 3 );
		sss_declaration->add_scalar	( "area" );
		sss_declaration->add_color	( "sss" );

		// retrieves fields ids for the map element TODO::SO NO NEED TO REDECLARE'M AS ABOVE::INIT THE DECL WITH THE EXISTING MIMAP
		cache->omap_area_id =	sss_declaration->get_field_id ( "area" );
		cache->omap_sss_id	=	sss_declaration->get_field_id ( "sss" );

		return;	// return early as we're shading directly from pcloud ////////////////////////////////
		*/
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////
	// plist GI layout

	if ( is_GI_init )		//already initialized by any other
	{
		if(cache->verbose>=3)
		mi_info("ROMBO::PCLOUD::GI::INIT::  <Pointlist already initialized>");

		cache->pointlist	= gxPointlistGI;
		cache->octree		= gxOctreeGI;
	}
	else
	{
		// locking ....................
		mi_lock ( state->global_lock );
		if ( is_GI_init )
		{
			if(cache->verbose>=3)
			mi_info("ROMBO::PCLOUD::GI::INIT::  <Pointlist almost initialized>");
			
			//maybe some other thread did init just before this lock
			cache->pointlist	= gxPointlistGI;
			cache->octree		= gxOctreeGI;
			mi_unlock ( state->global_lock );

		}else
		{
			if(cache->verbose>=2)
			mi_info("ROMBO::PCLOUD::GI::INIT::  <Initializing pointlist>");

			gxPointlistGI = new _pointlistGI();				// create a new pointlist
			_GIT::LayoutPointlistData(gxPointlistGI);		// data layout


			///////////////////////////////////////////////////////////////////////////////////////////////////
			if(cache->verbose) mi_info("ROMBO::PCLOUD::GI::INIT::  <Loading map(s)>");	// read map(s)

			// irrad map tags
			int map_array_id	= *mi_eval_integer( &paras->i_map );
			int map_array_nb	= *mi_eval_integer( &paras->n_map );

			miTag	* map_tags	= mi_eval_tag ( &paras->bake_map ) + map_array_id;

			bool readmap_aborted = false;
			for(int m=0; m < map_array_nb; m++)
			{
				char* imap_file_name = cpc_util_mi::GetCharFromMITag( map_tags[m] );
				string map_ext (cpc_util_mi::GetFileExtension( string(imap_file_name) ));

				if(map_ext.compare("ptc")==0)
				{
					// PTC::TODO:CUSTOM:LOADER:FOR:PTC:FILES
				}else
				{
					// miMAP
					if( !gxPointlistGI->ReadMIMapDataMRay(imap_file_name, cache->verbose) )
					{
						mi_error("ROMBO::PCLOUD:GI::POINTLIST::FILLING: Yep.. shit happens !");
						readmap_aborted = true;
						break;
					}	
				}
			}

			gxPointlistGI->PrintDataInfo(cache->verbose);
			cache->pointlist	= gxPointlistGI;						// point locally to global plist


			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Build octree
			if(cache->verbose) mi_info("ROMBO::PCLOUD::GI::INIT::  <Creating hierarchical structure>\n");
							
			gxOctreeGI = new _octreeGI();								// create new GI octree
			gxOctreeGI->SplitOctree( gxPointlistGI );					// splitting
			if(cache->verbose) gxOctreeGI->PrintDataInfo(cache->verbose);// debug

			cache->octree = gxOctreeGI;									// point locally to global octree

			//////////////////////////////////////////////////////////////////////////////////////////////////
			// Here we'd eval GI for every point in the pcloud (--- NOT USED ---)
			if(cache->read_only == 1){
				if(cache->verbose) mi_info("ROMBO::PCLOUD::GI::INIT::  Going to render this damn thing !!\n");

				// get machine cpu threads
				cache->nthreads = *mi_eval_integer(&paras->nthreads);
				if(!cache->nthreads){

					SYSTEM_INFO sysinfo; 
					GetSystemInfo( &sysinfo ); 

					cache->nthreads = sysinfo.dwNumberOfProcessors;
					if(cache->nthreads<=1){
						mi_error("ROMBO::PCLOUD:GI::  Application requires multi-processor architecture ATM. Aborting ..");
						cache->enable = miFALSE;
						return;
					}

					if(cache->verbose>=3) mi_info("ROMBO::PCLOUD:GI::  <Detected <%i> cores>", cache->nthreads);
				}


				iOctreeNodeGI * _octroot = gxOctreeGI->GetRoot();			// octree root

				// pre-compute first GI bounce
				Thread_CG_Effects_Wrapper<_GIT> ThreadManager;
				bool ABORT_CALLED = ThreadManager.StartMultiThreadComputing(gxPointlistGI, _octroot, cache);

				// transfer it to the actual irradiance slot
				for(int i=0; i<gxPointlistGI->GetTotalNodes(); i++)
				{
					iVector3 pIrrad ( gxPointlistGI->GetNodeVectorAt< _pointlistGI::NODERESULTSLOT >(i) );
					gxPointlistGI->SetNodeVectorAt< DATA_SLOT_IRRAD >(i, pIrrad);
				}
			}

			// Un-locking ...................
			is_GI_init = miTRUE;	// data is ready.. allow other threads to access it 
			is_GI_exit = miFALSE;	// update the static var has it doesn't get trashed until maya exit .. not when mray plugin finishes rendering !!

			mi_unlock ( state->global_lock );
		}
	} //end lock

 	//////////////////////////////////////////////////////////////////////////////////////////////////
   if(cache->verbose) mi_info("ROMBO::PCLOUD::GI::INIT::  <Everything ready for rendering>\n");
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Shader Exit
extern "C" DLLEXPORT void pcloud_GI_exit(
	miState * state,
	pcloud_GI_paras * paras )
{  
    if ( ! paras ) return;

	
	void **user; 
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_GI_cache* cache = static_cast< pcloud_GI_cache* >(*user);


	if ( is_GI_exit ){
		if(cache->verbose>=3)
		mi_info("ROMBO::PCLOUD:GI::EXIT::  <Data already flushed>");
	}
	else
	{
		// Locking ....................
		mi_lock ( state->global_lock );

		if ( is_GI_exit )
		{
			if(cache->verbose>=3)
			mi_info("ROMBO::PCLOUD:GI::EXIT::  <Data almost flushed>");
			
			//maybe some other thread did init just before this lock
			mi_unlock ( state->global_lock );
		}
		else
		{
			if(cache->verbose>=2)
			mi_info("ROMBO::PCLOUD:GI::EXIT::  <Flushing data>");

			// Clean-up memory
			if(!cache->isshaderball)
			if(cache->oMap) mi_db_delete(cache->oMap);		// delete data map from DB

			cache->pointlist->FreeStorage();				// clear pointlist storage
			cache->pointlist->ClearDataLayout();			// and data layout fields
			cache->octree->Clear();							// clear octree storage

			delete gxPointlistGI;							// delete global GI pointlist
			gxPointlistGI = miNULLTAG;						// and invalidate pointer
			delete gxOctreeGI;								// delete global GI octree
			gxOctreeGI = miNULLTAG;
			
			// Un-locking ...................
			is_GI_exit = miTRUE;
			is_GI_init = miFALSE; // update the static var has it doesn't get trashed until maya exit .. not when mray render finishes !!
			
			mi_unlock ( state->global_lock );
		}
	} // end lock
	
	cache->pointlist	= miNULLTAG;						// invalidate local pointers
	cache->octree		= miNULLTAG;
	mi_mem_release( *user );								// delete local user cache
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Shader Implementation
extern "C" DLLEXPORT	miBoolean   pcloud_GI (
	miColor	    *result,
	miState	    *state,
	struct pcloud_GI_paras	*paras)
{
    result->r = result->g = result->b = 0.0f;
    result->a = 1.0f;


	// * Get shader cache
	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_GI_cache* cache = static_cast< pcloud_GI_cache* >(*user);

	// exit earlier if ...  
	if(cache->isshaderball || cache->disabled) 
	{
		if(mi_is_connected(state, &paras->fg_bypass)) { *result = *mi_eval_color(&paras->fg_bypass); }
		return miTRUE;	// shaderball render
	}

	if(	state->type==miRAY_FINALGATHER)
	{
		if(mi_is_connected(state, &paras->fg_bypass)){ *result = *mi_eval_color(&paras->fg_bypass);	}
		return miTRUE;	// finalgathering
	}


	// Read pre-computed GI map /////////////////////////////////////////////////////////////////////
	if(cache->read_only == 0) // NOT USED NOR TESTED.. EVEN NOT NECESSARY :) We're fast enough already !
	{
		/*
		// ** Setup map lookup
		if( cache->oMap==miNULLTAG ){
		
			mi_error("No map saved to DB or invalid map tag !!");
			return miFALSE;
		}

		// map accessor
		Access_map  map = cache->oMap;
		if( map->is_empty() ){
			
			mi_error("Map is empty !!");
			return miFALSE;
		}

		// attach the lookup to the map
		Map_lookup	result_lookup ( map );
		{
			// runs a lookup inside the map
			result_lookup->search ( state->point, cache->points_in_lookup );

			// accumulates the colors of all the elements which have been found
			miUint		num = 0;
			miColor		element_sss;
			for ( ; ! result_lookup->at_end () ; result_lookup->next () )	{

				// retrieve radiosity data
				result_lookup->get ( cache->omap_radiosity_id , element_sss );

				result->r += element_sss.r;
				result->g += element_sss.g;
				result->b += element_sss.b;
			}

			// keeps track of how many elements were used in all
			num = result_lookup->size ();

			// divides by the total number of elements to get the average
			if ( num > 0 )  {
				result->r /= ( float ) num;
				result->g /= ( float ) num;
				result->b /= ( float ) num;
			}
		}
		*/
	}
	
	// Compute GI using octree as icache ////////////////////////////////////////////////////////////
	else if(cache->read_only >= 1)		// eval everything
	{

		// position and normal
		iVector3 P( state->point.x, state->point.y, state->point.z );
		iVector3 N( state->normal.x, state->normal.y, state->normal.z );

		// microrasterizer
		RadiosityIntegrator integrator( cache->buffer_res );
		//OcclusionIntegrator integrator( cache->buffer_res );

		microRasterize(	integrator, 
						P + N*cache->micro_eps, N, 
						cache->cosConeAngle, cache->sinConeAngle, cache->maxsolidangle, cache->exactRenderAngle,
						cache->pointlist, cache->octree->GetRoot() );

		iVector3 radResult( integrator.radiosity(N, M_PI_2, &result->a) );
		//iVector3 radResult( integrator.occlusion(N, M_PI_2) );
		result->r = radResult.GetX();
		result->g = radResult.GetY();
		result->b = radResult.GetZ();	
	}


	// user can scale the result
	result->r *= cache->multiply_factor;
	result->g *= cache->multiply_factor;
	result->b *= cache->multiply_factor;

	// turn it diffuse irradiance
	miColor diffuse_a	= *mi_eval_color	( &paras->diffuse_albedo );
	result->r *= diffuse_a.r;
	result->g *= diffuse_a.g;
	result->b *= diffuse_a.b;

	// add diffuse contribution
	miColor diffuse_c = *mi_eval_color	( &paras->diffuse_shader );
	result->r += diffuse_c.r;
	result->g += diffuse_c.g;
	result->b += diffuse_c.b;
	//result->a = 1.0f;
	return ( miTRUE );
}
