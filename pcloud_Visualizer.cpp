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
 *	pcloud_Visualizer
 *	pcloud_Visualizer_init
 *	pcloud_Visualizer_exit
 *	pcloud_Visualizer_version
 *
 */


#include "_cpc_mentalray.h"


///////////////////////////////////////////////////////////////////////////////////////////////
// * a custom distance functor, that will be used to evaluate the map elements
// during the lookups. The metric of this functor is quadratic, that is it
// returns the quadratic distance from the search point of the element being
// examined. The quadratic distance from a bounding box is defined in the
// 'Map_distance' class.

// An element is rejected if the search point falls outside its radius; when
// an element is rejected, 'miHUGE_SCALAR' is returned. As it's based on radius
// this end up showing the points as surfels.
class quad_distance_xfunctor : public mi::shader::Map_distance<3> 
{
public:

    // constructor
    quad_distance_xfunctor 
	(
		miVector		&point ,	// the current search point
		Map_field_id	radius_id,	// the id of the 'radius' field in the map
		miScalar		distance_mult
	)
	:	m_radius_id ( radius_id ),
		m_distance_mult ( distance_mult ) 
    {
		// 'm_point' is a protected member variable of the 'Map_distance' class,
		// it must be ALWAYS initialized to the search point!
		m_point[0] = point.x;
		m_point[1] = point.y;
		m_point[2] = point.z;
    }

    // returns the quadratic distance from the element being examined
    float   operator () ( const Map_iterator_base *element ) const
    {
		float	d = 0.0f;
		float	diff;
		float	position[DIMENSION];
		// 'DIMENSION' is a static variable of the 'Map_distance' class (here it's 3)

		// retrieves the position of the current element
		element->get_position ( position );

		for ( miUint i = 0 ; i < DIMENSION ; ++i )  {
			diff = this->m_point[i] - position[i];
			d += diff * diff;
		}
		// 'd' is the quadratic distance of the particle from the search point

		float	this_radius;

		// retrieves the radius of the current particle
		element->get ( m_radius_id , this_radius );

		// if the distance of the particle from the search point is greater than
		// the particle radius, the particle is rejected
		if(m_distance_mult != 0.0f)
			if ( d/m_distance_mult > this_radius * this_radius )
				return miHUGE_SCALAR;

		// otherwise, returns the squared distance
		return d;
    }

private:

    Map_field_id    m_radius_id;	// the id of the 'radius' field in the map
	miScalar		m_distance_mult;
};


///////////////////////////////////////////////////////////////////////////////////////////////
// * the parameter struct for the shader
struct pcloud_Visualizer_paras 
{
	miBoolean	verbose;

	miTag		map;	    // tag of the map to use
	miTag		map_attr;	// tag of the map field
	miInteger	attr_type;	// color or float

	miScalar	mult;	    // color result multiplier
	miScalar	p_search;	// extent of the lookup 
	miScalar	p_reject;	// reject points with radii<distance (opt:0)
};

// * the cache structure for the shader
struct pcloud_Visualizer_cache
{
	miTag tMap;		// We get the map from file, then we store in DB init
					// and access it directly from this tag in render loop
	//Map_field_id	map_mpsize_id;
	Map_field_id	map_area_id;
	Map_field_id	map_user_id;

	miInteger		map_attr_type;

	miScalar		multiply_factor;
	miScalar		map_search_dist;
	miScalar		reject_points;

	miBoolean		isshaderball;
	miBoolean		disabled;
};

///////////////////////////////////////////////////////////////////////////////////////////////
// Shader Initialization
extern "C" DLLEXPORT	int	    pcloud_Visualizer_version(void) {return(1);}
extern "C" DLLEXPORT	void	pcloud_Visualizer_init (
	miState						*state,
	pcloud_Visualizer_paras		*paras,
	miBoolean					*inst_init_req )
{
    if ( ! paras )	{
		*inst_init_req = miTRUE; return;	// shader instance initialization
    }


	// inittialize cache
	//pcloud_Visualizer_cache* cache = new pcloud_Visualizer_cache;
	pcloud_Visualizer_cache* cache = (pcloud_Visualizer_cache*) 
								mi_mem_allocate(sizeof(pcloud_Visualizer_cache));

	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	*user = cache;

	// check if in hypershader
	cache->isshaderball = miFALSE;
	const miCamera * camera = state->camera;
	if( camera->focal == 1.0 || camera->x_resolution<320){
		
		cache->isshaderball = miTRUE;
		return;
	}

	// check if in lightmap stage 
	// called during 2nd rays eval
	miStage stage;
	mi_query(miQ_STAGE, state, 0, &stage);
	if(stage == miSTAGE_LIGHTMAP){

		cache->disabled = miTRUE;
		return;
	}

	miBoolean verbose = *mi_eval_boolean( &paras->verbose );
	if(verbose){
		mi_info ( "--- Evaluate Point Cloud" );
		mi_info ( "---" );
	}


	// evaluates the tag of the map file name
	miTag	map_tag = *mi_eval_tag ( &paras->map );
	char	*name = ( char * ) mi_db_access ( map_tag );
	mi_db_unpin ( map_tag );


	// access the map from file
	Access_map  map ( name );
	if(1){
		//mi_info ( "--- Map -> Accessing map file: %s", name );
		if(map->is_empty()){ mi_info ( "--- Map -> is empty !" );cache->tMap = miNULLTAG;return;}
		else {
			if(verbose)
			mi_info ( "--- Map -> n. of elements: %u" , map->size() );
		}
	}

	// store the map in scene DB
	cache->tMap = map.store();

	// get a copy of the declaration
	Map_declaration	    declaration ( map );
	if(verbose){
		mi_info ( "--- Map -> dimensions: %u" , declaration->get_dimension () );
		mi_info ( "---" );
	}

	// standard map fields/channels/attributes (position is there by default)
	//cache->map_mpsize_id =		declaration->get_field_id ( "mpsize" );
	cache->map_area_id =		declaration->get_field_id ( "area" );


	// evaluates the tag of the map field string
	miTag	map_attr_tag = *mi_eval_tag ( &paras->map_attr );
	char* map_attr_name = ( char * ) mi_db_access ( map_attr_tag );
	mi_db_unpin ( map_attr_tag );

	cache->map_user_id =		declaration->get_field_id ( map_attr_name );

	// cache parameters
	cache->map_attr_type = *mi_eval_integer ( &paras->attr_type );
	cache->map_search_dist = *mi_eval_scalar ( &paras->p_search );
	cache->multiply_factor = *mi_eval_scalar ( &paras->mult );
	cache->reject_points = *mi_eval_scalar ( &paras->p_reject );


	// TEMP:DEBUG
	if(0)
	{
		Map_field_id fDir =		declaration->get_field_id ( "area" );
		Access_map_iterator	it ( map );
		int ix = 0; miVector vPos; float area; miVector vDir;
		for ( ; ! it->at_end () ; it->next () )	
		{
			it->get_position( vPos );
			it->get ( cache->map_area_id , area );
			it->get ( fDir , vDir );

			if(ix%500==0)
				mi_info( "--- Map -> point : %d, position: %.2f %.2f %.2f, dir: %f %f %f, area: %f", 
						ix, vPos.x, vPos.y, vPos.z, 
						vDir.x, vDir.y, vDir.z, area);
			ix ++;
		}
		mi_info ( "---" );
		mi_info ( "---" );
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Shader Exit
extern "C" DLLEXPORT void pcloud_Visualizer_exit(
	miState * state,
	pcloud_Visualizer_paras * paras )
{  
	if (!paras) return;

	void **user; 
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_Visualizer_cache* cache = static_cast< pcloud_Visualizer_cache* >(*user);

	mi_db_delete(cache->tMap);	// delete data map
	mi_mem_release( *user );	// delete user cache
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Shader Implementation
extern "C" DLLEXPORT	miBoolean   pcloud_Visualizer (
	miColor	    *result,
	miState	    *state)
{
    result->r = result->g = result->b = 0.0f;
    result->a = 1.0f;


	// * Get shader cache
	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_Visualizer_cache* cache = static_cast< pcloud_Visualizer_cache* >(*user);

	// exit earlier if ...  
	if(cache->isshaderball || cache->disabled) 
	{
		return miTRUE;	// shaderball render
	}

	if(	state->type==miRAY_FINALGATHER	|| 
		state->type==miRAY_REFLECT		||
		state->type==miRAY_REFRACT	)
	{
		return miTRUE;
	}


    // * Get parameters
    float	extent = cache->map_search_dist;


	// ** Setup map lookup
	if(!cache->tMap) return miFALSE;
	Access_map  map = cache->tMap;
	if( map->is_empty() ) return miFALSE;

    // attach the lookup to the map
    Map_lookup	result_lookup ( map );


	// build the distance functor to use in the search
	quad_distance_xfunctor    quad_distance ( state->point , cache->map_area_id, cache->reject_points );

	// Run a lookup inside the map, using the distance functor just built.

	// We're looking for just one particle here. Note that the squared extent
	// is used, since the metric of the 'quad_distance_xfunctor' functor is quadratic
	/*Map_status  status = */result_lookup->search ( quad_distance , 1 , extent * extent );

	if ( ! result_lookup->is_empty () )
	{
		if(cache->map_attr_type == 0) // color
		{
			// we found a particle
			miColor	    element_color;
			miScalar	element_radius;

			// retrieves the color of the first element in the lookup, which
			// in our case contains just one element
			result_lookup->get ( cache->map_user_id , element_color );
			result_lookup->get ( cache->map_area_id , element_radius );

			*result = element_color;
			result->a = element_radius;
		}else
		{
			miScalar	element_float;
			miScalar	element_radius;

			// retrieves the color of the first element in the lookup, which
			// in our case contains just one element
			result_lookup->get ( cache->map_user_id , element_float );
			result_lookup->get ( cache->map_area_id , element_radius );

			result->r = element_float;
			result->g = element_float;
			result->b = element_float;
			result->a = element_radius;
		}
	}	// if no particle found, then the output black color 

    
	// scales the result by the color multiplier
	result->r *= cache->multiply_factor;
	result->g *= cache->multiply_factor;
	result->b *= cache->multiply_factor;
	result->a = 1.0f;
	return ( miTRUE );
}
