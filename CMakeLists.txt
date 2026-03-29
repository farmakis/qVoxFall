option( PLUGIN_STANDARD_QVOXFALL "Install qVoxFall plugin" OFF )

if ( PLUGIN_STANDARD_QVOXFALL )
	project( QVOXFALL_PLUGIN)
	
	AddPlugin( NAME ${PROJECT_NAME} )
	
	add_subdirectory( external/gcl EXCLUDE_FROM_ALL )
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	target_link_libraries( ${PROJECT_NAME} GRAPH_CLUSTERING_LIB )
endif()
