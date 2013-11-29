# original copied from ORCA: http://orca-robotics.sourceforge.net/
# hacked around a bit to reflect a different structure... make that a lot!

# Benoit Larochelle: I took this script from CAST, and modified it to make it compile other packages, such as our NIFTi ROS packages


# This file contains a set of macros used for generating 
# C++ source from Slice files
#
# Author: Alex Brooks
#

#
# Appends the new_bit to the original.
# If the original is not set, it will be set to the new_bit.
#
MACRO( APPEND original new_bit )

  IF    ( NOT ${original} )
    SET( ${original} ${new_bit} )
  ELSE  ( NOT ${original} )
    SET( ${original} ${${original}} ${new_bit} )
  ENDIF( NOT ${original} )

ENDMACRO( APPEND original new_bit )


# Generate rules for SLICE->C++ files generation, for each
# of the named slice source files.
#
# cast_slice2cpp( generated_cpp_list generated_header_list SRC [INCDIR1 [INCDIR2...]] )
# 
# Returns lists of all the .cpp and .h files that will be generated.
#   (ie modifies the first two arguments: generated_cpp_list and generated_header_list)
#
# Sets up dependencies: 
#  - Each .cpp and .hpp file depends on all the .ice files.
#
# Each optional extra dir is added to the search path for includes

MACRO( slice2cpp4gui generated_cpp_list generated_header_list src_name)

    SET( slice_cpp_suffixes        .cpp )
    SET( slice_header_suffixes     .hpp  )
    SET( slice_suffixes            ${slice_cpp_suffixes} ${slice_header_suffixes} )
    SET( slice2cpp_command         $ENV{ICE_HOME}/bin/slice2cpp${EXE_EXTENSION} )

    SET( slice_src_dir        	   src)


    FOREACH(slice_inc_dir ${ARGN})
    	APPEND(slice_include_dirs "-I${slice_inc_dir}")
    ENDFOREACH(slice_inc_dir ${ARGN})

    APPEND(slice_include_dirs "-I${PROJECT_SOURCE_DIR}/src")


    # satellite projects need to include slice files from orca installation
    # NOTE: funky interaction between cmake and slice2cpp: cannot use "" around the slice_args!
    SET( slice_args ${SLICE_PROJECT_ARGS} ${slice_include_dirs} --stream --header-ext hpp --output-dir ${CMAKE_CURRENT_SOURCE_DIR} )

    APPEND(slice_src_full "${PROJECT_SOURCE_DIR}/${slice_src_dir}/${src_name}" )
    #MESSAGE( "slice_src_full ${PROJECT_SOURCE_DIR}/${slice_src_dir}/${src_name}" )

   
    # OUTPUT is the target we're generating rules for.
    STRING( REGEX REPLACE "\\.ice" ${slice_header_suffixes} output_header_basename "${src_name}" )
    SET( output_header_fullname "${CMAKE_CURRENT_SOURCE_DIR}/${output_header_basename}" )

    STRING( REGEX REPLACE "\\.ice" ${slice_cpp_suffixes} output_basename "${src_name}" )
    SET( output_fullname "${CMAKE_CURRENT_SOURCE_DIR}/${output_basename}" )

    #message( "DEBUG output_fullname ${output_fullname}" )

               
    #
    # Add this output to the list of generated files
    #
    APPEND( ${generated_header_list} ${output_header_fullname} )
    APPEND( ${generated_cpp_list} ${output_fullname} )
       

    SET( slice_source "${PROJECT_SOURCE_DIR}/${slice_src_dir}/${src_name}" )
    #MESSAGE( "DEBUG: Dealing with ${src_name}")


    #
    # Add a dependency from file.xxx from file.ice (HEADER)
    #
    MESSAGE( STATUS "Adding a dependency from ${output_header_basename} to ${src_name}" )
    ADD_CUSTOM_COMMAND(
        OUTPUT  ${output_header_fullname}
        DEPENDS ${slice_src_full}
        COMMENT ""
    )


    #
    # Add the command to generate file.xxx from file.ice (SOURCE)
    # Note: when the 'output' is needed, the 'command' will be called with the 'args'
    #
    MESSAGE( STATUS "Adding rule for generating ${output_basename}/${slice_header_suffixes} from ${src_name}" )
    ADD_CUSTOM_COMMAND(
        OUTPUT  ${output_fullname}
        COMMAND ${slice2cpp_command} ${slice_args} ${slice_source}
        DEPENDS ${slice_src_full} ${output_header_fullname}
        COMMENT "Generating ${output_basename}/${slice_header_suffixes} from ${src_name}"
        VERBATIM
    )
    
  
    #MESSAGE( STATUS "DEBUG: generated_cpp_list: ${${generated_cpp_list}}")
    #MESSAGE( STATUS "DEBUG: generated_header_list: ${${generated_header_list}}")
    
    #MESSAGE( STATUS "Will generate cpp header and source files from Slice definition using this command:" )
    #MESSAGE( STATUS "${slice2cpp_command} ${slice_args}" )

ENDMACRO( slice2cpp4gui generated_cpp_list generated_header_list )
