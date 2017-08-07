# - Try to find Pylon
# Once done this will define
#  PYLON_FOUND - System has Pylon
#  PYLON_INCLUDE_DIRS - The Pylon include directories
#  PYLON_LIBRARIES - The libraries needed to use Pylon

set( PYLON_LIBRARY "/opt/pylon5/lib" )

FIND_PATH( PYLON_INCLUDE_DIR pylon/PylonBase.h
            PATHS
            /opt/pylon5/include
            "$ENV{PYLON_ROOT}/include"
)

FIND_LIBRARY( PYLONBASE_LIBRARY 
                NAMES 
                pylonbase PylonBase_MD_VC100
                PATHS
                ${PYLON_LIBRARY}
)

FIND_LIBRARY( PYLON_UTILITY_LIBRARY 
                NAMES 
                pylonutility PylonUtility_MD_VC100
                PATHS
                ${PYLON_LIBRARY}
)

FIND_LIBRARY( PYLON_GCBASE_LIBRARY
                NAMES
                GCBase_gcc_v3_0_Basler_pylon_v5_0
                PATHS
                ${PYLON_LIBRARY}
)

FIND_LIBRARY( PYLON_GEN_API_LIBRARY
                NAMES
                GenApi_gcc_v3_0_Basler_pylon_v5_0
                PATHS
                ${PYLON_LIBRARY}
)

set(PYLON_LIBRARIES  ${PYLONBASE_LIBRARY} ${PYLON_UTILITY_LIBRARY} ${PYLON_GCBASE_LIBRARY} ${PYLON_GEN_API_LIBRARY})
set(PYLON_INCLUDE_DIRS ${PYLON_INCLUDE_DIR})

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(PYLON DEFAULT_MSG
  PYLON_INCLUDE_DIR
  PYLON_LIBRARY)

mark_as_advanced(PYLON_INCLUDE_DIR PYLON_LIBRARIES)
 
