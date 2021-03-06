cmake_minimum_required(VERSION 3.4.1)

SET( GCC_COVERAGE_LINK_FLAGS    "-v")
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
SET( CMAKE_SHARED_LINKER_FLAGS  "${CMAKE_SHARED_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
SET( CMAKE_STATIC_LINKER_FLAGS  "${CMAKE_STATIC_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
SET( OSG_CPP_EXCEPTIONS_AVAILABLE "Yes")

SET(KERNEL_LIBRARIES
    cvrKernel                     # For cvr, Kernel > Menu > Util
    cvrMenu                       # The order of Input, Config, Collaborative is unsure (current order works)
    cvrUtil                       #
    cvrInput                      #
    cvrConfig                     #
    cvrCollaborative              #
    mxml                          # mxml must be included before cvr
    log                   #
    osgPresentation               # For osg, the dependency relations:
    osgWidget                     # osgPresentation: https://github.com/WeichenLiu/CalVR_for_Android/blob/master/Note/osgPresentation.png
    osgUI                         # osgdb_osg: https://github.com/WeichenLiu/CalVR_for_Android/blob/master/Note/osgdb_osg.png
    osgManipulator                # osgdb_serializers_osg:
    osgAnimation                  # https://github.com/WeichenLiu/CalVR_for_Android/blob/master/Note/osgdb_serializers_osg.png
    osgViewer                     # (These three are the "roots" of the trees)
    osgVolume                     #
    osgSim                        #
    osgShadow                     #
    osgTerrain                    #
    osgParticle                   #
    osgFX                         #
    z                             #
    osgText                       #
    osgGA                         #
    #osgdb_obj
    osgdb_tiff
    osgdb_rgb
    osgdb_png
    osgdb_jpeg
    osgdb_osg                     #
    osgdb_serializers_osg         #
    osgDB                         #
    osgUtil                       #
    osg                           #
    OpenThreads                   #
    tiff
    png
    jpeg
    #zlib
    EGL                           #
    GLESv2                        #
    GLESv3
)

# order matters
SET(PHYSX_LIBS
    PxPvdSDK
	PsFastXml
	PhysX3CharacterKinematic
	PhysX3Extensions
	PhysX3Cooking
	PhysX3Vehicle
	PhysX3
	PhysX3Common
	PxFoundation
	cpufeatures
)

include(CMakeLists-staticDependencies.txt)
include(CMakeLists-staticOSG.txt)
include(CMakeLists-staticMXML.txt)
include(CMakeLists-staticPhysX.txt)
include(CMakeLists-buildCALVR.txt)
include(CMakeLists-buildCALVR-PLUGIN.txt)

# physx needs cpufeatures
include_directories(${ANDROID_NDK}/sources/android/cpufeatures)
add_library(cpufeatures ${ANDROID_NDK}/sources/android/cpufeatures/cpu-features.c)
target_link_libraries(cpufeatures dl)

#arcore
add_library(arcore SHARED IMPORTED)
set_target_properties(arcore PROPERTIES IMPORTED_LOCATION
                      ${ARCORE_LIBPATH}/${ANDROID_ABI}/libarcore_sdk_c.so)

include_directories(src/main/cpp/plugins/common)
include_directories(src/main/cpp/plugins/MenuBasics)
include_directories(src/main/cpp/plugins/PhysxBall)
include_directories(src/main/cpp/plugins/GlesDrawables)
include_directories(src/main/cpp/plugins/SpatialViz)

add_library(ARCalVR
            SHARED
            src/main/cpp/jni_interface.cpp
            src/main/cpp/allController.cpp
            src/main/cpp/bgDrawable.cpp
            src/main/cpp/perfMonitor.cpp
            )

target_link_libraries(ARCalVR
                      ${CVR_PLUGINS}
                      android
                      mediandk
                      arcore
                      ${PHYSX_LIBS}
                      ${KERNEL_LIBRARIES}
                      )