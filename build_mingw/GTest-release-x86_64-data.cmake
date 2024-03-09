########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

list(APPEND gtest_COMPONENT_NAMES GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
list(REMOVE_DUPLICATES gtest_COMPONENT_NAMES)
set(gtest_FIND_DEPENDENCY_NAMES "")

########### VARIABLES #######################################################################
#############################################################################################
set(gtest_PACKAGE_FOLDER_RELEASE "/home/plaiseek/.conan2/p/b/gtest604dbbf2183f2/p")
set(gtest_BUILD_MODULES_PATHS_RELEASE )


set(gtest_INCLUDE_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/include")
set(gtest_RES_DIRS_RELEASE )
set(gtest_DEFINITIONS_RELEASE )
set(gtest_SHARED_LINK_FLAGS_RELEASE )
set(gtest_EXE_LINK_FLAGS_RELEASE )
set(gtest_OBJECTS_RELEASE )
set(gtest_COMPILE_DEFINITIONS_RELEASE )
set(gtest_COMPILE_OPTIONS_C_RELEASE )
set(gtest_COMPILE_OPTIONS_CXX_RELEASE )
set(gtest_LIB_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/lib")
set(gtest_BIN_DIRS_RELEASE )
set(gtest_LIBRARY_TYPE_RELEASE STATIC)
set(gtest_IS_HOST_WINDOWS_RELEASE 1)
set(gtest_LIBS_RELEASE gmock_main gmock gtest_main gtest)
set(gtest_SYSTEM_LIBS_RELEASE )
set(gtest_FRAMEWORK_DIRS_RELEASE )
set(gtest_FRAMEWORKS_RELEASE )
set(gtest_BUILD_DIRS_RELEASE )
set(gtest_NO_SONAME_MODE_RELEASE FALSE)


# COMPOUND VARIABLES
set(gtest_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtest_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtest_COMPILE_OPTIONS_C_RELEASE}>")
set(gtest_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtest_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtest_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtest_EXE_LINK_FLAGS_RELEASE}>")


set(gtest_COMPONENTS_RELEASE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
########### COMPONENT GTest::gmock_main VARIABLES ############################################

set(gtest_GTest_gmock_main_INCLUDE_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/include")
set(gtest_GTest_gmock_main_LIB_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/lib")
set(gtest_GTest_gmock_main_BIN_DIRS_RELEASE )
set(gtest_GTest_gmock_main_LIBRARY_TYPE_RELEASE STATIC)
set(gtest_GTest_gmock_main_IS_HOST_WINDOWS_RELEASE 1)
set(gtest_GTest_gmock_main_RES_DIRS_RELEASE )
set(gtest_GTest_gmock_main_DEFINITIONS_RELEASE )
set(gtest_GTest_gmock_main_OBJECTS_RELEASE )
set(gtest_GTest_gmock_main_COMPILE_DEFINITIONS_RELEASE )
set(gtest_GTest_gmock_main_COMPILE_OPTIONS_C_RELEASE "")
set(gtest_GTest_gmock_main_COMPILE_OPTIONS_CXX_RELEASE "")
set(gtest_GTest_gmock_main_LIBS_RELEASE gmock_main)
set(gtest_GTest_gmock_main_SYSTEM_LIBS_RELEASE )
set(gtest_GTest_gmock_main_FRAMEWORK_DIRS_RELEASE )
set(gtest_GTest_gmock_main_FRAMEWORKS_RELEASE )
set(gtest_GTest_gmock_main_DEPENDENCIES_RELEASE GTest::gmock)
set(gtest_GTest_gmock_main_SHARED_LINK_FLAGS_RELEASE )
set(gtest_GTest_gmock_main_EXE_LINK_FLAGS_RELEASE )
set(gtest_GTest_gmock_main_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(gtest_GTest_gmock_main_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtest_GTest_gmock_main_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtest_GTest_gmock_main_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtest_GTest_gmock_main_EXE_LINK_FLAGS_RELEASE}>
)
set(gtest_GTest_gmock_main_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtest_GTest_gmock_main_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtest_GTest_gmock_main_COMPILE_OPTIONS_C_RELEASE}>")
########### COMPONENT GTest::gmock VARIABLES ############################################

set(gtest_GTest_gmock_INCLUDE_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/include")
set(gtest_GTest_gmock_LIB_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/lib")
set(gtest_GTest_gmock_BIN_DIRS_RELEASE )
set(gtest_GTest_gmock_LIBRARY_TYPE_RELEASE STATIC)
set(gtest_GTest_gmock_IS_HOST_WINDOWS_RELEASE 1)
set(gtest_GTest_gmock_RES_DIRS_RELEASE )
set(gtest_GTest_gmock_DEFINITIONS_RELEASE )
set(gtest_GTest_gmock_OBJECTS_RELEASE )
set(gtest_GTest_gmock_COMPILE_DEFINITIONS_RELEASE )
set(gtest_GTest_gmock_COMPILE_OPTIONS_C_RELEASE "")
set(gtest_GTest_gmock_COMPILE_OPTIONS_CXX_RELEASE "")
set(gtest_GTest_gmock_LIBS_RELEASE gmock)
set(gtest_GTest_gmock_SYSTEM_LIBS_RELEASE )
set(gtest_GTest_gmock_FRAMEWORK_DIRS_RELEASE )
set(gtest_GTest_gmock_FRAMEWORKS_RELEASE )
set(gtest_GTest_gmock_DEPENDENCIES_RELEASE GTest::gtest)
set(gtest_GTest_gmock_SHARED_LINK_FLAGS_RELEASE )
set(gtest_GTest_gmock_EXE_LINK_FLAGS_RELEASE )
set(gtest_GTest_gmock_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(gtest_GTest_gmock_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtest_GTest_gmock_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtest_GTest_gmock_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtest_GTest_gmock_EXE_LINK_FLAGS_RELEASE}>
)
set(gtest_GTest_gmock_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtest_GTest_gmock_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtest_GTest_gmock_COMPILE_OPTIONS_C_RELEASE}>")
########### COMPONENT GTest::gtest_main VARIABLES ############################################

set(gtest_GTest_gtest_main_INCLUDE_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/include")
set(gtest_GTest_gtest_main_LIB_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/lib")
set(gtest_GTest_gtest_main_BIN_DIRS_RELEASE )
set(gtest_GTest_gtest_main_LIBRARY_TYPE_RELEASE STATIC)
set(gtest_GTest_gtest_main_IS_HOST_WINDOWS_RELEASE 1)
set(gtest_GTest_gtest_main_RES_DIRS_RELEASE )
set(gtest_GTest_gtest_main_DEFINITIONS_RELEASE )
set(gtest_GTest_gtest_main_OBJECTS_RELEASE )
set(gtest_GTest_gtest_main_COMPILE_DEFINITIONS_RELEASE )
set(gtest_GTest_gtest_main_COMPILE_OPTIONS_C_RELEASE "")
set(gtest_GTest_gtest_main_COMPILE_OPTIONS_CXX_RELEASE "")
set(gtest_GTest_gtest_main_LIBS_RELEASE gtest_main)
set(gtest_GTest_gtest_main_SYSTEM_LIBS_RELEASE )
set(gtest_GTest_gtest_main_FRAMEWORK_DIRS_RELEASE )
set(gtest_GTest_gtest_main_FRAMEWORKS_RELEASE )
set(gtest_GTest_gtest_main_DEPENDENCIES_RELEASE GTest::gtest)
set(gtest_GTest_gtest_main_SHARED_LINK_FLAGS_RELEASE )
set(gtest_GTest_gtest_main_EXE_LINK_FLAGS_RELEASE )
set(gtest_GTest_gtest_main_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(gtest_GTest_gtest_main_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtest_GTest_gtest_main_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtest_GTest_gtest_main_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtest_GTest_gtest_main_EXE_LINK_FLAGS_RELEASE}>
)
set(gtest_GTest_gtest_main_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtest_GTest_gtest_main_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtest_GTest_gtest_main_COMPILE_OPTIONS_C_RELEASE}>")
########### COMPONENT GTest::gtest VARIABLES ############################################

set(gtest_GTest_gtest_INCLUDE_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/include")
set(gtest_GTest_gtest_LIB_DIRS_RELEASE "${gtest_PACKAGE_FOLDER_RELEASE}/lib")
set(gtest_GTest_gtest_BIN_DIRS_RELEASE )
set(gtest_GTest_gtest_LIBRARY_TYPE_RELEASE STATIC)
set(gtest_GTest_gtest_IS_HOST_WINDOWS_RELEASE 1)
set(gtest_GTest_gtest_RES_DIRS_RELEASE )
set(gtest_GTest_gtest_DEFINITIONS_RELEASE )
set(gtest_GTest_gtest_OBJECTS_RELEASE )
set(gtest_GTest_gtest_COMPILE_DEFINITIONS_RELEASE )
set(gtest_GTest_gtest_COMPILE_OPTIONS_C_RELEASE "")
set(gtest_GTest_gtest_COMPILE_OPTIONS_CXX_RELEASE "")
set(gtest_GTest_gtest_LIBS_RELEASE gtest)
set(gtest_GTest_gtest_SYSTEM_LIBS_RELEASE )
set(gtest_GTest_gtest_FRAMEWORK_DIRS_RELEASE )
set(gtest_GTest_gtest_FRAMEWORKS_RELEASE )
set(gtest_GTest_gtest_DEPENDENCIES_RELEASE )
set(gtest_GTest_gtest_SHARED_LINK_FLAGS_RELEASE )
set(gtest_GTest_gtest_EXE_LINK_FLAGS_RELEASE )
set(gtest_GTest_gtest_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(gtest_GTest_gtest_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtest_GTest_gtest_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtest_GTest_gtest_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtest_GTest_gtest_EXE_LINK_FLAGS_RELEASE}>
)
set(gtest_GTest_gtest_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtest_GTest_gtest_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtest_GTest_gtest_COMPILE_OPTIONS_C_RELEASE}>")