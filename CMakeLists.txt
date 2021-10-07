# Create library
set(COMPONENT_SRCDIRS
    depthai-spi-library
    common
    depthai-shared/src/datatype
    .
)

set(COMPONENT_ADD_INCLUDEDIRS
    depthai-shared/include
    depthai-spi-library
    common
    .
)

set(COMPONENT_REQUIRES "nvs_flash")

register_component()

# Add interface transitive property (C++14)
if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    target_compile_features(${COMPONENT_LIB} INTERFACE cxx_generic_lambdas)
else()
    target_compile_features(${COMPONENT_LIB} INTERFACE cxx_std_14)
endif()