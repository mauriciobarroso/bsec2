idf_component_register(SRCS "bsec2.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bme68x_lib)

add_prebuilt_library(libalgobsec lib/{IDF_TARGET}/libalgobsec.a
					PRIV_REQUIRES bme68x_lib)
target_link_libraries(${COMPONENT_LIB} PUBLIC libalgobsec)
