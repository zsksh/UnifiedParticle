SOURCE_GROUP (ANN REGULAR_EXPRESSION ANN/.*)
SOURCE_GROUP (HLBFGS REGULAR_EXPRESSION HLBFGS/.*)
SOURCE_GROUP (Eigen REGULAR_EXPRESSION Eigen/.*)

SOURCE_GROUP (core REGULAR_EXPRESSION core/.*)

file(GLOB_RECURSE SRCS *.cpp *.c *.h *.cu ) # all cpp/c/h files will be represented as a macro called SRCS
add_executable(surface_reconstruction  ${SRCS}) # all files is represented as ${SRCS}
target_link_libraries(surface_reconstruction  #we need geogram
PUBLIC
OpenMP::OpenMP_CXX
)
target_include_directories(surface_reconstruction PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
target_compile_features(surface_reconstruction PRIVATE cxx_std_11)



