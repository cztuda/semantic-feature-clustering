
SET(SRC
# 	argumentparser.h
# 	argumentparser.cpp
	)




if(NOT LIBONLY)
add_executable(quicktest quicktest.cpp)

add_executable(elzingatest elzingatest.cpp elzinga_grid.cpp)

endif()

#add_executable(parallelNormColumn normcolumntest.cpp normcolumn.cpp)
#target_compile_options(parallelNormColumn PRIVATE -O3)
