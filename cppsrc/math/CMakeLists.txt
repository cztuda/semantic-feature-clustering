SET(SRC
	definitions.h
	matrixOperations.cpp
	matrixOperations.h
	)


add_library(mathLib SHARED ${SRC})


get_filename_component(PATH_REL ${CMAKE_CURRENT_SOURCE_DIR} NAME)
foreach(elt ${SRC})
	set(math_SRC ${math_SRC} ${PATH_REL}/${elt})
endforeach()

set(models_SRC ${math_SRC} PARENT_SCOPE)
