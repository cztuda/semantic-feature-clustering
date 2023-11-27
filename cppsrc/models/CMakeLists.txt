########################################################################
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/ofc/models ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
########################################################################


SET(serialization
    ${MYCODE_BASE_DIR}/serialization/eigen_dense_serialization.h
    ${MYCODE_BASE_DIR}/serialization/eigen_sparse_serialization.h
    ${MYCODE_BASE_DIR}/serialization/serialization_helper.h
)


add_library(modelLib optimalcontrolproblem.cpp optimalcontrolproblem.h ../math/EigenPlus.cpp ../math/EigenPlus.h ${serialization})
target_link_libraries(modelLib functionLib)


add_subdirectory(furutapendulum_skew/)
add_subdirectory(manutec/)
# <<<INSERT_BEFORE>>>

    

get_filename_component(PATH_REL ${CMAKE_CURRENT_SOURCE_DIR} NAME)
foreach(elt ${SRC})
	set(models_SRC ${models_SRC} ${PATH_REL}/${elt})
endforeach()

