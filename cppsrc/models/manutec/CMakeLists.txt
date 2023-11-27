CMAKE_POLICY(SET CMP0079 NEW)

SET(problemsrc,
    _definition.h
    _dynamics.h
    _objective.h
    _constraints.h
    definition_header.h
    definition_functions.h
)

add_library(manutecproblem SHARED manutecproblem.h manutecproblem.cpp ${problemsrc})
target_link_libraries(manutecproblem 
    modelLib
    ${Boost_LIBRARIES} 
    dl
)

add_executable(manutec_derivative_library_creation derivativecreation.cpp ${problemsrc})
target_link_libraries(manutec_derivative_library_creation manutecproblem modelLib ${Boost_LIBRARIES} dl)

add_custom_target(run_manutec_derivative_library_creation
  COMMAND ./build/bin/manutec_derivative_library_creation 
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "run derivative creation for manutec in ${CMAKE_CURRENT_SOURCE_DIR}"
  SOURCES derivativecreation.cpp ${problemsrc}
)

add_dependencies(run_manutec_derivative_library_creation manutec_derivative_library_creation)

