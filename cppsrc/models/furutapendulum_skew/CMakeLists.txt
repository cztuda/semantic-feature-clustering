CMAKE_POLICY(SET CMP0079 NEW)

SET(problemsrc,
    _definition.h
    _dynamics.h
    _objective.h
    _constraints.h
    definition_header.h
    definition_functions.h
)

add_library(furutapendulum_skewproblem SHARED furutapendulum_skewproblem.h furutapendulum_skewproblem.cpp ${problemsrc})
target_link_libraries(furutapendulum_skewproblem 
    modelLib
    ${Boost_LIBRARIES}
    dl
)

add_executable(furutapendulum_skew_derivative_library_creation derivativecreation.cpp ${problemsrc})
target_link_libraries(furutapendulum_skew_derivative_library_creation furutapendulum_skewproblem modelLib ${Boost_LIBRARIES} dl)

add_custom_target(run_furutapendulum_skew_derivative_library_creation
  COMMAND ./build/bin/furutapendulum_skew_derivative_library_creation 
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "run derivative creation for furutapendulum_skew in ${CMAKE_CURRENT_SOURCE_DIR}"
  SOURCES derivativecreation.cpp ${problemsrc}
)

add_dependencies(run_furutapendulum_skew_derivative_library_creation furutapendulum_skew_derivative_library_creation)

