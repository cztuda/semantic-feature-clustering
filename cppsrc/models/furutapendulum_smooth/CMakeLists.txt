CMAKE_POLICY(SET CMP0079 NEW)

SET(problemsrc,
    _definition.h
    _dynamics.h
    _objective.h
    _constraints.h
    definition_header.h
    definition_functions.h
)

add_library(furutapendulum_smoothproblem SHARED furutapendulum_smoothproblem.h furutapendulum_smoothproblem.cpp ${problemsrc})
target_link_libraries(furutapendulum_smoothproblem 
    modelLib
    ${Boost_LIBRARIES} 
    dl
)

add_executable(furutapendulum_smooth_derivative_library_creation derivativecreation.cpp ${problemsrc})
target_link_libraries(furutapendulum_smooth_derivative_library_creation furutapendulum_smoothproblem modelLib ${Boost_LIBRARIES} dl)

add_custom_target(run_furutapendulum_smooth_derivative_library_creation
  COMMAND ./build/bin/furutapendulum_smooth_derivative_library_creation 
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "run derivative creation for furutapendulum_smooth in ${CMAKE_CURRENT_SOURCE_DIR}"
  SOURCES derivativecreation.cpp ${problemsrc}
)

add_dependencies(run_furutapendulum_smooth_derivative_library_creation furutapendulum_smooth_derivative_library_creation)

