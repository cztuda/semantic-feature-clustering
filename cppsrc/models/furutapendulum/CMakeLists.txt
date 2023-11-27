CMAKE_POLICY(SET CMP0079 NEW)

SET(problemsrc,
    _definition.h
    _dynamics.h
    _objective.h
    _constraints.h
    definition_header.h
    definition_functions.h
)

add_library(furutapendulumproblem SHARED furutapendulumproblem.h furutapendulumproblem.cpp ${problemsrc})
target_link_libraries(furutapendulumproblem 
	modelLib 
	${Boost_LIBRARIES}
	dl
)

add_executable(furutapendulum_derivative_library_creation derivativecreation.cpp ${problemsrc})
target_link_libraries(furutapendulum_derivative_library_creation furutapendulumproblem modelLib ${Boost_LIBRARIES} dl)

add_custom_target(run_furutapendulum_derivative_library_creation
  COMMAND ./build/furutapendulum/furutapendulum_derivative_library_creation 
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "run derivative creation for furutaPendulum in ${CMAKE_CURRENT_SOURCE_DIR}"
  SOURCES derivativecreation.cpp ${problemsrc}
)

add_dependencies(run_furutapendulum_derivative_library_creation furutapendulum_derivative_library_creation)
