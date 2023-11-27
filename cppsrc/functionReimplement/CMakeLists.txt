

SET(SRC
  Function.h
  Function.cpp
  SingleVariableFunction.h
  SingleVariableFunction.cpp
  Interval.h
  PiecewiseFunction.h
  PiecewiseFunction.cpp 
  Polynom.h
  Polynom.cpp
  NormalizedPolynom.cpp
  NormalizedPolynom.h
  TrajectoryReader.cpp
  TrajectoryReader.h
  trajectorycreation.cpp
)
                        
add_library(functionLib SHARED ${SRC})
target_link_libraries(functionLib ${Boost_LIBRARIES})
target_compile_options(functionLib PRIVATE -O3)

if(NOT LIBONLY)
add_executable(functionTest main.cpp ${SRC})

target_link_libraries(functionTest 
	functionLib ${Boost_LIBRARIES})
endif()
