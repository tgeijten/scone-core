set(SCONE_CONFIG_FILES
	sconelib_config.h
	"../../../VERSION.txt"
	scone_version.h.in
	)
set(CORE_BASE_FILES
	core/HasData.h
	core/HasName.h
	core/HasExternalResources.h
	core/HasExternalResources.cpp
	core/HasSignature.cpp
	core/HasSignature.h
	)
set(CORE_MATH_FILES
	core/Angle.h
	core/Vec3.h
	core/Quat.h
	core/Delayer.h
	core/math.h
	core/Range.h
	core/Statistic.h
	core/TimedValue.h
	core/Function.h
	core/PieceWiseConstantFunction.cpp
	core/PieceWiseConstantFunction.h
	core/PieceWiseLinearFunction.cpp
	core/PieceWiseLinearFunction.h
	core/Polynomial.h
	core/Polynomial.cpp
	core/ConstantFunction.h
	core/ConstantFunction.cpp
	core/SineWave.h
	core/SineWave.cpp
	core/RaisedCosine.h
	core/RaisedCosine.cpp
	core/RaiseFallProfileFunction.h
	core/RaiseFallProfileFunction.cpp
	core/InterpolataryCubicSpline.h
	core/InterpolataryCubicSpline.cpp
	core/Bezier.h
	core/Bezier.cpp
	core/random_tools.h
	)
set(CORE_SYSTEM_FILES
	core/FactoryProps.h
	core/Factories.h
	core/Factories.cpp
	core/memory_tools.h
	core/ResourceCache.h
	core/Profiler.cpp
	core/Profiler.h
	core/profiler_config.h
	core/profiler_config.cpp
	core/Exception.h
	core/platform.h
	core/Log.cpp
	core/Log.h
	core/version.cpp
	core/version.h
	core/types.h
	core/system_tools.cpp
	core/system_tools.h
	core/system_tests.cpp
	core/system_tests.h
	core/Settings.cpp
	core/Settings.h
	core/scone_settings_schema.h
	)
set(CORE_STORAGE_FILES
	core/Storage.h
	core/StorageIo.h
	core/StorageIo.cpp
	core/PropNode.h
	core/StringMap.h
	)
set(CORE_TOOL_FILES
	core/Event.cpp
	core/Event.h
	core/Benchmark.h
	core/Benchmark.cpp
	core/storage_tools.h
	core/storage_tools.cpp
	core/string_tools.cpp
	core/string_tools.h
	core/GaitCycle.cpp
	core/GaitCycle.h
	core/ModelConverter.cpp
	core/ModelConverter.h
	core/CachedVar.h
	core/ValuePtrMap.h
	core/ExternalResourceContainer.h
	core/ExternalResourceContainer.cpp
	)
set(CS_CONTROLLERS_FILES
	controllers/Controller.cpp
	controllers/Controller.h
	controllers/FeedForwardController.cpp
	controllers/FeedForwardController.h
	controllers/TrackingController.cpp
	controllers/TrackingController.h
	controllers/PerturbationController.cpp
	controllers/PerturbationController.h
	controllers/MirrorController.h
	controllers/MirrorController.cpp
	controllers/NoiseController.h
	controllers/NoiseController.cpp
	controllers/NeuralNetworkController.h
	controllers/NeuralNetworkController.cpp
	controllers/ExternalController.h
	controllers/ExternalController.cpp
	)
set(CS_CONTROLLERS_NC_FILES
	controllers/NeuralController.cpp
	controllers/NeuralController.h
	controllers/Neuron.cpp
	controllers/Neuron.h
	controllers/SensorNeuron.cpp
	controllers/SensorNeuron.h
	controllers/InterNeuron.cpp
	controllers/InterNeuron.h
	controllers/MotorNeuron.cpp
	controllers/MotorNeuron.h
	controllers/PatternNeuron.cpp
	controllers/PatternNeuron.h
	controllers/activation_functions.cpp
	controllers/activation_functions.h
	)
set(CS_CONTROLLERS_REFLEX_FILES
	controllers/ComPivotReflex.cpp
	controllers/ComPivotReflex.h
	controllers/ConditionalMuscleReflex.cpp
	controllers/ConditionalMuscleReflex.h
	controllers/DofReflex.cpp
	controllers/DofReflex.h
	controllers/BodyPointReflex.cpp
	controllers/BodyPointReflex.h
	controllers/BodyOrientationReflex.cpp
	controllers/BodyOrientationReflex.h
	controllers/MuscleReflex.cpp
	controllers/MuscleReflex.h
	controllers/Reflex.cpp
	controllers/Reflex.h
	controllers/ReflexController.cpp
	controllers/ReflexController.h
	controllers/SensorReflex.cpp
	controllers/SensorReflex.h
	)
set(CS_CONTROLLERS_STATE_FILES
	controllers/GaitStateController.cpp
	controllers/GaitStateController.h
	)
set(CS_CONTROLLERS_COMPOSITE_FILES
	controllers/CompositeController.cpp
	controllers/CompositeController.h
	controllers/SequentialController.cpp
	controllers/SequentialController.h
	)
set(CS_CONTROLLERS_SNEL_FILES
	controllers/SpinalController.cpp
	controllers/SpinalController.h
	)
set(MEASURE_FILES
	measures/BalanceMeasure.cpp
	measures/BalanceMeasure.h
	measures/BodyMeasure.cpp
	measures/BodyMeasure.h
	measures/CompositeMeasure.cpp
	measures/CompositeMeasure.h
	measures/DofLimitMeasure.cpp
	measures/DofLimitMeasure.h
	measures/DofMeasure.cpp
	measures/DofMeasure.h
	measures/EffortMeasure.cpp
	measures/EffortMeasure.h
	measures/GaitCycleMeasure.cpp
	measures/GaitCycleMeasure.h
	measures/GaitMeasure.cpp
	measures/GaitMeasure.h
	measures/HeightMeasure.cpp
	measures/HeightMeasure.h
	measures/JumpMeasure.cpp
	measures/JumpMeasure.h
	measures/JointLoadMeasure.cpp
	measures/JointLoadMeasure.h
	measures/MimicMeasure.cpp
	measures/MimicMeasure.h
	measures/MuscleMeasure.cpp
	measures/MuscleMeasure.h
	measures/ReactionForceMeasure.cpp
	measures/ReactionForceMeasure.h
	measures/RangePenalty.h
	measures/Measure.cpp
	measures/Measure.h
	measures/StepMeasure.cpp
	measures/StepMeasure.h
	measures/SimulationMeasure.cpp
	measures/SimulationMeasure.h
	)
set(OPT_API_FILES
	optimization/EsOptimizer.cpp
	optimization/EsOptimizer.h
	optimization/CmaOptimizerSpot.cpp
	optimization/CmaOptimizerSpot.h
	optimization/CmaPoolOptimizer.cpp
	optimization/CmaPoolOptimizer.h
	optimization/MesOptimizer.cpp
	optimization/MesOptimizer.h
	optimization/EvaOptimizer.cpp
	optimization/EvaOptimizer.h
	optimization/Objective.cpp
	optimization/Objective.h
	optimization/Optimizer.cpp
	optimization/Optimizer.h
	optimization/Params.h
	optimization/ParInitSettings.h
	optimization/ModelObjective.cpp
	optimization/ModelObjective.h
	optimization/SimulationObjective.cpp
	optimization/SimulationObjective.h
	optimization/TestObjective.cpp
	optimization/TestObjective.h
	optimization/ImitationObjective.cpp
	optimization/ImitationObjective.h
	optimization/ReplicationObjective.cpp
	optimization/ReplicationObjective.h
	optimization/SimilarityObjective.cpp
	optimization/SimilarityObjective.h
	optimization/opt_tools.cpp
	optimization/opt_tools.h
	)
set(SIM_MODELCOMPONENTS_FILES
	model/Body.cpp
	model/Body.h
	model/ContactForce.cpp
	model/ContactForce.h
	model/DelayBuffer.h
	model/DelayBuffer.cpp
	model/ForceValue.h
	model/Dof.cpp
	model/Dof.h
	model/Joint.cpp
	model/Joint.h
	model/Model.cpp
	model/Model.h
	model/Muscle.cpp
	model/Muscle.h
	model/Ligament.cpp
	model/Ligament.h
	model/Spring.h
	model/Spring.cpp
	model/State.cpp
	model/State.h
	model/ContactGeometry.h
	model/ContactGeometry.cpp
	model/DisplayGeometry.h
	model/StateComponent.h
	model/UserInput.h
	model/symmetry_tools.h
	)
set(SIM_SENSORSANDACTUATORS_FILES
	model/Actuator.cpp
	model/Actuator.h
	model/Sensor.h
	model/SensorDelayAdapter.cpp
	model/SensorDelayAdapter.h
	model/Sensors.cpp
	model/Sensors.h
	)
set(SIM_SIMULATIONANDCONTROL_FILES
	model/Simulation.cpp
	model/Simulation.h
	)
set(SIM_SYSTEM_FILES
	model/model_tools.cpp
	model/model_tools.h
	model/muscle_tools.cpp
	model/muscle_tools.h
	)
set(SIM_TOPOLOGY_FILES
	model/Location.cpp
	model/Location.h
	model/Leg.cpp
	model/Leg.h
	model/Side.h
	model/MuscleId.h
	)

set (SCONELIB_FILES
	${SCONE_CONFIG_FILES}
	${CORE_BASE_FILES}
	${CORE_MATH_FILES}
	${CORE_SYSTEM_FILES}
	${CORE_STORAGE_FILES}
	${CORE_TOOL_FILES}
	
	${CS_CONTROLLERS_FILES}
	${CS_CONTROLLERS_MRC_FILES}
	${CS_CONTROLLERS_NC_FILES}
	${CS_CONTROLLERS_REFLEX_FILES}
	${CS_CONTROLLERS_STATE_FILES}
	${CS_CONTROLLERS_COMPOSITE_FILES}

	${CS_FUNCTIONS_FILES}
	${MEASURE_FILES}
	${CS_TOOLS_FILES}
	
	${OPT_API_FILES}
	
	${SIM_MODELCOMPONENTS_FILES}
	${SIM_SENSORSANDACTUATORS_FILES}
	${SIM_SIMULATIONANDCONTROL_FILES}
	${SIM_SYSTEM_FILES}
	${SIM_TOPOLOGY_FILES}
	)

if (SCONE_SNEL)
	list( APPEND SCONELIB_FILES ${CS_CONTROLLERS_SNEL_FILES} )
endif()

# congifure scone_version.h
configure_file(scone_version.h.in scone_version.h @ONLY)

# add sources to single lib
add_library(sconelib SHARED ${SCONELIB_FILES})

# Require C++17 standard
set_target_properties(sconelib PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

target_include_directories(sconelib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")

target_link_libraries(sconelib xo spot)

set_target_properties(sconelib PROPERTIES PROJECT_LABEL sconelib )

if (SCONE_EXPERIMENTAL_FEATURES)
	target_compile_definitions(sconelib PUBLIC SCONE_EXPERIMENTAL_FEATURES)
endif()

if (SCONE_SNEL)
	target_include_directories(sconelib PUBLIC ${SNEL_INCLUDE_DIR})
	target_compile_definitions(sconelib PUBLIC SCONE_SNEL)
endif()

if (MSVC)
	target_precompile_headers(sconelib PRIVATE <string> <vector> <algorithm> <memory> <limits> <fstream>)
	file (GLOB_RECURSE PRECOMPILED_HEADER_FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_pch.*)
	source_group("CMakePCH" FILES ${PRECOMPILED_HEADER_FILES})

	target_compile_definitions( sconelib PRIVATE SCONE_EXPORTS )
	target_compile_definitions( sconelib PRIVATE _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS )
	target_compile_definitions( sconelib PRIVATE $<$<BOOL:${SCONE_ENABLE_PROFILER}>:SCONE_ENABLE_XO_PROFILING> )
	target_compile_options( sconelib PRIVATE "/MP" ) # multithreaded compilation on MSVC
endif()

install(
	TARGETS sconelib xo spot
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)

# To create folders in Visual Studio.
source_group("tools" FILES ${CORE_BASE_FILES})
source_group("math" FILES ${CORE_MATH_FILES})
source_group("system" FILES ${CORE_SYSTEM_FILES})
source_group("tools" FILES ${CORE_TOOL_FILES})
source_group("tools" FILES ${CORE_STORAGE_FILES})

source_group("controllers" FILES ${CS_CONTROLLERS_FILES})
source_group("controllers" FILES ${CS_SYSTEM_FILES})
source_group("controllers" FILES ${CS_CONTROLLERS_EXPERIMENTAL_FILES})
if (SCONE_SNEL)
	source_group("controllers" FILES ${CS_CONTROLLERS_SNEL_FILES})
endif()
source_group("controllers\\State Control" FILES ${CS_CONTROLLERS_STATE_FILES})
source_group("controllers\\Composite Control" FILES ${CS_CONTROLLERS_COMPOSITE_FILES})
source_group("controllers\\ReflexController" FILES ${CS_CONTROLLERS_REFLEX_FILES})
source_group("controllers\\NeuralController" FILES ${CS_CONTROLLERS_NC_FILES})
source_group("controllers\\Meta Reflex Control" FILES ${CS_CONTROLLERS_MRC_FILES})

source_group("measures" FILES ${MEASURE_FILES})

source_group("optimization" FILES ${OPT_API_FILES})
source_group("optimization" FILES ${OPT_SYSTEM_FILES})

source_group("model" FILES ${SIM_MODELCOMPONENTS_FILES})
source_group("model" FILES ${SIM_SENSORSANDACTUATORS_FILES})
source_group("model" FILES ${SIM_SIMULATIONANDCONTROL_FILES})
source_group("model" FILES ${SIM_SYSTEM_FILES})
source_group("model" FILES ${SIM_TOPOLOGY_FILES})

source_group("" FILES ${SCONE_CONFIG_FILES} )
