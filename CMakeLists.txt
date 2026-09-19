cmake_minimum_required(VERSION 3.28)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(odNEAT VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ODNEAT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(ODNEAT_BUILD_DEMOS "Build experiment demos" ON)
option(BUILD_TESTING "Build CTest unit tests" ON)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    if(ODNEAT_WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

add_library(odneat
    src/innovation_clock.cpp
    src/genome.cpp
    src/compatibility.cpp
    src/crossover.cpp
    src/mutation.cpp
    src/network.cpp
    src/energy.cpp
    src/population.cpp
    src/tabu_list.cpp
    src/broadcast.cpp
    src/agent.cpp
    src/tasks/task_energies.cpp
    src/sim/simulation.cpp
)
target_include_directories(odneat PUBLIC include)
target_compile_features(odneat PUBLIC cxx_std_23)

if(ODNEAT_BUILD_DEMOS)
    add_executable(run_experiment demos/run_experiment.cpp)
    target_link_libraries(run_experiment PRIVATE odneat)

    add_executable(fault_injection demos/fault_injection.cpp)
    target_link_libraries(fault_injection PRIVATE odneat)

    add_executable(ablation demos/ablation.cpp)
    target_link_libraries(ablation PRIVATE odneat)
endif()

if(BUILD_TESTING)
    include(CTest)
    enable_testing()

    add_executable(odneat_tests
        tests/test_main.cpp
        tests/innovation_clock_test.cpp
        tests/genome_test.cpp
        tests/compatibility_test.cpp
        tests/crossover_test.cpp
        tests/mutation_test.cpp
        tests/network_test.cpp
        tests/energy_test.cpp
        tests/population_test.cpp
        tests/tabu_test.cpp
        tests/broadcast_test.cpp
        tests/agent_test.cpp
        tests/task_energies_test.cpp
        tests/simulation_test.cpp
        tests/integration_test.cpp
    )
    target_link_libraries(odneat_tests PRIVATE odneat)

    function(odneat_add_suite suiteName)
        add_test(NAME ${suiteName} COMMAND odneat_tests ${suiteName})
    endfunction()

    odneat_add_suite(test_innovation_clock)
    odneat_add_suite(test_genome)
    odneat_add_suite(test_compatibility)
    odneat_add_suite(test_crossover)
    odneat_add_suite(test_mutation)
    odneat_add_suite(test_network)
    odneat_add_suite(test_energy)
    odneat_add_suite(test_population)
    odneat_add_suite(test_tabu)
    odneat_add_suite(test_broadcast)
    odneat_add_suite(test_agent)
    odneat_add_suite(test_task_energies)
    odneat_add_suite(test_simulation)
    odneat_add_suite(test_integration)
endif()
