cmake_minimum_required(VERSION 3.28)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(odNEAT VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

option(ODNEAT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(ODNEAT_BUILD_DEMOS "Build experiment demos" ON)
option(ODNEAT_BUILD_EXAMPLES "Build library usage examples" ON)
option(ODNEAT_USE_SYSTEM_GLAZE "Use an installed glaze package instead of FetchContent" OFF)
option(ODNEAT_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer (Clang/GCC, via the sanitize preset)" OFF)
option(BUILD_TESTING "Build CTest unit tests" ON)

if(MSVC)
    add_compile_options(/W4 /EHsc)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    if(ODNEAT_WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

if(ODNEAT_SANITIZE)
    if(MSVC)
        message(FATAL_ERROR "ODNEAT_SANITIZE requires Clang or GCC; pass /fsanitize=address manually for MSVC-only AddressSanitizer builds")
    endif()
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

if(ODNEAT_USE_SYSTEM_GLAZE)
    find_package(glaze 8.4.0 REQUIRED)
else()
    FetchContent_Declare(
        glaze
        GIT_REPOSITORY https://github.com/stephenberry/glaze.git
        GIT_TAG v8.4.0
        GIT_SHALLOW TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(glaze)
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
    src/serialization.cpp
    src/tasks/task_energies.cpp
    src/sim/simulation.cpp
)
add_library(odneat::odneat ALIAS odneat)
target_include_directories(odneat PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(odneat PUBLIC cxx_std_23)
# Glaze is a private header-only backend: needed to build serialization.cpp but
# never appears in public headers, so it must not propagate to install consumers.
target_link_libraries(odneat PRIVATE $<BUILD_INTERFACE:glaze::glaze>)

if(MSVC)
    # Glaze requires a conformant preprocessor on MSVC.
    target_compile_options(odneat PRIVATE /Zc:preprocessor)
endif()

if(ODNEAT_BUILD_DEMOS)
    add_executable(run_experiment demos/run_experiment.cpp)
    target_link_libraries(run_experiment PRIVATE odneat)

    add_executable(fault_injection demos/fault_injection.cpp)
    target_link_libraries(fault_injection PRIVATE odneat)

    add_executable(ablation demos/ablation.cpp)
    target_link_libraries(ablation PRIVATE odneat)
endif()

if(ODNEAT_BUILD_EXAMPLES)
    add_executable(minimal_embed examples/minimal_embed.cpp)
    target_link_libraries(minimal_embed PRIVATE odneat)
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
        tests/serialization_test.cpp
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
    odneat_add_suite(test_serialization)

    if(ODNEAT_BUILD_EXAMPLES)
        # The embed example doubles as a smoke test: it must keep building, running
        # and round-tripping a checkpoint (also covers the odneat.h umbrella header).
        add_test(NAME example_minimal_embed COMMAND minimal_embed)
    endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS odneat EXPORT odneatTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(EXPORT odneatTargets FILE odneatTargets.cmake NAMESPACE odneat:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/odneat)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/odneatConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/odneatConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/odneat
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/odneatConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/odneatConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/odneatConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/odneat
)
