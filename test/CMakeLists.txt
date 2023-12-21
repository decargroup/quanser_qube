# https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md
# vcpkg method in x86 Native Tools terminal
# vcpkg.exe install catch2:x64-windows -> get 64 bit version of Catch2

find_package(Catch2 3 REQUIRED)

add_executable(TestPrbs test_prbs.cpp)
set_property(TARGET TestPrbs PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
target_include_directories(TestPrbs PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(TestPrbs Prbs Catch2::Catch2WithMain)
add_test(NAME TestPrbs COMMAND TestPrbs)

add_executable(TestLagBuffer test_lag_buffer.cpp)
set_property(TARGET TestLagBuffer PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
target_include_directories(TestLagBuffer PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(TestLagBuffer LagBuffer LtiSys Catch2::Catch2WithMain)
add_test(NAME TestLagBuffer COMMAND TestLagBuffer)