cmake_minimum_required(VERSION 3.20)
project(HybridRocket6DoF VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hrocket_core
    src/barrowman.cpp
    src/csv.cpp
    src/dispersion.cpp
    src/io.cpp
    src/simulator.cpp
)
target_include_directories(hrocket_core PUBLIC include)

add_executable(hrocket_cli src/main_cli.cpp)
target_link_libraries(hrocket_cli PRIVATE hrocket_core)

if(WIN32)
    add_executable(hrocket_gui WIN32 src/gui_win32.cpp)
    target_link_libraries(hrocket_gui PRIVATE hrocket_core user32 gdi32 comdlg32)
    if(MINGW)
        target_link_options(hrocket_gui PRIVATE -municode)
    endif()
endif()

enable_testing()
add_executable(hrocket_tests tests/core_tests.cpp)
target_link_libraries(hrocket_tests PRIVATE hrocket_core)
add_test(NAME hrocket_tests COMMAND hrocket_tests)

install(TARGETS hrocket_core hrocket_cli
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
if(WIN32)
    install(TARGETS hrocket_gui RUNTIME DESTINATION bin)
endif()
install(DIRECTORY samples DESTINATION .)
install(FILES README.md AGENTS.md skills.md DESTINATION .)
