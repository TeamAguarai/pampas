cmake_minimum_required(VERSION 3.31)

project(Aguarai)

# Buscar automáticamente todos los archivos en las carpetas src e include
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

# Crear una biblioteca estática
add_library(vehicle_control STATIC ${SOURCES} ${HEADERS})

target_include_directories(Aguarai PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)