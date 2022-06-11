cmake_minimum_required(VERSION 3.9.0)

project(920control) 

add_executable(${PROJECT_NAME} 920fsm.c)

target_link_libraries(${PROJECT_NAME} bcm2835) 
