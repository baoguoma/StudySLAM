add_executable(main main.cpp)
target_link_libraries(main test_output)
target_link_libraries(main ${THIRD_PARTY_LIBS})