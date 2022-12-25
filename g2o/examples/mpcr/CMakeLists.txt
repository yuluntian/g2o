add_executable(multiple_point_clouds_registration
        multiple_point_clouds_registration.cpp
        )

target_link_libraries(multiple_point_clouds_registration core types_slam3d solver_dense)
if(CHOLMOD_FOUND)
    target_link_libraries(multiple_point_clouds_registration solver_cholmod)
else()
    target_link_libraries(multiple_point_clouds_registration solver_eigen)
endif()