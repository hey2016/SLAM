add_executable(run_frontend_offline
        run_frontend_offline.cc
)

target_link_libraries(run_frontend_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loop_offline
        run_loop_offline.cc
)

target_link_libraries(run_loop_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_offline
        run_slam_offline.cc
)

target_link_libraries(run_slam_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_online
        run_slam_online.cc
)

target_link_libraries(run_slam_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_offline
        run_loc_offline.cc
)

target_link_libraries(run_loc_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(test_ui
        test_ui.cc
)

target_link_libraries(test_ui
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(check_miao_loop_no_commit
        check_miao_loop_no_commit.cc
)

target_link_libraries(check_miao_loop_no_commit
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(check_source_scan_accum_selection
        check_source_scan_accum_selection.cc
)

target_link_libraries(check_source_scan_accum_selection
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(check_adjacent_pose_gate
        check_adjacent_pose_gate.cc
)

target_link_libraries(check_adjacent_pose_gate
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

install(TARGETS run_slam_offline run_slam_online run_loc_offline run_loc_online check_miao_loop_no_commit
        check_source_scan_accum_selection check_adjacent_pose_gate
        DESTINATION lib/${PROJECT_NAME})
