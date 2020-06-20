add_rostest_gtest(road_recognizer_test
    test/road_recognizer.test
    src/road_recognizer_test.cpp
    ../src/road_recognizer.cpp
)
target_link_libraries(road_recognizer_test
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

catkin_add_gtest(beam_model_test
    src/beam_model_test.cpp
)
target_link_libraries(beam_model_test
    ${catkin_LIBRARIES}
    beam_model
)

catkin_add_gtest(intersection_detection_test
    src/intersection_detection_test.cpp
    ../src/intersection_detector.cpp
)
target_link_libraries(intersection_detection_test
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    beam_model
)