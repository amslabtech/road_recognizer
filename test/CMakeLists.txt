add_rostest_gtest(road_recognizer_test
    test/road_recognizer.test
    src/road_recognizer_test.cpp
    ../src/road_recognizer.cpp
)
target_link_libraries(road_recognizer_test
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

catkin_add_gtest(peak_detector_test
    src/peak_detector_test.cpp
)
target_link_libraries(peak_detector_test
    ${catkin_LIBRARIES}
    peak_detector
)

add_rostest_gtest(intersection_detection_test
    test/intersection_detection.test
    src/intersection_detection_test.cpp
    ../src/intersection_detector.cpp
)
target_link_libraries(intersection_detection_test
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    peak_detector
)