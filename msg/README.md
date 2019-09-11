# Messages
## OtsuBinary.msg
- bool emergency
  - when "sum_diff_from_avr"(refer to "intensity_partition_node.cpp") is more than ".otsubinary_sum_of_diff_from_avr_threshold", ".emergency" is "true"
- float32 range_division_num
  - the number of division of range, which is used for setting polar grid cell
- float32 theta_division_num
  - the number of division of horizontal angle, which is used for setting polar grid cell
- float32 range_max
  - the maximum range to recognize road, which is used for setting polar grid cell
- float32 otsubinary_separation_threshold
  - the threshold for judging the validity of classification of point cloud
- float32 otsubinary_sum_of_diff_from_avr_threshold
  - the sum of ".analysis.otsubinary_diff_from_thresholds_avr"
- road_recognizer/Intensity[] intensity
  - the messages about intensity at each range
- road_recognizer/Analysis[] analysis
  - the messages about analysis of otsu binarization

### Intensity.msg
- threshold
  - the threshold for classifing point cloud on the grass
- min
  - the value of minimum intensity of obtained points
- max
  - the value of maximum intensity of obtained points

### Analysis.msg
- float32 separation
  - the degree of separation
- float32 skewness
  - the skewness of whole histogram of intensity
-  float32 intensity_std_deviation
  - the deviation of whole histogram of intensity
- float32 otsubinary_diff_from_thresholds_avr
  - the value of difference between ".intensity.threshold" and "range_mu_otsu"(refer to intensity_partition_node.cpp)
- road_recognizer/Distribution[] distribution
  - the message about histogram of intensity
#### Distribution.msg
- float32 intensity
  - the frequency of each intensity
