std_msgs/Header header

# 线段两端点（激光雷达坐标系下的直角坐标 x,y,z）
geometry_msgs/Point start
geometry_msgs/Point end
geometry_msgs/Point midpoint

float32 length
float32 bearing_start
float32 bearing_end
uint8   ring
uint16  num_points
float32 avg_intensity