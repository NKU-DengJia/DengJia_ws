std_msgs/Header header  
float64 vel_req             # 表示期望的纵向速度
float64 turn_radius_req     # 表示期望的转弯半径
bool steer_mod_req            # 0中心转向，1正常行驶
float64 center_steer_req      # 中心转向请求，-180 deg ~ 180 deg，目标航向角