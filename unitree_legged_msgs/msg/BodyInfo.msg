Header header
geometry_msgs/Point pos_act           #actual position x y z
geometry_msgs/Quaternion quat_act     #actual orientation in quaternion
geometry_msgs/Vector3 euler_act       #actual orientation in euler angles
geometry_msgs/Twist vel_act           #actual linear velocity x y z and angular velocity around x y z
geometry_msgs/Twist acc_act           #actual linear acceleration x y z and angular acceleration around x y z

geometry_msgs/Point pos_des           #desired position x y z
geometry_msgs/Vector3 euler_des       #desired orientation in euler angles
geometry_msgs/Twist vel_des           #desired linear velocity x y z and angular velocity around x y z
geometry_msgs/Twist acc_des           #desired linear acceleration x y z and angular acceleration around x y z

geometry_msgs/Point test
geometry_msgs/Point test1
geometry_msgs/Point test2
geometry_msgs/Point test3
geometry_msgs/Point test4
geometry_msgs/Point test5

unitree_legged_msgs/StateError state_error
float32 pos_z_global                  #body pose z axis

