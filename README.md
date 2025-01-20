


To get the foxglove bridge working use the following command. This will make a websocket available so hat foxglove can coenct to it for easier introspection

`ros2 launch foxglove_bridge foxglove_bridge_launch.xml `

to replay the bag file use the following

`ros2 bag play ./bag_files/case_study/case_study.db3 --loop`

to start the module use the following, this will remap the topics in the bag file to simple `camera_X/image` topics

```
ros2 run dc_abyss_solutions_test multi_camera_node --ros-args \
    --remap /camera_1/image:=/platypus/camera_1/dec/manual_white_balance \
    --remap /camera_2/image:=/platypus/camera_2/dec/manual_white_balance \
    --remap /camera_3/image:=/platypus/camera_3/dec/manual_white_balance 
```