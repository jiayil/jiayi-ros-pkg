Commands to run  

roscore  
~~roslaunch tracking_mapping camera_node.launch~~  
rosrun rviz rviz  

rosrun tracking_mapping broadcast_video num  
num: none default video  
     0    cam  
     1    video  
     2    image  
or slideshow  
rosrun tracking_mapping broadcast_video data/metric/image_0_0_770_ 0 5 .jpg  

rosrun tracking_mapping tracking_mapping _image_transport:=compressed  


