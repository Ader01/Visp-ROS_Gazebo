Использование библиотеки алгоритмов распознавания  QR кодов Visp в Gazebo ROS 


  Скачиваем пакеты
$ git clone https://github.com/lagadic/visp_ros.git (master branch)
$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg visp_ros
$ git clone https://github.com/lagadic/visp.git

Создаем соответствующие пакеты visp_ros  и visp в своем рабочем пространстве

$ roslaunch sova_space simple_cam_gazebo.launch world:=... 
$ rosrun visp_app ros-grabber