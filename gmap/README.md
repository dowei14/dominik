# openslam gmapping, map_server, navigation


requirements:
git clone https://github.com/ros-perception/openslam_gmapping.git
git clone https://github.com/ros-planning/navigation.git -b indigo-devel
git clone https://github.com/ros-perception/slam_gmapping.git
libsdl-image1.2-dev


frobomind/frobomind_make /dominik/gmap
dominik/gmap/bin/frobit_gmap

rosrun map_server map_saver -f mymap

rosrun map_server map_server mymap.yaml

rosrun amcl amcl scan:=base_scan
