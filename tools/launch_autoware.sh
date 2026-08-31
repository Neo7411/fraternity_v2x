echo "Autoware környezet betöltése..."
source /opt/autoware/setup.bash

echo "Planning simulator indítása a háttérben..."
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/maps/highway2 &