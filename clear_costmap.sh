#!/bin/bash
x=1
y=0

while [ $x -gt $y ]
do
echo "clearing cost maps carter 3"
rosservice call /move_base_carter3/clear_costmaps "{}"
sleep 30
echo "clearing cost maps carter 2"
rosservice call /move_base_carter2/clear_costmaps "{}"
sleep 30
echo "clearing cost maps carter 1"
rosservice call /move_base_carter1/clear_costmaps "{}"
sleep 180
done
