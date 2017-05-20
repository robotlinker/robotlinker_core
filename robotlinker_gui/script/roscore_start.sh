echo "starting the ros server..."

#kill $(ps aux | grep '[r]oscore' | awk '{print $2}')

killall -9 roscore
killall -9 rosmaster
rosnode kill -a

roscore
