#!/usr/bin/env bash
# =============================================================================
# UAV SLAM Performance Benchmark — ROS2 Jazzy / Raspberry Pi 5
# =============================================================================
# Usage:
#   chmod +x benchmark_uav.sh
#   ./benchmark_uav.sh [duration_seconds]
#
# Run AFTER launching the UAV stack:
#   ros2 launch uav_slam_launch slam_only.launch.py
# =============================================================================

DURATION=${1:-60}
LOG_DIR="$HOME/uav_benchmark_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

echo "=============================================="
echo " UAV SLAM Benchmark — ROS2 Jazzy"
echo " Duration: ${DURATION}s  |  Logs: $LOG_DIR"
echo "=============================================="

# 1. CPU + Memory
echo "[1/5] Monitoring CPU + Memory..."
(
  for i in $(seq 1 $DURATION); do
    CPU=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | tr -d '%us,')
    MEM=$(free -m | awk '/Mem:/{print $3}')
    echo "$(date +%s) cpu=${CPU} mem_mb=${MEM}"
    sleep 1
  done
) > "$LOG_DIR/cpu_mem.log" &
CPU_PID=$!

# 2. Temperature
echo "[2/5] Monitoring temperature..."
(
  for i in $(seq 1 $DURATION); do
    if command -v vcgencmd &>/dev/null; then
      TEMP=$(vcgencmd measure_temp | tr -d 'temp=°C')
      THROTTLE=$(vcgencmd get_throttled)
    else
      TEMP=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "N/A")
      THROTTLE="N/A"
    fi
    echo "$(date +%s) temp=${TEMP} throttle=${THROTTLE}"
    sleep 2
  done
) > "$LOG_DIR/temperature.log" &
TEMP_PID=$!

# 3. Topic frequencies
echo "[3/5] Measuring topic frequencies (5s each)..."
{
  echo "=== Topic Hz ==="
  for topic in \
    /camera/color/image_raw \
    /camera/aligned_depth_to_color/image_raw \
    /odom \
    /map \
    /rtabmap/odom \
    /mavros/vision_pose/pose \
    /uav/coverage_path \
    /uav/filtered_detections; do
    echo -n "$topic: "
    timeout 5 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | tail -1 || echo "not publishing"
  done
} > "$LOG_DIR/topic_hz.log"

# 4. TF validation
echo "[4/5] Validating TF tree..."
{
  echo "=== TF Tree ==="
  timeout 5 ros2 run tf2_tools view_frames 2>/dev/null && echo "frames.pdf generated" || echo "tf2_tools failed"
  echo ""
  echo "=== map → base_link ==="
  timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | head -15
  echo ""
  echo "=== odom → base_link ==="
  timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -15
} > "$LOG_DIR/tf_validation.log"

# 5. Node list
echo "[5/5] Capturing node + topic list..."
ros2 node list  > "$LOG_DIR/node_list.log"
ros2 topic list > "$LOG_DIR/topic_list.log"

wait $CPU_PID $TEMP_PID 2>/dev/null

# Summary
echo ""
echo "=============================================="
echo " BENCHMARK RESULTS"
echo "=============================================="

if [ -f "$LOG_DIR/cpu_mem.log" ]; then
  AVG_CPU=$(awk -F'cpu=' '{if(NF>1) {split($2,a," "); sum+=a[1]; count++}} END {if(count>0) printf "%.1f", sum/count}' "$LOG_DIR/cpu_mem.log")
  MAX_CPU=$(awk -F'cpu=' '{if(NF>1) {split($2,a," "); if(a[1]+0>max) max=a[1]+0}} END {printf "%.1f", max}' "$LOG_DIR/cpu_mem.log")
  AVG_MEM=$(awk -F'mem_mb=' '{if(NF>1) {split($2,a," "); sum+=a[1]; count++}} END {if(count>0) printf "%.0f", sum/count}' "$LOG_DIR/cpu_mem.log")
  echo " CPU avg: ${AVG_CPU}%  max: ${MAX_CPU}%"
  echo " RAM avg: ${AVG_MEM} MB"
  if awk "BEGIN{exit !(${MAX_CPU:-0} > 80)}"; then
    echo " WARNING: CPU exceeded 80% — reduce Vis/MaxFeatures or FPS"
  else
    echo " CPU within target (<80%)"
  fi
fi

if [ -f "$LOG_DIR/temperature.log" ]; then
  echo ""
  echo " Last temperature readings:"
  tail -3 "$LOG_DIR/temperature.log"
fi

echo ""
echo " Full logs: $LOG_DIR"
echo "=============================================="
echo " TARGET METRICS:"
echo "   CPU:         < 80%"
echo "   RAM:         < 1500 MB"
echo "   /odom rate:  ~15 Hz"
echo "   /map rate:   ~1 Hz"
echo "   Temp:        < 80°C"
echo "=============================================="
