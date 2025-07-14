#!/bin/bash

echo "🧪 Testing Clean Launch System..."
echo "=================================="

# Apply the same environment fixes
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=20  # Only warnings and errors
export RCUTILS_COLORIZED_OUTPUT=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null

# Source ROS2 environment
source ~/bcr_ws/install/setup.bash

echo "✅ Environment configured - Starting basic test..."

# Test just one static transform publisher with new format
echo "🔧 Testing static transform publisher..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id base_link --child-frame-id laser &
TEST_PID=$!

echo "📊 Running for 10 seconds to check for errors..."
sleep 10

echo "🛑 Stopping test..."
kill $TEST_PID 2>/dev/null

echo "✅ Test completed! Check above output for any remaining errors."
echo "If you see much fewer/no RTPS_TRANSPORT_SHM errors, the fix is working!" 