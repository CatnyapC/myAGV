const path = require('node:path');
const setup = 'source /opt/ros/noetic/setup.bash && source "$HOME/myagv_ros/devel/setup.bash" && ';
const processConfig = (name, command) => ({
  name, cwd: __dirname, script: '/bin/bash', interpreter: 'none',
  args: ['-c', setup + command], autorestart: true, exp_backoff_restart_delay: 1000,
});

module.exports = { apps: [
  processConfig('myagv-ros', '/usr/bin/python3 -c "import RPi.GPIO as G; G.setmode(G.BCM); G.setup(21, G.OUT); G.output(21, G.HIGH); G.setup(20, G.OUT); G.output(20, G.HIGH)" && exec roslaunch myagv_odometry myagv_active.launch'),
  processConfig('myagv-localization', 'exec roslaunch --wait ./web_hardware.launch map_file:="${MYAGV_MAP_FILE:-$HOME/maps/room.yaml}" mapping:="$(test -f web_runtime/mapping.json && echo true || echo false)"'),
  { ...processConfig('myagv-bridge', 'exec /usr/bin/python3 ros_web_bridge.py'),
    env: { MYAGV_HARDWARE_DIR: path.join(__dirname, 'web_runtime/ros'),
           MYAGV_FRONT_DEVICE: process.env.MYAGV_FRONT_DEVICE || '/dev/video0',
           MYAGV_ARM_DEVICE: process.env.MYAGV_ARM_DEVICE || '/dev/video1' } },
] };
