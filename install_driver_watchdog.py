"""Patch the installed ROS1 chassis source; rebuild/restart it after this command."""
from pathlib import Path
import sys


def patch(source):
    if 'MYAGV_WEB_WATCHDOG' in source:
        return source.replace('ros::Rate loop_rate(100);', 'ros::WallRate loop_rate(100);')
    changes = {
        'double angularZ = 0.0;': 'double angularZ = 0.0;\nros::WallTime lastCommand; // MYAGV_WEB_WATCHDOG',
        'linearX = msg.linear.x;': 'lastCommand = ros::WallTime::now();\n\tlinearX = msg.linear.x;',
        'ros::Rate loop_rate(100);': 'ros::WallRate loop_rate(100);\n\tn.setParam("/myagv_web/driver_watchdog", 1);',
        'myAGV.execute(linearX, linearY, angularZ);':
            'if ((ros::WallTime::now() - lastCommand).toSec() > 0.5)\n'
            '\t\t\tlinearX = linearY = angularZ = 0.0;\n'
            '\t\tmyAGV.execute(linearX, linearY, angularZ);',
        '\treturn 0;': '\tmyAGV.execute(0, 0, 0);\n\treturn 0;',
    }
    for old, new in changes.items():
        if source.count(old) != 1:
            raise ValueError('Unexpected driver source; refused partial patch: ' + old)
        source = source.replace(old, new)
    return source


if __name__ == '__main__':
    path = Path(sys.argv[1])
    original = path.read_text()
    updated = patch(original)
    if updated != original:
        backup = path.with_suffix(path.suffix + '.before-web-watchdog')
        if backup.exists() and 'MYAGV_WEB_WATCHDOG' not in original:
            raise SystemExit('Backup exists; inspect previous installation first')
        if not backup.exists():
            backup.write_text(original)
        path.write_text(updated)
    print('Driver watchdog source ready; rebuild and restart chassis driver')
