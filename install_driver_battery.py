"""Expose battery voltage from the installed ROS driver's existing MCU stream."""
from pathlib import Path
import sys


def patch(source):
    if 'MYAGV_WEB_BATTERY' in source:
        return source
    changes = {
        '#include "myagv_odometry/myAGV.h"':
            '#include "myagv_odometry/myAGV.h"\n#include <std_msgs/Float32.h>\n'
            'static ros::Publisher batteryVoltage; // MYAGV_WEB_BATTERY',
        'pub_v = n.advertise<std_msgs::Int8>("Voltage", 1000);':
            'pub_v = n.advertise<std_msgs::Int8>("Voltage", 1000);\n'
            '    batteryVoltage = n.advertise<std_msgs::Float32>("/myagv/battery_voltage", 1);',
        'vx = (static_cast<double>(buf[index]) - 128.0) * 0.01;':
            '// MCU bytes 16/17 contain the two pack voltages in tenths of a volt.\n'
            '    std_msgs::Float32 battery;\n'
            '    battery.data = (buf[16] > buf[17] ? buf[16] : buf[17]) / 10.0f;\n'
            '    batteryVoltage.publish(battery);\n\n'
            '    vx = (static_cast<double>(buf[index]) - 128.0) * 0.01;',
    }
    if 'unsigned char buf[27]' not in source:
        raise ValueError('Unsupported MCU frame; expected 27-byte driver payload')
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
        backup = path.with_suffix(path.suffix + '.before-web-battery')
        if backup.exists():
            raise SystemExit('Backup exists; inspect previous installation first')
        backup.write_text(original)
        path.write_text(updated)
