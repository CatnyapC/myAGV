"""Patch the installed ROS1 chassis source; rebuild/restart it after this command."""
from pathlib import Path
import sys


def patch(source):
    if 'MYAGV_WEB_WATCHDOG' in source:
        source = source.replace('ros::Rate loop_rate(100);', 'ros::WallRate loop_rate(100);')
    else:
        source = patch_watchdog(source)
    if 'MYAGV_WEB_MOTOR_RECOVERY' not in source:
        old = 'n.setParam("/myagv_web/driver_watchdog", 1);'
        if source.count(old) != 1:
            raise ValueError('Unexpected driver watchdog declaration')
        source = source.replace(old, old + '''
    n.setParam("/myagv_web/motor_restore_requested", false); // MYAGV_WEB_MOTOR_RECOVERY
    n.setParam("/myagv_web/motor_overcurrent", false);
    n.setParam("/myagv_web/driver_recovery", 1);''')
    if '"/myagv_web/angular_speed_fault"' not in source:
        old = 'n.setParam("/myagv_web/driver_watchdog", 1);'
        if source.count(old) != 1:
            raise ValueError('Unexpected driver watchdog declaration')
        source = source.replace(old, old + '\n    n.setParam("/myagv_web/angular_speed_fault", false);')
    return source


def patch_watchdog(source):
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


def patch_recovery(source):
    if 'MYAGV_WEB_MOTOR_RECOVERY' in source:
        return source
    start = source.index('void MyAGV::restoreRun()')
    end = source.index('\n}', start) + 2
    original = source[start:end]
    if original.count('std::cin') != 1 or original.count('restore();') != 1:
        raise ValueError('Unexpected motor recovery source; refused patch')
    return source[:start] + '''void MyAGV::restoreRun()
{
    // MYAGV_WEB_MOTOR_RECOVERY: latch until an explicit, physically verified recovery.
    writeSpeed(0, 0, 0);
    n.setParam("/myagv_web/motor_restore_requested", false);
    n.setParam("/myagv_web/motor_overcurrent", true);
    while (ros::ok()) {
        bool requested = false;
        n.getParam("/myagv_web/motor_restore_requested", requested);
        if (requested) {
            restore();
            writeSpeed(0, 0, 0);
            n.setParam("/myagv_web/motor_restore_requested", false);
            n.setParam("/myagv_web/motor_overcurrent", false);
            return;
        }
        ros::WallDuration(0.05).sleep();
    }
}

''' + source[end:]


def patch_speed_guard(source):
    if 'MYAGV_WEB_ANGULAR_SPEED_GUARD' in source:
        return source
    old = '''    writeSpeed(linearX, linearY, angularZ);
    readSpeed(); // easy to report error'''
    preflight = '''    // MYAGV_WEB_ANGULAR_SPEED_GUARD: raw MCU commands are not calibrated SI speeds.
    static bool speedFault = false;
    if (!std::isfinite(linearX) || !std::isfinite(linearY) || !std::isfinite(angularZ)) {
        speedFault = true;
        n.setParam("/myagv_web/angular_speed_fault", true);
    }
'''
    condition = '''!speedFault && angularZ != 0 &&
            std::abs(vtheta) > std::max(0.15, 2 * std::abs(angularZ) + 0.05)'''
    latch = '''
        ROS_ERROR("Chassis angular overspeed: requested %.3f rad/s, measured %.3f rad/s; motion locked", angularZ, vtheta);
        speedFault = true;
        n.setParam("/myagv_web/angular_speed_fault", true);
'''
    new = preflight + '''    if (speedFault) linearX = linearY = angularZ = 0;
    writeSpeed(linearX, linearY, angularZ);
    bool fresh = readSpeed();
    if (fresh && ''' + condition + ') {' + latch + '''    }
    if (!fresh || speedFault) writeSpeed(0, 0, 0);'''
    if source.count(old) != 1:
        old = '''        writeSpeed(linearX, linearY, angularZ);
        publisherOdom(dt);'''
        new = preflight + '    if (' + condition + ') {' + latch + '''    }
    if (speedFault) linearX = linearY = angularZ = 0;
    writeSpeed(linearX, linearY, angularZ);
        publisherOdom(dt);'''
        closing = '    lastTime = currentTime;\n}'
        if source.count(closing) != 1:
            raise ValueError('Unexpected driver feedback loop; refused partial patch')
        source = source.replace(closing, '    else { writeSpeed(0, 0, 0); }\n' + closing)
    if source.count(old) != 1 or source.count('#include "myagv_odometry/myAGV.h"') != 1:
        raise ValueError('Unexpected driver velocity source; refused partial patch')
    return source.replace('#include "myagv_odometry/myAGV.h"',
                          '#include "myagv_odometry/myAGV.h"\n#include <algorithm>\n#include <cmath>').replace(old, new)


if __name__ == '__main__':
    path = Path(sys.argv[1])
    library = path.with_name('myAGV.cpp')
    changes = [(path, path.read_text()), (library, library.read_text())]
    updates = [patch(changes[0][1]), patch_speed_guard(patch_recovery(changes[1][1]))]
    for (path, original), updated in zip(changes, updates):
        if updated == original:
            continue
        backup = path.with_suffix(path.suffix + '.before-web-watchdog')
        if backup.exists() and 'MYAGV_WEB_WATCHDOG' not in original:
            raise SystemExit('Backup exists; inspect previous installation first')
        if not backup.exists():
            backup.write_text(original)
        path.write_text(updated)
    print('Driver watchdog, angular speed guard and motor recovery ready; rebuild and restart chassis driver')
