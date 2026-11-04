"""Patch the installed ROS1 chassis source; rebuild/restart it after this command."""
from pathlib import Path
import sys
import re


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
    source = source.replace('n.subscribe("cmd_vel", 50, cmdCallback)', 'n.subscribe("cmd_vel", 1, cmdCallback)')
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
    if 'MYAGV_WEB_MOTION_V2' in source:
        return source
    # Remove the earlier angular-only guard before installing the shared gate.
    if 'MYAGV_WEB_ANGULAR_SPEED_GUARD' in source:
        legacy = re.compile(r'    // MYAGV_WEB_ANGULAR_SPEED_GUARD:.*?    if \(!fresh \|\| speedFault\) writeSpeed\(0, 0, 0\);', re.S)
        source, replaced = legacy.subn('    writeSpeed(linearX, linearY, angularZ);\n    readSpeed(); // easy to report error', source)
        if replaced != 1:
            raise ValueError('Unexpected previous angular guard; refused upgrade')
    anchor = '#include "myagv_odometry/myAGV.h"'
    if source.count(anchor) != 1:
        raise ValueError('Unexpected driver includes')
    legacy = '    writeSpeed(linearX, linearY, angularZ);\n    readSpeed(); // easy to report error'
    modern = re.compile(r'    if \(true ==\s+readSpeed\(\)\)\s*\{\s*writeSpeed\(linearX, linearY, angularZ\);')
    block = '''    // MYAGV_WEB_MOTION_V2
    static MyAGVMotionSafety safety;
    const auto clockNow = []() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    const auto previous = safety.sent;
    const bool wasBraking = safety.braking;
    const bool wasFault = safety.fault != nullptr;
    auto output = safety.command({{linearX, linearY, angularZ}}, clockNow());
    writeSpeed(output[0], output[1], output[2]);
    bool fresh = readSpeed();
    safety.observe({{vx, vy, vtheta}}, fresh, clockNow());
    if (safety.fault) {
        writeSpeed(0, 0, 0);
        n.setParam("/myagv_web/angular_speed_fault", true);
        if (!wasFault) ROS_ERROR("Chassis motion locked: %s; requested %.3f %.3f %.3f; feedback %.3f %.3f %.3f",
            safety.fault, linearX, linearY, angularZ, vx, vy, vtheta);
    }
    if (output != previous || safety.braking != wasBraking) {
        ROS_INFO("Chassis command requested %.3f %.3f %.3f; sent %.3f %.3f %.3f; encoded %d %d %d; feedback %.3f %.3f %.3f; braking %d",
            linearX, linearY, angularZ, output[0], output[1], output[2],
            static_cast<int>(output[0]*100)+128, static_cast<int>(output[1]*100)+128,
            static_cast<int>(output[2]*100)+128, vx, vy, vtheta, safety.braking);
    }
'''
    if source.count(legacy) == 1:
        source = source.replace(legacy, block + '    if (!fresh) return;')
    elif len(modern.findall(source)) == 1:
        source = modern.sub(lambda _: block + '    if (fresh) {', source)
    else:
        raise ValueError('Unexpected driver velocity source; refused partial patch')
    return source.replace(anchor, anchor + '\n#include "myagv_motion_safety.h"\n#include <chrono>')


def patch_serial_timeout(source):
    if 'MYAGV_WEB_SERIAL_DEADLINE' in source:
        return source
    start = source.index('bool MyAGV::readSpeed()')
    end = source.index('\n}', start) + 2
    body = source[start:end]
    body = body.replace('{', '{\n    // MYAGV_WEB_SERIAL_DEADLINE\n    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);', 1)
    pattern = r'ret = boost::asio::read\(sp, boost::asio::buffer\((buf_header|buf)\), er2\);'
    body, count = re.subn(pattern, lambda m:
        'ret = myagvReadExact(sp.native_handle(), %s, sizeof(%s), deadline);\n    if (ret != sizeof(%s)) return false;' % (m[1], m[1], m[1]), body)
    if count != 3 or 'boost::asio::read(' in body:
        raise ValueError('Unexpected serial reader; refused partial patch')
    # A header must consist of adjacent bytes; a payload FE is not a second header.
    body = body.replace('            if (buf_header[0] != header[0]) {\n                continue;\n            }', '            if (buf_header[0] != header[0]) break;')
    body = body.replace('        header_found = true;', '        header_found = header_2_found;')
    return (source[:start] + body + source[end:]).replace(
        '#include "myagv_odometry/myAGV.h"', '#include "myagv_odometry/myAGV.h"\n#include "myagv_serial_read.h"')


def install(path):
    library = path.with_name('myAGV.cpp')
    changes = [(path, path.read_text()), (library, library.read_text())]
    updates = [patch(changes[0][1]), patch_serial_timeout(patch_speed_guard(patch_recovery(changes[1][1])))]
    # Validate every backup before writing anything; never overwrite earlier backups.
    for (target, original), updated in zip(changes, updates):
        backup = target.with_suffix(target.suffix + '.before-web-motion-v2')
        if original != updated and backup.exists() and backup.read_text() != original:
            raise ValueError('Motion backup differs; inspect previous installation first: ' + str(backup))
    headers = {name: Path(__file__).with_name(name).read_text()
               for name in ('myagv_motion_safety.h', 'myagv_serial_read.h')}
    for name, content in headers.items():
        path.with_name(name).write_text(content)
    for (target, original), updated in zip(changes, updates):
        if updated == original:
            continue
        backup = target.with_suffix(target.suffix + '.before-web-motion-v2')
        if not backup.exists():
            backup.write_text(original)
        target.write_text(updated)


if __name__ == '__main__':
    install(Path(sys.argv[1]))
    print('Chassis motion gate, bounded feedback and recovery ready; rebuild before restart')
