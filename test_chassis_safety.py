"""Native chassis transitions, feedback deadlines and patch upgrades; no hardware motion."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from install_driver_watchdog import install, patch_serial_timeout, patch_speed_guard

ROOT = Path(__file__).parent


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
class ChassisSafetyTest(unittest.TestCase):
    def run_cpp(self, code):
        with tempfile.TemporaryDirectory() as directory:
            source, binary = Path(directory) / 'check.cpp', Path(directory) / 'check'
            source.write_text(code)
            subprocess.run(['c++', '-std=c++11', '-pthread', '-I', str(ROOT), str(source), '-o', str(binary)],
                           check=True, capture_output=True)
            subprocess.run([str(binary)], check=True, timeout=10)

    def test_motion_transitions_and_faults(self):
        self.run_cpp(r'''
#include "myagv_motion_safety.h"
#include <cassert>
#include <limits>
using V = MyAGVMotionSafety::Velocity;
const V zero{{0,0,0}}, forward{{.03,0,0}}, reverse{{-.03,0,0}}, turn{{0,0,.03}};
void stopped(MyAGVMotionSafety &g, double start) {
    for (int i=0; i<7; ++i) {
        g.command(zero,start+i*.1);
        g.observe(zero,true,start+i*.1);
    }
}
int main() {
    for (V next : {reverse, turn, V{{0,.03,0}}, V{{.03,0,.03}}}) {
        MyAGVMotionSafety g;
        stopped(g,0);
        // First start also requires a full stop interval after the transition.
        assert(g.command(forward,.7)==zero);
        for (int i=0; i<6; ++i) { g.observe(zero,true,.7+i*.1); g.command(forward,.7+i*.1); }
        assert(g.command(forward,1.21)==forward);
        g.observe(forward,true,1.22);
        assert(g.command(next,1.23)==zero);
        g.observe(forward,true,1.24);
        assert(g.command(next,1.3)==zero);
        for (int i=0; i<5; ++i) {
            g.observe(zero,true,1.4+i*.1);
            assert(g.command(next,1.4+i*.1)==zero);
        }
        g.observe(zero,true,1.9);
        assert(g.command(next,1.91)==next);
        assert(!g.fault);
    }
    // Latest request wins during braking; cancelled commands never resume.
    MyAGVMotionSafety g;
    stopped(g,0);
    g.command(turn,.7);
    for(int i=0;i<6;++i) {g.observe(zero,true,.7+i*.1);g.command(zero,.7+i*.1);}
    assert(g.command(zero,1.21)==zero);
    assert(!g.fault);
    // Missing/invalid feedback latches; a later good sample cannot unlock it.
    for (int scenario=0;scenario<5;++scenario) {
        MyAGVMotionSafety bad;
        stopped(bad,0);
        if(scenario==0) bad.observe(zero,false,.7);
        if(scenario==1) bad.observe(V{{0,0,.8}},true,.7);
        if(scenario==2) bad.command(V{{0,0,std::numeric_limits<double>::quiet_NaN()}},.7);
        if(scenario==3) bad.observe(V{{0,0,std::numeric_limits<double>::infinity()}},true,.7);
        if(scenario==4) {bad.command(turn,.7);bad.command(turn,6);}
        assert(bad.fault);
        bad.observe(zero,true,7);
        assert(bad.command(forward,7)==zero);
    }
    // A single zero sample followed by a gap cannot establish a stop.
    MyAGVMotionSafety gap;
    gap.command(turn,0); gap.observe(zero,true,0);
    assert(gap.command(turn,.6)==zero);
    gap.observe(zero,true,.6);
    assert(gap.command(turn,.7)==zero);
    // Overspeed during straight travel and after zero is checked too.
    MyAGVMotionSafety brake;
    stopped(brake,0);
    brake.command(V{{0,0,.3}},.7);
    for(int i=0;i<6;++i) {brake.observe(zero,true,.7+i*.1);brake.command(V{{0,0,.3}},.7+i*.1);}
    brake.command(V{{0,0,.3}},1.21);
    brake.observe(V{{0,0,.3}},true,1.22);
    assert(brake.command(zero,1.23)==zero);
    brake.observe(V{{0,0,.3}},true,1.3); assert(!brake.fault);
    brake.observe(V{{0,0,.3}},true,1.5); assert(brake.fault);
}
''')

    def test_serial_deadline_partial_frame_and_disconnect(self):
        self.run_cpp(r'''
#include "myagv_serial_read.h"
#include <cassert>
#include <sys/socket.h>
#include <thread>
using Clock=std::chrono::steady_clock;
int main() {
    int fd[2]; assert(socketpair(AF_UNIX,SOCK_STREAM,0,fd)==0);
    unsigned char buf[3]; int flags=fcntl(fd[0],F_GETFL);
    auto start=Clock::now();
    assert(myagvReadExact(fd[0],buf,3,start+std::chrono::milliseconds(40))==0);
    assert(Clock::now()-start < std::chrono::milliseconds(500));
    assert(fcntl(fd[0],F_GETFL)==flags);
    std::thread writer([&](){write(fd[1],"a",1); usleep(10000); write(fd[1],"bc",2);});
    assert(myagvReadExact(fd[0],buf,3,Clock::now()+std::chrono::milliseconds(200))==3);
    writer.join(); assert(buf[0]=='a' && buf[2]=='c');
    write(fd[1],"a",1);
    assert(myagvReadExact(fd[0],buf,3,Clock::now()+std::chrono::milliseconds(40))==0);
    close(fd[1]);
    assert(myagvReadExact(fd[0],buf,1,Clock::now()+std::chrono::milliseconds(40))==0);
    assert(fcntl(fd[0],F_GETFL)==flags); close(fd[0]);
}
''')

    def test_generated_execute_both_layouts(self):
        legacy = '''#include "myagv_odometry/myAGV.h"
void MyAGV::execute(double linearX, double linearY, double angularZ) {
    writeSpeed(linearX, linearY, angularZ);
    readSpeed(); // easy to report error
}
'''
        modern = legacy.replace('''    writeSpeed(linearX, linearY, angularZ);
    readSpeed(); // easy to report error''', '''    double dt = 0;
    if (true ==  readSpeed())
    {
        writeSpeed(linearX, linearY, angularZ);
        publisherOdom(dt);
    }
    lastTime = currentTime;''')
        for source in (legacy, modern):
            guarded = patch_speed_guard(source)
            self.assertEqual(guarded, patch_speed_guard(guarded))
            self.run_cpp(r'''
#include <array>
#include <cassert>
#include <thread>
#include <chrono>
#include <vector>
#define ROS_ERROR(...) ((void)0)
#define ROS_INFO(...) ((void)0)
struct MyAGV {
    struct Params {bool fault=false; void setParam(const char*,bool v){fault=v;}} n;
    double vx=0,vy=0,vtheta=0,currentTime=0,lastTime=0;
    bool fresh=true;
    std::vector<std::array<double,3>> commands;
    void writeSpeed(double x,double y,double z){commands.push_back({{x,y,z}});}
    bool readSpeed(){return fresh;}
    void publisherOdom(double){}
    void execute(double,double,double);
};
''' + guarded.replace('#include "myagv_odometry/myAGV.h"', '') + r'''
int main(){
    MyAGV d;
    for(int i=0;i<8;++i){d.execute(0,0,.03);std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    assert(d.commands.back()[2]==.03);
    d.vtheta=.8;d.execute(0,0,.03);
    assert(d.n.fault && d.commands.back()[2]==0);
    d.vtheta=0;d.execute(.03,0,0);
    assert(d.commands.back()[0]==0);
}
''')


class InstallerTest(unittest.TestCase):
    def test_reader_refuses_unknown_layout(self):
        with self.assertRaises(ValueError):
            patch_serial_timeout('bool MyAGV::readSpeed()\n{\n return true;\n}')

    def test_preflight_failure_does_not_change_first_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'myAGVSub.cpp'
            source='invalid driver source'
            path.write_text(source)
            path.with_name('myAGV.cpp').write_text('invalid library')
            with self.assertRaises(ValueError): install(path)
            self.assertEqual(path.read_text(),source)
            self.assertFalse(path.with_suffix('.cpp.before-web-motion-v2').exists())


READER = '''#include "myagv_odometry/myAGV.h"
bool MyAGV::readSpeed()
{
    unsigned char buf_header[1] = {0}, buf[27] = {0};
    size_t ret;
    bool header_found = false;
    while (!header_found) {
        ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (ret != 1) continue;
        if (buf_header[0] != header[0]) continue;
        bool header_2_found = false;
        while (!header_2_found) {
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            if (ret != 1) continue;
            if (buf_header[0] != header[0]) {
                continue;
            }
            header_2_found = true;
        }
        header_found = true;
    }
    ret = boost::asio::read(sp, boost::asio::buffer(buf), er2);
    if (ret != 27) return false;
    unsigned int sum = 0;
    for (int i=0; i<26; ++i) sum += buf[i];
    return (sum % 256) == buf[26];
}
'''


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
class ReaderPatchTest(unittest.TestCase):
    run_cpp = ChassisSafetyTest.run_cpp

    def test_frame_parser_deadline_checksum_and_header(self):
        patched = patch_serial_timeout(READER)
        self.assertEqual(patch_serial_timeout(patched), patched)
        self.run_cpp(r'''
#include <cassert>
#include <sys/socket.h>
struct Port {int fd; int native_handle(){return fd;}} sp;
const unsigned char header[2]={0xfe,0xfe};
struct MyAGV {bool readSpeed();};
''' + patched.replace('#include "myagv_odometry/myAGV.h"','') + r'''
int main(){
    int fd[2]; assert(socketpair(AF_UNIX,SOCK_STREAM,0,fd)==0); sp.fd=fd[0];
    MyAGV driver;
    unsigned char frame[29]={0xfe,0xfe};
    frame[2]=frame[3]=frame[4]=128; frame[28]=128;
    write(fd[1],frame,sizeof(frame)); assert(driver.readSpeed());
    frame[28]=0;write(fd[1],frame,sizeof(frame));assert(!driver.readSpeed());
    unsigned char badHeader[3]={0xfe,1,0xfe};
    write(fd[1],badHeader,3);write(fd[1],frame+2,27);
    auto start=std::chrono::steady_clock::now();
    assert(!driver.readSpeed());
    assert(std::chrono::steady_clock::now()-start<std::chrono::milliseconds(500));
    write(fd[1],frame,5);assert(!driver.readSpeed());
    close(fd[0]);close(fd[1]);
}
''')

    def test_backup_upgrade_preflight_and_idempotence(self):
        sub = '''double angularZ = 0.0;
linearX = msg.linear.x;
ros::Rate loop_rate(100);
ros::Subscriber sub = n.subscribe("cmd_vel", 50, cmdCallback);
myAGV.execute(linearX, linearY, angularZ);
\treturn 0;'''
        library = READER + '''void MyAGV::restoreRun()
{
    std::cin >> value;
    restore();
}
void MyAGV::execute(double linearX, double linearY, double angularZ)
{
    writeSpeed(linearX, linearY, angularZ);
    readSpeed(); // easy to report error
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'myAGVSub.cpp'
            cpp = path.with_name('myAGV.cpp')
            path.write_text(sub);cpp.write_text(library)
            previous = cpp.with_suffix('.cpp.before-web-watchdog')
            previous.write_text('original vendor backup')
            conflict = cpp.with_suffix('.cpp.before-web-motion-v2')
            conflict.write_text('different source')
            with self.assertRaisesRegex(ValueError,'backup differs'):install(path)
            self.assertEqual(path.read_text(),sub)
            self.assertEqual(cpp.read_text(),library)
            self.assertFalse(path.with_name('myagv_motion_safety.h').exists())
            conflict.unlink()
            install(path)
            self.assertIn('"cmd_vel", 1,',path.read_text())
            self.assertEqual(previous.read_text(),'original vendor backup')
            self.assertEqual(conflict.read_text(),library)
            updated = cpp.read_text()
            install(path)
            self.assertEqual(cpp.read_text(),updated)
            self.assertEqual(conflict.read_text(),library)
