// kinematics.cpp —— 构建自检: 验证轮速换算/帧字节与真实 Python 复算一致
#include "kinematics.h"
#include "protocol.h"
#include "pid.h"
#include <cstdio>
#include <cassert>

void kinematics_selftest() {
    // 例: vx=0.1 -> wheel_linear=[0.1,-0.1,-0.1,0.1]
    double v[3] = {0.1, 0.0, 0.0};
    double wl[4];
    kin::inverse_kinematics(v, wl);
    int8_t wire[4];
    kin::wheel_linear_to_wire(wl, wire);
    // 取负后: [-10,10,10,-10] = f6 0a 0a f6
    assert(wire[0] == -10 && wire[1] == 10 && wire[2] == 10 && wire[3] == -10);
    auto p = proto::mc602::motor4_speed_payload(wire);
    assert(p.size() == 6);
    assert(p[0] == 0x01 && p[1] == 0x02);
    assert(p[2] == static_cast<uint8_t>(0xf6) && p[3] == 0x0a &&
           p[4] == 0x0a && p[5] == static_cast<uint8_t>(0xf6));
    auto frame = proto::pack_mc_frame(p);
    assert(frame.size() == 10 && frame[2] == 0x0a);
    // 线路帧应完全等于 77 68 0a 01 02 f6 0a 0a f6 0a
    const uint8_t expect[] = {0x77, 0x68, 0x0a, 0x01, 0x02, 0xf6, 0x0a, 0x0a, 0xf6, 0x0a};
    for (size_t i = 0; i < frame.size(); ++i) assert(frame[i] == expect[i]);

    // get_bytes 层示例(输入为已取负的虚拟值): [31,-32,-33,34] -> 01 02 1f e0 df 22
    int8_t neg[4] = {31, -32, -33, 34};
    auto p2 = proto::mc602::motor4_speed_payload(neg);
    const uint8_t expect2[] = {0x01, 0x02, 0x1f, 0xe0, 0xdf, 0x22};
    for (size_t i = 0; i < p2.size(); ++i) assert(p2[i] == expect2[i]);

    // 编码器读帧: 4 帧, 每帧 7 字节, 首帧 04 01 01 00 00 00 00
    auto enc = proto::mc602::encoder_read_all();
    assert(enc.size() == 28);
    assert(enc[0] == 0x04 && enc[1] == 0x01 && enc[2] == 0x01);
    for (size_t i = 3; i < 7; ++i) assert(enc[i] == 0x00);
    assert(enc[7 + 2] == 0x02);

    // PID 首次调用 d=0 (differential_on_measurement 语义), P 项 = Kp*error
    // setpoint=0, input=0.05 -> error=-0.05 -> P=-0.3; dt≈0 -> I≈0; D=0
    PID pid(6, 0.3, 0.1, 0.0, -0.6, 0.6);
    double o1 = pid.call(0.05);
    assert(o1 < 0.0 && o1 > -0.31);  // 约 -0.3

    printf("[rtd] kinematics selftest OK\n");
}
