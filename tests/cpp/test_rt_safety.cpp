#include "realtime_control/rt_common.hpp"
#include "realtime_control/shared_memory.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <thread>
#include <vector>

using namespace flexiv_rt;

// ============================================================================
// QuatAngularDist
// ============================================================================

TEST(QuatAngularDist, IdentityToIdentity) {
    // pose = [x, y, z, qw, qx, qy, qz]
    std::array<double, 7> p1 = {0, 0, 0, 1, 0, 0, 0};
    std::array<double, 7> p2 = {0, 0, 0, 1, 0, 0, 0};
    EXPECT_NEAR(QuatAngularDist(p1, p2), 0.0, 1e-12);
}

TEST(QuatAngularDist, SmallRotationZ10deg) {
    // 10 degree rotation around Z
    double half = 10.0 * M_PI / 180.0 / 2.0;
    std::array<double, 7> p1 = {0, 0, 0, 1, 0, 0, 0};
    std::array<double, 7> p2 = {0, 0, 0, std::cos(half), 0, 0, std::sin(half)};
    EXPECT_NEAR(QuatAngularDist(p1, p2), 10.0 * M_PI / 180.0, 1e-10);
}

TEST(QuatAngularDist, Rotation90deg) {
    // 90 degree rotation around X
    double half = M_PI / 4.0;
    std::array<double, 7> p1 = {0, 0, 0, 1, 0, 0, 0};
    std::array<double, 7> p2 = {0, 0, 0, std::cos(half), std::sin(half), 0, 0};
    EXPECT_NEAR(QuatAngularDist(p1, p2), M_PI / 2.0, 1e-10);
}

TEST(QuatAngularDist, Rotation180deg) {
    // 180 degree rotation around Y
    std::array<double, 7> p1 = {0, 0, 0, 1, 0, 0, 0};
    std::array<double, 7> p2 = {0, 0, 0, 0, 0, 1, 0};
    EXPECT_NEAR(QuatAngularDist(p1, p2), M_PI, 1e-10);
}

TEST(QuatAngularDist, AntipodalQuaternions) {
    // q and -q represent the same rotation -> distance should be 0
    double half = 30.0 * M_PI / 180.0 / 2.0;
    std::array<double, 7> p1 = {0, 0, 0,  std::cos(half), std::sin(half), 0, 0};
    std::array<double, 7> p2 = {0, 0, 0, -std::cos(half), -std::sin(half), 0, 0};
    EXPECT_NEAR(QuatAngularDist(p1, p2), 0.0, 1e-10);
}

// ============================================================================
// CheckFinite
// ============================================================================

TEST(CheckFinite, AllValidVector) {
    std::vector<double> v = {1.0, -2.5, 0.0, 3.14};
    EXPECT_TRUE(CheckFinite(v));
}

TEST(CheckFinite, VectorWithNaN) {
    std::vector<double> v = {1.0, std::numeric_limits<double>::quiet_NaN(), 3.0};
    EXPECT_FALSE(CheckFinite(v));
}

TEST(CheckFinite, VectorWithInf) {
    std::vector<double> v = {1.0, std::numeric_limits<double>::infinity(), 3.0};
    EXPECT_FALSE(CheckFinite(v));
}

TEST(CheckFinite, EmptyVector) {
    std::vector<double> v;
    EXPECT_TRUE(CheckFinite(v));
}

TEST(CheckFinite, ValidArray7) {
    std::array<double, 7> a = {0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(CheckFinite(a));
}

TEST(CheckFinite, ArrayWithNaN) {
    std::array<double, 7> a = {0.1, 0.2, 0.3, 1.0, 0.0,
                               std::numeric_limits<double>::quiet_NaN(), 0.0};
    EXPECT_FALSE(CheckFinite(a));
}

// ============================================================================
// CheckJointJump
// ============================================================================

TEST(CheckJointJump, WithinThreshold) {
    std::vector<double> cmd  = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    std::vector<double> last = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    EXPECT_FALSE(CheckJointJump(cmd, last, 0.01));
}

TEST(CheckJointJump, OneJointExceeds) {
    std::vector<double> cmd  = {0.0, 0.1, 0.2, 0.35, 0.4, 0.5, 0.6};
    std::vector<double> last = {0.0, 0.1, 0.2, 0.30, 0.4, 0.5, 0.6};
    // delta on joint 3 = 0.05 > 0.01
    EXPECT_TRUE(CheckJointJump(cmd, last, 0.01));
}

TEST(CheckJointJump, SizeMismatch) {
    std::vector<double> cmd  = {0.0, 0.1, 0.2};
    std::vector<double> last = {0.0, 0.1};
    EXPECT_TRUE(CheckJointJump(cmd, last, 0.01));
}

TEST(CheckJointJump, Empty) {
    std::vector<double> cmd;
    std::vector<double> last;
    EXPECT_FALSE(CheckJointJump(cmd, last, 0.01));
}

// ============================================================================
// CheckCartesianJump
// ============================================================================

TEST(CheckCartesianJump, BothWithin) {
    std::array<double, 7> cmd  = {0.5, 0.0, 0.3, 1, 0, 0, 0};
    std::array<double, 7> last = {0.5, 0.0, 0.3, 1, 0, 0, 0};
    EXPECT_FALSE(CheckCartesianJump(cmd, last, 0.01, 0.05));
}

TEST(CheckCartesianJump, PositionExceeds) {
    std::array<double, 7> cmd  = {0.5, 0.1, 0.3, 1, 0, 0, 0};
    std::array<double, 7> last = {0.5, 0.0, 0.3, 1, 0, 0, 0};
    // pos delta = 0.1 > 0.01
    EXPECT_TRUE(CheckCartesianJump(cmd, last, 0.01, 0.05));
}

TEST(CheckCartesianJump, RotationExceeds) {
    // 45 degree rotation around Z
    double half = 45.0 * M_PI / 180.0 / 2.0;
    std::array<double, 7> cmd  = {0.5, 0.0, 0.3, std::cos(half), 0, 0, std::sin(half)};
    std::array<double, 7> last = {0.5, 0.0, 0.3, 1, 0, 0, 0};
    // rot delta ≈ π/4 ≈ 0.785 > 0.05
    EXPECT_TRUE(CheckCartesianJump(cmd, last, 0.01, 0.05));
}

TEST(CheckCartesianJump, Identical) {
    std::array<double, 7> pose = {0.1, 0.2, 0.3, 1, 0, 0, 0};
    EXPECT_FALSE(CheckCartesianJump(pose, pose, 0.01, 0.05));
}

// ============================================================================
// Cache-line alignment constants
// ============================================================================

TEST(CacheLineAlign, ConstantIs64) {
    EXPECT_EQ(kCacheLineSize, 64u);
}

// ============================================================================
// RealTimeBuffer — basic SPSC semantics
// ============================================================================

using IntBuf8 = RealTimeBuffer<int, 8>;
using DoubleBuf4 = RealTimeBuffer<double, 4>;
using Pose7Buf8 = RealTimeBuffer<std::array<double, 7>, 8>;

TEST(RealTimeBuffer, EmptyOnInit) {
    IntBuf8 buf;
    EXPECT_TRUE(buf.empty());
}

TEST(RealTimeBuffer, WriteAndRead) {
    DoubleBuf4 buf;
    EXPECT_TRUE(buf.try_write(3.14));
    EXPECT_FALSE(buf.empty());
    double out = 0;
    EXPECT_TRUE(buf.try_read(out));
    EXPECT_DOUBLE_EQ(out, 3.14);
    EXPECT_TRUE(buf.empty());
}

TEST(RealTimeBuffer, ReadFromEmptyFails) {
    IntBuf8 buf;
    int out = -1;
    EXPECT_FALSE(buf.try_read(out));
    EXPECT_EQ(out, -1);  // unchanged
}

TEST(RealTimeBuffer, FIFOOrder) {
    IntBuf8 buf;
    for (int i = 0; i < 8; ++i)
        EXPECT_TRUE(buf.try_write(i * 10));
    for (int i = 0; i < 8; ++i) {
        int out = -1;
        EXPECT_TRUE(buf.try_read(out));
        EXPECT_EQ(out, i * 10);
    }
}

TEST(RealTimeBuffer, FullBufferRejectsWrite) {
    DoubleBuf4 buf;
    for (int i = 0; i < 4; ++i)
        EXPECT_TRUE(buf.try_write(static_cast<double>(i)));
    EXPECT_FALSE(buf.try_write(99.0));  // capacity = 4, should fail
}

TEST(RealTimeBuffer, WrapAround) {
    // Write and read more items than capacity to test index wrap-around
    DoubleBuf4 buf;
    for (int round = 0; round < 10; ++round) {
        double val = static_cast<double>(round);
        EXPECT_TRUE(buf.try_write(val));
        double out = -1;
        EXPECT_TRUE(buf.try_read(out));
        EXPECT_DOUBLE_EQ(out, val);
    }
}

TEST(RealTimeBuffer, Clear) {
    IntBuf8 buf;
    buf.try_write(1);
    buf.try_write(2);
    EXPECT_FALSE(buf.empty());
    buf.clear();
    EXPECT_TRUE(buf.empty());
    int out = -1;
    EXPECT_FALSE(buf.try_read(out));
}

TEST(RealTimeBuffer, ArrayElement) {
    Pose7Buf8 buf;
    std::array<double, 7> pose = {0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(buf.try_write(pose));
    std::array<double, 7> out = {};
    EXPECT_TRUE(buf.try_read(out));
    EXPECT_EQ(out, pose);
}

// ============================================================================
// RealTimeBuffer — cache-line alignment of head / tail
// ============================================================================

TEST(RealTimeBuffer, HeadTailAlignment) {
    EXPECT_EQ(offsetof(IntBuf8, head) % kCacheLineSize, 0u);
    EXPECT_EQ(offsetof(IntBuf8, tail) % kCacheLineSize, 0u);
    EXPECT_NE(offsetof(IntBuf8, head), offsetof(IntBuf8, tail));
}

TEST(RealTimeBuffer, DataAlignment) {
    EXPECT_EQ(offsetof(IntBuf8, data) % kCacheLineSize, 0u);
}

// ============================================================================
// RealTimeBuffer — concurrent SPSC (1 producer thread, 1 consumer thread)
// ============================================================================

TEST(RealTimeBuffer, ConcurrentSPSC) {
    using Buf = RealTimeBuffer<uint64_t, 256>;
    Buf buf;
    constexpr uint64_t kCount = 100000;

    std::thread producer([&] {
        for (uint64_t i = 0; i < kCount; ++i) {
            while (!buf.try_write(i)) {
                // spin — buffer full
            }
        }
    });

    std::vector<uint64_t> received;
    received.reserve(kCount);
    std::thread consumer([&] {
        uint64_t val;
        while (received.size() < kCount) {
            if (buf.try_read(val)) {
                received.push_back(val);
            }
        }
    });

    producer.join();
    consumer.join();

    // Verify all items arrived in order
    ASSERT_EQ(received.size(), kCount);
    for (uint64_t i = 0; i < kCount; ++i) {
        EXPECT_EQ(received[i], i) << "Mismatch at index " << i;
    }
}

// ============================================================================
// ClampCartesianPose
// ============================================================================

TEST(ClampCartesianPose, PositionWithin) {
    // Small move within limit: 0.5 m/s * 0.001s = 0.5mm per cycle
    std::array<double, 7> pose = {0.0001, 0.0, 0.0, 1, 0, 0, 0};  // 0.1mm
    std::array<double, 7> last = {0.0, 0.0, 0.0, 1, 0, 0, 0};
    EXPECT_FALSE(ClampCartesianPose(pose, last));
    EXPECT_NEAR(pose[0], 0.0001, 1e-12);  // unchanged
}

TEST(ClampCartesianPose, PositionExceeds) {
    // Large move: 10mm in one cycle, limit = 0.5mm
    std::array<double, 7> pose = {0.01, 0.0, 0.0, 1, 0, 0, 0};
    std::array<double, 7> last = {0.0, 0.0, 0.0, 1, 0, 0, 0};
    EXPECT_TRUE(ClampCartesianPose(pose, last));
    double dist = std::sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
    double max_d = kCartMaxLinearVel * kRtDt;
    EXPECT_NEAR(dist, max_d, 1e-10);
}

TEST(ClampCartesianPose, PositionExceedsDiagonal) {
    // Diagonal move: 10mm in X and Y
    std::array<double, 7> pose = {0.01, 0.01, 0.0, 1, 0, 0, 0};
    std::array<double, 7> last = {0.0, 0.0, 0.0, 1, 0, 0, 0};
    EXPECT_TRUE(ClampCartesianPose(pose, last));
    double dist = std::sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
    EXPECT_NEAR(dist, kCartMaxLinearVel * kRtDt, 1e-10);
    // Direction preserved: X and Y should be equal
    EXPECT_NEAR(pose[0], pose[1], 1e-12);
}

TEST(ClampCartesianPose, RotationWithin) {
    // Small rotation within limit: 1.0 rad/s * 0.001s = 0.001 rad per cycle
    double half = 0.0005 / 2.0;  // 0.0005 rad rotation
    std::array<double, 7> pose = {0, 0, 0, std::cos(half), 0, 0, std::sin(half)};
    std::array<double, 7> last = {0, 0, 0, 1, 0, 0, 0};
    EXPECT_FALSE(ClampCartesianPose(pose, last));
}

TEST(ClampCartesianPose, RotationExceeds) {
    // Large rotation: 45 degrees in one cycle, limit = 0.001 rad
    double half = 45.0 * M_PI / 180.0 / 2.0;
    std::array<double, 7> pose = {0, 0, 0, std::cos(half), 0, 0, std::sin(half)};
    std::array<double, 7> last = {0, 0, 0, 1, 0, 0, 0};
    EXPECT_TRUE(ClampCartesianPose(pose, last));
    double angle = QuatAngularDist(pose, last);
    double max_r = kCartMaxAngularVel * kRtDt;
    EXPECT_NEAR(angle, max_r, 1e-6);
}

// ============================================================================
// ClampCartesianVelocity
// ============================================================================

TEST(ClampCartesianVelocity, AllWithin) {
    std::array<double, 6> vel      = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3};
    std::array<double, 6> last_vel = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3};
    EXPECT_FALSE(ClampCartesianVelocity(vel, last_vel));
}

TEST(ClampCartesianVelocity, LinearVelocityExceeds) {
    std::array<double, 6> vel      = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 1 m/s > 0.5
    std::array<double, 6> last_vel = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0};  // at limit, no acc trigger
    EXPECT_TRUE(ClampCartesianVelocity(vel, last_vel));
    double v = std::sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
    EXPECT_NEAR(v, kCartMaxLinearVel, 1e-10);
}

TEST(ClampCartesianVelocity, AngularVelocityExceeds) {
    std::array<double, 6> vel      = {0.0, 0.0, 0.0, 2.0, 0.0, 0.0};  // 2 rad/s > 1.0
    std::array<double, 6> last_vel = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};  // at limit, no acc trigger
    EXPECT_TRUE(ClampCartesianVelocity(vel, last_vel));
    double w = std::sqrt(vel[3]*vel[3] + vel[4]*vel[4] + vel[5]*vel[5]);
    EXPECT_NEAR(w, kCartMaxAngularVel, 1e-10);
}

TEST(ClampCartesianVelocity, LinearAccelerationExceeds) {
    // Velocity jump from 0 to 0.4: dv = 0.4, max_dv = 2.0 * 0.001 = 0.002
    std::array<double, 6> vel      = {0.4, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> last_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(ClampCartesianVelocity(vel, last_vel));
    double max_dv = kCartMaxLinearAcc * kRtDt;
    EXPECT_NEAR(vel[0], max_dv, 1e-10);
}

// ============================================================================
// ClampJointPosition
// ============================================================================

TEST(ClampJointPosition, AllWithin) {
    std::vector<double> pos  = {0.0, 0.001, 0.002, 0.001, 0.0, 0.001, 0.002};
    std::vector<double> last = {0.0, 0.0,   0.0,   0.0,   0.0, 0.0,   0.0};
    // max delta = 2.0 * 0.001 = 0.002 rad
    EXPECT_FALSE(ClampJointPosition(pos, last));
}

TEST(ClampJointPosition, OneJointExceeds) {
    std::vector<double> pos  = {0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0};  // 0.1 rad jump
    std::vector<double> last = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(ClampJointPosition(pos, last));
    double max_delta = kJointMaxVel * kRtDt;
    EXPECT_NEAR(pos[3], max_delta, 1e-10);
    EXPECT_NEAR(pos[0], 0.0, 1e-12);  // other joints unchanged
}

TEST(ClampJointPosition, NegativeDelta) {
    std::vector<double> pos  = {0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0};
    std::vector<double> last = {0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(ClampJointPosition(pos, last));
    double max_delta = kJointMaxVel * kRtDt;
    EXPECT_NEAR(pos[3], -max_delta, 1e-10);
}

// ============================================================================
// ClampJointVelocity
// ============================================================================

TEST(ClampJointVelocity, AllWithin) {
    std::vector<double> vel      = {0.001, 0.002, 0.001, 0.001, 0.002, 0.001, 0.001};
    std::vector<double> last_vel = {0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0};
    // max dv = 3.0 * 0.001 = 0.003 rad/s
    EXPECT_FALSE(ClampJointVelocity(vel, last_vel));
}

TEST(ClampJointVelocity, AccelerationExceeds) {
    // Velocity jump: 0 to 1.0 rad/s, max_dv = 0.003
    std::vector<double> vel      = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> last_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(ClampJointVelocity(vel, last_vel));
    double max_dv = kJointMaxAcc * kRtDt;
    EXPECT_NEAR(vel[1], max_dv, 1e-10);
    EXPECT_NEAR(vel[0], 0.0, 1e-12);  // other joints unchanged
}
