/*
 * OpenCNC - Trajectory Planner Unit Tests
 */

#include <gtest/gtest.h>
#include "traj_planner.h"
#include "traj_segment.h"
#include "traj_queue.h"

using namespace cnc::trajectory;

class PlannerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Configure planner
        PlannerConfig config;
        config.maxVelocity = 5000.0 / 60.0;  // mm/s
        config.maxAcceleration = 500.0;       // mm/s²
        config.junctionDeviation = 0.02;
        config.arcTolerance = 0.002;
        
        planner = std::make_unique<TrajectoryPlanner>(config);
    }
    
    std::unique_ptr<TrajectoryPlanner> planner;
};

TEST_F(PlannerTest, PlanLinearMove) {
    Position start{0, 0, 0};
    Position end{100, 0, 0};
    double feedrate = 1000.0 / 60.0;  // mm/s
    
    auto segments = planner->planLinear(start, end, feedrate);
    
    EXPECT_FALSE(segments.empty());
    
    // Check segment connects start to end
    EXPECT_NEAR(segments.front().start.x, start.x, 0.001);
    EXPECT_NEAR(segments.back().end.x, end.x, 0.001);
}

TEST_F(PlannerTest, PlanDiagonalMove) {
    Position start{0, 0, 0};
    Position end{100, 100, 50};
    double feedrate = 1000.0 / 60.0;
    
    auto segments = planner->planLinear(start, end, feedrate);
    
    EXPECT_FALSE(segments.empty());
}

TEST_F(PlannerTest, PlanArcCW) {
    Position start{0, 10, 0};
    Position end{10, 0, 0};
    Position center{0, 0, 0};
    double feedrate = 500.0 / 60.0;
    
    auto segments = planner->planArc(start, end, center, true, feedrate);
    
    EXPECT_FALSE(segments.empty());
}

TEST_F(PlannerTest, PlanArcCCW) {
    Position start{10, 0, 0};
    Position end{0, 10, 0};
    Position center{0, 0, 0};
    double feedrate = 500.0 / 60.0;
    
    auto segments = planner->planArc(start, end, center, false, feedrate);
    
    EXPECT_FALSE(segments.empty());
}

TEST_F(PlannerTest, VelocityLimiting) {
    Position start{0, 0, 0};
    Position end{100, 0, 0};
    double requestedFeed = 10000.0 / 60.0;  // Higher than max
    
    auto segments = planner->planLinear(start, end, requestedFeed);
    
    EXPECT_FALSE(segments.empty());
    
    // Cruise velocity should be limited to max
    for (const auto& seg : segments) {
        EXPECT_LE(seg.cruiseVelocity, planner->getConfig().maxVelocity + 0.001);
    }
}

TEST_F(PlannerTest, AccelerationProfile) {
    Position start{0, 0, 0};
    Position end{50, 0, 0};  // Short move
    double feedrate = 1000.0 / 60.0;
    
    auto segments = planner->planLinear(start, end, feedrate);
    
    EXPECT_FALSE(segments.empty());
    
    // First segment should accelerate
    EXPECT_GE(segments.front().cruiseVelocity, segments.front().entryVelocity);
}

TEST_F(PlannerTest, JunctionVelocity) {
    // Plan two connected moves with a corner
    Position p1{0, 0, 0};
    Position p2{100, 0, 0};
    Position p3{100, 100, 0};
    double feedrate = 1000.0 / 60.0;
    
    auto seg1 = planner->planLinear(p1, p2, feedrate);
    auto seg2 = planner->planLinear(p2, p3, feedrate);
    
    // Junction velocity at 90° corner should be reduced
    // The exit velocity of seg1 should match entry velocity of seg2
    // (This requires forward/backward planning pass)
}

class RingBufferTest : public ::testing::Test {
protected:
    RingBuffer<int, 8> buffer;
};

TEST_F(RingBufferTest, InitiallyEmpty) {
    EXPECT_TRUE(buffer.empty());
    EXPECT_FALSE(buffer.full());
    EXPECT_EQ(buffer.size(), 0);
}

TEST_F(RingBufferTest, PushAndPop) {
    EXPECT_TRUE(buffer.push(42));
    EXPECT_FALSE(buffer.empty());
    EXPECT_EQ(buffer.size(), 1);
    
    int value;
    EXPECT_TRUE(buffer.pop(value));
    EXPECT_EQ(value, 42);
    EXPECT_TRUE(buffer.empty());
}

TEST_F(RingBufferTest, FillBuffer) {
    for (int i = 0; i < 7; i++) {  // Capacity - 1
        EXPECT_TRUE(buffer.push(i));
    }
    
    EXPECT_TRUE(buffer.full());
    EXPECT_FALSE(buffer.push(99));  // Should fail when full
}

TEST_F(RingBufferTest, FIFO_Order) {
    for (int i = 0; i < 5; i++) {
        buffer.push(i);
    }
    
    for (int i = 0; i < 5; i++) {
        int value;
        EXPECT_TRUE(buffer.pop(value));
        EXPECT_EQ(value, i);
    }
}

TEST_F(RingBufferTest, Wraparound) {
    // Fill and drain multiple times to test wraparound
    for (int round = 0; round < 3; round++) {
        for (int i = 0; i < 5; i++) {
            EXPECT_TRUE(buffer.push(i + round * 10));
        }
        
        for (int i = 0; i < 5; i++) {
            int value;
            EXPECT_TRUE(buffer.pop(value));
            EXPECT_EQ(value, i + round * 10);
        }
    }
}

TEST_F(RingBufferTest, Peek) {
    buffer.push(1);
    buffer.push(2);
    buffer.push(3);
    
    EXPECT_EQ(*buffer.peek(), 1);
    
    int value;
    buffer.pop(value);
    EXPECT_EQ(*buffer.peek(), 2);
}

TEST_F(RingBufferTest, Clear) {
    buffer.push(1);
    buffer.push(2);
    buffer.push(3);
    
    buffer.clear();
    
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.size(), 0);
}
