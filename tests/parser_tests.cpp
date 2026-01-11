/*
 * OpenCNC - G-code Parser Unit Tests
 */

#include <gtest/gtest.h>
#include "ngc_parser.h"

using namespace cnc::gcode;

class ParserTest : public ::testing::Test {
protected:
    Parser parser;
};

TEST_F(ParserTest, ParseEmptyLine) {
    auto result = parser.parseLine("");
    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.commands.empty());
}

TEST_F(ParserTest, ParseComment) {
    auto result = parser.parseLine("; This is a comment");
    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.commands.empty());
}

TEST_F(ParserTest, ParseRapidMove) {
    auto result = parser.parseLine("G0 X10 Y20 Z5");
    EXPECT_TRUE(result.ok);
    EXPECT_FALSE(result.commands.empty());
    
    // Check that motion mode changed to rapid
    EXPECT_EQ(parser.getModalState().motion, MotionMode::Rapid);
}

TEST_F(ParserTest, ParseLinearMove) {
    auto result = parser.parseLine("G1 X100 Y50 F1000");
    EXPECT_TRUE(result.ok);
    
    EXPECT_EQ(parser.getModalState().motion, MotionMode::Linear);
    EXPECT_DOUBLE_EQ(parser.getModalState().feedrate, 1000.0);
}

TEST_F(ParserTest, ParseArcCW) {
    // First set up a starting position
    parser.parseLine("G0 X0 Y0");
    
    auto result = parser.parseLine("G2 X10 Y0 I5 J0 F500");
    EXPECT_TRUE(result.ok);
    
    EXPECT_EQ(parser.getModalState().motion, MotionMode::ArcCW);
}

TEST_F(ParserTest, ParseArcCCW) {
    parser.parseLine("G0 X0 Y0");
    
    auto result = parser.parseLine("G3 X10 Y0 I5 J0 F500");
    EXPECT_TRUE(result.ok);
    
    EXPECT_EQ(parser.getModalState().motion, MotionMode::ArcCCW);
}

TEST_F(ParserTest, ParseSpindleOn) {
    auto result = parser.parseLine("M3 S12000");
    EXPECT_TRUE(result.ok);
    
    auto state = parser.getModalState();
    EXPECT_TRUE(state.spindleOn);
    EXPECT_TRUE(state.spindleCW);
    EXPECT_EQ(state.spindleSpeed, 12000);
}

TEST_F(ParserTest, ParseSpindleOff) {
    parser.parseLine("M3 S10000");
    auto result = parser.parseLine("M5");
    EXPECT_TRUE(result.ok);
    
    EXPECT_FALSE(parser.getModalState().spindleOn);
}

TEST_F(ParserTest, ParseCoolant) {
    auto result1 = parser.parseLine("M7");  // Mist
    EXPECT_TRUE(result1.ok);
    EXPECT_TRUE(parser.getModalState().coolantMist);
    
    auto result2 = parser.parseLine("M8");  // Flood
    EXPECT_TRUE(result2.ok);
    EXPECT_TRUE(parser.getModalState().coolantFlood);
    
    auto result3 = parser.parseLine("M9");  // Off
    EXPECT_TRUE(result3.ok);
    EXPECT_FALSE(parser.getModalState().coolantMist);
    EXPECT_FALSE(parser.getModalState().coolantFlood);
}

TEST_F(ParserTest, ParseAbsoluteMode) {
    auto result = parser.parseLine("G90");
    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(parser.getModalState().absolute);
}

TEST_F(ParserTest, ParseRelativeMode) {
    auto result = parser.parseLine("G91");
    EXPECT_TRUE(result.ok);
    EXPECT_FALSE(parser.getModalState().absolute);
}

TEST_F(ParserTest, ParseUnits) {
    parser.parseLine("G21");  // Metric
    EXPECT_TRUE(parser.getModalState().metric);
    
    parser.parseLine("G20");  // Imperial
    EXPECT_FALSE(parser.getModalState().metric);
}

TEST_F(ParserTest, ParseWorkOffset) {
    auto result = parser.parseLine("G54");
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(parser.getModalState().workOffset, 1);
    
    parser.parseLine("G55");
    EXPECT_EQ(parser.getModalState().workOffset, 2);
}

TEST_F(ParserTest, ParseDwell) {
    auto result = parser.parseLine("G4 P0.5");
    EXPECT_TRUE(result.ok);
}

TEST_F(ParserTest, ParseToolChange) {
    auto result = parser.parseLine("T1 M6");
    EXPECT_TRUE(result.ok);
}

TEST_F(ParserTest, ParseMultipleWordsOnLine) {
    auto result = parser.parseLine("G0 G90 G54 X0 Y0 Z10 M3 S10000");
    EXPECT_TRUE(result.ok);
    
    auto state = parser.getModalState();
    EXPECT_EQ(state.motion, MotionMode::Rapid);
    EXPECT_TRUE(state.absolute);
    EXPECT_EQ(state.workOffset, 1);
    EXPECT_TRUE(state.spindleOn);
}

TEST_F(ParserTest, ParseInvalidGCode) {
    auto result = parser.parseLine("G999 X10");
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.error.empty());
}

TEST_F(ParserTest, ParseLineNumbers) {
    auto result = parser.parseLine("N100 G0 X10 Y20");
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.lineNumber, 100);
}

TEST_F(ParserTest, Reset) {
    parser.parseLine("G1 X100 Y50 F1000");
    parser.parseLine("M3 S12000");
    
    parser.reset();
    
    auto state = parser.getModalState();
    EXPECT_EQ(state.motion, MotionMode::Rapid);  // Default
    EXPECT_FALSE(state.spindleOn);
    EXPECT_DOUBLE_EQ(state.feedrate, 0.0);
}
