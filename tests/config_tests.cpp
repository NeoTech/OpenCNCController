/*
 * OpenCNC - Configuration Parser Unit Tests
 */

#include <gtest/gtest.h>
#include "cnc_config.h"

using namespace cnc::config;

class ConfigValueTest : public ::testing::Test {};

TEST_F(ConfigValueTest, DefaultIsNull) {
    ConfigValue v;
    EXPECT_TRUE(v.isNull());
}

TEST_F(ConfigValueTest, BoolValue) {
    ConfigValue v(true);
    EXPECT_TRUE(v.isBool());
    EXPECT_TRUE(v.asBool());
    
    ConfigValue v2(false);
    EXPECT_FALSE(v2.asBool());
}

TEST_F(ConfigValueTest, IntValue) {
    ConfigValue v(42);
    EXPECT_TRUE(v.isInt());
    EXPECT_EQ(v.asInt(), 42);
}

TEST_F(ConfigValueTest, DoubleValue) {
    ConfigValue v(3.14159);
    EXPECT_TRUE(v.isDouble());
    EXPECT_DOUBLE_EQ(v.asDouble(), 3.14159);
}

TEST_F(ConfigValueTest, StringValue) {
    ConfigValue v("hello");
    EXPECT_TRUE(v.isString());
    EXPECT_EQ(v.asString(), "hello");
}

TEST_F(ConfigValueTest, IntAsDouble) {
    ConfigValue v(100);
    EXPECT_TRUE(v.isInt());
    EXPECT_DOUBLE_EQ(v.asDouble(), 100.0);
}

TEST_F(ConfigValueTest, ArrayValue) {
    ConfigArray arr = {ConfigValue(1), ConfigValue(2), ConfigValue(3)};
    ConfigValue v(arr);
    
    EXPECT_TRUE(v.isArray());
    EXPECT_EQ(v.asArray().size(), 3);
    EXPECT_EQ(v[0].asInt(), 1);
    EXPECT_EQ(v[2].asInt(), 3);
}

TEST_F(ConfigValueTest, TableValue) {
    ConfigTable table;
    table["name"] = ConfigValue("test");
    table["value"] = ConfigValue(123);
    
    ConfigValue v(table);
    
    EXPECT_TRUE(v.isTable());
    EXPECT_TRUE(v.contains("name"));
    EXPECT_TRUE(v.contains("value"));
    EXPECT_EQ(v["name"].asString(), "test");
    EXPECT_EQ(v["value"].asInt(), 123);
}

TEST_F(ConfigValueTest, GetWithDefault) {
    ConfigValue v(42);
    EXPECT_EQ(v.get<int>(0), 42);
    
    ConfigValue empty;
    EXPECT_EQ(empty.get<int>(99), 99);
}

class TomlParserTest : public ::testing::Test {
protected:
    TomlParser parser;
};

TEST_F(TomlParserTest, ParseEmptyString) {
    auto result = parser.parse("");
    EXPECT_TRUE(result.empty());
}

TEST_F(TomlParserTest, ParseComments) {
    auto result = parser.parse("# This is a comment\n# Another comment");
    EXPECT_TRUE(result.empty());
}

TEST_F(TomlParserTest, ParseKeyValue) {
    auto result = parser.parse("key = \"value\"");
    EXPECT_TRUE(result.count("key"));
    EXPECT_EQ(result.at("key").asString(), "value");
}

TEST_F(TomlParserTest, ParseInteger) {
    auto result = parser.parse("count = 42");
    EXPECT_EQ(result.at("count").asInt(), 42);
}

TEST_F(TomlParserTest, ParseFloat) {
    auto result = parser.parse("pi = 3.14159");
    EXPECT_DOUBLE_EQ(result.at("pi").asDouble(), 3.14159);
}

TEST_F(TomlParserTest, ParseBoolean) {
    auto result = parser.parse("enabled = true\ndisabled = false");
    EXPECT_TRUE(result.at("enabled").asBool());
    EXPECT_FALSE(result.at("disabled").asBool());
}

TEST_F(TomlParserTest, ParseTable) {
    auto result = parser.parse("[section]\nname = \"test\"");
    
    EXPECT_TRUE(result.count("section"));
    EXPECT_TRUE(result.at("section").isTable());
    EXPECT_EQ(result.at("section").asTable().at("name").asString(), "test");
}

TEST_F(TomlParserTest, ParseNestedTable) {
    auto result = parser.parse("[a.b.c]\nvalue = 123");
    
    EXPECT_TRUE(result.count("a"));
    EXPECT_TRUE(result.at("a").asTable().count("b"));
    EXPECT_TRUE(result.at("a").asTable().at("b").asTable().count("c"));
    EXPECT_EQ(result.at("a").asTable().at("b").asTable().at("c").asTable().at("value").asInt(), 123);
}

TEST_F(TomlParserTest, ParseArrayOfTables) {
    auto result = parser.parse(
        "[[items]]\n"
        "name = \"first\"\n"
        "[[items]]\n"
        "name = \"second\"\n"
    );
    
    EXPECT_TRUE(result.count("items"));
    EXPECT_TRUE(result.at("items").isArray());
    EXPECT_EQ(result.at("items").asArray().size(), 2);
    EXPECT_EQ(result.at("items").asArray()[0].asTable().at("name").asString(), "first");
    EXPECT_EQ(result.at("items").asArray()[1].asTable().at("name").asString(), "second");
}

TEST_F(TomlParserTest, ParseSimpleArray) {
    auto result = parser.parse("values = [1, 2, 3, 4, 5]");
    
    EXPECT_TRUE(result.at("values").isArray());
    EXPECT_EQ(result.at("values").asArray().size(), 5);
    EXPECT_EQ(result.at("values").asArray()[0].asInt(), 1);
}

class MachineConfigTest : public ::testing::Test {};

TEST_F(MachineConfigTest, LoadFromString) {
    std::string toml = R"(
[machine]
name = "Test CNC"
version = "1.0"

[[axis]]
name = "X"
steps_per_mm = 800.0
max_velocity = 5000.0
max_acceleration = 500.0
max_travel = 300.0

[[axis]]
name = "Y"
steps_per_mm = 800.0
max_velocity = 5000.0
max_acceleration = 500.0
max_travel = 200.0

[spindle]
max_rpm = 24000
enabled = true

[communication]
port = "COM3"
baud_rate = 115200
)";
    
    MachineConfig config = MachineConfig::loadFromString(toml);
    
    EXPECT_EQ(config.name, "Test CNC");
    EXPECT_EQ(config.version, "1.0");
    
    EXPECT_EQ(config.axes.size(), 2);
    EXPECT_EQ(config.axes[0].name, "X");
    EXPECT_DOUBLE_EQ(config.axes[0].stepsPerMm, 800.0);
    EXPECT_DOUBLE_EQ(config.axes[0].maxVelocity, 5000.0);
    EXPECT_EQ(config.axes[1].name, "Y");
    
    EXPECT_TRUE(config.spindle.enabled);
    EXPECT_EQ(config.spindle.maxRpm, 24000);
    
    EXPECT_EQ(config.communication.port, "COM3");
    EXPECT_EQ(config.communication.baudRate, 115200);
}

TEST_F(MachineConfigTest, DefaultValues) {
    std::string toml = R"(
[[axis]]
name = "X"
)";
    
    MachineConfig config = MachineConfig::loadFromString(toml);
    
    // Check defaults are applied
    EXPECT_DOUBLE_EQ(config.axes[0].stepsPerMm, 800.0);
    EXPECT_DOUBLE_EQ(config.axes[0].maxVelocity, 5000.0);
    EXPECT_DOUBLE_EQ(config.axes[0].maxAcceleration, 500.0);
    EXPECT_EQ(config.axes[0].homeDirection, -1);
}
