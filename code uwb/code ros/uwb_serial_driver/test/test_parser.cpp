#include <string>

#include <gtest/gtest.h>

#include "uwb_serial_driver/parser.hpp"

using uwb_serial_driver::ParseError;
using uwb_serial_driver::parse_line;

TEST(Parser, ParsesCurrentMakerfabsTagOutput)
{
  const auto frame = parse_line(
    "T0,mask:5,seq:7,beacon:12,range:(125,0,234),ancid:(0,-1,2)");
  ASSERT_EQ(frame.tag_id, 0);
  ASSERT_EQ(frame.sequence, 7);
  ASSERT_EQ(frame.beacon_sequence, 12);
  ASSERT_EQ(frame.measurements.size(), 2U);
  EXPECT_EQ(frame.measurements[0].anchor_id, 0);
  EXPECT_DOUBLE_EQ(frame.measurements[0].range_m, 1.25);
  EXPECT_EQ(frame.measurements[1].anchor_id, 2);
  EXPECT_DOUBLE_EQ(frame.measurements[1].range_m, 2.34);
}

TEST(Parser, ParsesMeterValuesSeparatedBySemicolons)
{
  const auto frame = parse_line(
    "T0,mask:F,seq:7,beacon:12,range_m:(1.25;0.34;2.17;1.22),ancid:(0,1,2,3)");
  ASSERT_EQ(frame.measurements.size(), 4U);
  EXPECT_EQ(frame.measurements[0].anchor_id, 0);
  EXPECT_DOUBLE_EQ(frame.measurements[0].range_m, 1.25);
  EXPECT_EQ(frame.measurements[1].anchor_id, 1);
  EXPECT_DOUBLE_EQ(frame.measurements[1].range_m, 0.34);
  EXPECT_EQ(frame.measurements[2].anchor_id, 2);
  EXPECT_DOUBLE_EQ(frame.measurements[2].range_m, 2.17);
  EXPECT_EQ(frame.measurements[3].anchor_id, 3);
  EXPECT_DOUBLE_EQ(frame.measurements[3].range_m, 1.22);
}

TEST(Parser, ParsesAnchorMeterOutputWithFailCount)
{
  const auto frame = parse_line(
    "T0,mask:F,seq:7,fail:2,range_m:(1.25;0.34;2.17;1.22),ancid:(0,1,2,3)");
  ASSERT_EQ(frame.fail_count, 2);
  ASSERT_EQ(frame.measurements.size(), 4U);
  EXPECT_DOUBLE_EQ(frame.measurements[0].range_m, 1.25);
  EXPECT_DOUBLE_EQ(frame.measurements[3].range_m, 1.22);
}

TEST(Parser, ParsesAnchorIdentityInSerialRecord)
{
  const auto frame = parse_line(
    "T0,mask:F,seq:7,fail:2,range_m:(1.25;0.34;2.17;1.22),ancid:(0,1,2,3),"
    "device_role:anchor,device_index:3,device_radio_id:163");
  EXPECT_EQ(frame.device_role, "anchor");
  ASSERT_EQ(frame.device_index, 3);
  ASSERT_EQ(frame.device_radio_id, 163);
  ASSERT_EQ(frame.tag_id, 0);
  ASSERT_EQ(frame.measurements.size(), 4U);
}

TEST(Parser, ParsesTagIdentityInSerialRecord)
{
  const auto frame = parse_line(
    "T1,mask:1,seq:7,beacon:12,range_m:(1.25;0;0;0),ancid:(0,-1,-1,-1),"
    "device_role:tag,device_index:1,device_radio_id:177");
  EXPECT_EQ(frame.device_role, "tag");
  ASSERT_EQ(frame.device_index, 1);
  ASSERT_EQ(frame.device_radio_id, 177);
}

TEST(Parser, ParsesAnchorIdentityInJsonRecord)
{
  const auto frame = parse_line(
    R"({"device_role":"anchor","device_index":3,"device_radio_id":163,)"
    R"("tag_id":1,"measurements":[{"anchor_id":0,"range_m":1.25}]})");
  EXPECT_EQ(frame.device_role, "anchor");
  ASSERT_EQ(frame.device_index, 3);
  ASSERT_EQ(frame.device_radio_id, 163);
}

TEST(Parser, RejectsInconsistentAnchorRadioIdentity)
{
  EXPECT_THROW(
    parse_line(
      "T0,mask:1,seq:7,range_m:(1.25;0;0;0),ancid:(0,-1,-1,-1),"
      "device_role:anchor,device_index:3,device_radio_id:160"),
    ParseError);
}

TEST(Parser, MaskStillExcludesMissingAnchorsInMeterFormat)
{
  const auto frame = parse_line(
    "T0,mask:5,seq:7,beacon:12,range_m:(1.25;0;2.17;0),ancid:(0,-1,2,-1)");
  ASSERT_EQ(frame.measurements.size(), 2U);
  EXPECT_EQ(frame.measurements[0].anchor_id, 0);
  EXPECT_DOUBLE_EQ(frame.measurements[0].range_m, 1.25);
  EXPECT_EQ(frame.measurements[1].anchor_id, 2);
  EXPECT_DOUBLE_EQ(frame.measurements[1].range_m, 2.17);
}

TEST(Parser, ParsesExtendedJsonQualityTimestampsAndPayload)
{
  const auto frame = parse_line(
    R"({"protocol":"uwb_json_v1","tag_id":3,"sequence":8,"timestamp_ms":1234,)"
    R"("measurements":[{"anchor_id":2,"range_m":1.5,)"
    R"("quality":{"fp_power_dbm":-75.2},"timestamps":{"poll_tx":12},)"
    R"("clock_offset":0.001}],"payload":{"temperature_c":24.5}})");
  ASSERT_EQ(frame.tag_id, 3);
  ASSERT_EQ(frame.device_timestamp_ms, 1234U);
  ASSERT_EQ(frame.measurements.size(), 1U);
  EXPECT_DOUBLE_EQ(frame.measurements[0].quality["fp_power_dbm"].get<double>(), -75.2);
  EXPECT_EQ(frame.measurements[0].timestamps["poll_tx"].get<int>(), 12);
  EXPECT_DOUBLE_EQ(*frame.measurements[0].clock_offset, 0.001);
  EXPECT_DOUBLE_EQ(frame.payload["temperature_c"].get<double>(), 24.5);
}

TEST(Parser, ParsesExistingSingleRangeJson)
{
  const auto frame = parse_line(
    R"({"protocol":"uwb_serial_v1","sequence":4,"source_id":1,)"
    R"("range_m":2.25,"payload_hex":"54455354"})");
  ASSERT_EQ(frame.measurements.size(), 1U);
  EXPECT_EQ(frame.measurements[0].anchor_id, 1);
  EXPECT_DOUBLE_EQ(frame.measurements[0].range_m, 2.25);
  EXPECT_EQ(frame.payload["encoding"], "hex");
  EXPECT_EQ(frame.payload["data"], "54455354");
}

TEST(Parser, RejectsInvalidRange)
{
  EXPECT_THROW(parse_line(R"({"anchor_id":1,"range_m":-1})"), ParseError);
}

TEST(Parser, RejectsMismatchedLegacyArrays)
{
  EXPECT_THROW(parse_line("T0,mask:1,seq:1,range:(100,200),ancid:(0)"), ParseError);
}
