#pragma once

#include <cstdint>
#include <vector>
#include <map>

namespace esphome {
namespace ups_hid {

// Describes a single data field within an HID report
struct HidField {
    uint8_t report_type;         // 1=Input, 2=Output, 3=Feature
    uint8_t report_id;
    uint16_t usage_page;
    uint16_t usage_id;
    uint16_t bit_offset;         // bit position within report data (after report ID byte)
    uint16_t bit_size;           // number of bits for this field
    int32_t logical_min;
    int32_t logical_max;
    uint16_t parent_collection;  // usage ID of parent collection (e.g. Input=0x1A, Output=0x1C)
    bool is_constant;            // true if field is padding/constant (no data)
};

// Parses a raw HID report descriptor into a list of fields
class HidDescriptorParser {
public:
    bool parse(const uint8_t* data, size_t len);

    const std::vector<HidField>& get_fields() const { return fields_; }

    // Find a field by usage page + usage ID, optionally within a specific parent collection
    const HidField* find_field(uint16_t usage_page, uint16_t usage_id,
                               uint8_t report_type = 3,
                               uint16_t parent_collection = 0) const;

    // Find all fields matching usage page + usage ID
    std::vector<const HidField*> find_fields(uint16_t usage_page, uint16_t usage_id,
                                              uint8_t report_type = 3) const;

    // Get all unique report IDs for a given report type
    std::vector<uint8_t> get_report_ids(uint8_t report_type) const;

    // Get total byte size for a report (excluding report ID byte)
    size_t get_report_byte_size(uint8_t report_type, uint8_t report_id) const;

    // Parse diagnostics
    uint32_t total_items{0}, main_items{0}, input_items{0}, feature_items{0};
    uint32_t constant_skipped{0}, zero_usage_skipped{0};

    // Store raw descriptor for later dumping
    std::vector<uint8_t> raw_descriptor;

private:
    std::vector<HidField> fields_;

    // Track max bit offset per (report_type, report_id) for size calculation
    std::map<uint16_t, size_t> report_max_bits_; // key = (type<<8)|id
};

} // namespace ups_hid
} // namespace esphome
