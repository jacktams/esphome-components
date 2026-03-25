#include "hid_descriptor_parser.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <set>

namespace esphome {
namespace ups_hid {

static const char *const PARSER_TAG = "ups_hid.hid_parser";

// HID item type codes
enum HidItemType {
    ITEM_TYPE_MAIN = 0,
    ITEM_TYPE_GLOBAL = 1,
    ITEM_TYPE_LOCAL = 2,
};

// HID main item tags
enum HidMainTag {
    MAIN_INPUT = 0x08,
    MAIN_OUTPUT = 0x09,
    MAIN_FEATURE = 0x0B,
    MAIN_COLLECTION = 0x0A,
    MAIN_END_COLLECTION = 0x0C,
};

// HID global item tags
enum HidGlobalTag {
    GLOBAL_USAGE_PAGE = 0x00,
    GLOBAL_LOGICAL_MIN = 0x01,
    GLOBAL_LOGICAL_MAX = 0x02,
    GLOBAL_PHYSICAL_MIN = 0x03,
    GLOBAL_PHYSICAL_MAX = 0x04,
    GLOBAL_UNIT_EXPONENT = 0x05,
    GLOBAL_UNIT = 0x06,
    GLOBAL_REPORT_SIZE = 0x07,
    GLOBAL_REPORT_ID = 0x08,
    GLOBAL_REPORT_COUNT = 0x09,
    GLOBAL_PUSH = 0x0A,
    GLOBAL_POP = 0x0B,
};

// Stackable global state for Push/Pop
struct GlobalState {
    uint16_t usage_page{0};
    int32_t logical_min{0};
    int32_t logical_max{0};
    uint32_t report_size{0};
    uint32_t report_count{0};
    uint8_t report_id{0};
};

// HID local item tags
enum HidLocalTag {
    LOCAL_USAGE = 0x00,
    LOCAL_USAGE_MIN = 0x01,
    LOCAL_USAGE_MAX = 0x02,
};

static int32_t parse_signed(const uint8_t* data, uint8_t size) {
    switch (size) {
        case 1: return static_cast<int8_t>(data[0]);
        case 2: return static_cast<int16_t>(data[0] | (data[1] << 8));
        case 4: return static_cast<int32_t>(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        default: return 0;
    }
}

static uint32_t parse_unsigned(const uint8_t* data, uint8_t size) {
    switch (size) {
        case 1: return data[0];
        case 2: return data[0] | (data[1] << 8);
        case 4: return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
        default: return 0;
    }
}

bool HidDescriptorParser::parse(const uint8_t* data, size_t len) {
    fields_.clear();
    report_max_bits_.clear();

    // Global state (stackable via Push/Pop)
    GlobalState gs;
    std::vector<GlobalState> global_stack;

    // Local state (reset after each Main item)
    std::vector<uint16_t> usages;
    std::vector<uint16_t> prev_usages;  // saved from previous Main item
    uint16_t usage_min = 0;
    uint16_t usage_max = 0;

    // Collection stack
    std::vector<uint16_t> collection_stack;

    // Per-report bit offset tracking: key = (report_type << 8) | report_id
    std::map<uint16_t, uint16_t> bit_offsets;

    // Debug: dump entire raw descriptor in 64-byte chunks
    ESP_LOGI(PARSER_TAG, "Raw HID descriptor: %zu bytes", len);
    for (size_t offset = 0; offset < len; offset += 64) {
        std::string hex;
        for (size_t i = offset; i < std::min(len, offset + 64); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", data[i]);
            hex += buf;
        }
        ESP_LOGI(PARSER_TAG, "  @%04zu: %s", offset, hex.c_str());
    }

    // Save raw descriptor for later dumping via API logs
    raw_descriptor.assign(data, data + len);

    // Reset parse counters
    total_items = main_items = input_items = feature_items = 0;
    constant_skipped = zero_usage_skipped = 0;
    uint32_t fields_added = 0;

    size_t pos = 0;
    while (pos < len) {
        uint8_t prefix = data[pos];

        // Long item (0xFE prefix) - skip
        if (prefix == 0xFE) {
            if (pos + 2 >= len) break;
            uint8_t long_size = data[pos + 1];
            pos += 3 + long_size;
            continue;
        }

        total_items++;
        // Short item
        uint8_t item_size = prefix & 0x03;
        if (item_size == 3) item_size = 4; // size code 3 means 4 bytes
        uint8_t item_type = (prefix >> 2) & 0x03;
        uint8_t item_tag = (prefix >> 4) & 0x0F;

        pos++; // skip prefix
        if (pos + item_size > len) break;

        const uint8_t* item_data = data + pos;
        uint32_t value = (item_size > 0) ? parse_unsigned(item_data, item_size) : 0;

        switch (item_type) {
        case ITEM_TYPE_GLOBAL:
            switch (item_tag) {
            case GLOBAL_USAGE_PAGE:
                gs.usage_page = value & 0xFFFF;
                break;
            case GLOBAL_LOGICAL_MIN:
                gs.logical_min = parse_signed(item_data, item_size);
                break;
            case GLOBAL_LOGICAL_MAX:
                gs.logical_max = parse_signed(item_data, item_size);
                break;
            case GLOBAL_REPORT_SIZE:
                gs.report_size = value;
                break;
            case GLOBAL_REPORT_COUNT:
                gs.report_count = value;
                break;
            case GLOBAL_REPORT_ID:
                gs.report_id = value & 0xFF;
                break;
            case GLOBAL_PUSH:
                global_stack.push_back(gs);
                break;
            case GLOBAL_POP:
                if (!global_stack.empty()) {
                    gs = global_stack.back();
                    global_stack.pop_back();
                }
                break;
            default:
                break;
            }
            break;

        case ITEM_TYPE_LOCAL:
            switch (item_tag) {
            case LOCAL_USAGE:
                if (item_size <= 2) {
                    usages.push_back(value & 0xFFFF);
                } else {
                    // 4-byte usage: high word is usage page override
                    gs.usage_page = (value >> 16) & 0xFFFF;
                    usages.push_back(value & 0xFFFF);
                }
                break;
            case LOCAL_USAGE_MIN:
                usage_min = value & 0xFFFF;
                break;
            case LOCAL_USAGE_MAX:
                usage_max = value & 0xFFFF;
                break;
            default:
                break;
            }
            break;

        case ITEM_TYPE_MAIN:
            switch (item_tag) {
            case MAIN_COLLECTION:
                // Push collection usage (the last local usage before collection)
                if (!usages.empty()) {
                    collection_stack.push_back(usages.back());
                } else {
                    collection_stack.push_back(0);
                }
                // Clear local state (including prev_usages — new collection scope)
                usages.clear();
                prev_usages.clear();
                usage_min = usage_max = 0;
                break;

            case MAIN_END_COLLECTION:
                if (!collection_stack.empty()) {
                    collection_stack.pop_back();
                }
                break;

            case MAIN_INPUT:
            case MAIN_OUTPUT:
            case MAIN_FEATURE: {
                main_items++;
                uint8_t rtype = (item_tag == MAIN_INPUT) ? 1 :
                                (item_tag == MAIN_OUTPUT) ? 2 : 3;
                if (rtype == 1) input_items++;
                if (rtype == 3) feature_items++;
                bool is_constant = (value & 0x01) != 0; // bit 0 = Constant

                uint16_t offset_key = (rtype << 8) | gs.report_id;
                if (bit_offsets.find(offset_key) == bit_offsets.end()) {
                    bit_offsets[offset_key] = 0;
                }

                // Fill usages from usage range if individual usages not provided
                // Only expand if range was explicitly set (usage_max > 0)
                if (usages.empty() && usage_max > 0 && usage_min <= usage_max) {
                    for (uint16_t u = usage_min; u <= usage_max && usages.size() < gs.report_count; u++) {
                        usages.push_back(u);
                    }
                }

                // Eaton/MGE pattern: Feature + Input items back-to-back for
                // the same field. The first Main item consumes the usages, the
                // second has none. Reuse the previous usages when current has none.
                if (usages.empty() && !prev_usages.empty() && !is_constant) {
                    usages = prev_usages;
                }

                // Emit one HidField per usage (or per report_count if no usages)
                uint16_t parent = collection_stack.empty() ? 0 : collection_stack.back();
                for (uint32_t i = 0; i < gs.report_count; i++) {
                    HidField field{};
                    field.report_type = rtype;
                    field.report_id = gs.report_id;
                    field.usage_page = gs.usage_page;
                    field.usage_id = (i < usages.size()) ? usages[i] : 0;
                    field.bit_offset = bit_offsets[offset_key];
                    field.bit_size = gs.report_size;
                    field.logical_min = gs.logical_min;
                    field.logical_max = gs.logical_max;
                    field.parent_collection = parent;
                    field.is_constant = is_constant;

                    if (is_constant) {
                        constant_skipped++;
                    } else if (field.usage_id == 0) {
                        zero_usage_skipped++;
                    } else {
                        fields_.push_back(field);
                        fields_added++;
                    }

                    bit_offsets[offset_key] += gs.report_size;
                }

                // Track max bits for report size calculation
                uint16_t max_key = offset_key;
                size_t total = bit_offsets[offset_key];
                if (report_max_bits_.find(max_key) == report_max_bits_.end() || total > report_max_bits_[max_key]) {
                    report_max_bits_[max_key] = total;
                }

                // Save usages for potential reuse by next Main item
                // (Eaton Feature+Input pairs share the same usage),
                // then clear local state per HID spec
                prev_usages = usages;
                usages.clear();
                usage_min = usage_max = 0;
                break;
            }
            default:
                break;
            }
            break;
        }

        pos += item_size;
    }

    ESP_LOGI(PARSER_TAG, "Parsed HID descriptor: %zu fields from %zu bytes "
             "(items=%u, main=%u, input=%u, feature=%u, const_skip=%u, zero_skip=%u)",
             fields_.size(), len, total_items, main_items, input_items, feature_items,
             constant_skipped, zero_usage_skipped);
    return !fields_.empty();
}

const HidField* HidDescriptorParser::find_field(uint16_t usage_page, uint16_t usage_id,
                                                 uint8_t report_type,
                                                 uint16_t parent_collection) const {
    for (const auto& f : fields_) {
        if (f.usage_page == usage_page && f.usage_id == usage_id && f.report_type == report_type) {
            if (parent_collection == 0 || f.parent_collection == parent_collection) {
                return &f;
            }
        }
    }
    return nullptr;
}

std::vector<const HidField*> HidDescriptorParser::find_fields(uint16_t usage_page, uint16_t usage_id,
                                                               uint8_t report_type) const {
    std::vector<const HidField*> result;
    for (const auto& f : fields_) {
        if (f.usage_page == usage_page && f.usage_id == usage_id && f.report_type == report_type) {
            result.push_back(&f);
        }
    }
    return result;
}

std::vector<uint8_t> HidDescriptorParser::get_report_ids(uint8_t report_type) const {
    std::set<uint8_t> ids;
    for (const auto& f : fields_) {
        if (f.report_type == report_type) {
            ids.insert(f.report_id);
        }
    }
    return std::vector<uint8_t>(ids.begin(), ids.end());
}

size_t HidDescriptorParser::get_report_byte_size(uint8_t report_type, uint8_t report_id) const {
    uint16_t key = (report_type << 8) | report_id;
    auto it = report_max_bits_.find(key);
    if (it != report_max_bits_.end()) {
        return (it->second + 7) / 8;
    }
    return 0;
}

} // namespace ups_hid
} // namespace esphome
