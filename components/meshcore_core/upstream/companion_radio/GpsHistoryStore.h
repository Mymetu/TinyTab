#pragma once

#ifndef GPS_HISTORY_MAX_RECORDS
#define GPS_HISTORY_MAX_RECORDS 0
#endif

#ifndef GPS_HISTORY_SEGMENT_COUNT
#define GPS_HISTORY_SEGMENT_COUNT 13
#endif

#ifndef GPS_HISTORY_RECORDS_PER_SEGMENT
#define GPS_HISTORY_RECORDS_PER_SEGMENT 240
#endif

#if GPS_HISTORY_MAX_RECORDS > 0 && ENV_INCLUDE_GPS == 1 && defined(ESP32)
#include <Arduino.h>
#include <FS.h>

#if GPS_HISTORY_SEGMENT_COUNT * GPS_HISTORY_RECORDS_PER_SEGMENT <= GPS_HISTORY_MAX_RECORDS
#error "GPS history needs at least one spare segment for smooth rotation"
#endif

struct GpsHistoryRecord {
  uint32_t timestamp;
  int32_t latitude_e6;
  int32_t longitude_e6;
  int32_t altitude_cm;
};

static_assert(sizeof(GpsHistoryRecord) == 16, "GPS history BLE record must remain 16 bytes");

class GpsHistoryStore {
public:
  bool begin(fs::FS* filesystem, uint32_t new_session_id);
  bool append(const GpsHistoryRecord& record);
  bool read(uint32_t sequence, GpsHistoryRecord& record);

  uint32_t getSessionId() const { return _session_id; }
  uint32_t getOldestSequence() const { return _oldest_sequence; }
  uint32_t getNextSequence() const { return _next_sequence; }
  uint16_t getCount() const { return _record_count; }
  bool isReady() const { return _ready; }

private:
  static const uint16_t RECORDS_PER_SEGMENT = GPS_HISTORY_RECORDS_PER_SEGMENT;

  struct SegmentHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t session_id;
    uint32_t generation;
    uint32_t first_sequence;
    uint32_t crc32;
  };

  struct DiskRecord {
    uint32_t sequence;
    GpsHistoryRecord record;
    uint32_t crc32;
  };

  struct SegmentInfo {
    bool valid;
    bool appendable;
    uint8_t slot;
    uint16_t count;
    uint32_t session_id;
    uint32_t generation;
    uint32_t first_sequence;
  };

  static_assert(sizeof(SegmentHeader) == 24, "GPS history segment header layout changed");
  static_assert(sizeof(DiskRecord) == 24, "GPS history disk record layout changed");

  fs::FS* _fs = nullptr;
  SegmentInfo _segments[GPS_HISTORY_SEGMENT_COUNT];
  bool _ready = false;
  int8_t _current_slot = -1;
  uint32_t _session_id = 0;
  uint32_t _oldest_sequence = 0;
  uint32_t _next_sequence = 0;
  uint16_t _record_count = 0;

  static uint32_t calculateCrc32(const uint8_t* data, size_t length);
  static void makeSegmentPath(uint8_t slot, char* path, size_t path_size);
  bool scanSegment(uint8_t slot, SegmentInfo& info);
  bool createNextSegment();
  void updateBounds();
};

#endif
