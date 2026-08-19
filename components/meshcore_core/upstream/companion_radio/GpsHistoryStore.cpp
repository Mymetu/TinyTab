#include "GpsHistoryStore.h"

#if GPS_HISTORY_MAX_RECORDS > 0 && ENV_INCLUDE_GPS == 1 && defined(ESP32)

#include <string.h>

static const uint32_t GPS_HISTORY_MAGIC = 0x31534847; // "GHS1"
static const uint16_t GPS_HISTORY_FORMAT_VERSION = 1;

uint32_t GpsHistoryStore::calculateCrc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ (0xEDB88320 & (0U - (crc & 1U)));
    }
  }
  return ~crc;
}

void GpsHistoryStore::makeSegmentPath(uint8_t slot, char* path, size_t path_size) {
  snprintf(path, path_size, "/gps%02u.bin", slot);
}

bool GpsHistoryStore::scanSegment(uint8_t slot, SegmentInfo& info) {
  memset(&info, 0, sizeof(info));
  info.slot = slot;

  char path[16];
  makeSegmentPath(slot, path, sizeof(path));
  if (!_fs->exists(path)) return false;

  File file = _fs->open(path, FILE_READ);
  if (!file || file.size() < sizeof(SegmentHeader)) {
    if (file) file.close();
    return false;
  }

  SegmentHeader header;
  if (file.read((uint8_t*)&header, sizeof(header)) != sizeof(header)) {
    file.close();
    return false;
  }

  uint32_t header_crc = calculateCrc32((const uint8_t*)&header, sizeof(header) - sizeof(header.crc32));
  if (header.magic != GPS_HISTORY_MAGIC ||
      header.version != GPS_HISTORY_FORMAT_VERSION ||
      header.header_size != sizeof(SegmentHeader) ||
      header.crc32 != header_crc) {
    file.close();
    return false;
  }

  info.valid = true;
  info.session_id = header.session_id;
  info.generation = header.generation;
  info.first_sequence = header.first_sequence;

  while (info.count < RECORDS_PER_SEGMENT &&
         file.size() - file.position() >= sizeof(DiskRecord)) {
    DiskRecord disk_record;
    if (file.read((uint8_t*)&disk_record, sizeof(disk_record)) != sizeof(disk_record)) break;

    uint32_t record_crc =
        calculateCrc32((const uint8_t*)&disk_record, sizeof(disk_record) - sizeof(disk_record.crc32));
    if (disk_record.sequence != info.first_sequence + info.count ||
        disk_record.crc32 != record_crc) {
      break;
    }
    info.count++;
  }

  size_t valid_size = sizeof(SegmentHeader) + info.count * sizeof(DiskRecord);
  info.appendable = info.count < RECORDS_PER_SEGMENT && file.size() == valid_size;
  file.close();
  return true;
}

void GpsHistoryStore::updateBounds() {
  uint32_t cursor = _next_sequence;
  uint32_t count = 0;

  while (count < GPS_HISTORY_MAX_RECORDS) {
    int8_t preceding_slot = -1;
    for (uint8_t slot = 0; slot < GPS_HISTORY_SEGMENT_COUNT; slot++) {
      const SegmentInfo& info = _segments[slot];
      if (!info.valid || info.session_id != _session_id || info.count == 0 ||
          info.first_sequence + info.count != cursor) {
        continue;
      }
      if (preceding_slot < 0 ||
          info.generation > _segments[preceding_slot].generation) {
        preceding_slot = slot;
      }
    }

    if (preceding_slot < 0) break;
    const SegmentInfo& preceding = _segments[preceding_slot];
    count += preceding.count;
    cursor = preceding.first_sequence;
  }

  if (count > GPS_HISTORY_MAX_RECORDS) count = GPS_HISTORY_MAX_RECORDS;
  _record_count = count;
  _oldest_sequence = _next_sequence - count;
}

bool GpsHistoryStore::begin(fs::FS* filesystem, uint32_t new_session_id) {
  _fs = filesystem;
  _ready = false;
  _current_slot = -1;
  _session_id = 0;
  _oldest_sequence = 0;
  _next_sequence = 0;
  _record_count = 0;
  memset(_segments, 0, sizeof(_segments));

  if (!_fs) return false;

  int8_t newest_slot = -1;
  for (uint8_t slot = 0; slot < GPS_HISTORY_SEGMENT_COUNT; slot++) {
    scanSegment(slot, _segments[slot]);
    if (!_segments[slot].valid) continue;

    if (newest_slot < 0 ||
        _segments[slot].generation > _segments[newest_slot].generation ||
        (_segments[slot].generation == _segments[newest_slot].generation &&
         _segments[slot].first_sequence > _segments[newest_slot].first_sequence)) {
      newest_slot = slot;
    }
  }

  if (newest_slot >= 0) {
    _session_id = _segments[newest_slot].session_id;
    _current_slot = newest_slot;
    _next_sequence = _segments[newest_slot].first_sequence + _segments[newest_slot].count;

    for (uint8_t slot = 0; slot < GPS_HISTORY_SEGMENT_COUNT; slot++) {
      if (_segments[slot].valid && _segments[slot].session_id != _session_id) {
        _segments[slot].valid = false;
      }
    }
  } else {
    _session_id = new_session_id == 0 ? 1 : new_session_id;
  }

  updateBounds();
  _ready = true;
  return true;
}

bool GpsHistoryStore::createNextSegment() {
  uint8_t target_slot = _current_slot < 0 ? 0 : (_current_slot + 1) % GPS_HISTORY_SEGMENT_COUNT;
  uint32_t generation = _current_slot < 0 ? 0 : _segments[_current_slot].generation + 1;

  char path[16];
  makeSegmentPath(target_slot, path, sizeof(path));
  if (_fs->exists(path) && !_fs->remove(path)) return false;

  memset(&_segments[target_slot], 0, sizeof(SegmentInfo));
  _segments[target_slot].slot = target_slot;
  updateBounds();

  SegmentHeader header;
  header.magic = GPS_HISTORY_MAGIC;
  header.version = GPS_HISTORY_FORMAT_VERSION;
  header.header_size = sizeof(SegmentHeader);
  header.session_id = _session_id;
  header.generation = generation;
  header.first_sequence = _next_sequence;
  header.crc32 = calculateCrc32((const uint8_t*)&header, sizeof(header) - sizeof(header.crc32));

  File file = _fs->open(path, FILE_WRITE, true);
  if (!file) return false;
  size_t written = file.write((const uint8_t*)&header, sizeof(header));
  file.flush();
  file.close();
  if (written != sizeof(header)) {
    _fs->remove(path);
    return false;
  }

  _segments[target_slot].valid = true;
  _segments[target_slot].appendable = true;
  _segments[target_slot].slot = target_slot;
  _segments[target_slot].session_id = _session_id;
  _segments[target_slot].generation = generation;
  _segments[target_slot].first_sequence = _next_sequence;
  _current_slot = target_slot;
  updateBounds();
  return true;
}

bool GpsHistoryStore::append(const GpsHistoryRecord& record) {
  if (!_ready) return false;

  if (_current_slot < 0 ||
      !_segments[_current_slot].appendable ||
      _segments[_current_slot].count >= RECORDS_PER_SEGMENT) {
    if (!createNextSegment()) return false;
  }

  DiskRecord disk_record;
  disk_record.sequence = _next_sequence;
  disk_record.record = record;
  disk_record.crc32 =
      calculateCrc32((const uint8_t*)&disk_record, sizeof(disk_record) - sizeof(disk_record.crc32));

  char path[16];
  makeSegmentPath(_current_slot, path, sizeof(path));
  File file = _fs->open(path, FILE_APPEND, true);
  if (!file) return false;
  size_t written = file.write((const uint8_t*)&disk_record, sizeof(disk_record));
  file.flush();
  file.close();

  if (written != sizeof(disk_record)) {
    SegmentInfo recovered;
    if (scanSegment(_current_slot, recovered)) {
      _segments[_current_slot] = recovered;
    } else {
      _segments[_current_slot].appendable = false;
    }
    updateBounds();
    return false;
  }

  _segments[_current_slot].count++;
  _segments[_current_slot].appendable =
      _segments[_current_slot].count < RECORDS_PER_SEGMENT;
  _next_sequence++;
  updateBounds();
  return true;
}

bool GpsHistoryStore::read(uint32_t sequence, GpsHistoryRecord& record) {
  if (!_ready || sequence < _oldest_sequence || sequence >= _next_sequence) return false;

  for (uint8_t slot = 0; slot < GPS_HISTORY_SEGMENT_COUNT; slot++) {
    const SegmentInfo& info = _segments[slot];
    if (!info.valid || info.session_id != _session_id ||
        sequence < info.first_sequence || sequence >= info.first_sequence + info.count) {
      continue;
    }

    char path[16];
    makeSegmentPath(slot, path, sizeof(path));
    File file = _fs->open(path, FILE_READ);
    if (!file) return false;

    size_t offset = sizeof(SegmentHeader) +
                    (sequence - info.first_sequence) * sizeof(DiskRecord);
    if (!file.seek(offset)) {
      file.close();
      return false;
    }

    DiskRecord disk_record;
    bool success = file.read((uint8_t*)&disk_record, sizeof(disk_record)) == sizeof(disk_record);
    file.close();
    if (!success) return false;

    uint32_t crc =
        calculateCrc32((const uint8_t*)&disk_record, sizeof(disk_record) - sizeof(disk_record.crc32));
    if (disk_record.sequence != sequence || disk_record.crc32 != crc) return false;

    record = disk_record.record;
    return true;
  }

  return false;
}

#endif
