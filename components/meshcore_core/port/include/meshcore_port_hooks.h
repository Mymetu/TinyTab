#pragma once

#include <cstdint>

extern "C" void meshcore_port_on_channel_message(uint32_t timestamp,
                                                   const char *text,
                                                   bool is_local,
                                                   int16_t snr_quarter_db,
                                                   uint8_t router_count);
extern "C" void meshcore_port_on_contact_advert(const uint8_t *public_key,
                                                  const char *name,
                                                  uint8_t type,
                                                  bool route_known,
                                                  uint8_t hop_count,
                                                  int16_t rssi_dbm,
                                                  int16_t snr_quarter_db);
extern "C" void meshcore_port_on_contact_loaded(const uint8_t *public_key,
                                                  const char *name,
                                                  uint8_t type,
                                                  bool route_known,
                                                  uint8_t hop_count);
extern "C" void meshcore_port_on_contact_removed(const uint8_t *public_key);
extern "C" void meshcore_port_on_contact_message(const uint8_t *public_key);
