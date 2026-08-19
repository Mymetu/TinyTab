#pragma once

#include <cstdint>

#include "Dispatcher.h"
#include "MeshCore.h"
#include "helpers/SensorManager.h"
#include "meshcore_radio.hpp"

class IdfRadioAdapter : public mesh::Radio {
public:
    IdfRadioAdapter();

    void bind(SX1262 *radio);
    void begin() override;
    int recvRaw(uint8_t *bytes, int size) override;
    uint32_t getEstAirtimeFor(int length) override;
    float packetScore(float snr, int packet_length) override;
    bool startSendRaw(const uint8_t *bytes, int length) override;
    bool isSendComplete() override;
    void onSendFinished() override;
    bool isInRecvMode() const override;
    bool isReceiving() override;
    float getLastRSSI() const override;
    float getLastSNR() const override;
    int getNoiseFloor() const override;

    void setParams(float frequency, float bandwidth, uint8_t spreading_factor,
                   uint8_t coding_rate);
    void setTxPower(int8_t dbm);
    void setRxBoostedGainMode(bool enabled);
    bool getRxBoostedGainMode() const;
    uint32_t getRngSeed();
    uint32_t getPacketsRecv() const { return packets_received_; }
    uint32_t getPacketsSent() const { return packets_sent_; }
    uint32_t getPacketsRecvErrors() const { return packet_receive_errors_; }
    void resetStats() { packets_received_ = packets_sent_ = packet_receive_errors_ = 0; }

private:
    enum class State : uint8_t { idle, receive, transmit };
    static void onRadioInterrupt();
    void startReceive();

    SX1262 *radio_;
    volatile bool interrupt_ready_;
    State state_;
    bool rx_boosted_;
    uint8_t spreading_factor_;
    uint32_t packets_received_;
    uint32_t packets_sent_;
    uint32_t packet_receive_errors_;
    static IdfRadioAdapter *instance_;
};

extern IdfRadioAdapter radio_driver;
extern mesh::MainBoard &board;
extern SensorManager sensors;

bool radio_init();
uint32_t radio_get_rng_seed();
void radio_set_params(float frequency, float bandwidth, uint8_t spreading_factor,
                      uint8_t coding_rate);
void radio_set_tx_power(int8_t dbm);
mesh::LocalIdentity radio_new_identity();
