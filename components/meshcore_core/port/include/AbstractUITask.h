#pragma once

#include <cstdint>

enum class UIEventType {
    none,
    contactMessage,
    channelMessage,
    roomMessage,
    newContactMessage,
    ack,
};

class AbstractUITask {
public:
    virtual ~AbstractUITask() = default;
    virtual void msgRead(int) {}
    virtual void newMsg(uint8_t, const char *, const char *, int) {}
    virtual void notify(UIEventType = UIEventType::none) {}
    virtual void loop() {}
};
