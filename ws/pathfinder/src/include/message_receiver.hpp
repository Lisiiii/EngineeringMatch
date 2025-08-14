#pragma once

namespace engineering_match::base {

class IMessageReceiver {
public:
    ~IMessageReceiver() = default;
    virtual void on_message_received() = 0;
};
}