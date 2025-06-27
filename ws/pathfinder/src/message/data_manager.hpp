#include "message_handler.hpp"
#include <memory>
namespace engineering_match::data {
class DataManager {
public:
    DataManager(std::shared_ptr<message::MessageHandler> message_handler)
        : message_handler_(message_handler) {};

    std::shared_ptr<message::MessageHandler> message_handler_;
};
}