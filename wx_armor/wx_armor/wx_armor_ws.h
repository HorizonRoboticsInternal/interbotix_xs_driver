#pragma once

#include "drogon/HttpAppFramework.h"
#include "drogon/PubSubService.h"
#include "drogon/WebSocketController.h"

namespace horizon::wx_armor {

class WxArmorWebController
    : public drogon::WebSocketController<WxArmorWebController> {
 public:
  virtual void handleNewMessage(const drogon::WebSocketConnectionPtr &,
                                std::string &&,
                                const drogon::WebSocketMessageType &) override;

  virtual void handleConnectionClosed(
      const drogon::WebSocketConnectionPtr &) override;

  virtual void handleNewConnection(
      const drogon::HttpRequestPtr &,
      const drogon::WebSocketConnectionPtr &) override;

  WS_PATH_LIST_BEGIN
  WS_PATH_ADD("/api/engage", drogon::Get);
  WS_PATH_LIST_END
};

}  // namespace horizon::wx_armor
