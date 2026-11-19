#pragma once

#include <esp_http_server.h>

class AudioCodec;
class GestureSensor;
class ServoController;

class ControlPanel {
public:
    ControlPanel(ServoController& servos, AudioCodec& audio, GestureSensor* gesture);
    ~ControlPanel();
    esp_err_t Start();

private:
    static esp_err_t ServePage(httpd_req_t* req);
    static esp_err_t ServeState(httpd_req_t* req);
    static esp_err_t HandleCommand(httpd_req_t* req);
    static esp_err_t HandleFollow(httpd_req_t* req);

    ServoController& servos_;
    AudioCodec& audio_;
    GestureSensor* gesture_;
    httpd_handle_t server_ = nullptr;
};
