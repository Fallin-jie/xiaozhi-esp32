#include "control_panel.h"

#include "audio_codec.h"
#include "gesture_sensor.h"
#include "servo_controller.h"
#include "settings.h"

#include <charconv>
#include <memory>
#include <cmath>
#include "cJSON.h"
#include <esp_mac.h>
#include <cstdio>
#include <cstring>
#include <string_view>

extern "C" const char _binary_control_panel_html_start[];

ControlPanel::ControlPanel(ServoController& servos, AudioCodec& audio, GestureSensor* gesture)
    : servos_(servos), audio_(audio), gesture_(gesture) {}

ControlPanel::~ControlPanel() {
    if (server_) httpd_stop(server_);
}

esp_err_t ControlPanel::Start() {
    if (server_) return ESP_OK;
    Settings settings("head_motion");
    servos_.SetManualSpeed(settings.GetInt("manual_speed", ServoController::kDefaultSpeed));
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 2;
    config.lru_purge_enable = true;
    esp_err_t err = httpd_start(&server_, &config);
    if (err != ESP_OK) return err;

    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = ServePage, .user_ctx = this},
        {.uri = "/api/state", .method = HTTP_GET, .handler = ServeState, .user_ctx = this},
        {.uri = "/api/control", .method = HTTP_POST, .handler = HandleCommand, .user_ctx = this},
        {.uri = "/api/follow", .method = HTTP_POST, .handler = HandleFollow, .user_ctx = this},
    };
    for (const auto& route : routes) {
        err = httpd_register_uri_handler(server_, &route);
        if (err != ESP_OK) {
            httpd_stop(server_);
            server_ = nullptr;
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t ControlPanel::ServePage(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
    return httpd_resp_sendstr(req, _binary_control_panel_html_start);
}

esp_err_t ControlPanel::ServeState(httpd_req_t* req) {
    auto& panel = *static_cast<ControlPanel*>(req->user_ctx);
    auto position = panel.servos_.GetPosition();
    const char* gesture = "null";  // Sensor or polling task unavailable.
    if (panel.gesture_) gesture = panel.gesture_->ResponseEnabled() ? "true" : "false";
    uint8_t mac[6]{};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char state[384];
    std::snprintf(state, sizeof(state),
        R"({"x":%d,"y":%d,"minX":%d,"maxX":%d,"minY":%d,"maxY":%d,)"
        R"("robot":"%02x%02x%02x%02x%02x%02x","mode":"%s","boot":%lu,"generation":%lu,"targetX":%.1f,"targetY":%.1f,"moving":%s,)"
        R"("step":%d,"manualSpeed":%d,"manual":%s,"volume":%d,"gesture":%s})",
        position.x, position.y, SERVO_MIN_X, SERVO_MAX_X, SERVO_MIN_Y, SERVO_MAX_Y,
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], position.mode, (unsigned long)position.boot,
        (unsigned long)position.generation, position.target_x, position.target_y, position.moving ? "true" : "false",
        ServoController::kManualStep, position.manual_speed, position.manual ? "true" : "false",
        panel.audio_.output_volume(), gesture);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, state);
}

esp_err_t ControlPanel::HandleCommand(httpd_req_t* req) {
    // A browser must use a same-origin fetch to send this custom header.
    char header[4];
    char body[24];
    if (httpd_req_get_hdr_value_str(req, "X-Desk-Control", header, sizeof(header)) != ESP_OK ||
        std::strcmp(header, "1") != 0 || req->content_len == 0 || req->content_len > sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid control request");
        return ESP_FAIL;
    }
    size_t received = 0;
    while (received < req->content_len) {
        int n = httpd_req_recv(req, body + received, req->content_len - received);
        if (n <= 0) return ESP_FAIL;
        received += n;
    }

    std::string_view command(body, received);
    auto& panel = *static_cast<ControlPanel*>(req->user_ctx);
    using Move = ServoController::ManualMove;
    if (command == "left")        panel.servos_.MoveManually(Move::Left);
    else if (command == "right")  panel.servos_.MoveManually(Move::Right);
    else if (command == "up")     panel.servos_.MoveManually(Move::Up);
    else if (command == "down")   panel.servos_.MoveManually(Move::Down);
    else if (command == "center") panel.servos_.MoveManually(Move::Center);
    else if (command == "auto")   panel.servos_.ResumeAutomatic();
    else if (command == "gesture=on" || command == "gesture=off") {
        if (!panel.gesture_) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Gesture sensor unavailable");
        panel.gesture_->SetResponseEnabled(command == "gesture=on");
    } else if (command.starts_with("volume=")) {
        command.remove_prefix(7);
        int volume = 0;
        const char* end = command.data() + command.size();
        auto result = std::from_chars(command.data(), end, volume);
        if (result.ec != std::errc{} || result.ptr != end || volume < 0 || volume > 100) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid volume");
        }
        if (volume != panel.audio_.output_volume()) panel.audio_.SetOutputVolume(volume);
    } else {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown command");
    }
    return ServeState(req);
}

esp_err_t ControlPanel::HandleFollow(httpd_req_t* req) {
    char header[4], body[257];
    if (httpd_req_get_hdr_value_str(req, "X-Desk-Control", header, sizeof(header)) != ESP_OK ||
        std::strcmp(header, "1") || !req->content_len || req->content_len >= sizeof(body))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
    size_t received = 0;
    while (received < req->content_len) {
        int n = httpd_req_recv(req, body + received, req->content_len - received);
        if (n <= 0) return ESP_FAIL;
        received += n;
    }
    body[received] = 0;
    std::unique_ptr<cJSON, decltype(&cJSON_Delete)> json(cJSON_ParseWithOpts(body, nullptr, true), cJSON_Delete);
    auto field = [&](const char* name) { return cJSON_GetObjectItemCaseSensitive(json.get(), name); };
    auto number = [&](const char* name, uint32_t& out) {
        auto item = field(name);
        if (!cJSON_IsNumber(item)) return false;
        double v = item->valuedouble;
        if (v < 0 || v > UINT32_MAX || v != std::floor(v)) return false;
        out = static_cast<uint32_t>(v);
        return true;
    };
    const char* raw = cJSON_GetStringValue(field("op"));
    std::string_view op(raw ? raw : "");
    uint32_t session, seq, boot, generation, request, x = 0, y = 0;
    auto& servo = static_cast<ControlPanel*>(req->user_ctx)->servos_;
    bool accepted = false;
    bool angles = field("x") || field("y");
    if (angles && (!number("x", x) || !number("y", y) ||
        x < SERVO_MIN_X || x > SERVO_MAX_X || y < SERVO_MIN_Y || y > SERVO_MAX_Y))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Angles outside limits");
    if (op == "speed") {
        uint32_t value;
        if (angles || !number("value", value) || value > 20)
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid manual speed");
        auto previous = servo.GetPosition().manual_speed;
        if (!servo.SetManualSpeed(value))
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid manual speed");
        if (previous != static_cast<int>(value)) Settings("head_motion", true).SetInt("manual_speed", value);
        return ServeState(req);
    }
    if (op == "manual" && angles) accepted = servo.ManualAngles(x, y);
    else if (op == "start" && number("boot", boot) && number("generation", generation) && number("request", request)) {
        session = servo.StartFollow(boot, generation, request);
        if (session) {
            char result[40];
            std::snprintf(result, sizeof(result), R"({"session":%lu})", (unsigned long)session);
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, result);
        }
    } else if (op == "update" && number("session", session) && number("seq", seq) &&
               (!field("hold") || cJSON_IsBool(field("hold"))) && !(angles && cJSON_IsTrue(field("hold"))))
        accepted = servo.UpdateFollow(session, seq, cJSON_IsTrue(field("hold")), angles ? int(x) : -1, y);
    else if (op == "stop" && number("session", session)) accepted = servo.StopFollow(session);
    else return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid operation");
    if (accepted) return ServeState(req);
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "Control session changed");
}
