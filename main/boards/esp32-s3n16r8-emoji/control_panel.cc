#include "control_panel.h"

#include "application.h"
#include "audio_codec.h"
#include "gesture_sensor.h"
#include "servo_controller.h"
#include "settings.h"

#include <charconv>
#include <atomic>
#include <memory>
#include <cmath>
#include "cJSON.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <cstdio>
#include <cstring>
#include <string_view>

extern "C" const char _binary_control_panel_html_start[];
extern "C" const char _binary_visitor_alert_ogg_start[];
extern "C" const char _binary_visitor_alert_ogg_end[];

namespace {
std::atomic<bool> queueing_voice{false};
struct VoiceClip {
    size_t size;
    uint8_t data[];
};

size_t PacketSize(const uint8_t* header) {
    return (size_t(header[2]) << 8) | header[3];
}

bool ValidVoiceClip(const VoiceClip& clip) {
    size_t offset = 0, frames = 0;
    while (offset < clip.size) {
        const auto* header = clip.data + offset;
        if (clip.size - offset < sizeof(BinaryProtocol3) || header[0] || header[1]) return false;
        size_t size = PacketSize(header);
        offset += sizeof(BinaryProtocol3);
        if (!size || size > 1275 || size > clip.size - offset || ++frames > 2000) return false;
        offset += size;
    }
    return frames != 0;
}

void PlayVoice(void* arg) {
    auto* clip = static_cast<VoiceClip*>(arg);
    if (clip) {
        auto& audio = Application::GetInstance().GetAudioService();
        size_t offset = 0;
        while (offset < clip->size) {
            size_t size = PacketSize(clip->data + offset);
            offset += sizeof(BinaryProtocol3);
            auto packet = std::make_unique<AudioStreamPacket>();
            packet->sample_rate = 16000;
            packet->frame_duration = 60;
            packet->payload.assign(clip->data + offset, clip->data + offset + size);
            audio.PushPacketToDecodeQueue(std::move(packet), true);
            offset += size;
        }
        ESP_LOGI("Voice", "Queued synthesized speech (%u bytes)", unsigned(clip->size));
        free(clip);
    } else {
        Application::GetInstance().PlaySound({_binary_visitor_alert_ogg_start,
            static_cast<size_t>(_binary_visitor_alert_ogg_end - _binary_visitor_alert_ogg_start)});
    }
    queueing_voice = false;
    vTaskDelete(nullptr);
}

bool SpeakerAvailable() {
    auto& app = Application::GetInstance();
    return !queueing_voice && app.GetDeviceState() != kDeviceStateSpeaking && app.GetAudioService().IsIdle();
}

bool StartVoice(VoiceClip* clip) {
    queueing_voice = true;
    if (xTaskCreate(PlayVoice, "local_voice", 4096, clip, 3, nullptr) == pdPASS) return true;
    queueing_voice = false;
    return false;
}
}

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
        {.uri = "/api/voice", .method = HTTP_POST, .handler = HandleVoice, .user_ctx = this},
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
    char state[512];
    std::snprintf(state, sizeof(state),
        R"({"x":%d,"y":%d,"minX":%d,"maxX":%d,"minY":%d,"maxY":%d,)"
        R"("robot":"%02x%02x%02x%02x%02x%02x","mode":"%s","boot":%lu,"generation":%lu,"targetX":%.1f,"targetY":%.1f,"moving":%s,)"
        R"("step":%d,"manualSpeed":%d,"manualSpeedMin":%d,"manualSpeedMax":%d,)"
        R"("centerOffsetX":%d,"centerOffsetY":%d,"manual":%s,"volume":%d,"gesture":%s,"visitorAlert":%lu,"textVoice":true})",
        position.x, position.y, position.min_x, position.max_x, position.min_y, position.max_y,
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], position.mode, (unsigned long)position.boot,
        (unsigned long)position.generation, position.target_x, position.target_y, position.moving ? "true" : "false",
        ServoController::kManualStep, position.manual_speed, ServoController::kMinManualSpeed, ServoController::kMaxManualSpeed,
        position.offset_x, position.offset_y, position.manual ? "true" : "false",
        panel.audio_.output_volume(), gesture, static_cast<unsigned long>(panel.last_visitor_alert_));
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

esp_err_t ControlPanel::HandleVoice(httpd_req_t* req) {
    char guard[4], kind[12];
    auto read_id = [&](const char* key, uint32_t& id) {
        char text[12];
        if (httpd_req_get_hdr_value_str(req, key, text, sizeof(text)) != ESP_OK) return false;
        const char* end = text + strlen(text);
        auto result = std::from_chars(text, end, id);
        return result.ec == std::errc{} && result.ptr == end;
    };
    uint32_t boot, request;
    if (httpd_req_get_hdr_value_str(req, "X-Desk-Control", guard, sizeof(guard)) != ESP_OK ||
        strcmp(guard, "1") || !read_id("X-Voice-Boot", boot) || !read_id("X-Voice-Request", request) ||
        !request || httpd_req_get_hdr_value_str(req, "X-Voice-Kind", kind, sizeof(kind)) != ESP_OK ||
        (strcmp(kind, "manual") && strcmp(kind, "visitor")) ||
        req->content_len < 5 || req->content_len > 256 * 1024)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid voice request");
    auto& panel = *static_cast<ControlPanel*>(req->user_ctx);
    if (panel.servos_.GetPosition().boot != boot) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Device restarted");
    }
    std::unique_ptr<VoiceClip, decltype(&free)> clip(static_cast<VoiceClip*>(
        heap_caps_malloc(sizeof(VoiceClip) + req->content_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)), free);
    if (!clip) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Voice buffer unavailable");
    clip->size = req->content_len;
    for (size_t received = 0; received < clip->size;) {
        int n = httpd_req_recv(req, reinterpret_cast<char*>(clip->data + received), clip->size - received);
        if (n <= 0) return ESP_FAIL;
        received += n;
    }
    if (!ValidVoiceClip(*clip)) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid voice frames");
    const bool visitor = !strcmp(kind, "visitor");
    auto& last = visitor ? panel.last_visitor_alert_ : panel.last_manual_voice_;
    if (last != request) {
        auto now = esp_timer_get_time();
        if (!SpeakerAvailable() || (visitor && now < panel.next_voice_at_us_)) {
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "Speaker busy");
        }
        if (!StartVoice(clip.get())) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot start playback");
        clip.release();  // The audio task owns the buffer.
        last = request;
        if (visitor) panel.next_voice_at_us_ = now + 4000000;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, R"({"accepted":true})");
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
    auto& panel = *static_cast<ControlPanel*>(req->user_ctx);
    auto& servo = panel.servos_;
    bool accepted = false;
    bool angles = field("x") || field("y");
    if (op == "visitor_alert") {
        if (angles || !number("boot", boot) || !number("request", request) || request == 0)
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected boot and alert request");
        if (boot != servo.GetPosition().boot) {
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "Device restarted");
        }
        auto now = esp_timer_get_time();
        if (request != panel.last_visitor_alert_ && now >= panel.next_voice_at_us_ && SpeakerAvailable()) {
            if (!StartVoice(nullptr))
                return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot start voice playback");
            panel.last_visitor_alert_ = request;
            panel.next_voice_at_us_ = now + 4000000;
        }
        return ServeState(req);
    }
    if (op == "save_center" || op == "reset_center") {
        if (angles || !number("boot", boot) || !number("generation", generation))
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected boot and generation");
        auto err = servo.CalibrateCenter(op == "reset_center", boot, generation);
        if (err == ESP_ERR_INVALID_ARG)
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Center correction exceeds 15 degrees");
        if (err == ESP_ERR_INVALID_STATE) {
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "Select manual mode and wait until stationary");
        }
        if (err != ESP_OK) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot store servo center");
        return ServeState(req);
    }
    if (angles && (!number("x", x) || !number("y", y) ||
        x < SERVO_MIN_X || x > SERVO_MAX_X || y < SERVO_MIN_Y || y > SERVO_MAX_Y))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Angles outside limits");
    if (op == "speed") {
        uint32_t value;
        if (angles || !number("value", value) || value > ServoController::kMaxManualSpeed)
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
