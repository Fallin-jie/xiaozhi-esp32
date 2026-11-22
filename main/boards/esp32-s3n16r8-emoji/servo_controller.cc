/**
 * @file servo_controller.cc
 * @brief 舵机控制模块实现
 */

#include "servo_controller.h"
#include <esp_log.h>
#include <esp_random.h>
#include <nvs.h>
#include <cmath>

#define TAG "ServoController"

namespace {
constexpr int kMaxCenterOffset = 15;
static_assert(SERVO_MIN_X - kMaxCenterOffset >= 0 && SERVO_MAX_X + kMaxCenterOffset <= 180);
static_assert(SERVO_MIN_Y - kMaxCenterOffset >= 0 && SERVO_MAX_Y + kMaxCenterOffset <= 180);
constexpr char kMotionNamespace[] = "head_motion";
constexpr char kCenterKey[] = "center_xy";
}

ServoController::ServoController() {
}

ServoController::~ServoController() {
    if (timer_) { esp_timer_stop(timer_); esp_timer_delete(timer_); }
}

void ServoController::Initialize() {
    // 配置LEDC定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_BIT_WIDTH,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 配置LEDC通道
    for (int i = 0; i < SERVO_CHANNEL_COUNT; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_MODE,
            .channel = servo_channels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    boot_ = esp_random() | 1U;
    nvs_handle_t handle;
    if (nvs_open(kMotionNamespace, NVS_READONLY, &handle) == ESP_OK) {
        uint16_t centers;
        if (nvs_get_u16(handle, kCenterKey, &centers) == ESP_OK) {
            int dx = (centers >> 8) - SERVO_CENTER_X;
            int dy = (centers & 0xff) - SERVO_CENTER_Y;
            if (std::abs(dx) <= kMaxCenterOffset && std::abs(dy) <= kMaxCenterOffset) {
                offset_x_ = dx;
                offset_y_ = dy;
            }
        }
        nvs_close(handle);
    }
    WriteAngle(0, output_.x);
    WriteAngle(1, output_.y);
    esp_timer_create_args_t args{};
    args.callback = [](void* self) { static_cast<ServoController*>(self)->Tick(); };
    args.arg = this;
    args.name = "head_motion";
    args.skip_unhandled_events = true;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_, 20000));
}

thread_local uint32_t ServoController::scope_ = 0;
ServoController::AutoScope::AutoScope(uint32_t epoch) : previous_(scope_) {
    if (!scope_) scope_ = epoch;
}
ServoController::AutoScope::~AutoScope() { scope_ = previous_; }
uint32_t ServoController::AutoEpoch() const {
    std::lock_guard lock(mutex_);
    return scope_ ? scope_ : epoch_;
}
bool ServoController::AutoAllowed() const {
    return mode_ == Mode::Auto && (!scope_ || scope_ == epoch_);
}

void ServoController::WriteAngle(int channel, float angle) {
    angle += channel == 0 ? offset_x_ : offset_y_;
    float pulse = SERVO_MIN_PULSEWIDTH + angle * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.f;
    auto duty = std::lround(pulse * ((1 << LEDC_TIMER_BIT_WIDTH) - 1) * LEDC_FREQUENCY / 1000000.f);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, servo_channels[channel], duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, servo_channels[channel]));
}

void ServoController::Tick() {
    std::lock_guard lock(mutex_);
    const auto now = esp_timer_get_time();
    Expire(now);
    // Small timer jitter is normal. Account for early ticks and discard missed time.
    const auto elapsed_us = now - last_tick_;
    if (elapsed_us < 10000) return;
    const float dt = std::min<int64_t>(elapsed_us, 20000) / 1000000.f;
    last_tick_ = now;
    if (output_.x == target_.x && output_.y == target_.y && count_) {
        target_ = queue_[first_];
        first_ = (first_ + 1) % queue_.size();
        --count_;
    }
    float dx = target_.x - output_.x, dy = target_.y - output_.y;
    float distance = std::abs(dx) + std::abs(dy);
    if (distance == 0) { speed_ = 0; return; }

    const float limit = mode_ == Mode::Manual ? manual_speed_ : kDefaultSpeed;
    const float acceleration = 2.f * limit;  // Half a second to reach cruising speed.
    speed_ = std::min({limit, speed_ + acceleration * dt, std::sqrt(2 * acceleration * distance)});
    float step = speed_ * dt;
    if (step >= distance) { output_ = target_; speed_ = 0; }
    else { output_.x += dx * step / distance; output_.y += dy * step / distance; }
    WriteAngle(0, output_.x);
    WriteAngle(1, output_.y);
}

void ServoController::Switch(Mode mode) {
    mode_ = mode;
    ++epoch_;
    first_ = count_ = 0;
    session_ = 0;
    target_ = Limit(output_);
    speed_ = 0;
}
void ServoController::Expire(int64_t now) {
    if (mode_ == Mode::Follow && now >= deadline_) Switch(Mode::Auto);
}
ServoController::Position ServoController::GetPosition() const {
    std::lock_guard lock(mutex_);
    return {int(std::lround(output_.x)), int(std::lround(output_.y)), mode_ == Mode::Manual,
            target_.x, target_.y, mode_ == Mode::Follow ? "follow" : mode_ == Mode::Manual ? "manual" : "auto",
            boot_, epoch_, output_.x != target_.x || output_.y != target_.y || count_ > 0, manual_speed_,
            offset_x_, offset_y_, SERVO_MIN_X, SERVO_MAX_X, SERVO_MIN_Y, SERVO_MAX_Y};
}
ServoController::Angles ServoController::Limit(Angles point) {
    // Travel is measured from the calibrated neutral. WriteAngle translates it to PWM.
    return {std::clamp(point.x, float(SERVO_MIN_X), float(SERVO_MAX_X)),
            std::clamp(point.y, float(SERVO_MIN_Y), float(SERVO_MAX_Y))};
}

esp_err_t ServoController::CalibrateCenter(bool reset, uint32_t boot, uint32_t generation) {
    std::lock_guard lock(mutex_);
    if (boot != boot_ || generation != epoch_ || mode_ != Mode::Manual ||
        output_.x != target_.x || output_.y != target_.y || count_) return ESP_ERR_INVALID_STATE;
    const int dx = reset ? 0 : offset_x_ + int(std::lround(output_.x)) - SERVO_CENTER_X;
    const int dy = reset ? 0 : offset_y_ + int(std::lround(output_.y)) - SERVO_CENTER_Y;
    if (std::abs(dx) > kMaxCenterOffset || std::abs(dy) > kMaxCenterOffset) return ESP_ERR_INVALID_ARG;
    if (dx == offset_x_ && dy == offset_y_) return ESP_OK;

    nvs_handle_t handle;
    auto err = nvs_open(kMotionNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    uint16_t centers = ((SERVO_CENTER_X + dx) << 8) | (SERVO_CENTER_Y + dy);
    err = nvs_set_u16(handle, kCenterKey, centers);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) return err;

    // Rebase the coordinates at rest so saving or resetting never steps the PWM.
    output_.x += offset_x_ - dx;
    output_.y += offset_y_ - dy;
    // Resetting neutral at an old endpoint can leave the head outside the new travel.
    // Tick brings it to the nearest endpoint under the normal speed limit.
    target_ = Limit(output_);
    offset_x_ = dx;
    offset_y_ = dy;
    ++epoch_;
    return ESP_OK;
}
bool ServoController::SetManualSpeed(int degrees_per_second) {
    if (degrees_per_second < kMinManualSpeed || degrees_per_second > kMaxManualSpeed) return false;
    std::lock_guard lock(mutex_);
    manual_speed_ = degrees_per_second;
    return true;
}

bool ServoController::InRange(int x, int y) {
    return x >= SERVO_MIN_X && x <= SERVO_MAX_X && y >= SERVO_MIN_Y && y <= SERVO_MAX_Y;
}
void ServoController::MoveManually(ManualMove move) {
    std::lock_guard lock(mutex_);
    if (mode_ != Mode::Manual) Switch(Mode::Manual); else ++epoch_;
    auto previous = target_;
    switch (move) {
        case ManualMove::Left: target_.x -= kManualStep; break;
        case ManualMove::Right: target_.x += kManualStep; break;
        case ManualMove::Up: target_.y -= kManualStep; break;
        case ManualMove::Down: target_.y += kManualStep; break;
        case ManualMove::Center: target_ = {SERVO_CENTER_X, SERVO_CENTER_Y}; break;
    }
    target_ = Limit(target_);
    ResetSpeedOnTurn(previous);
}
void ServoController::ResetSpeedOnTurn(Angles previous) {
    if ((previous.x-output_.x)*(target_.x-output_.x) < 0 ||
        (previous.y-output_.y)*(target_.y-output_.y) < 0) speed_ = 0;
}
bool ServoController::ManualAngles(int x, int y) {
    if (!InRange(x, y)) return false;
    std::lock_guard lock(mutex_);
    if (mode_ != Mode::Manual) Switch(Mode::Manual); else ++epoch_;
    auto previous = target_;
    target_ = Limit({float(x), float(y)});
    ResetSpeedOnTurn(previous);
    return true;
}
void ServoController::ResumeAutomatic() {
    std::lock_guard lock(mutex_);
    Switch(Mode::Auto);
}
uint32_t ServoController::StartFollow(uint32_t boot, uint32_t generation, uint32_t request) {
    std::lock_guard lock(mutex_);
    auto now = esp_timer_get_time();
    Expire(now);
    if (boot != boot_ || !request) return 0;
    if (mode_ == Mode::Follow) return generation + 1 == epoch_ && request == request_ ? session_ : 0;
    if (generation != epoch_) return 0;
    Switch(Mode::Follow);
    request_ = request;
    session_ = esp_random() | 1U;
    seq_ = 0;
    deadline_ = now + 3000000;
    return session_;
}
bool ServoController::UpdateFollow(uint32_t session, uint32_t seq, bool hold, int x, int y) {
    std::lock_guard lock(mutex_);
    auto now = esp_timer_get_time();
    Expire(now);
    if (mode_ != Mode::Follow || session != session_ || seq <= seq_) return false;
    if (x != -1 && !InRange(x, y)) return false;
    if (hold) { target_ = Limit(output_); speed_ = 0; }
    else if (x != -1) {
        auto previous = target_;
        target_ = Limit({float(x), float(y)});
        ResetSpeedOnTurn(previous);
    }
    seq_ = seq;
    deadline_ = now + 3000000;
    return true;
}
bool ServoController::StopFollow(uint32_t session) {
    std::lock_guard lock(mutex_);
    if (mode_ != Mode::Follow || session != session_) return false;
    Switch(Mode::Auto);
    return true;
}

ServoController::Angles ServoController::Last() const {
    return count_ ? queue_[(first_ + count_ - 1) % queue_.size()] : target_;
}
void ServoController::Append(Angles point) {
    point = Limit(point);
    auto end = Last();
    if (point.x == end.x && point.y == end.y) return;
    if (!count_ && output_.x == target_.x && output_.y == target_.y) target_ = point;
    else if (count_ < queue_.size()) queue_[(first_ + count_++) % queue_.size()] = point;
    else queue_[(first_ + count_ - 1) % queue_.size()] = point;
}
bool ServoController::SetServoAngle(int channel, int angle) {
    std::lock_guard lock(mutex_);
    if (!AutoAllowed() || channel < 0 || channel >= SERVO_CHANNEL_COUNT) return false;
    auto p = Last();
    if (channel == 0) p.x = angle; else p.y = angle;
    Append(p);
    return true;
}
void ServoController::HeadMove(int x, int y, int) {
    std::lock_guard lock(mutex_);
    if (AutoAllowed()) { auto p = Last(); Append({p.x + x, p.y + y}); }
}
void ServoController::HeadCenter(int) {
    std::lock_guard lock(mutex_);
    if (AutoAllowed()) Append({SERVO_CENTER_X, SERVO_CENTER_Y});
}
void ServoController::HeadNod(int) {
    std::lock_guard lock(mutex_);
    if (!AutoAllowed()) return;
    float x = Last().x;
    for (int i = 0; i < 3; ++i) { Append({x, 110}); Append({x, 70}); }
    Append({x, 90});
}
void ServoController::HeadShake(int) {
    std::lock_guard lock(mutex_);
    if (!AutoAllowed()) return;
    float y = Last().y;
    for (int i = 0; i < 2; ++i) { Append({70, y}); Append({110, y}); }
    Append({90, y});
}
void ServoController::HeadRoll(int) {
    std::lock_guard lock(mutex_);
    if (!AutoAllowed()) return;
    for (Angles p : {Angles{90,90}, {90,107}, {130,95}, {90,83}, {50,95}, {90,107}, {90,90}}) Append(p);
}
void ServoController::HeadUp(int offset) { HeadMove(0, -offset); }
void ServoController::HeadDown(int offset) { HeadMove(0, offset); }
void ServoController::HeadLeft(int offset) { HeadMove(-offset, 0); }
void ServoController::HeadRight(int offset) { HeadMove(offset, 0); }
