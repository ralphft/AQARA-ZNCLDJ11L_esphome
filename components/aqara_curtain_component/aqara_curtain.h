#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
#include <vector>
#include <cstdint>

namespace esphome {
namespace aqara_curtain {

// ── Protocol constants ──────────────────────────────────────────────────────

static const uint8_t MSG_OPEN[]    = {0x03, 0x01, 0xb9, 0x24};
static const uint8_t MSG_CLOSE[]   = {0x03, 0x02, 0xf9, 0x25};
static const uint8_t MSG_PAUSE[]   = {0x03, 0x03, 0x38, 0xe5};
static const uint8_t MSG_UNCAL[]   = {0x03, 0x07, 0x39, 0x26};
static const uint8_t MSG_REQPOS[]  = {0x01, 0x02, 0x01, 0x85, 0x42};
static const uint8_t MSG_REQDIR[]  = {0x01, 0x03, 0x01, 0x84, 0xd2};
static const uint8_t MSG_REQSTA[]  = {0x01, 0x05, 0x01, 0x87, 0x72};
static const uint8_t MSG_REQCAL[]  = {0x01, 0x09, 0x01, 0x82, 0x72};
static const uint8_t MSG_SETDIR0[] = {0x02, 0x03, 0x01, 0x00, 0xd2, 0x27};
static const uint8_t MSG_SETDIR1[] = {0x02, 0x03, 0x01, 0x01, 0x13, 0xe7};

static const uint8_t PREAMBLE[] = {0x55, 0xfe, 0xfe};

// ── CRC16/Modbus ────────────────────────────────────────────────────────────

inline uint16_t crc16_modbus(const uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t) data[pos];
        for (int i = 8; i != 0; i--) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else               { crc >>= 1; }
        }
    }
    return crc;
}

// ── ESPHome Cover ────────────────────────────────────────────────────────────

class AqaraCurtainComponent;

class AqaraCurtainCover : public cover::Cover {
public:
    void set_parent(AqaraCurtainComponent *parent) { parent_ = parent; }

    cover::CoverTraits get_traits() override {
        auto traits = cover::CoverTraits();
        traits.set_supports_position(true);
        traits.set_supports_stop(true);
        traits.set_is_assumed_state(false);
        return traits;
    }

    void control(const cover::CoverCall &call) override;

protected:
    AqaraCurtainComponent *parent_{nullptr};
};

// ── Main component ───────────────────────────────────────────────────────────

class AqaraCurtainComponent : public Component, public uart::UARTDevice {
public:
    void set_cover(AqaraCurtainCover *cover) {
        cover_ = cover;
        cover_->set_parent(this);
    }

    void request_open() {
        request_motion_(cover::COVER_OPERATION_OPENING, 100, false);
    }

    void request_close() {
        request_motion_(cover::COVER_OPERATION_CLOSING, 0, false);
    }

    void request_stop() {
        waiting_for_stop_ = false;
        pending_use_position_ = false;
        pending_operation_ = cover::COVER_OPERATION_IDLE;
        cmd_pause();
        set_operation_(cover::COVER_OPERATION_IDLE);
    }

    void request_position(uint8_t pct, cover::CoverOperation desired_operation) {
        request_motion_(desired_operation, pct, true);
    }

    // ── Component lifecycle ─────────────────────────────────────────────────

    void setup() override {
        // Request current state from motor
        send_raw(MSG_REQCAL, sizeof(MSG_REQCAL));
        send_raw(MSG_REQDIR, sizeof(MSG_REQDIR));
        send_raw(MSG_REQPOS, sizeof(MSG_REQPOS));
        send_raw(MSG_REQSTA, sizeof(MSG_REQSTA));
    }

    void loop() override {
        // Read incoming bytes
        while (available()) {
            uint8_t byte = read();
            if (idx_ < 16) buff_[idx_++] = byte;
            process_buffer_();
        }

        // Periodic poll (~1 Hz when idle, faster when moving)
        uint32_t now = millis();
        uint32_t interval = (moving_) ? 200 : 1000;
        if (now - last_poll_ >= interval) {
            last_poll_ = now;
            send_raw(MSG_REQPOS, sizeof(MSG_REQPOS));
            send_raw(MSG_REQSTA, sizeof(MSG_REQSTA));
            send_raw(MSG_REQCAL, sizeof(MSG_REQCAL));
        }
    }

    // ── Public command API ──────────────────────────────────────────────────

    void cmd_open()  { send_raw(MSG_OPEN,  sizeof(MSG_OPEN));  moving_ = true; }
    void cmd_close() { send_raw(MSG_CLOSE, sizeof(MSG_CLOSE)); moving_ = true; }
    void cmd_pause() { send_raw(MSG_PAUSE, sizeof(MSG_PAUSE)); }

    void cmd_set_position(uint8_t pct) {
        // Build: 0x55 0xFE 0xFE 0x03 0x04 <pct> <crcL> <crcH>
        uint8_t payload[6] = {0x55, 0xfe, 0xfe, 0x03, 0x04, pct};
        uint16_t crc = crc16_modbus(payload, 6);
        // send as full frame
        write_array(PREAMBLE, 3);
        uint8_t hdr[2] = {0x03, 0x04};
        write_array(hdr, 2);
        write_byte(pct);
        write_byte((uint8_t)(crc & 0xFF));
        write_byte((uint8_t)(crc >> 8));
        moving_ = true;
    }

    void cmd_uncalibrate() { send_raw(MSG_UNCAL, sizeof(MSG_UNCAL)); calibrated_ = false; }

    void cmd_set_reversed(bool reversed) {
        if (reversed) send_raw(MSG_SETDIR1, sizeof(MSG_SETDIR1));
        else          send_raw(MSG_SETDIR0, sizeof(MSG_SETDIR0));
        send_raw(MSG_REQDIR, sizeof(MSG_REQDIR));
    }

    bool is_calibrated() const { return calibrated_; }
    bool is_reversed()   const { return reversed_; }
    bool is_position_aware() const { return aware_; }

protected:
    AqaraCurtainCover *cover_{nullptr};

    uint8_t  buff_[16]{};
    int      idx_{0};

    bool     calibrated_{false};
    bool     reversed_{false};
    bool     moving_{false};
    bool     aware_{false};       // motor knows its position
    bool     waiting_for_stop_{false};
    bool     pending_use_position_{false};
    uint8_t  pending_position_{0};
    cover::CoverOperation pending_operation_{cover::COVER_OPERATION_IDLE};
    uint32_t last_poll_{0};

    // ── Low-level send ──────────────────────────────────────────────────────

    void send_raw(const uint8_t *payload, int len) {
        write_array(PREAMBLE, 3);
        write_array(payload, len);
    }

    void set_operation_(cover::CoverOperation operation) {
        moving_ = (operation != cover::COVER_OPERATION_IDLE);
        if (cover_ != nullptr) cover_->current_operation = operation;
    }

    bool is_reversing_(cover::CoverOperation desired_operation) const {
        if (!moving_ || cover_ == nullptr) return false;
        return cover_->current_operation != cover::COVER_OPERATION_IDLE &&
               cover_->current_operation != desired_operation;
    }

    void dispatch_motion_(cover::CoverOperation operation, uint8_t pct, bool use_position) {
        waiting_for_stop_ = false;
        pending_use_position_ = false;
        pending_operation_ = cover::COVER_OPERATION_IDLE;

        if (use_position) {
            cmd_set_position(pct);
        } else if (operation == cover::COVER_OPERATION_OPENING) {
            cmd_open();
        } else if (operation == cover::COVER_OPERATION_CLOSING) {
            cmd_close();
        }

        set_operation_(operation);
    }

    void request_motion_(cover::CoverOperation operation, uint8_t pct, bool use_position) {
        if (is_reversing_(operation)) {
            waiting_for_stop_ = true;
            pending_use_position_ = use_position;
            pending_position_ = pct;
            pending_operation_ = operation;
            cmd_pause();
            return;
        }

        dispatch_motion_(operation, pct, use_position);
    }

    void maybe_dispatch_pending_() {
        if (!waiting_for_stop_ || moving_) return;

        const auto pending_operation = pending_operation_;
        const auto pending_position = pending_position_;
        const bool pending_use_position = pending_use_position_;

        waiting_for_stop_ = false;
        pending_use_position_ = false;
        pending_operation_ = cover::COVER_OPERATION_IDLE;

        dispatch_motion_(pending_operation, pending_position, pending_use_position);
    }

    // ── Buffer processing ───────────────────────────────────────────────────

    void shift_(int n) {
        for (int i = n; i < 16; i++) buff_[i - n] = buff_[i];
        for (int i = 16 - n; i < 16; i++) buff_[i] = 0;
        idx_ -= n;
    }

    bool crc_ok_(int len) {
        return crc16_modbus(buff_, len) == 0;
    }

    void process_buffer_() {
        while (idx_ > 0) {
            // Must start with 0x88 or 0x55
            if (buff_[0] != 0x88 && buff_[0] != 0x55) { shift_(1); continue; }

            // Short heartbeat/ack 0x88 0xF8
            if (buff_[0] == 0x88 && idx_ >= 2) {
                if (buff_[1] == 0xf8) { shift_(2); continue; }
                else                  { shift_(1); continue; }
            }
            if (buff_[0] == 0x88) break; // need more data

            // Full frame: 0x55 0xFE 0xFE ...
            if (idx_ >= 3 && !(buff_[1] == 0xfe && buff_[2] == 0xfe)) { shift_(1); continue; }
            if (idx_ < 4) break;

            uint8_t type = buff_[3];
            if ((type == 0 || type > 4) && idx_ > 3) { shift_(1); continue; }
            if (idx_ < 5) break;

            uint8_t sub = buff_[4];
            if ((sub == 0 || sub > 9) && idx_ > 4) { shift_(1); continue; }

            // Type 3: command confirmations (7 or 8 bytes)
            if (type == 3) {
                if (idx_ >= 7 && crc_ok_(7)) { parse_(7); shift_(7); continue; }
                if (idx_ >= 8 && crc_ok_(8)) { parse_(8); shift_(8); continue; }
                if (idx_ >= 8)               { shift_(1); continue; }
                break;
            }

            // Type 1 or 2: request answers (8 or 9 bytes)
            if (type == 1 || type == 2) {
                if (idx_ >= 8 && crc_ok_(8)) { parse_(8); shift_(8); continue; }
                if (idx_ >= 9 && crc_ok_(9)) { parse_(9); shift_(9); continue; }
                if (idx_ >= 9)               { shift_(1); continue; }
                break;
            }

            // Type 4 sub 3: setdir confirmation (8 bytes)
            if (type == 4 && sub == 3 && idx_ >= 8) {
                if (crc_ok_(8)) parse_(8);
                shift_(8); continue;
            }

            // Type 4 sub 2: full status report (16 bytes)
            if (type == 4 && sub == 2 && idx_ >= 16) {
                if (crc_ok_(16)) parse_(16);
                shift_(16); continue;
            }

            if (idx_ >= 16) { shift_(1); continue; } // safety
            break;
        }
    }

    // ── Parse a validated frame ─────────────────────────────────────────────

    void parse_(int len) {
        if (len < 5) return;

        uint8_t type = buff_[3];
        uint8_t sub  = buff_[4];

        // Full status report
        if (type == 0x04 && sub == 0x02 && len == 16 && buff_[5] == 0x08) {
            int pos   = buff_[6];
            int dir   = buff_[7];
            int sta   = buff_[9];
            int cal   = buff_[13];
            bool obst = (sta == 4);
            if (obst) sta = 0;

            calibrated_ = cal;
            reversed_   = dir;

            // sta: 0=stopped 1=opening 2=closing -> cover state
            auto operation = cover::COVER_OPERATION_IDLE;
            if (sta == 1) operation = cover::COVER_OPERATION_OPENING;
            else if (sta == 2) operation = cover::COVER_OPERATION_CLOSING;
            set_operation_(operation);

            update_position_(pos);
            maybe_dispatch_pending_();
            return;
        }

        // Answers to individual requests (0x55 0xFE 0xFE 0x01 <sub> 0x01 <val>)
        if (type == 0x01 && len >= 7 && buff_[5] == 0x01) {
            uint8_t val = buff_[6];

            if (sub == 0x02) { // position
                if (val == 0xff) {
                    aware_ = false;
                    if (cover_) {
                        cover_->publish_state();
                    }
                } else {
                    if (val <  3) val = 0;
                    if (val > 97) val = 100;
                    aware_ = true;
                    update_position_(val);
                }
            }

            if (sub == 0x03) { // direction
                reversed_ = val;
            }

            if (sub == 0x05) { // status (0=stopped 1=opening 2=closing)
                auto operation = cover::COVER_OPERATION_IDLE;
                if (val == 1) operation = cover::COVER_OPERATION_OPENING;
                else if (val == 2) operation = cover::COVER_OPERATION_CLOSING;

                set_operation_(operation);
                if (cover_) {
                    cover_->publish_state();
                }
                maybe_dispatch_pending_();
            }

            if (sub == 0x09) { // calibration
                calibrated_ = val;
            }
        }
    }

    void update_position_(uint8_t pct) {
        if (!cover_) return;
        if (pct < 3) pct = 0;
        if (pct > 97) pct = 100;
        // ESPHome: 0.0 = CLOSED, 1.0 = OPEN; motor: 0 = closed, 100 = open
        cover_->position = pct / 100.0f;
        cover_->publish_state();
    }
};

// ── AqaraCurtainCover::control (needs full AqaraCurtainComponent definition) ─

inline void AqaraCurtainCover::control(const cover::CoverCall &call) {
    if (!parent_) return;

    if (call.get_stop()) {
        parent_->request_stop();
        current_operation = cover::COVER_OPERATION_IDLE;
        publish_state();
        return;
    }

    if (call.get_position().has_value()) {
        float pos = *call.get_position();
        if (pos < 0.0f) pos = 0.0f;
        if (pos > 1.0f) pos = 1.0f;

        uint8_t pct = static_cast<uint8_t>(pos * 100.0f + 0.5f);

        if (pct >= 100) {
            parent_->request_open();
            current_operation = cover::COVER_OPERATION_OPENING;
        } else if (pct == 0) {
            parent_->request_close();
            current_operation = cover::COVER_OPERATION_CLOSING;
        } else {
            auto desired_operation = cover::COVER_OPERATION_OPENING;
            if (parent_->is_position_aware()) {
                desired_operation = (pos > position) ? cover::COVER_OPERATION_OPENING
                                                     : cover::COVER_OPERATION_CLOSING;
            } else {
                desired_operation = (pos >= 0.5f) ? cover::COVER_OPERATION_OPENING
                                                  : cover::COVER_OPERATION_CLOSING;
            }

            if (parent_->is_calibrated()) {
                parent_->request_position(pct, desired_operation);
            } else if (desired_operation == cover::COVER_OPERATION_OPENING) {
                parent_->request_open();
            } else {
                parent_->request_close();
            }

            current_operation = desired_operation;
        }

        publish_state();
    }
}

}  // namespace aqara_curtain
}  // namespace esphome

