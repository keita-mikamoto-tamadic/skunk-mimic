#include <iostream>
#include <cmath>
#include <stdexcept>
#include "robot_control_manager.hpp"

// --- テスト基盤 ---
static int g_passed = 0, g_failed = 0;

#define RUN_TEST(fn) do { \
    std::cout << #fn << "... "; \
    try { fn(); std::cout << "PASS\n"; g_passed++; } \
    catch (...) { std::cout << "FAIL\n"; g_failed++; } \
} while(0)

#define ASSERT_EQ(a, b)        if ((a) != (b)) throw std::runtime_error("ASSERT_EQ")
#define ASSERT_TRUE(c)         if (!(c)) throw std::runtime_error("ASSERT_TRUE")
#define ASSERT_FALSE(c)        if ((c)) throw std::runtime_error("ASSERT_FALSE")
#define ASSERT_NEAR(a, b, eps) if (std::abs((a)-(b)) > (eps)) throw std::runtime_error("ASSERT_NEAR")

// --- ヘルパー ---
static AxisConfig makeAxis(double init_pos) {
    return {.device_id = 1, .motdir = 1, .initial_position = init_pos,
            .reset_position = 0.0, .velocity_limit = 1.0,
            .accel_limit = 1.0, .torque_limit = 1.0};
}

static AxisStatus makeStatus(double pos) {
    return {.position = pos, .velocity = 0.0, .torque = 0.0, .fault = 0};
}

// =========================================================
// Phase 1: 基本
// =========================================================
void test_initial_state_is_off() {
    RobotControlManager sm;
    ASSERT_EQ(sm.getState(), State::OFF);
}

void test_configure_keeps_state() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)});
    ASSERT_EQ(sm.getState(), State::OFF);
}

// =========================================================
// Phase 2: 状態遷移
// =========================================================
void test_servo_on_off_to_stop() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    ASSERT_EQ(sm.getState(), State::STOP);
}

void test_servo_on_from_stop_ignored() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::SERVO_ON);
    ASSERT_EQ(sm.getState(), State::STOP);
}

void test_ready_from_stop() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    ASSERT_EQ(sm.getState(), State::READY);
}

void test_ready_from_off_ignored() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::READY);
    ASSERT_EQ(sm.getState(), State::OFF);
}

void test_run_from_ready() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.handleStateCommand(StateCommand::RUN);
    ASSERT_EQ(sm.getState(), State::RUN);
}

void test_run_from_stop_ignored() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::RUN);
    ASSERT_EQ(sm.getState(), State::STOP);
}

void test_stop_from_run() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.handleStateCommand(StateCommand::RUN);
    sm.handleStateCommand(StateCommand::STOP);
    ASSERT_EQ(sm.getState(), State::STOP);
}

void test_stop_from_off_ignored() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::STOP);
    ASSERT_EQ(sm.getState(), State::OFF);
}

void test_servo_off_from_any() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.handleStateCommand(StateCommand::RUN);
    sm.handleStateCommand(StateCommand::SERVO_OFF);
    ASSERT_EQ(sm.getState(), State::OFF);
}

// =========================================================
// Phase 3: robotController 基本出力
// =========================================================
void test_ctrl_off_motor_off() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)});
    auto ctrl = sm.robotController();
    ASSERT_EQ(ctrl.commands.size(), (size_t)1);
    ASSERT_EQ(ctrl.commands[0].state, MotorState::OFF);
    ASSERT_NEAR(ctrl.commands[0].ref_val, 0.0, 1e-9);
}

void test_ctrl_stop_holds_position() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)});
    sm.updateMotorStatus({makeStatus(0.75)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    auto ctrl = sm.robotController();
    ASSERT_EQ(ctrl.commands[0].state, MotorState::STOP);
    ASSERT_NEAR(ctrl.commands[0].ref_val, 0.75, 1e-9);
}

void test_ctrl_run_holds_initial() {
    RobotControlManager sm;
    sm.configure({makeAxis(2.5)}, nullptr, 1.0, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.robotController();  // 補間完了
    sm.handleStateCommand(StateCommand::RUN);
    auto ctrl = sm.robotController();
    ASSERT_EQ(ctrl.commands[0].state, MotorState::POSITION);
    ASSERT_NEAR(ctrl.commands[0].ref_val, 2.5, 1e-9);
}

void test_ctrl_command_count() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0), makeAxis(1.0), makeAxis(2.0)});
    auto ctrl = sm.robotController();
    ASSERT_EQ(ctrl.commands.size(), (size_t)3);
}

// =========================================================
// Phase 4: READY 補間
// =========================================================
void test_ready_starts_from_current() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)}, nullptr, 0.5, 1.0);
    sm.updateMotorStatus({makeStatus(0.5)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    auto ctrl = sm.robotController();
    ASSERT_NEAR(ctrl.commands[0].ref_val, 0.5, 1e-9);
}

void test_ready_interpolates() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)}, nullptr, 0.3, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);

    auto c1 = sm.robotController();  // progress=0 → ref=0.0, progress+=0.3
    ASSERT_NEAR(c1.commands[0].ref_val, 0.0, 1e-9);

    auto c2 = sm.robotController();  // progress=0.3 → ref=0.3, progress+=0.3
    ASSERT_NEAR(c2.commands[0].ref_val, 0.3, 1e-9);
}

void test_ready_clamps_at_one() {
    RobotControlManager sm;
    sm.configure({makeAxis(2.0)}, nullptr, 0.5, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);

    sm.robotController();  // progress: 0 → 0.5
    sm.robotController();  // progress: 0.5 → 1.0
    auto c3 = sm.robotController();  // progress=1.0 → ref=2.0
    ASSERT_NEAR(c3.commands[0].ref_val, 2.0, 1e-9);
}

void test_ready_multi_axis() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0), makeAxis(2.0)}, nullptr, 0.5, 1.0);
    sm.updateMotorStatus({makeStatus(0.0), makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);

    sm.robotController();  // progress: 0 → 0.5
    auto c2 = sm.robotController();  // progress=0.5 → ref=[0.5, 1.0]
    ASSERT_NEAR(c2.commands[0].ref_val, 0.5, 1e-9);
    ASSERT_NEAR(c2.commands[1].ref_val, 1.0, 1e-9);
}

void test_ready_reentry_resets() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)}, nullptr, 0.3, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.robotController();  // progress: 0 → 0.3
    sm.robotController();  // progress: 0.3 → 0.6

    sm.handleStateCommand(StateCommand::STOP);
    sm.updateMotorStatus({makeStatus(0.6)});
    sm.handleStateCommand(StateCommand::READY);

    auto c1 = sm.robotController();  // progress=0 → ref=0.6
    ASSERT_NEAR(c1.commands[0].ref_val, 0.6, 1e-9);
}

// =========================================================
// Phase 5: ready_complete フラグ
// =========================================================
void test_ready_complete_false_during_interp() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)}, nullptr, 0.3, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    auto ctrl = sm.robotController();
    ASSERT_FALSE(ctrl.ready_complete);
}

void test_ready_complete_true_after_interp() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)}, nullptr, 1.0, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    auto ctrl = sm.robotController();  // progress: 0 → 1.0
    ASSERT_TRUE(ctrl.ready_complete);
}

void test_ready_complete_persists_in_run() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)}, nullptr, 1.0, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.robotController();
    sm.handleStateCommand(StateCommand::RUN);
    auto ctrl = sm.robotController();
    ASSERT_TRUE(ctrl.ready_complete);
}

// =========================================================
// Phase 6: フォルト注入
// =========================================================
void test_no_evaluator_ignores_fault() {
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)});
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.updateMotorStatus({{.position = 0.0, .velocity = 0.0, .torque = 0.0, .fault = 99}});
    ASSERT_EQ(sm.getState(), State::STOP);
}

void test_evaluator_triggers_stop() {
    auto eval = [](uint8_t fault) -> std::optional<State> {
        if (fault != 0) return State::STOP;
        return std::nullopt;
    };
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)}, eval);
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.handleStateCommand(StateCommand::RUN);
    sm.updateMotorStatus({{.position = 0.0, .velocity = 0.0, .torque = 0.0, .fault = 1}});
    ASSERT_EQ(sm.getState(), State::STOP);
}

void test_evaluator_nullopt_no_change() {
    auto eval = [](uint8_t) -> std::optional<State> { return std::nullopt; };
    RobotControlManager sm;
    sm.configure({makeAxis(0.0)}, eval);
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.updateMotorStatus({{.position = 0.0, .velocity = 0.0, .torque = 0.0, .fault = 42}});
    ASSERT_EQ(sm.getState(), State::STOP);
}

void test_evaluator_first_fault_wins() {
    auto eval = [](uint8_t fault) -> std::optional<State> {
        if (fault == 1) return State::STOP;
        if (fault == 2) return State::OFF;
        return std::nullopt;
    };
    RobotControlManager sm;
    sm.configure({makeAxis(0.0), makeAxis(0.0)}, eval);
    sm.handleStateCommand(StateCommand::SERVO_ON);
    sm.handleStateCommand(StateCommand::READY);
    sm.handleStateCommand(StateCommand::RUN);
    sm.updateMotorStatus({
        {.position = 0.0, .velocity = 0.0, .torque = 0.0, .fault = 1},
        {.position = 0.0, .velocity = 0.0, .torque = 0.0, .fault = 2}
    });
    ASSERT_EQ(sm.getState(), State::STOP);
}

// =========================================================
// Phase 7: エッジケース
// =========================================================
void test_empty_axes() {
    RobotControlManager sm;
    sm.configure({});
    auto ctrl = sm.robotController();
    ASSERT_EQ(ctrl.commands.size(), (size_t)0);
}

void test_full_lifecycle() {
    RobotControlManager sm;
    sm.configure({makeAxis(1.0)}, nullptr, 1.0, 1.0);
    sm.updateMotorStatus({makeStatus(0.0)});

    ASSERT_EQ(sm.getState(), State::OFF);
    sm.handleStateCommand(StateCommand::SERVO_ON);
    ASSERT_EQ(sm.getState(), State::STOP);
    sm.handleStateCommand(StateCommand::READY);
    ASSERT_EQ(sm.getState(), State::READY);

    auto ctrl = sm.robotController();
    ASSERT_TRUE(ctrl.ready_complete);

    sm.handleStateCommand(StateCommand::RUN);
    ASSERT_EQ(sm.getState(), State::RUN);

    ctrl = sm.robotController();
    ASSERT_EQ(ctrl.commands[0].state, MotorState::POSITION);
    ASSERT_NEAR(ctrl.commands[0].ref_val, 1.0, 1e-9);

    sm.handleStateCommand(StateCommand::STOP);
    ASSERT_EQ(sm.getState(), State::STOP);
    sm.handleStateCommand(StateCommand::SERVO_OFF);
    ASSERT_EQ(sm.getState(), State::OFF);
}

// =========================================================
// main
// =========================================================
int main() {
    std::cout << "=== RobotControlManager Unit Tests ===\n\n";

    // Phase 1: 基本
    RUN_TEST(test_initial_state_is_off);
    RUN_TEST(test_configure_keeps_state);
    // Phase 2: 状態遷移
    RUN_TEST(test_servo_on_off_to_stop);
    RUN_TEST(test_servo_on_from_stop_ignored);
    RUN_TEST(test_ready_from_stop);
    RUN_TEST(test_ready_from_off_ignored);
    RUN_TEST(test_run_from_ready);
    RUN_TEST(test_run_from_stop_ignored);
    RUN_TEST(test_stop_from_run);
    RUN_TEST(test_stop_from_off_ignored);
    RUN_TEST(test_servo_off_from_any);
    // Phase 3: robotController 基本出力
    RUN_TEST(test_ctrl_off_motor_off);
    RUN_TEST(test_ctrl_stop_holds_position);
    RUN_TEST(test_ctrl_run_holds_initial);
    RUN_TEST(test_ctrl_command_count);
    // Phase 4: READY 補間
    RUN_TEST(test_ready_starts_from_current);
    RUN_TEST(test_ready_interpolates);
    RUN_TEST(test_ready_clamps_at_one);
    RUN_TEST(test_ready_multi_axis);
    RUN_TEST(test_ready_reentry_resets);
    // Phase 5: ready_complete
    RUN_TEST(test_ready_complete_false_during_interp);
    RUN_TEST(test_ready_complete_true_after_interp);
    RUN_TEST(test_ready_complete_persists_in_run);
    // Phase 6: フォルト注入
    RUN_TEST(test_no_evaluator_ignores_fault);
    RUN_TEST(test_evaluator_triggers_stop);
    RUN_TEST(test_evaluator_nullopt_no_change);
    RUN_TEST(test_evaluator_first_fault_wins);
    // Phase 7: エッジケース
    RUN_TEST(test_empty_axes);
    RUN_TEST(test_full_lifecycle);

    std::cout << "\n=== " << g_passed << " passed, " << g_failed << " failed ===\n";
    return g_failed ? 1 : 0;
}
