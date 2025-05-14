#include "flight_phases.h"

void check_flight_phase(flight_fsm_t *fsm_state, vf32_t acc_data, vf32_t gyro_data, estimation_output_t state_data, const control_settings_t *settings) {
    // Save old FSM State
    flight_fsm_e old_fsm_state = fsm_state->flight_state;

    // Check FSM State
    switch (fsm_state->flight_state) {
        case READY:
            check_ready_phase(fsm_state, acc_data, settings);
            break;
        case THRUSTING:
            check_thrusting_phase(fsm_state, state_data);
            break;
        case COASTING:
            check_coasting_phase(fsm_state, state_data);
            break;
        case DROGUE:
            check_drogue_phase(fsm_state, state_data);
            break;
        case MAIN:
            check_main_phase(fsm_state, state_data);
            break;
        case TOUCHDOWN:
        default:
            break;
    }

    fsm_state->state_changed = old_fsm_state != fsm_state->flight_state;
}

// Implement the other phase check functions similarly to the original code
// ...

static void clear_fsm_memory(flight_fsm_t *fsm_state) {
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
}

static void change_state_to(flight_fsm_e new_state, cats_event_e event_to_trigger, flight_fsm_t *fsm_state) {
    // Implement state change logic
    fsm_state->flight_state = new_state;
    clear_fsm_memory(fsm_state);
    // Trigger events as necessary
}