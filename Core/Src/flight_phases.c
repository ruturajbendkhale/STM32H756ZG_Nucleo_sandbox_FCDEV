#include "flight_phases.h"
#include <math.h>
#include "main.h" // For HAL_GetTick()


void check_flight_phase(flight_fsm_t *fsm_state, vf32_t acc_data, vf32_t gyro_data, estimation_output_t state_data, const control_settings_t *settings, bool launch_pin_high) {
    // Save old FSM State
    flight_fsm_e old_fsm_state = fsm_state->flight_state;

    // Check FSM State
    switch (fsm_state->flight_state) {
        case READY:
            check_ready_phase(fsm_state, acc_data, settings, launch_pin_high);
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
void trigger_event(cats_event_e event, flight_fsm_t *fsm_state) {
    switch (event) {
        case EV_LIFTOFF:
            // Code to handle liftoff event //no action needed
            break;
        case EV_MAX_V:
            // Code to handle maximum velocity event //no action needed
            break;
        case EV_APOGEE:
            // Code to handle apogee event // TRIGGER the Nosecone Seperation Motor for 1 Seconds
            fsm_state->apogee_flag = true; // Set apogee flag
            fsm_state->apogee_trigger_time_ms = HAL_GetTick(); // Record apogee time
            
            // Activate Nosecone Motor
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Turn PB4 ON
            fsm_state->nosecone_motor_active = true;
            fsm_state->nosecone_motor_start_time_ms = HAL_GetTick();
            break;
        case EV_MAIN_DEPLOYMENT:
            // Code to handle main deployment event // TRIGGER the Main Parachute Deployment Motor for 2 Seconds
            fsm_state->main_deployment_flag = true; // Set main deployment flag
            
            // Activate Main Parachute Motor
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Turn PB5 ON
            fsm_state->main_parachute_motor_active = true;
            fsm_state->main_parachute_motor_start_time_ms = HAL_GetTick();
            break;
        case EV_TOUCHDOWN:
            // Code to handle touchdown event // Stop the Data Logging
            break;
        default:
            break;
    }
}

static void check_ready_phase(flight_fsm_t *fsm_state, vf32_t acc_data, const control_settings_t *settings, bool launch_pin_high) {
    /* Check if we move from READY To THRUSTING */
    
    // Condition 1: Launch Pin is HIGH (disconnected from ground)
    if (launch_pin_high) {
        change_state_to(THRUSTING, EV_LIFTOFF, fsm_state);
        return; // Exit immediately if launch pin triggers liftoff
    }

    // Condition 2: Acceleration threshold met for a duration
    /* The absolute value of the acceleration is used here to make sure that we detect liftoff */
    const float32_t accel_x = acc_data.x * acc_data.x;
    const float32_t accel_y = acc_data.y * acc_data.y;
    const float32_t accel_z = acc_data.z * acc_data.z;
    // Calculate magnitude squared for comparison with squared threshold
    const float32_t acceleration_sq = accel_x + accel_y + accel_z;

    // num iterations, if the acceleration is bigger than the threshold for 0.1 s we detect liftoff
    uint16_t LIFTOFF_SAFETY_COUNTER = 10; // 10 loops * 10ms/loop = 100ms = 0.1s

    if (acceleration_sq > (settings->liftoff_acc_threshold * settings->liftoff_acc_threshold)) {
        fsm_state->memory[0]++;
    } else {
        fsm_state->memory[0] = 0;
    }

    if (fsm_state->memory[0] > LIFTOFF_SAFETY_COUNTER) {
        change_state_to(THRUSTING, EV_LIFTOFF, fsm_state);
    }
}

static void check_thrusting_phase(flight_fsm_t *fsm_state, estimation_output_t state_data) {
    /* When acceleration is below 0, liftoff concludes */
    // num iterations, acceleration needs to be smaller than 0 for at least 0.1 s for the transition THRUSTING -> COASTING
    uint16_t COASTING_SAFETY_COUNTER = 10; // 10ms counter
    if (state_data.acceleration < 0) {
        fsm_state->memory[1]++;
    } else {
        fsm_state->memory[1] = 0;
    }

    if (fsm_state->memory[1] > COASTING_SAFETY_COUNTER) {
        change_state_to(COASTING, EV_MAX_V, fsm_state);
    }
}

static void check_coasting_phase(flight_fsm_t *fsm_state, estimation_output_t state_data) {
    /* When velocity is below 0, coasting concludes */
    // num iterations, velocity needs to be smaller than 0 for at least 0.3 s for the transition COASTING -> DROGUE
    uint16_t APOGEE_SAFETY_COUNTER = 30; // 30ms counter    
    //uint32_t thrust_trigger_time = 0;
    /* DROGUE */
    // num iterations, height needs to be smaller than user-defined for at least 0.3 s for the transition DROGUE -> MAIN
    // tick counts [ms]
    //uint16_t MIN_TICK_COUNTS_BETWEEN_THRUSTING_APOGEE = 1500;
    if (state_data.velocity < 0) {
        fsm_state->memory[2]++;
    } else {
        fsm_state->memory[2] = 0; // Reset if velocity is not below 0
    }

    if (fsm_state->memory[2] > APOGEE_SAFETY_COUNTER) {
        // Directly transition to DROGUE 
        change_state_to(DROGUE, EV_APOGEE, fsm_state);
    }   
}

static void check_drogue_phase(flight_fsm_t *fsm_state, estimation_output_t state_data) {
    float32_t Main_height = 500; // 500m
    uint16_t MAIN_SAFETY_COUNTER = 30; // 30ms counter
    bool time_condition = false;
    bool altitude_condition = false;

    // Check time-based condition
    if (fsm_state->apogee_flag) {
        if ((HAL_GetTick() - fsm_state->apogee_trigger_time_ms) >= 5000) { // 5 seconds
            time_condition = true;
        }
    }

    // Check altitude-based condition
    if (state_data.height < Main_height) {
        fsm_state->memory[3]++;
        if (fsm_state->memory[3] > MAIN_SAFETY_COUNTER) {
            altitude_condition = true;
        }
    } else {
        fsm_state->memory[3] = 0;
    }

    // Transition if either condition is met
    if (time_condition || altitude_condition) {
        change_state_to(MAIN, EV_MAIN_DEPLOYMENT, fsm_state);
    }
}

static void check_main_phase(flight_fsm_t *fsm_state, estimation_output_t state_data) {
    /* If the velocity is very small we have touchdown */
    float32_t VELOCITY_BOUND_TOUCHDOWN = 3.0f;
    uint16_t TOUCHDOWN_SAFETY_COUNTER = 100;
    if (fabsf(state_data.velocity) < VELOCITY_BOUND_TOUCHDOWN) {
        /* Touchdown achieved */
        fsm_state->memory[4]++;
    } else {
        /* Touchdown not achieved */
        fsm_state->memory[4] = 0;
    }

    if (fsm_state->memory[4] > TOUCHDOWN_SAFETY_COUNTER) {
        change_state_to(TOUCHDOWN, EV_TOUCHDOWN, fsm_state);
    }
}

static void clear_fsm_memory(flight_fsm_t *fsm_state) {
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->memory[3] = 0;
    fsm_state->memory[4] = 0;
}

static void change_state_to(flight_fsm_e new_state, cats_event_e event_to_trigger, flight_fsm_t *fsm_state) {
    // Implement state change logic
    fsm_state->flight_state = new_state;
    clear_fsm_memory(fsm_state);
    // Trigger the corresponding event
    trigger_event(event_to_trigger, fsm_state);
}

// New function to manage timed actuators like motors
void manage_timed_actuators(flight_fsm_t *fsm_state) {
    // --- Nosecone Motor Control (PB4) ---
    if (fsm_state->nosecone_motor_active) {
        if ((HAL_GetTick() - fsm_state->nosecone_motor_start_time_ms) >= 1000) { // 1000 ms = 1 second
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // Turn PB4 OFF
            fsm_state->nosecone_motor_active = false;
        }
    }

    // --- Main Parachute Motor Control (PB5) ---
    if (fsm_state->main_parachute_motor_active) {
        if ((HAL_GetTick() - fsm_state->main_parachute_motor_start_time_ms) >= 2000) { // 2000 ms = 2 seconds
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Turn PB5 OFF
            fsm_state->main_parachute_motor_active = false;
        }
    }

    // Add other timed actuator logic here if needed in the future
    // For example, main parachute motor
}
