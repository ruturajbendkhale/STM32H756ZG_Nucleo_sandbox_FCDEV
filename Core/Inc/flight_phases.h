
#ifndef FLIGHT_PHASES_H
#define FLIGHT_PHASES_H

#include <stdint.h> // For standard integer types
#include <math.h> // For mathematical functions

typedef float float32_t;
typedef struct {
    float x;
    float y;
    float z;
} vf32_t;

typedef struct {
    float acceleration;
    float velocity;
    float height;
} estimation_output_t;

typedef struct {
    float liftoff_acc_threshold;
    // Add other settings as needed
} control_settings_t;


// Define flight states
typedef enum {
    READY,
    THRUSTING,
    COASTING,
    DROGUE,
    MAIN,
    TOUCHDOWN
} flight_fsm_e;

// Define the flight state machine structure
typedef struct {
    flight_fsm_e flight_state;
    float32_t memory[3]; // Adjust size as needed
    // Add other necessary fields
} flight_fsm_t;

// Function prototypes
void check_flight_phase(flight_fsm_t *fsm_state, vf32_t acc_data, vf32_t gyro_data, estimation_output_t state_data, const control_settings_t *settings);
static void check_ready_phase(flight_fsm_t *fsm_state, vf32_t acc_data, const control_settings_t *settings);
static void check_thrusting_phase(flight_fsm_t *fsm_state, estimation_output_t state_data);
static void check_coasting_phase(flight_fsm_t *fsm_state, estimation_output_t state_data);
static void check_drogue_phase(flight_fsm_t *fsm_state, estimation_output_t state_data);
static void check_main_phase(flight_fsm_t *fsm_state, estimation_output_t state_data);
static void clear_fsm_memory(flight_fsm_t *fsm_state);
static void change_state_to(flight_fsm_e new_state, cats_event_e event_to_trigger, flight_fsm_t *fsm_state);

#endif // FLIGHT_PHASES_C_H