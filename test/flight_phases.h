
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
} control_settings_t;

typedef enum {
    EV_READY,
    EV_LIFTOFF,
    EV_MAX_V,
    EV_TOUCHDOWN,
    EV_APOGEE,
    EV_MAIN_DEPLOYMENT,
    // Add other events as needed
} cats_event_e;
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
    uint8_t state_changed; // Add this line to track state changes
    uint32_t thrust_trigger_time;
    uint32_t iteration_count; // Add this line to count iterations
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

#endif // FLIGHT_PHASES_H