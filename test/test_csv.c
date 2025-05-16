#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "flight_phases.h" // Include your state machine header
#include "flight_phases.c"

#define MAX_DATA_POINTS 2800

typedef struct {
    float acc_x;    // Acceleration in the x direction
    float acc_y;    // Acceleration in the y direction
    float acc_z;    // Acceleration in the z direction
    float acc_tot;  // Absolute acceleration
} AccelerationData;

AccelerationData acceleration_data[MAX_DATA_POINTS];
int data_count = 0;

// Function to read acceleration data from a CSV file
void read_acceleration_data(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error opening file: %s\n", filename);
        return;
    }

    char line[100];
    while (fgets(line, sizeof(line), file) && data_count < MAX_DATA_POINTS) {
        // Assuming the CSV format is: acc_x,acc_y,acc_z,acc_tot
        float acc_x, acc_y, acc_z, acc_tot;
        if (sscanf(line, "%f,%f,%f,%f", &acc_x, &acc_y, &acc_z, &acc_tot) == 4) {
            acceleration_data[data_count].acc_x = acc_x;
            acceleration_data[data_count].acc_y = acc_y;
            acceleration_data[data_count].acc_z = acc_z;
            acceleration_data[data_count].acc_tot = acc_tot;
            data_count++;
        }
    }

    fclose(file);
}

// Function to test the state machine with the acceleration data
void test_state_machine_with_acceleration_data() {
    flight_fsm_t fsm_state = {
        .flight_state = READY, // Start in the READY state
        .memory = {0, 0, 0}    // Initialize memory
    };

    vf32_t gyro_data = {0.0f, 0.0f, 0.0f}; // Assuming you have gyro data
    estimation_output_t state_data = {0.0f, 0.0f, 0.0f}; // State data
    control_settings_t settings = {0}; // Control settings

    // Set up your control settings
    settings.liftoff_acc_threshold = 1.5f; // Example threshold

    // Iterate through the acceleration data
    for (int i = 0; i < data_count; i++) {
        vf32_t acc_data = {acceleration_data[i].acc_x, acceleration_data[i].acc_y, acceleration_data[i].acc_z};

        // Call the flight phase check function
        check_flight_phase(&fsm_state, acc_data, gyro_data, state_data, &settings);

        // Print the current state after processing the acceleration data
        printf("Data Point %d - Acc: (%.2f, %.2f, %.2f) - State: %d\n", 
               i, acc_data.x, acc_data.y, acc_data.z, fsm_state.flight_state);
    }
}

// Main function to run tests
int main(void) {
    printf("Running State Machine Tests...\n");
    read_acceleration_data("test/2020-09-08_NovaPayloader5-flight20.csv");
    test_state_machine_with_acceleration_data();
    printf("All Tests Completed.\n");

    // Main loop (if needed)
    while (1) {
    }

    return 0;
}