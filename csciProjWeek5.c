#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define DISTANCE_THRESHOLD 10.0

// Initialize arrays and variables 
WbDeviceTag left_motor, right_motor; //motor variables
WbDeviceTag distance_sensors[8]; //distance sensor array
WbDeviceTag light_sensors[3]; //light sensor array

double light_values[3]; //array to store values
double distance_values[8]; //array to store values

// Robot initialization
void initialize_robot() {
  wb_robot_init();
  
  //initializing the motors 
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Enabling distance sensors
  char distance_sensor_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < 8; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensor_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP); 
  }

  // Enabling light sensors
  char light_sensor_names[3][4] = {"ls0", "ls1", "ls2"};
  for (int i = 0; i < 3; i++) {
    light_sensors[i] = wb_robot_get_device(light_sensor_names[i]);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }
}

//Function to Move robot forward
void move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

//Function to Stop the robot
void stop_robot() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

//Function to Turn left
void turn_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED/2);
  wb_motor_set_velocity(right_motor, MAX_SPEED/2);
  wb_robot_step(TIME_STEP*15); // adjust the duration to turn 90 degrees
}

// Function to check if it is at a dead-end
int is_dead_end() {
  // Assuming dead-end detection if all front sensors detect obstacles
  return (distance_values[0] > DISTANCE_THRESHOLD && distance_values[1] > DISTANCE_THRESHOLD &&
          distance_values[2] > DISTANCE_THRESHOLD && distance_values[3] > DISTANCE_THRESHOLD);
}

// Measure and return average light intensity at current position
double measure_brightness() {
  double total_light = 0.0;
  for (int i = 0; i < 3; i++) {
    light_values[i] = wb_light_sensor_get_value(light_sensors[i]);
    total_light += light_values[i];
  }
  return total_light / 3.0;
}


/*Main Function*/
int main(int argc, char **argv) {
  initialize_robot();
  
  // Set initial robot position (as mentioned in your requirements)
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("e-puck"), "translation"), (const double[]){-1.4, 0.35, 0});

  double dead_end_brightness[10]; // Array to store brightness of dead-ends
  int dead_end_count = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    // Get distance sensor readings
    for (int i = 0; i < 8; i++) {
      distance_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);
    }

    // Check if robot is in a dead-end
    if (is_dead_end()) {
      stop_robot();
      double current_brightness = measure_brightness();
      dead_end_brightness[dead_end_count++] = current_brightness;
      turn_left(); // Turn left after detecting a dead-end to continue exploration
    } else {
      move_forward();
    }
  }

  // Determine the brightest dead-end 
  double max_brightness = dead_end_brightness[0];
  int brightest_dead_end = 0;
  for (int i = 1; i < dead_end_count; i++) {
    if (dead_end_brightness[i] > max_brightness) {
      max_brightness = dead_end_brightness[i];
      brightest_dead_end = i;
    }
  }

  // Navigate to the brightest dead-end (implement the logic based on the recorded paths if necessary)

  wb_robot_cleanup();
  return 0;
}