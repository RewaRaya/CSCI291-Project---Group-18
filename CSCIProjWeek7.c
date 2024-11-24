#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>

#define TIME_STEP 64
#define NUM_DEAD_ENDS 10
#define MAX_SPEED (0.5 * 6.28)    // Define maximum speed as 0.5 * 6.28
#define TURN_DURATION 800
#define MIN_OBSTACLE_DISTANCE 200  // Increased threshold for detecting dead-ends

// Motor and sensor handles
WbDeviceTag left_motor, right_motor;
WbDeviceTag distance_sensors[8];
WbDeviceTag light_sensors[8];

// Track visited dead-ends and highest light intensity
unsigned char visited_dead_ends = 0;
double max_light_intensity = 0;

// Initialize motor and sensors
void init_devices() {
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  const char *distance_sensor_names[8] = {
    "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"
  };
  
  const char *light_sensor_names[8] = {
    "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"
  };
  
  // Enable distance and light sensors
  unsigned char i;
  for (i = 0; i < 8; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensor_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
    
    light_sensors[i] = wb_robot_get_device(light_sensor_names[i]);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
}

// Move forward
void move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

// Stop robot
void stop_robot() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

// Turn the robot 180 degrees to backtrack
void turn_around() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  wb_robot_step(TURN_DURATION);
  stop_robot();
}

// Check if the robot is at a dead-end (front sensors detect obstacles)
bool is_at_dead_end() {
  return wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
         wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE;
}

// Simple obstacle avoidance with forward bias
void obstacle_avoidance() {
  double left_obstacle = wb_distance_sensor_get_value(distance_sensors[5]);
  double right_obstacle = wb_distance_sensor_get_value(distance_sensors[2]);

  if (left_obstacle > 100) {
    // Slight right turn to avoid obstacle on the left
    wb_motor_set_velocity(left_motor, MAX_SPEED * 0.7);
    wb_motor_set_velocity(right_motor, MAX_SPEED * 0.3);
  } else if (right_obstacle > 100) {
    // Slight left turn to avoid obstacle on the right
    wb_motor_set_velocity(left_motor, MAX_SPEED * 0.3);
    wb_motor_set_velocity(right_motor, MAX_SPEED * 0.7);
  } else {
    move_forward();  // Move forward if no side obstacles are detected
  }
}

// Measure light intensity and update maximum if current is greater
void measure_light_intensity() {
  double total_light = 0;
  unsigned char i;
  for (i = 0; i < 8; i++) {
    total_light += wb_light_sensor_get_value(light_sensors[i]);
  }
  if (total_light > max_light_intensity) {
    max_light_intensity = total_light;
  }
}

int main() {
  wb_robot_init();
  init_devices();
  
  // Initial delay to allow the robot to leave the starting point
  wb_robot_step(500);
  
  while (wb_robot_step(TIME_STEP) != -1) {
    if (visited_dead_ends >= NUM_DEAD_ENDS) break;

    // Check if at a dead-end
    if (is_at_dead_end()) {
      stop_robot();
      measure_light_intensity();
      visited_dead_ends++;
      printf("Dead-end %d: Light intensity %f\n", visited_dead_ends, max_light_intensity);
      
      // Turn around to leave the dead-end
      turn_around();
      wb_robot_step(500); // Short delay to leave the dead-end
    } else {
      // Navigate and avoid obstacles
      obstacle_avoidance();
    }
  }
  
  // Report the brightest dead-end light intensity
  printf("Maximum light intensity found at a dead-end: %f\n", max_light_intensity);
  
  wb_robot_cleanup();
  return 0;
}