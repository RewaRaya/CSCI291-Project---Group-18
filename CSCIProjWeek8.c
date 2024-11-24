#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED (0.5 * 6.28)    // Define maximum speed as 0.5 * 6.28
#define TURN_DURATION 800
#define MIN_OBSTACLE_DISTANCE 200  // Threshold for detecting obstacles

// Motor and sensor handles
WbDeviceTag left_motor, right_motor;
WbDeviceTag distance_sensors[8];
WbDeviceTag light_sensors[8];

// Track dead-ends and light intensities dynamically
#define MAX_DEAD_ENDS 20  // Maximum number of dead-ends that can be tracked
double dead_end_light_intensities[MAX_DEAD_ENDS];  // Store light intensity values at each dead-end
int visited_dead_ends = 0;  // Track visited dead-ends
double max_light_intensity = 0;  // Store the highest light intensity
int brightest_dead_end_index = -1;  // Index of the brightest dead-end

// State flags
enum RobotState { EXPLORING, BACKTRACKING } robot_state = EXPLORING;

// Initialize motor and sensors
void init_devices() {
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  const char *distance_sensor_names[8] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  
  const char *light_sensor_names[8] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};
  
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

// Turn the robot 180 degrees (to backtrack)
void turn_around() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  wb_robot_step(TURN_DURATION);
  stop_robot();
}

// Check if the robot is at a true dead-end (both front sensors detect obstacles)
bool is_at_dead_end() {
  return wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
         wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE;
}

// Measure light intensity and update maximum if current is greater
void measure_light_intensity() {
  double total_light = 0;
  unsigned char i;
  for (i = 0; i < 8; i++) {
    total_light += wb_light_sensor_get_value(light_sensors[i]);
  }

  // Store the light intensity at the current dead-end
  dead_end_light_intensities[visited_dead_ends] = total_light;

  // Update max light intensity and track the dead-end with the highest intensity
  if (total_light > max_light_intensity) {
    max_light_intensity = total_light;
    brightest_dead_end_index = visited_dead_ends;
  }

  printf("Dead-end %d: Light intensity = %f\n", visited_dead_ends, total_light);
}

void backtrack_to_brightest_dead_end() {
  // Implement backtracking logic to the brightest dead-end
  // For simplicity, assuming the robot can reverse its path (you could add more sophisticated logic here)
  printf("Backtracking to brightest dead-end %d\n", brightest_dead_end_index);
  turn_around(); // Backtrack by turning around
  // After backtracking, stop and perform any additional logic to reach the brightest dead-end
}

int main() {
  wb_robot_init();
  init_devices();
  
  // Initial delay to allow the robot to leave the starting point
  wb_robot_step(500);

  while (wb_robot_step(TIME_STEP) != -1) {
    // If robot has visited all dead-ends, break the loop
    if (visited_dead_ends >= MAX_DEAD_ENDS) {
      break;
    }

    // If robot is at a dead-end and is exploring
    if (is_at_dead_end() && robot_state == EXPLORING) {
      // Measure light intensity when we reach a new dead-end
      measure_light_intensity();
      visited_dead_ends++;
      
      // Switch to backtracking mode after reaching a dead-end
      robot_state = BACKTRACKING;
      
      // Perform a 180-degree turn to backtrack
      turn_around();
      wb_robot_step(500); // Short delay to leave the dead-end
    } 
    // If robot is backtracking
    else if (robot_state == BACKTRACKING) {
      // Move forward after backtracking (to continue exploring)
      move_forward();
      
      // Once it has backtracked and moved forward again, switch back to exploring
      robot_state = EXPLORING;
    }
    // If robot is exploring and not at a dead-end
    else if (robot_state == EXPLORING) {
      /*Case 1*/ 
       if ((wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE ||
          wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE )&&
          (wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE ||
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE) &&
          (wb_distance_sensor_get_value(distance_sensors[2]) < MIN_OBSTACLE_DISTANCE ||
          wb_distance_sensor_get_value(distance_sensors[5]) < MIN_OBSTACLE_DISTANCE) &&
          (wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE ||
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE)){ 
          
          
          //turn right 90 degrees
          turn_90_r();
          move_forward();
          continue;
      }
      
      /*Case 2*/
      else if (wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE){
          
          //Continue moving forward
          move_forward();
      }
      
      /*Case 3*/
      else if (wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[2]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE){
          
          //turn right 90 degrees
          turn_90_r();
          move_forward();
          continue;
      }
      
      /*Case 4*/
      else if (wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE){
          
          //move forward to get outside deadend
          move_forward();
          continue;
      }
      
      /*Case 5*/
      else if (wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[2]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE
          ){
          
          //turn right 90 degrees
          turn_90_r();
          move_forward();
          continue;
      }
      
      /*Case 6*/
      else if (wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE ){
          
          //turn left 90 degrees
          turn_90_l();
          move_forward();
          continue;
      }
      
      /*Case 7*/
      else if (wb_distance_sensor_get_value(distance_sensors[0]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[2]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE ) {
          
          move_forward();
          //turn left 90 degrees
          turn_90_l();
          move_forward();
          continue;
          
      }
          
      /*Case 8*/
      else if (wb_distance_sensor_get_value(distance_sensors[0]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[1]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[6]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE){
            
            move_forward();
        }
    }
  }
  
  // Report the brightest dead-end light intensity
  printf("Maximum light intensity found at a dead-end: %f\n", max_light_intensity);
  
  // Backtrack to the dead-end with the highest light intensity
  backtrack_to_brightest_dead_end();
  
  wb_robot_cleanup();
  return 0;
}