#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28
#define PAUSE_DURATION 60.0

// Motor and sensor handles
WbDeviceTag left_motor, right_motor;
WbDeviceTag distance_sensors[8];
WbDeviceTag light_sensors[8];

// Track dead-ends and light intensities dynamically
#define MAX_DEAD_ENDS 22  // Maximum number of dead-ends that can be tracked


double dead_end_light_intensities[MAX_DEAD_ENDS];  // Store light intensity values at each dead-end
int visited_dead_ends = 0;  // Track visited dead-ends
double max_light_intensity = 0;  // Store the highest light intensity
int brightest_dead_end_index = -1;  // Index of the brightest dead-end

// State flags
enum RobotState { EXPLORING, BACKTRACKING } robot_state = EXPLORING;

// Initialize motor and sensors
void init_devices() {
  //initiating robot motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  /*if motors not found case:*/
  if (left_motor == -1 || right_motor == -1) {
    printf("Error: Motors not found!\n");
    return;
  }
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
  //initiating sensors
  const char *distance_sensor_names[8] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  const char *light_sensor_names[8] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};
  for (int i = 0; i < 8; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensor_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
    
    light_sensors[i] = wb_robot_get_device(light_sensor_names[i]);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }
}

// Move forward
void move_forward() {
  wb_motor_set_velocity(left_motor, 0.5*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.5*MAX_SPEED);
}

// Stop robot
void stop_robot() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}



// Check if the robot is at a true dead-end (both front sensors detect obstacles)
bool is_at_dead_end() {
  double front_wall = (wb_distance_sensor_get_value(distance_sensors[0]) > 90.0 &&
                       wb_distance_sensor_get_value(distance_sensors[7]) > 90.0);
  double side_walls = (wb_distance_sensor_get_value(distance_sensors[5]) > 90.0 &&
                       wb_distance_sensor_get_value(distance_sensors[1]) > 70.0 &&
                       wb_distance_sensor_get_value(distance_sensors[2]) < 100.0);
  double rear_walls = (wb_distance_sensor_get_value(distance_sensors[4]) > 70.0 &&
                       wb_distance_sensor_get_value(distance_sensors[3]) > 70.0);
  return (front_wall && side_walls && rear_walls);
}

// Measure light intensity and update maximum if current is greater
void measure_light_intensity() {
  double light = 0; //variaible to hold readings
  double light_max = 0; //variable to hold max light
  for (int i = 0; i < 8; i++) {
    light = wb_light_sensor_get_value(light_sensors[i]);
    if(light > light_max){
      light_max = light; //update the max. light read at dead-end i.  
    }
  }
  
  // Store the light intensity at the current dead-end
  dead_end_light_intensities[visited_dead_ends] = light_max;
  
  // Update max light intensity and track the dead-end with the highest intensity
  if (light_max > max_light_intensity) {
    max_light_intensity = light_max;
    brightest_dead_end_index = visited_dead_ends;
  }

  printf("Dead-end %d: Light intensity = %f\n", visited_dead_ends+1, light_max);
}

/*Function for left-hand algorithm*/
void follow_wall(){
  //declaring variables  
  double front_wall = wb_distance_sensor_get_value(distance_sensors[0]) > 80.0 ||
                      wb_distance_sensor_get_value(distance_sensors[7]) > 80.0;
  double left_wall  = wb_distance_sensor_get_value(distance_sensors[5]) > 80.0;
  double right_wall = wb_distance_sensor_get_value(distance_sensors[2]) > 80.0;          
     
  /*Obstacle infront of robot*/
  if(front_wall) {
    if (left_wall && right_wall){
      //moving backwards
      wb_motor_set_velocity(left_motor, -0.5*MAX_SPEED);
      wb_motor_set_velocity(right_motor, -0.5*MAX_SPEED);
    }
          
    else if(left_wall){
      //turning right
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, 0.1*MAX_SPEED);
    }
          
    else if(right_wall){
      //turning left
      wb_motor_set_velocity(left_motor, 0.1*MAX_SPEED);
      wb_motor_set_velocity(right_motor, MAX_SPEED);
    }
          
    /*case of nothing front of robot*/
    else {
      //move back and turn left
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, -0.1*MAX_SPEED);
    } 
  }
        
  else {
    if(left_wall && !right_wall){
      //Turn right
      wb_motor_set_velocity(left_motor, 0.1*MAX_SPEED);
      wb_motor_set_velocity(right_motor, MAX_SPEED);
    }
          
    else if(!left_wall && !right_wall){
      //Move forward slowely
      wb_motor_set_velocity(left_motor, 0.5*MAX_SPEED);
      wb_motor_set_velocity(right_motor, 0.5*MAX_SPEED);
    }
          
    else if(!left_wall && right_wall){
      //Turn left
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, 0.1*MAX_SPEED);
    }
  }
}

void turn_around() {
  wb_motor_set_velocity(left_motor,  -0.5*MAX_SPEED);
  wb_motor_set_velocity(right_motor, -0.5*MAX_SPEED);
}

bool measure_light_back(){
  double light_back = 0; //variaible to hold readings
  double light_maxb = 0; //variable to hold max light
  int dead_end_back = 0;
  
  //enabling second light sensor
  WbDeviceTag back_light_sensors[8];
  const char *back_light_sensor_names[8] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};
  for (int i = 0; i < 8; i++){
    back_light_sensors[i] = wb_robot_get_device(back_light_sensor_names[i]);
    wb_light_sensor_enable(back_light_sensors[i], TIME_STEP);
  }

  
  for (int i = 0; i < 8; i++) {
    light_back = wb_light_sensor_get_value(back_light_sensors[i]);
    if(light_back > light_maxb){
      light_maxb = light_back; //update the max. light read at dead-end i.  
    }
  }
  printf("Dead-end %d: Light intensity = %f\n", dead_end_back+1, light_maxb);
  
  //Use tolerance to decrease noise
  double tolerance_maxLight = max_light_intensity*0.01464;
  
  // Update max light intensity and track the dead-end with the highest intensity
  if (light_maxb >= max_light_intensity + tolerance_maxLight) {
    return true;
  }
  //otherwise:
    return false;
}

/*Function for backtracking*/
void follow_mazeBack(){
  int dead_end_back = 0; //counter to track the dead-ends revisited
  
  robot_state = EXPLORING;
  while (wb_robot_step(TIME_STEP) != -1) {
    // If robot is at a dead-end and is exploring
    if (is_at_dead_end() && robot_state == EXPLORING) {
      printf("Reached dead-end %d during backtracking.\n", dead_end_back+1);
      // Increment backtrack count after visiting a dead-end
      
      dead_end_back++;
      
      // Measure light intensity when we reach a new dead-end
      //if same light intensity as maximum:
      if (measure_light_back() == true){
        printf("Brightest dead-end found! Stopping at this location.\n");
        stop_robot();
        break;
      }
      //case of false
      else {
        printf("Dead-end light intensity does not match. Turning around.\n");
        follow_wall();
      }
    }
    else
      follow_wall();
  }
}

int main() {
  wb_robot_init();
  init_devices();
  
  // Initial delay to allow the robot to leave the starting point
  wb_robot_step(500);

  while (wb_robot_step(TIME_STEP) != -1) {
    // If robot has visited all dead-ends, break the loop
    if (visited_dead_ends >= MAX_DEAD_ENDS) {
      printf("All dead-ends visited. Starting backtracking.\n");
      robot_state = BACKTRACKING;
    }

    // If robot is at a dead-end and is exploring
    if (is_at_dead_end() && robot_state == EXPLORING) {
      // Measure light intensity when we reach a new dead-end
      measure_light_intensity();
      visited_dead_ends++;
     
      wb_robot_step(500); // Short delay to leave the dead-end
    } 
    
    // If robot is backtracking
    else if (robot_state == BACKTRACKING) {
      // Move forward after backtracking (to continue exploring)
      follow_mazeBack();
      break;
    }
    // If robot is exploring and not at a dead-end
    else if (robot_state == EXPLORING) {
        follow_wall();
    }
  }
  
  // Report the brightest dead-end light intensity
  printf("Maximum light intensity found at a dead-end: %f\n", max_light_intensity);
  
  wb_robot_cleanup();
  return 0;
}