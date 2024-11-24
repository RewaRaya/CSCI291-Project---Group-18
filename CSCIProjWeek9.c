#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <math.h>


//Declare Constants
#define TIME_STEP 64
#define MAX_SPEED 0.5*6.28   
#define TURN_DURATION 1000
#define MIN_OBSTACLE_DISTANCE 73  // Threshold for detecting obstacles
#define MAX_DEAD_ENDS 20 // Track dead-ends and light intensities dynamically
//Maximum number of dead-ends that can be tracked (2 cycles)

// Motor and sensor handles declaration
WbDeviceTag left_motor, right_motor;
WbDeviceTag distance_sensors[8];
WbDeviceTag light_sensors[8];
WbDeviceTag gps;

//Initiate Arrays and Variables
double dead_end_light_intensities[MAX_DEAD_ENDS];  // Store light intensity values at each dead-end
int visited_dead_ends = 0;  // Track visited dead-ends 
double max_light_intensity = 0;  // Store the highest light intensity 
int brightest_dead_end_index = -1;  // Index of the brightest dead-end
double dead_end_positions[MAX_DEAD_ENDS][3]; //GPS Positions at Dead-ends
// Global variable for storing GPS data (X, Y, Z)
double current_gps_position[3] = {0.0, 0.0, 0.0};

// enum to State flags for robot
enum RobotState { EXPLORING, BACKTRACKING } robot_state = EXPLORING;

// Function to initialize motor and sensors 
void init_devices() {
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");

  //sensors declaration
  char distance_sensor_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  char light_sensor_names[8][4] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};
  
  // Enable distance and light sensors
  for (int i = 0; i < 8; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensor_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
    
    light_sensors[i] = wb_robot_get_device(light_sensor_names[i]);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
  // Enable GPS and initialize GPS
  gps = wb_robot_get_device("gps");
  if (gps == NULL) {  // Check for an invalid device handle
    printf("Error: GPS device not found!\n");
    return;  // Exit the program if GPS is not found
  } //ignore mismatch data type error
  //gps is WbDeviceTag while NULL is integer
  wb_gps_enable(gps, TIME_STEP);
}

// Move forward function
void move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor,MAX_SPEED);
  wb_robot_step(TIME_STEP);
}

// Stop robot function
void stop_robot() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

// Turn the robot 180 degrees (to backtrack)
void turn_around() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  wb_robot_step(TURN_DURATION);
  
}


//Function for robot to Turn 90 degrees to the right. 
void turn_90_r() {
  // Rotate right by setting left motor to max speed and right motor to negative max speed
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  int duration = TURN_DURATION/2; 
  wb_robot_step(duration); 
  //TURN_DURATION is calibrated for 180 degrees so 180/2=90
}

void turn_90_l() {
  // Rotate left by setting right motor to max speed and left motor to negative max speed
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  int duration = TURN_DURATION/2;
  wb_robot_step(duration);
}

// Check if the robot is at a true dead-end (both front sensors detect obstacles)
bool is_at_dead_end() {
  return wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
         wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE &&
         wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
         wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
         wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
         wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE ;
}

// Function to Measure light intensity and update maximum if current is greater
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

/*GPS FUNCTIONS*/

// Function to store GPS coordinates
void store_gps_coordinates() {
  //GPS values obtained through pointer
  const double *gps_values = wb_gps_get_values(gps);
  
  //Copy the values obtained from function into global variable
  current_gps_position[0] = gps_values[0]; // X 
  current_gps_position[1] = gps_values[1]; // Y 
  current_gps_position[2] = gps_values[2]; // Z 

  // Assign the position to the dead_end_positions array
  dead_end_positions[visited_dead_ends][0] = current_gps_position[0];
  dead_end_positions[visited_dead_ends][1] = current_gps_position[1];
  dead_end_positions[visited_dead_ends][2] = current_gps_position[2];

  // Print the position for debugging
   printf("Dead-end %d position: X = %f, Y = %f, Z = %f\n", 
         visited_dead_ends + 1, current_gps_position[0], current_gps_position[1], current_gps_position[2]);
}


// Function to navigate back to a specific GPS location
void backtrack_to_position(int dead_end_index) {
  printf("Backtracking to brightest dead-end %d\n", dead_end_index + 1);

  // Get the target position for the dead-end
  double target_position[3];
  target_position[0] = dead_end_positions[dead_end_index][0]; //X
  target_position[1] = dead_end_positions[dead_end_index][1]; //Y
  target_position[2] = dead_end_positions[dead_end_index][2]; //Z

  // Start backtracking
  while (wb_robot_step(TIME_STEP) != -1) {
    //Obtain current GPS values with pointer
    const double *gps_values = wb_gps_get_values(gps);
    //Storing the obtained values into the global array:
    current_gps_position[0] = gps_values[0]; // X
    current_gps_position[1] = gps_values[1]; // Y
    current_gps_position[2] = gps_values[2]; // Z

    // Calculate the distance to the target position (considering X, Y, and Z coordinates)
    double dx = target_position[0] - current_gps_position[0];
    double dy = target_position[1] - current_gps_position[1];
    double dz = target_position[2] - current_gps_position[2];
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    //sqrt() function requires the math.h header file

    // If the robot is close enough to the target, stop
    if (distance < 0.01) {  // Threshold for accuracy
      printf("Reached target position.\n");
      stop_robot();
      break;
    }
    
    // Otherwise, move forward
    move_forward();
  }
}

/*MAIN FUNCTION*/

int main() {
  wb_robot_init();
  init_devices();
  
  // Initial delay to allow the robot to leave the starting point
  wb_robot_step(500);

  while (wb_robot_step(TIME_STEP) != -1) {
    // If robot has visited all dead-ends, break the loop
    if (visited_dead_ends >= MAX_DEAD_ENDS) {
      break; //breaks loop when maximum dead-ends number has reached.
    }

    // If robot is at a dead-end and is exploring
    if (is_at_dead_end() && robot_state == EXPLORING) {
      //Dead-end is detected
      // Measure light intensity and store GPS coordinates
      measure_light_intensity();
      store_gps_coordinates();
      visited_dead_ends++; //update the number of dead-ends visited
      
      // Perform a 180-degree turn to exit dead-end
      turn_around();
      // Switch to backtracking mode after reaching a dead-end
      robot_state = BACKTRACKING;
      wb_robot_step(500); // Short delay for smooth transition 
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
        // Continue moving forward until a dead-end is found    
     int x;
     int n=0;
     
      while(robot_state == EXPLORING && n == 0) {
        /*Case 1*/
        if(wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE ){
          x=1;
        }
        
        /*Case 2*/
        else if(wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE){
          x=2;
        }
        
        /*Case 3*/
        else if(wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE){
          x=3;
        }
        
        /*Case 4*/
        else if(wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE){
          x=4;
        }
        
        /*Case 5*/
        else if(wb_distance_sensor_get_value(distance_sensors[3]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE){
          x=5;
        }
        
        /*Case 6*/
        else if(wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[2]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[3]) < MIN_OBSTACLE_DISTANCE){
          x=6;
        }
        
        /*Case 7*/
        else if(wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[4]) < MIN_OBSTACLE_DISTANCE){
          x=7;
        } 
      
        /*Case 9*/
        else if(wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[0]) < MIN_OBSTACLE_DISTANCE &&
          wb_distance_sensor_get_value(distance_sensors[7]) < MIN_OBSTACLE_DISTANCE ){
          x=9;
        }
        
        /*Case 8*/
        else {
          x=8;}
      
        switch(x){
          case 1:
            //Turn right when one wall is in front and no on side or behind
            turn_90_r();
            move_forward();
            n = 1;
            break;
            
          case 2:
            //Continue going straight at intersections with left turn
            move_forward();
            n = 1;
            break;
            
          case 3:
            //Turn Right at intersection with right turn
            turn_90_r();
            move_forward();
            n = 1;
            break;
            
          case 4:
            //At dead-end
            is_at_dead_end();
            n = 1;
            break;
            
          case 5:
            //Continue straight after turning due to dead-end found 
            move_forward();
            n = 1;
            break;
            
          case 6:
            //Turning right at corner with ps5
            turn_90_r();
            move_forward();
            n = 1;
            break;
            
          case 7:
            //Turning left at corner with ps2
            turn_90_l();
            move_forward();
            n = 1;
            break;
          
          case 8:
            //At point with 4 exits
            turn_90_l();
            move_forward();
            n = 1;
            break;
          
          case 9:
            //When in  corridor continue moving straight
            move_forward();
            n = 1;
            break;
      
        }//switch closing bracket
        
        while(n == 1){
          move_forward();
          if (wb_distance_sensor_get_value(distance_sensors[0]) > MIN_OBSTACLE_DISTANCE ||
            wb_distance_sensor_get_value(distance_sensors[1]) > MIN_OBSTACLE_DISTANCE ||
            wb_distance_sensor_get_value(distance_sensors[3]) > MIN_OBSTACLE_DISTANCE ||
            wb_distance_sensor_get_value(distance_sensors[4]) > MIN_OBSTACLE_DISTANCE ||
            wb_distance_sensor_get_value(distance_sensors[6]) > MIN_OBSTACLE_DISTANCE ||
            wb_distance_sensor_get_value(distance_sensors[7]) > MIN_OBSTACLE_DISTANCE ||
            wb_distance_sensor_get_value(distance_sensors[2]) > MIN_OBSTACLE_DISTANCE ||
            wb_distance_sensor_get_value(distance_sensors[5]) > MIN_OBSTACLE_DISTANCE){
        
              n = 0;
          }        
       
        }//inner while
      }//while loop closing bracket      
    }
  }
  
  // Report the brightest dead-end light intensity
  printf("Maximum light intensity found at a dead-end: %f\n", max_light_intensity);
  
  // Backtrack to the dead-end with the highest light intensity
  backtrack_to_brightest_dead_end();
  
  wb_robot_cleanup();
  return 0;
}