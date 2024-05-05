import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.Mouse;
import com.cyberbotics.webots.controller.MouseState;
import java.lang.Math;
import java.io.IOException;

public class Drone3Controller extends Robot {

    final int DRONE_NUMBER = 3;
    final int[] base_values = {-1, -1, 1};
    
    final double[][] box_coords = {{3,3,1},{3,-3,1},{-3,-3,1},{-3,3,1}};
  
    final int TIME_STEP = 32; // Simulation time step in milliseconds
    

   Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller, cameraRollMotor, cameraPitchMotor;
   Motor motors[];
   LED frontLeftLED, frontRightLED;
   InertialUnit imu;
   GPS gps;
   Gyro gyro;
   Keyboard keyboard;
   Mouse mouse;
    
  // Constants, empirically found.
  final double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
  final double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  final double k_vertical_p = 3.0;        // P constant of the vertical PID.
  final double k_roll_p = 50.0;           // P constant of the roll PID.
  final double k_pitch_p = 30.0;          // P constant of the pitch PID.
// Variables.
  double target_altitude = 1.0;  // The target altitude. Can be changed by the user.
  double target_x = base_values[0];
  double target_y = -1*base_values[1];
  
  double target_yawn = -1.0;
  
  double kp_x = 5;
  double kd_x = 5;
  
  double kp_y = 5;
  double kd_y = 5;
  
  


  public Drone3Controller() {
    //super();
    //initializeDevices();
 } 
  
  
  public static double CLAMP(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
  }
  
  public void displayWelcomeMessage() {
    // Wait one second
    double previousTime = 0.0;
    while (step(TIME_STEP) != -1) {
      if (getTime() - previousTime > 3.0) { break; }
    }
  }
  
  public void blinkLEDS() {
    final double timeInSecs = getTime(); // in seconds.
    final boolean ledState = ((int) timeInSecs) % 2 == 0;
  
    frontLeftLED.set(ledState ? 1 : 0);
    frontRightLED.set(ledState ? 0 : 1);
  }
  
  
  public void stabilizeCamera(double rollVelocity, double pitchVelocity) {
    cameraRollMotor.setPosition(-0.115 * rollVelocity);
    cameraPitchMotor.setPosition(-0.1 * pitchVelocity);
  }

  // Main control loop
  public void run() {

    imu = getInertialUnit("inertial unit");
    imu.enable(TIME_STEP);
    gps = getGPS("gps");
    gps.enable(TIME_STEP);
    gyro = getGyro("gyro");
    gyro.enable(TIME_STEP);
    keyboard = new Keyboard();
    keyboard.enable(1);
    mouse = new Mouse();
    mouse.enable(TIME_STEP);
    mouse.enable3dPosition();
    
    frontLeftLED = new LED("front left led");
    frontRightLED = new LED("front right led");
    frontRightPropeller = getMotor("front right propeller");
    frontLeftPropeller = getMotor("front left propeller");
    rearRightPropeller = getMotor("rear right propeller");
    rearLeftPropeller = getMotor("rear left propeller");
    motors = new Motor[] {
      frontRightPropeller,
      frontLeftPropeller,
      rearRightPropeller,
      rearLeftPropeller
    };
    
    for (Motor motor : motors) {
      motor.setPosition(Double.POSITIVE_INFINITY);
      motor.setVelocity(1); // 10% of velocity to start
    }
    
    cameraRollMotor = getMotor("camera roll");
    cameraPitchMotor = getMotor("camera pitch");

    displayWelcomeMessage();
    
    //work in progress
    
    double error_x = 0;
    double last_error_x = 0;
    double d_error_x = 0;
    
    double error_y = 0;
    double last_error_y = 0;
    double d_error_y = 0;
    
    
    double input_x = 0;
    double x_iterator = 0.05;
    int x_it_count = 0;
    
    double input_y = 0;
    double y_iterator = 0.05;
    int y_it_count = 0;
    
    double input_alt = 0;
    int alt_it_count = 0;
    
    
    while (step(TIME_STEP) != -1) {
    
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;

      final double roll = imu.getRollPitchYaw()[0];
      final double pitch = imu.getRollPitchYaw()[1];
      final double posX = gps.getValues()[0];
      final double posY = gps.getValues()[1];
      final double altitude = gps.getValues()[2];
      final double roll_acceleration = gyro.getValues()[0];
      final double pitch_acceleration = gyro.getValues()[1];
      
      
      MouseState mouseState = mouse.getState();
      double mouseX = mouseState.getX();
      double mouseY = mouseState.getY();
      double mouseZ = mouseState.getZ();
      if (mouseState.getRight() && mouseState.getLeft()){
        System.out.printf("MouseX = %f.2 | MouseY = %f.2, | MouseZ = %f.2 | Clicked = %s \n", mouseX, mouseY, mouseZ, mouseState.getRight());
        
        input_x = mouseX;
        x_it_count = (int)((input_x - target_x)/x_iterator);           
        
        input_y = mouseY;
        y_it_count = (int)((input_y + target_y)/y_iterator);

        //input_alt = mouseZ;
        //alt_it_count = (int)((input_alt - target_altitude)/0.05);

      }
      

      //Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();

      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      stabilizeCamera(roll_acceleration, pitch_acceleration);
      
      int key = keyboard.getKey();
      //System.out.println(key);
    while (key > 0) {
      switch (key) {
        case Keyboard.UP:
          //pitch_disturbance = -2.0;
          target_x += x_iterator;
          System.out.println(target_x);
          break;
        case Keyboard.DOWN:
          //pitch_disturbance = 2.0;
          target_x -= x_iterator;
          break;
        case Keyboard.RIGHT:
          //roll_disturbance = -1.0;
          target_y += y_iterator;
          break;
        case Keyboard.LEFT:
          //roll_disturbance = 1.0;
          target_y -= y_iterator;
          break;
        case (Keyboard.SHIFT + Keyboard.RIGHT):
          yaw_disturbance = -1.3;
          break;
        case (Keyboard.SHIFT + Keyboard.LEFT):
          yaw_disturbance = 1.3;
          break;
        case (Keyboard.SHIFT + Keyboard.UP):
          target_altitude += 0.05;
          //System.out.printf("target altitude: %f [m]\n", target_altitude);
          break;
        case (Keyboard.SHIFT + Keyboard.DOWN):
          target_altitude -= 0.05;
          System.out.printf("target altitude: %f [m]\n", target_altitude);
          break;
          
         case (Keyboard.SHIFT+'T'):
           input_x = 5;
           x_it_count = (int)((input_x - target_x)/x_iterator);
           
           input_y = 5;
           y_it_count = (int)((input_y + target_y)/y_iterator);
           
           input_alt = 5;
           alt_it_count = (int)((input_alt - target_altitude)/0.05);
           
           System.out.println("Test1");
           break;
           
         case (Keyboard.SHIFT+'Y'):
           input_x = -7;
           x_it_count = (int)((input_x - target_x)/x_iterator);
           
           input_y = -7;
           y_it_count = (int)((input_y + target_y)/y_iterator);
           
           input_alt = 15;
           alt_it_count = (int)((input_alt - target_altitude)/0.05);
           
           System.out.println("Test2");
           break;
           
          case (Keyboard.SHIFT+'U'):
           input_x = 32;
           x_it_count = (int)((input_x - target_x)/x_iterator);
           
           input_y = -40;
           y_it_count = (int)((input_y + target_y)/y_iterator);
           
           input_alt = 50;
           alt_it_count = (int)((input_alt - target_altitude)/0.05);
           
           System.out.println("Test Extreme");
           break;
           
          case (Keyboard.SHIFT+'X'):
           input_x = box_coords[DRONE_NUMBER-1][0];
           x_it_count = (int)((input_x - target_x)/x_iterator);
           
           input_y = -1*box_coords[DRONE_NUMBER-1][1];
           y_it_count = (int)((input_y + target_y)/y_iterator);
           
           input_alt = box_coords[DRONE_NUMBER-1][2];
           alt_it_count = (int)((input_alt - target_altitude)/0.05);
           
           System.out.println("Test Box");
           break;
           
          case (Keyboard.SHIFT+'B'):
           input_x = base_values[0];
           x_it_count = (int)((input_x - target_x)/x_iterator);
           
           input_y = -1*base_values[1];
           y_it_count = (int)((input_y + target_y)/y_iterator);
           
           input_alt = base_values[2];
           alt_it_count = (int)((input_alt - target_altitude)/0.05);
           
           System.out.println("Back To Base");
           break;
      }
      key = keyboard.getKey();
    }
    
    
    /*
      if (input_x < target_x){
         target_x -= 0.05;
      } else if (input_x > target_x){
        target_x += 0.05;
      } else {
        input_x = target_x;
      }
      
      */
      if (x_it_count < 0){
         target_x -= x_iterator;
         x_it_count++;
      } else if (x_it_count > 0){
        target_x += x_iterator;
        x_it_count--;
      } 
      //System.out.println(posX);
      
      if (y_it_count < 0){
         target_y += y_iterator;
         y_it_count++;
      } else if (y_it_count > 0){
        target_y -= y_iterator;
        y_it_count--;
      } 
      
      if (alt_it_count < 0){
         target_altitude -= 0.05;
         alt_it_count++;
      } else if (alt_it_count > 0){
        target_altitude += 0.05;
        alt_it_count--;
      } 
      
      if(x_it_count == 0 && y_it_count == 0 && alt_it_count == 0){
        //System.out.println("Drone made it to target point:");
        //System.out.printf(" TARGET -- X: %f | Y: %f | Altitude: %f \n", target_x, target_y, target_altitude);
        //System.out.printf("CURRENT -- X: %f | Y: %f | Altitude: %f \n", posX, posY, altitude);
      }
      
      
      
      //System.out.println("X: "+posX+" | Y: "+posY+" | Altitude: "+altitude);
      //System.out.printf("X: %f | Y: %f | Altitude: %f \n", posX, posY, altitude);

      // Compute the roll, pitch, yaw and vertical inputs.
      
      error_y = CLAMP(target_y + posY, -1, 1);
      d_error_y = error_y - last_error_y;
      last_error_y = error_y;
      double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + 2*roll_acceleration + roll_disturbance;
      roll_input = roll_input +(kp_y * error_y) + (kd_y * d_error_y);
      
      
      
      
      
      error_x = CLAMP(target_x - posX, -1, 1);
      d_error_x = error_x - last_error_x;
      last_error_x = error_x;
      double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_acceleration + pitch_disturbance;
      pitch_input = pitch_input + (kp_x * error_x) + (kd_x * d_error_x);

      final double yaw_input = yaw_disturbance;
      final double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
      final double vertical_input = k_vertical_p * Math.pow(clamped_difference_altitude, 3.0);
      
      // Actuate the motors taking into consideration all the computed inputs.
       // Actuate the motors taking into consideration all the computed inputs.
       final double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
       final double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
       final double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
       final double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
      
      frontLeftPropeller.setVelocity(front_left_motor_input);
      frontRightPropeller.setVelocity(-front_right_motor_input);
      rearLeftPropeller.setVelocity(-rear_left_motor_input);
      rearRightPropeller.setVelocity(rear_right_motor_input);
    }
  }
  
  public static void main(String[] args) {
    Drone3Controller droneController = new Drone3Controller();
    droneController.run();
  }
}