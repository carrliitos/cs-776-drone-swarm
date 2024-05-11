import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.Keyboard;
import java.lang.Math;
import java.io.IOException;

public class DroneControllerTest extends Robot {
  
    final int TIME_STEP = 32; // Simulation time step in milliseconds
    

   Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller, cameraRollMotor, cameraPitchMotor;
   Motor motors[];
   LED frontLeftLED, frontRightLED;
   InertialUnit imu;
   GPS gps;
   Gyro gyro;
   Keyboard keyboard;
    
  // Constants, empirically found.
  final double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
  final double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  final double k_vertical_p = 3.0;        // P constant of the vertical PID.
  final double k_roll_p = 50.0;           // P constant of the roll PID.
  final double k_pitch_p = 30.0;          // P constant of the pitch PID.
// Variables.
  double target_altitude = 1.0;  // The target altitude. Can be changed by the user.

  public DroneControllerTest() {
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
    
    while (step(TIME_STEP) != -1) {

      final double roll = imu.getRollPitchYaw()[0];
      final double pitch = imu.getRollPitchYaw()[1];
      final double altitude = gps.getValues()[2];
      final double roll_acceleration = gyro.getValues()[0];
      final double pitch_acceleration = gyro.getValues()[1];
      
      double roll_disturbance = 0.0;
      double pitch_disturbance = 0.0;
      double yaw_disturbance = 0.0;

      //Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();

      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      stabilizeCamera(roll_acceleration, pitch_acceleration);
      
      int key = keyboard.getKey();
      //System.out.println(key);
    while (key > 0) {
      switch (key) {
        case Keyboard.UP:
          pitch_disturbance = -2.0;
          break;
        case Keyboard.DOWN:
          pitch_disturbance = 2.0;
          break;
        case Keyboard.RIGHT:
          yaw_disturbance = -1.3;
          break;
        case Keyboard.LEFT:
          yaw_disturbance = 1.3;
          break;
        case (Keyboard.SHIFT + Keyboard.RIGHT):
          roll_disturbance = -1.0;
          break;
        case (Keyboard.SHIFT + Keyboard.LEFT):
          roll_disturbance = 1.0;
          break;
        case (Keyboard.SHIFT + Keyboard.UP):
          target_altitude += 0.05;
          System.out.printf("target altitude: %.2f [m]%n", targetAltitude);
          break;
        case (Keyboard.SHIFT + Keyboard.DOWN):
          target_altitude -= 0.05;
          //system.out.print("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = keyboard.getKey();
    }

      // Compute the roll, pitch, yaw and vertical inputs.
      final double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance;
      final double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_acceleration + pitch_disturbance;
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
    DroneControllerTest droneControllerTest = new DroneControllerTest();
    droneControllerTest.run();
  }
}