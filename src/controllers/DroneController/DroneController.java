import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;

/**
 * PID Reading: https://oscarliang.com/pid/
 *
 * Drone Controller: Simplistic Drone Controller
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 *
 * front right propeller
 * front left propeller
 * rear right propeller
 * rear left propeller
 * front left LED
 * front right LED
 **/
public class DroneController extends Robot {
  private static final int TIME_STEP = 64; // Simulation time step in milliseconds
  private static final double MAX_VELOCITY = 2.0;

  private Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller;
  private Motor motors[];
  private LED frontLeftLED, frontRightLED;
  private InertialUnit imu;
  private GPS gps;
  private Gyro gyro;
  
  // Constants
  private static final double K_VERTICAL_THRUST = 68.5; // with this thrust, the drone lifts.
  private static final double K_VERTICAL_OFFSET = 0.6; // Vertical offset where the robot actually targets to stabilize itself.
  private static final double K_VERTICAL_P = 3.0; // P constant of the vertical PID.
  private static final double K_ROLL_P = 50.0; // P constant of the roll PID.
  private static final double K_PITCH_P = 30.0; // P constant of the pitch PID.
  
  public DroneController() {
    super();
    initializeDevices();
  }
  
  private void initializeDevices() {
    imu = getInertialUnit("inertial unit");
    imu.enable(TIME_STEP);
    gps = getGPS("gps");
    gps.enable(TIME_STEP);
    gyro = getGyro("gyro");
    gyro.enable(TIME_STEP);
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
      motor.setVelocity(0.10 * MAX_VELOCITY);
    }
  }
  
  private void displayWelcomeMessage() {
    System.out.println("Starting the drone...");
    while (step(TIME_STEP) != -1) {
      if (getTime() > 1.0) {
        break;
      }
    }
  }
  
  private void blinkLEDS() {
    final double timeInSecs = getTime(); // in seconds.
    final boolean ledState = ((int) timeInSecs) % 2 == 0;
    
    frontLeftLED.set(ledState ? 1 : 0);
    frontRightLED.set(ledState ? 0 : 1);
  }
  
  private double[] getRobotState() {
    try {
      final double roll = imu.getRollPitchYaw()[0];
      final double pitch = imu.getRollPitchYaw()[1];
      final double altitude = gps.getValues()[2];
      final double rollVelocity = gyro.getValues()[0];
      final double pitchVelocity = gyro.getValues()[1];
      
      return new double[]{roll, pitch, altitude, rollVelocity, pitchVelocity};
    } catch (Exception e) {
      System.err.println("Error retrieving robot state: " + e.getMessage());
      return new double[]{0.0, 0.0, 0.0, 0.0, 0.0}; // Default values
    }
  }
  
  private void stabilizations(double roll, double rollVelocity, double rollDisturbance) {
    ;
  }
  
  private void actuators() {
    ;
  }
  
  // Main control loop
  public void run() {
    displayWelcomeMessage();
    while (step(TIME_STEP) != -1) {
      // Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();
      
      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      
      // Transform the keyboard input to disturbances on the stabilization algorithm.

      // Compute the roll, pitch, yaw and vertical inputs.
      
      // Actuate the motors taking into consideration all the computed inputs.
    }
  }
  
  public static void main(String[] args) {
    DroneController droneController = new DroneController();
    droneController.run();
  }
}
