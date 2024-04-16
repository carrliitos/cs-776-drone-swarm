import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;

public class Drone extends Robot {
  private static final int TIME_STEP = 64; // Simulation time step in milliseconds
  private String imuName;
  private String gpsName;
  private String gyroName;
  private String frontLeftLEDName;
  private String frontRightLEDName;
  private String frontRightPropellerName;
  private String frontLeftPropellerName;
  private String rearRightPropellerName;
  private String rearLeftPropellerName;
  private String cameraRollMotorName;
  private String cameraPitchMotorName;

  private InertialUnit imu;
  private GPS gps;
  private Gyro gyro;
  private LED frontLeftLED, frontRightLED;
  private Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller, cameraRollMotor, cameraPitchMotor;
  private Motor motors[];

  public Drone(String imuName, String gpsName, String gyroName, String frontLeftLEDName, 
               String frontRightLEDName, String frontRightPropellerName, String frontLeftPropellerName,
               String rearRightPropellerName, String rearLeftPropellerName, String cameraRollMotorName, 
               String cameraPitchMotorName) {
    super();
    this.imuName = imuName;
    this.gpsName = gpsName;
    this.gyroName = gyroName;
    this.frontLeftLEDName = frontLeftLEDName;
    this.frontRightLEDName = frontRightLEDName;
    this.frontRightPropellerName = frontRightPropellerName;
    this.frontLeftPropellerName = frontLeftPropellerName;
    this.rearRightPropellerName = rearRightPropellerName;
    this.rearLeftPropellerName = rearLeftPropellerName;
    this.cameraRollMotorName = cameraRollMotorName;
    this.cameraPitchMotorName = cameraPitchMotorName;

    initializeDevices(imuName, gpsName, gyroName, frontLeftLEDName, frontRightLEDName, frontRightPropellerName, 
                      frontLeftPropellerName, rearRightPropellerName, rearLeftPropellerName, cameraRollMotorName, 
                      cameraPitchMotorName);
  }

  private void initializeDevices(String imuName, String gpsName, String gyroName, String frontLeftLEDName, 
                                 String frontRightLEDName, String frontRightPropellerName, String frontLeftPropellerName, 
                                 String rearRightPropellerName, String rearLeftPropellerName, String cameraRollMotorName,
                                 String cameraPitchMotorName) {
    imu = getInertialUnit(imuName);
    imu.enable(TIME_STEP);
    gps = getGPS(gpsName);
    gps.enable(TIME_STEP);
    gyro = getGyro(gyroName);
    gyro.enable(TIME_STEP);
    frontLeftLED = new LED(frontLeftLEDName);
    frontRightLED = new LED(frontRightLEDName);
    frontRightPropeller = getMotor(frontRightPropellerName);
    frontLeftPropeller = getMotor(frontLeftPropellerName);
    rearRightPropeller = getMotor(rearRightPropellerName);
    rearLeftPropeller = getMotor(rearLeftPropellerName);
    motors = new Motor[] {
      frontRightPropeller,
      frontLeftPropeller,
      rearRightPropeller,
      rearLeftPropeller
    };
    for (Motor motor : motors) {
      motor.setPosition(Double.POSITIVE_INFINITY);
      motor.setVelocity(100);
    }
    cameraRollMotor = getMotor(cameraRollMotorName);
    cameraPitchMotor = getMotor(cameraPitchMotorName);
  }

  public void displayWelcomeMessage() {
    System.out.println("Starting drone: " + getName());
    double previousTime = 0.0;
    while (step(TIME_STEP) != -1) {
      if (getTime() - previousTime > 1.0) { break; }
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

  public double[] getRobotState() {
    try {
      final double roll = imu.getRollPitchYaw()[0];
      final double pitch = imu.getRollPitchYaw()[1];
      final double posX = gps.getValues()[0];
      final double posY = gps.getValues()[1];
      final double altitude = gps.getValues()[2];
      final double rollVelocity = gyro.getValues()[0];
      final double pitchVelocity = gyro.getValues()[1];
      double[] positions = { roll, pitch, posX, posY, altitude, rollVelocity, pitchVelocity };

      return positions;
    } catch (Exception e) {
      System.err.println("Error retrieving robot state: " + e.getMessage());
      return new double[]{0.0, 0.0, 0.0, 0.0, 0.0}; // Default values
    }
  }
}
