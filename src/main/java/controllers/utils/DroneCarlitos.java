public class DroneCarlitos {
  private Drone drone;

  public DroneCarlitos(String imuName, String gpsName, String gyroName, String frontLeftLEDName,
                       String frontRightLEDName, String frontRightPropellerName, String frontLeftPropellerName,
                       String rearRightPropellerName, String rearLeftPropellerName, String cameraRollMotorName,
                       String cameraPitchMotorName) {
    drone = new Drone (imuName, gpsName, gyroName, frontLeftLEDName, frontRightLEDName, frontRightPropellerName, 
                       frontLeftPropellerName, rearRightPropellerName, rearLeftPropellerName,
                       cameraRollMotorName, cameraPitchMotorName);
  }

  public void run() {
    drone.displayWelcomeMessage();
  }

  public static void main(String[] args) {
    String inertialUnit = "inertial unit";
    String gps = "gps";
    String gyro = "gyro";
    String frontLeftLED = "front left led";
    String frontRightLED = "front right led";
    String frontLeftPropeller = "front left propeller";
    String frontRightPropeller = "front right propeller";
    String rearLeftPropeller = "rear left propeller";
    String rearRightPropeller = "rear right propeller";
    String cameraRoll = "camera roll";
    String cameraPitch = "camera pitch";
    DroneCarlitos droneCarlitos = new DroneCarlitos(inertialUnit, gps, gyro, frontLeftLED, frontRightLED,
                                                    frontRightPropeller, frontLeftPropeller, rearRightPropeller, 
                                                    rearLeftPropeller, cameraRoll, cameraPitch);
    droneCarlitos.run();
  }
}
