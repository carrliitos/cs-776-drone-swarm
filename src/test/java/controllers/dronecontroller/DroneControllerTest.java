import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DroneControllerTest {

  @Test
  void testClampWithinRange() {
    double value = 5.0;
    double low = 1.0;
    double high = 10.0;
    double result = DroneController.clamp(value, low, high);

    assertEquals(value, result);
  }

  @Test
  void testClampBelowRange() {
    double value = 0.5;
    double low = 1.0;
    double high = 10.0;
    double result = DroneController.clamp(value, low, high);

    assertEquals(low, result);
  }

  @Test
  void testClampAboveRange() {
    double value = 15.0;
    double low = 1.0;
    double high = 10.0;
    double result = DroneController.clamp(value, low, high);

    assertEquals(high, result);
  }
}
