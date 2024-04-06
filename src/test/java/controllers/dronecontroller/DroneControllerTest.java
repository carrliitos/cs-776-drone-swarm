import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class DroneControllerTest {

    @Test
    public void testClamp() {
        assertEquals(3.0, DroneController.clamp(3.0, 1.0, 5.0), 0.0001); // value within range
        assertEquals(1.0, DroneController.clamp(-1.0, 1.0, 5.0), 0.0001); // value below low
        assertEquals(5.0, DroneController.clamp(6.0, 1.0, 5.0), 0.0001); // value above high
        assertEquals(1.0, DroneController.clamp(0.0, 1.0, 5.0), 0.0001); // value at low
        assertEquals(5.0, DroneController.clamp(5.0, 1.0, 5.0), 0.0001); // value at high
    }
}
