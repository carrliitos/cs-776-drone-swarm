import static org.junit.Assert.*;
import org.junit.Test;

import java.util.stream.Collectors;
import java.util.Arrays;
import java.io.IOException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.lang.reflect.Field;

public class CsvWriterTest {
  private String outputFile = System.getProperty("user.dir") + "src/test/resources/data/data/test.csv";;

  private class CsvWriterMock extends CsvWriter {
    public CsvWriterMock(String outputFile) throws IOException {
      super(outputFile);
    }

    @Override
    public void close() throws IOException {
      throw new IOException("Mocked IOException");
    }
  }

  @Test
  public void testCsvWriterInitialization() {
    try {
      CsvWriter csvWriter = new CsvWriter(outputFile);
      assertNotNull(csvWriter);
    } catch (IOException e) {
      fail("CsvWriter initialization failed: " + e.getMessage());
    }
  }

  @Test
  public void testCsvWriterInvalidPath() {
    String invalidOutputFile = System.getProperty("user.dir") + "src/test/resources/data/nonexistent-directory/test.csv";
    try {
      CsvWriter csvWriter = new CsvWriter(invalidOutputFile);
    } catch (IOException e) {
      fail("Expected IOException");
    }
  }

  @Test
  public void testWriteHeaders() {
    String[] headers = {"Header1", "Header2", "Header3"};

    try {
      CsvWriter csvWriter = new CsvWriter(outputFile);
      assertNotNull(csvWriter);

      csvWriter.writeHeaders(headers);
      csvWriter.close();

      BufferedReader reader = new BufferedReader(new FileReader(outputFile));
      String headerLine = reader.readLine();
      reader.close();

      assertEquals(String.join(",", headers), headerLine);
    } catch (IOException e) {
      fail("Error writing headers: " + e.getMessage());
    }
  }

  @Test
  public void testWriteData() {
    double[] data = {1.0, 2.0, 3.0};

    try {
      CsvWriter csvWriter = new CsvWriter(outputFile);
      assertNotNull(csvWriter);

      csvWriter.writeData(data);
      csvWriter.close();

      BufferedReader reader = new BufferedReader(new FileReader(outputFile));
      String dataLine = reader.readLine();
      reader.close();

      String expectedLine = Arrays.stream(data)
                    .mapToObj(d -> String.format("%.8f", d))
                    .collect(Collectors.joining(","));
      assertEquals(expectedLine, dataLine);
    } catch (IOException e) {
      fail("Error writing data: " + e.getMessage());
    }
  }

  @Test(expected = IOException.class)
  public void testClose() throws IOException {
    CsvWriter csvWriter = new CsvWriterMock(outputFile);
    assertNotNull(csvWriter);

    csvWriter.close();

    csvWriter.writeHeaders(new String[]{"Header"});
  }

}
