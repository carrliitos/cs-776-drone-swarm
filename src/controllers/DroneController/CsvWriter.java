import java.io.FileWriter;
import java.io.IOException;

public class CsvWriter {
  private FileWriter writer;

  public CsvWriter(String outputFile) {
    try {
      writer = new FileWriter(outputFile);
    } catch (IOException e) {
      System.err.println("Error creating CsvWriter: " + e.getMessage());
    }
  }

  public void writeHeaders(String[] headers) {
    try {
      for (int i = 0; i < headers.length; i++) {
        writer.append(headers[i]);
        if (i < headers.length - 1) {
          writer.append(',');
        }
      }
      writer.append('\n');
    } catch (IOException e) {
      System.err.println("Error writing headers: " + e.getMessage());
    }
  }

  public void writeData(double[] data) {
    try {
      for (int i = 0; i < data.length; i++) {
        writer.append(String.valueOf(data[i]));
        if (i < data.length - 1) {
          writer.append(',');
        }
      }
      writer.append('\n');
    } catch (IOException e) {
      System.err.println("Error writing data: " + e.getMessage());
    }
  }

  public void close() {
    try {
      writer.close();
    } catch (IOException e) {
      System.err.println("Error closing CsvWriter: " + e.getMessage());
    }
  }
}
