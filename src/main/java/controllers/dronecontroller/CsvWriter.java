import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class CsvWriter {
  private FileWriter writer;

  public CsvWriter(String outputFile) throws IOException {
    Path outputPath = Paths.get(outputFile);
    if (!Files.exists(outputPath.getParent())) {
      Files.createDirectories(outputPath.getParent());
    }

    writer = new FileWriter(outputFile);
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
        String formattedValue = String.format("%.8f", data[i]);
        writer.append(formattedValue);
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
