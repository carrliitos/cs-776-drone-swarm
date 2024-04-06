import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;

public class RealTimeDataApp extends Application {
  private static final String[] COLUMN_NAMES = { "" };
  private static final String INITIAL_DATA = "Initial Data";
  private static String currentData = INITIAL_DATA;
  private static TableView<String[]> tableView;
  private static TextArea textArea;
  private static XYChart.Series<Number, Number> series;
  private static int valueCount = 0;

  @Override
  public void start(Stage primaryStage) {
    BorderPane root = new BorderPane();

    // Table
    tableView = new TableView<>();
    root.setCenter(tableView);
    for (String columnName : COLUMN_NAMES) {
      TableColumn<String[], String> column = new TableColumn<>(columnName);
      final int columnIndex = tableView.getColumns().size();
      column.setCellValueFactory(cellData -> {
        String[] row = cellData.getValue();
        return row != null && columnIndex < row.length ? new javafx.beans.property.SimpleStringProperty(row[columnIndex]) : null;
      });
      tableView.getColumns().add(column);
    }

    // Text area
    textArea = new TextArea();
    textArea.setEditable(false);
    root.setBottom(textArea);

    // Chart
    NumberAxis xAxis = new NumberAxis();
    NumberAxis yAxis = new NumberAxis();
    LineChart<Number, Number> lineChart = new LineChart<>(xAxis, yAxis);
    series = new XYChart.Series<>();
    lineChart.getData().add(series);
    root.setTop(lineChart);

    primaryStage.setScene(new Scene(root, 800, 600));
    primaryStage.show();

    // Update data every frame
    new AnimationTimer() {
      @Override
      public void handle(long now) {
        if (valueCount > 0) {
          String[] data = tableView.getItems().get(valueCount - 1);
          double xValue = Double.parseDouble(data[0]);
          double yValue = Double.parseDouble(data[1]);
          series.getData().add(new XYChart.Data<>(xValue, yValue));
          if (series.getData().size() > 50) {
            series.getData().remove(0);
          }
        }
      }
    }.start();
  }

  public void setHeaders(String[] headers) {
    tableView.getColumns().clear();
    for (String header : headers) {
      TableColumn<String[], String> column = new TableColumn<>(header);
      final int columnIndex = tableView.getColumns().size();
      column.setCellValueFactory(cellData -> {
        String[] row = cellData.getValue();
        return row != null && columnIndex < row.length ? new javafx.beans.property.SimpleStringProperty(row[columnIndex]) : null;
      });
      tableView.getColumns().add(column);
    }
  }

  public void visualize(double[] data) {
    String[] rowData = new String[data.length];
    for (int i = 0; i < data.length; i++) {
      rowData[i] = String.valueOf(data[i]);
    }
    tableView.getItems().add(rowData);

    StringBuilder sb = new StringBuilder();
    for (double value : data) {
      sb.append(value).append(", ");
    }
    sb.setLength(sb.length() - 2); // Remove the last comma and space
    sb.append("\n");
    textArea.appendText(sb.toString());
    valueCount++;
  }

  public void addData(double x, double y) {
    series.getData().add(new XYChart.Data<>(x, y));
    if (series.getData().size() > 50) {
      series.getData().remove(0);
    }
  }
}
