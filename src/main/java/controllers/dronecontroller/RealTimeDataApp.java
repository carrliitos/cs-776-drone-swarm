import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class RealTimeDataApp {
  private static final String X_AXIS_LABEL = "Time";
  private static final String Y_AXIS_LABEL = "Value";
  private static final int WIDTH = 800;
  private static final int HEIGHT = 600;
  private List<Double> xData = new ArrayList<>();
  private List<Double> yData = new ArrayList<>();
  private int maxDataPoints = 1000; // Maximum number of data points to display
  private int currentDataIndex = 0;
  private double Y_MIN = -10;
  private double Y_MAX = 50;
  private double currentX = 0.0;
  private JPanel chartPanel;
  private JLabel statusLabel;

  public RealTimeDataApp(String title) {
    JFrame frame = new JFrame(title);
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.setLayout(new BorderLayout());
    chartPanel = new JPanel() {
      @Override
      protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        plotData(g);
      }
    };
    chartPanel.setPreferredSize(new Dimension(WIDTH, HEIGHT));
    frame.add(chartPanel, BorderLayout.CENTER);
    statusLabel = new JLabel("No data available");
    frame.add(statusLabel, BorderLayout.SOUTH);
    frame.pack();
    frame.setVisible(true);
  }

  private void plotData(Graphics g) {
    if (!xData.isEmpty() && !yData.isEmpty()) {
      Graphics2D g2d = (Graphics2D) g.create();

      // Set up scaling based on the data range and panel size
      double xMin = xData.get(0);
      double xMax = xData.get(xData.size() - 1);
      double yMin = Math.max(Y_MIN, yData.stream().min(Double::compare).orElse(0.0));
      double yMax = Math.min(Y_MAX, yData.stream().max(Double::compare).orElse(0.0));
      double xScale = WIDTH / (xMax - xMin);
      double yScale = HEIGHT / (yMax - yMin);

      // Plot the data points
      g2d.setColor(Color.BLUE);
      for (int i = 1; i < xData.size(); i++) {
        int x1 = (int) ((xData.get(i - 1) - xMin) * xScale);
        int y1 = HEIGHT - (int) ((yData.get(i - 1) - yMin) * yScale);
        int x2 = (int) ((xData.get(i) - xMin) * xScale);
        int y2 = HEIGHT - (int) ((yData.get(i) - yMin) * yScale);
        g2d.drawLine(x1, y1, x2, y2);
      }

      // Draw x and y labels
      g2d.setColor(Color.BLACK);
      g2d.drawString(X_AXIS_LABEL, WIDTH / 2 - 20, HEIGHT - 5);
      g2d.rotate(-Math.PI / 2);
      g2d.drawString(Y_AXIS_LABEL, -HEIGHT / 2 - 20, 10);
      g2d.rotate(Math.PI / 2);

      // Draw x tick marks
      g2d.setColor(Color.GRAY);
      for (double xTick = xMin; xTick <= xMax; xTick++) {
        int x = (int) ((xTick - xMin) * xScale);
        g2d.drawLine(x, HEIGHT - 5, x, HEIGHT + 5);
      }

      // Draw y tick marks and labels
      for (double yTick = Math.floor(yMin / 10) * 10; yTick <= Math.ceil(yMax / 10) * 10; yTick += 5) {
        int y = HEIGHT - (int) ((yTick - yMin) * yScale);
        g2d.drawLine(WIDTH - 5, y, WIDTH + 5, y);
        g2d.drawString(String.format("%.0f", yTick), -HEIGHT + 10, y + 5);
      }

      // Draw (0, 0) axis label
      g2d.setColor(Color.BLACK);
      int zeroX = (int) (-xMin * xScale);
      int zeroY = HEIGHT - (int) (-yMin * yScale);
      g2d.drawString("(0, 0)", zeroX + 5, zeroY - 5);

      g2d.dispose();
    } else {
      // Display a message if no data is available
      Graphics2D g2d = (Graphics2D) g.create();
      g2d.setColor(Color.BLACK);
      g2d.drawString("No data available", 10, 20);
      g2d.dispose();
    }
  }

  public void visualize(double y) {
      double x = currentX++;
      xData.add(x);
      yData.add(Math.min(Math.max(y, Y_MIN), Y_MAX));

      if (xData.size() > maxDataPoints) {
          xData.remove(0);
          yData.remove(0);
      }

      chartPanel.repaint();
      statusLabel.setText(String.format("Latest data point: x=%.2f, y=%.2f", x, y));
  }
}
