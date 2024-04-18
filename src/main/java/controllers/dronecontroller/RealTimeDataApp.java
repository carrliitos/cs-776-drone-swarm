import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class RealTimeDataApp {
  private static final String X_AXIS_LABEL = "Time";
  private static final String Y_AXIS_LABEL = "Signals";
  private static final int WIDTH = 1000;
  private static final int HEIGHT = 500;
  private List<Double> xData = new ArrayList<>();
  private List<List<Double>> yDataList = new ArrayList<>();
  private int maxDataPoints = 100; // Maximum number of data points to display
  private int currentDataIndex = 0;
  private double Y_MIN = -6;
  private double Y_MAX = 6;
  private double currentX = 0.0;
  private JPanel chartPanel;
  private JLabel statusLabel;

  public RealTimeDataApp(String title, String[] labels) {
    JFrame frame = new JFrame(title);
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.setLayout(new BorderLayout());
    chartPanel = new JPanel() {
      @Override
      protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        plotData(g, labels);
      }
    };
    chartPanel.setPreferredSize(new Dimension(WIDTH, HEIGHT));
    frame.add(chartPanel, BorderLayout.CENTER);
    statusLabel = new JLabel("No data available");
    frame.add(statusLabel, BorderLayout.SOUTH);
    frame.pack();
    frame.setVisible(true);
    for (int i = 0; i < 4; i++) {
      yDataList.add(new ArrayList<>());
    }
  }

  private void plotData(Graphics g, String[] labels) {
    if (!xData.isEmpty()) {
      Graphics2D g2d = (Graphics2D) g.create();
      g2d.setStroke(new BasicStroke(2.0f));
      double xMin = xData.get(0);
      double xMax = xData.get(xData.size() - 1);
      double yMin = Y_MIN;
      double yMax = Y_MAX;

      for (List<Double> dataList : yDataList) {
        if (!dataList.isEmpty()) {
          yMin = Math.min(yMin, dataList.stream().min(Double::compare).orElse(0.0));
          yMax = Math.max(yMax, dataList.stream().max(Double::compare).orElse(0.0));
        }
      }
      double xScale = WIDTH / (xMax - xMin);
      double yScale = HEIGHT / (yMax - yMin);

      Color[] colors = {Color.BLUE, Color.RED, Color.GREEN, Color.ORANGE};
      for (int i = 0; i < 4; i++) {
        g2d.setColor(colors[i]);
        for (int j = 1; j < xData.size(); j++) {
          int x1 = (int) ((xData.get(j - 1) - xMin) * xScale);
          int y1 = HEIGHT - (int) ((yDataList.get(i).get(j - 1) - yMin) * yScale);
          int x2 = (int) ((xData.get(j) - xMin) * xScale);
          int y2 = HEIGHT - (int) ((yDataList.get(i).get(j) - yMin) * yScale);
          g2d.drawLine(x1, y1, x2, y2);
        }
        g2d.setColor(colors[i]);
        g2d.drawString(labels[i], 10, 20 + i * 20);
      }
      // Draw x and y labels
      g2d.setColor(Color.BLACK);
      g2d.drawString(X_AXIS_LABEL, WIDTH / 2 - 20, HEIGHT - 5);
      g2d.rotate(-Math.PI / 2);
      g2d.drawString(Y_AXIS_LABEL, -HEIGHT / 2 - 20, 10);
      g2d.rotate(Math.PI / 2);

      // Draw x tick marks and labels
      g2d.setColor(Color.GRAY);
      for (double xTick = Math.ceil(xMin / 5) * 5; xTick <= xMax; xTick += 5) {
        int x = (int) ((xTick - xMin) * xScale);
        g2d.drawLine(x, HEIGHT - 5, x, HEIGHT + 5);
        g2d.drawString(String.format("%.0f", xTick), x - 10, HEIGHT + 20);
      }

      // Draw y tick marks and labels
      for (double yTick = Math.floor(yMin / 10) * 10; yTick <= Math.ceil(yMax / 10) * 10; yTick += 5) {
        int y = HEIGHT - (int) ((yTick - yMin) * yScale);
        g2d.drawLine(WIDTH - 5, y, WIDTH + 5, y);
        g2d.drawString(String.format("%.0f", yTick), -HEIGHT + 10, y + 5);
      }
      g2d.setColor(Color.GRAY);
      int y0 = HEIGHT - (int) ((0 - yMin) * yScale);
      int yPosLine = HEIGHT - (int) ((5 - yMin) * yScale);
      int yNegLine = HEIGHT - (int) ((-5 - yMin) * yScale);
      g2d.drawLine(0, y0, WIDTH, y0);
      g2d.drawString("y=0", 5, y0 - 5);
      g2d.drawLine(0, yPosLine, WIDTH, yPosLine);
      g2d.drawString("y=5", 5, yPosLine - 5);
      g2d.drawLine(0, yNegLine, WIDTH, yNegLine);
      g2d.drawString("y=5", 5, yNegLine - 5);
      g2d.dispose();
    } else {
      Graphics2D g2d = (Graphics2D) g.create();
      g2d.setColor(Color.BLACK);
      g2d.drawString("No data available", 10, 20);
      g2d.dispose();
    }
  }

  public void visualize(double[] yValues) {
    double x = currentX++;
    xData.add(x);

    if (xData.size() > maxDataPoints) {
      xData.remove(0);
      for (List<Double> dataList : yDataList) {
        dataList.remove(0);
      }
    }

    for (int i = 0; i < 4; i++) {
      yDataList.get(i).add(Math.min(Math.max(yValues[i], Y_MIN), Y_MAX));
    }

    chartPanel.repaint();
    statusLabel.setText(String.format("Latest data point: x=%.2f", x));
  }
}
