import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableModel;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

public class RealTimeDataApp {
  private static String[] COLUMN_NAMES = { "" };
  private static final String INITIAL_DATA = "Initial Data";
  private static String currentData = INITIAL_DATA;
  private static DefaultTableModel tableModel;
  private static JTable table;

  public RealTimeDataApp() {
    SwingUtilities.invokeLater(() -> {
      JFrame frame = new JFrame("Real-Time Data");
      frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
      frame.addWindowListener(new WindowAdapter() {
        @Override
        public void windowClosing(WindowEvent e) {
          // Handle window closing event
          // For now, we just hide the frame
          frame.setVisible(false);
        }
      });

      tableModel = new DefaultTableModel(null, COLUMN_NAMES);
      table = new JTable(tableModel);
      JScrollPane scrollPane = new JScrollPane(table);
      frame.add(scrollPane);

      frame.pack();
      frame.setVisible(true);
    });
  }

  public void setHeaders(String[] headers) {
    COLUMN_NAMES = headers;
    tableModel.setColumnIdentifiers(COLUMN_NAMES);
  }

  public void visualize(double[] data) {
    Object[] rowData = new Object[data.length];
    for (int i = 0; i < data.length; i++) {
      rowData[i] = String.format("%.8f", data[i]);
    }
    tableModel.addRow(rowData);
    table.scrollRectToVisible(table.getCellRect(table.getRowCount() - 1, 0, true));
  }
}
