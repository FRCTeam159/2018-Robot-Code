import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class PlotTestClient implements ITableListener {
	ArrayList<PathData> list = new ArrayList<PathData>();
	int traces=0;
	int index=0;
	int points=0;
	int count=0;
	int plot_count=0;
	int id=0;

	static NetworkTable table;
	public static void main(String[] args) {
		new PlotTestClient().run();
	}
	
	public PlotTestClient() {
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("10.1.59.2");
		table = NetworkTable.getTable("datatable");
	}

	public void run() {
		table.addTableListener(this,true);
		while (true) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException ex) {
				System.out.println("exception)");
			}
		}
	}

	private static void createAndShowGui(int id, ArrayList<PathData> list, int traces) {
		JFrame frame = new PlotPath(list, traces);
		frame.setTitle("Path Plot "+id);

		System.out.println("Showing plot: Size = " + list.size());
		//frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
		list = new ArrayList();
		table.flush();
	}

	@Override
	public void valueChanged(ITable arg0, String arg1, Object arg2, boolean arg3) {
		if (arg1.contentEquals("NewPlot"+plot_count)) {
			list.clear();
			double info[] = arg0.getNumberArray("NewPlot"+plot_count, new double[0]);
			id = (int) info[0];
			traces = (int) info[1];
			points = (int) info[2];
			index=0;
			count=0;
			System.out.println("NewPlot:" + id + " " + traces + " " + points);

		}
		if (arg1.contentEquals("PlotData"+count)) {
			double data[] = arg0.getNumberArray("PlotData"+count, new double[0]);
			PathData pd = new PathData();

			index = (int) data[0];
			pd.tm= data[1];
			for(int i=0;i<data.length-2;i++) {
				pd.d[i]=data[i+2];
			}
			list.add(pd);
			count++;
			//System.out.println("PlotData:" + index);

		}
		if((count >= points) && (points > 0) && (id==plot_count)) {
			count=0;
			index=0;
			plot_count++;
			SwingUtilities.invokeLater(new Runnable() {
				public void run() {
					System.out.println("points " + points);
					createAndShowGui(id, list, traces);
				}
			});
		}
		
	}
}
