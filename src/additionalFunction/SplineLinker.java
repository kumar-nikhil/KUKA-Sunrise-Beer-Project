package additionalFunction;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;

/**
 * @author Seulki-Kim , 2015. 11. 5. , KUKA Robotics Korea<p>
 * use static method => <br><b> SplineLinker.splineLinker(ArrayList)
 */
public class SplineLinker {
	
	/**
	 * @param frames	list of Frame to make spline links
	 * @return <b>Queue</b>	Spline	separated spline segments by 20 motions<p>
	 * usage<br>
	 * <code><pre>for (Spline spline : splineQueue) {
	 * 	lbr.move(spline);
	 * }</pre></code>
	 * 
	 */
	public static Queue<Spline> splineLinker(ArrayList<Frame> frames) {
		Queue<Spline> splineQueue = new LinkedList<Spline>();
		Queue<Frame> queue = new LinkedList<Frame>();
		for (Frame frame : frames) {
			queue.offer(frame);
		}
		while (!queue.isEmpty()) {
			SPL[] spline = new SPL[Math.min(queue.size(), 20)];
			for (int i = 0; i < spline.length; i++) {
				spline[i] = new SPL(queue.poll());
			}
			splineQueue.offer(new Spline(spline));
		}
//		for (Spline spline : splineQueue) {
//			_lbr.move(spline);
//		}
		
		return splineQueue;
	}
}
