package additionalFunction;

import com.kuka.task.ITaskLogger;


/**
 * @author Seulki-Kim , 2016. 4. 25. , KUKA Robotics Korea<p>
 */
public class CycleTimer {
	double cycleTime_start;
	double cycleTime;

	String nameOfTimer;
	ITaskLogger logger;

	public CycleTimer(ITaskLogger logger) {
		this.nameOfTimer = "";
		this.logger = logger;
		cycleTime_start = 0;
		cycleTime = 0;
	}
	public CycleTimer(String nameOfTimer, ITaskLogger logger) {
		this.nameOfTimer = nameOfTimer;
		this.logger = logger;
		cycleTime_start = 0;
		cycleTime = 0;
	}

	public double getCycleTime() {
		return cycleTime;
	}
	
	public void start() {
		cycleTime_start = System.nanoTime();
		logger.info("----------[" + nameOfTimer + "] Cycle time check start----------");
		
	}
	
	public void end() {
		cycleTime = (System.nanoTime() - cycleTime_start) / 1000000000;
		String strC = String.format("%,.3f", cycleTime);
		logger.info("----------[" + nameOfTimer + "] Cycle time : [" + strC + "] sec----------");		
	}
	
}
