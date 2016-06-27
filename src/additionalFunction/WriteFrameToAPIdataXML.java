package additionalFunction;

import java.util.ArrayList;
import java.util.Map;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.task.ITaskLogger;

/**
 * @author Seulki-Kim , 2015. 8. 7. , KUKA Robotics Korea<p>
 * WriteFraToAPIdataXML class
 */
public class WriteFrameToAPIdataXML {
	@SuppressWarnings("unused")
	private static RoboticsAPIApplication app;
	private static ITaskLogger logger;
	private ObjectFrame baseFrame;
	
	private IPersistenceEngine engine;
	private XmlApplicationDataSource defaultDataSource;

	// constructor
	/**
	 * @param app		-	RoboticsAPIApplication
	 * @param baseFrame	-	Base Frame for the new frame. new frame will be stored under this frame
	 * @param logger		ITaskLogger
	 */
	public WriteFrameToAPIdataXML(RoboticsAPIApplication app, ObjectFrame baseFrame, ITaskLogger logger) {
		WriteFrameToAPIdataXML.app = app;
		WriteFrameToAPIdataXML.logger = logger;
		this.baseFrame = baseFrame;

		// get the object to manipulate the RoboticsAPI.data.xml
		engine = app.getContext().getEngine(IPersistenceEngine.class);
		defaultDataSource = (XmlApplicationDataSource) engine.getDefaultDataSource();
	}
	// end of constructor
	
	
	//
	// method
	//
	/**
	 * @param app	application class. Usually, just use <code><b>this</b></code>
	 * @param base	Parent frame for the new base. It could be <code><b>World.Current.getRootFrame()</b></code>
	 * @param name	Name of the new base
	 * <p>ex>	<code>WriteFrameToAPIdataXML.makeBaseFrame(<br>this, getApplicationData().getFrame(base), newBaseName);</code>	
	 */
	public static void makeBaseFrame(RoboticsAPIApplication app, ObjectFrame base, String name) {
		IPersistenceEngine engine = app.getContext().getEngine(IPersistenceEngine.class);
		XmlApplicationDataSource defaultDataSource = (XmlApplicationDataSource) engine.getDefaultDataSource();
		
		String frameName = name.substring(1);
		ObjectFrame newFrame = null;
		Map<String, Object> m = base.getAdditionalFrameData();
		
		newFrame = defaultDataSource.addFrame(frameName, base);
		newFrame.getAdditionalFrameData().putAll(m);
		defaultDataSource.saveFile(false);
		newFrame.setAdditionalParameter("UseAsBaseForJogging", true);
	}

	/**
	 *<code> public <b>writeFrame</b>(Frame frame, String name)</code><p>
	 * writes the frame which has the <code>name</code> under baseFrame<br>
	 * comment is also written from the <code>frame</code>
	 * @param frame	-	Frame to be wrote
	 * @param name	-	Frame name
	 */
	public void writeFrame(Frame frame, String name) {
		
		// Create a new frame under the Base Frame
		// base jogging deactivate, get tcp data, activate again
		baseFrame.setAdditionalParameter("UseAsBaseForJogging", false);
		Map<String, Object> m = baseFrame.getAdditionalFrameData();
		String frameName = name;
		ObjectFrame newFrame = null;
		newFrame = defaultDataSource.addFrame(frameName, baseFrame);

		try {	// copy frame's comment if there is.
			newFrame.setAdditionalParameter("Comment", (frame.getAdditionalParameter("Comment").toString()) );
		} catch (Exception e) {
		}

		// Transformate the newFrame from the baseFrame
		defaultDataSource.changeFrameTransformation(newFrame, baseFrame.transformationTo(frame));
//		defaultDataSource.getFrameParameters("/TouchBase/"+frameName).putAll(m);
		newFrame.getAdditionalFrameData().putAll(m);
		
		defaultDataSource.saveFile(false);
		if ( logger != null ) {
			logger.info("[" + name + "] frame written on API.data.xml");							
		}
		baseFrame.setAdditionalParameter("UseAsBaseForJogging", true);
		
	}
	
	/**
	 *<code> public <b>writeFrame</b>(Frame frame, String name)</code><p>
	 * writes the frame which has the <code>name</code> under baseFrame<br>
	 * comment is also written from the <code>frame</code>
	 * @param frames	-	List of frames to be wrote
	 * @param name	-	Frame name prefix -> <code>name_001 ... name_999</code>
	 */
	public void writeFrame(ArrayList<Frame> frames, String name) {

		// base jogging deactivate, get tcp data, activate again
		baseFrame.setAdditionalParameter("UseAsBaseForJogging", false);
		Map<String, Object> m = baseFrame.getAdditionalFrameData();

		int number = 0;
		for (Frame frame : frames) {
			number++;
			String frameName = String.format(name + "_%03d", number);
			ObjectFrame newFrame = null;
			newFrame = defaultDataSource.addFrame(frameName, baseFrame);
			
			try {	// copy frame's comment if there is.
				newFrame.setAdditionalParameter("Comment", (frame.getAdditionalParameter("Comment").toString()) );
			} catch (Exception e) {
			}
			
			// Transformate the newFrame to the currentPosition
			defaultDataSource.changeFrameTransformation(newFrame, baseFrame.transformationTo(frame));
//			defaultDataSource.getFrameParameters("/CSVBase/"+frameName).putAll(m);
			newFrame.getAdditionalFrameData().putAll(m);
		}
		defaultDataSource.saveFile(false);
		logger.info("[" + number + "] frames written on API.data.xml");				
		baseFrame.setAdditionalParameter("UseAsBaseForJogging", true);
		
	}
		
	
	/**
	 *<code> public <b>deleteFrame</b>(String name)</code><p>
	 * Deletes the frame which has the <code>name</code> under baseFrame
	 * @param name	-	name of the frame to be deleted
	 */
	public void deleteFrame(String name) {
		if ( logger != null ) {
			logger.info("Deleting [/" + baseFrame.getName() + "/" + name + "] frame");
		}
		ObjectFrame removeFrame = baseFrame.getChild(name);
		defaultDataSource.removeFrame(removeFrame);
		defaultDataSource.saveFile();
	}

	/**
	 *<code> public <b>deleteAllFrames</b>()</code><p>
	 * Deletes all frames under baseFrame
	 */
	public void deleteAllFrames() {
		if ( logger != null ) {
			logger.info("Deleting [/" + baseFrame.getChildren().size() + "/" +  "] frames");
		}
		while ( !baseFrame.getChildren().isEmpty() ) {
			ObjectFrame removeFrame = baseFrame.getChildren().get(0);
			defaultDataSource.removeFrame(removeFrame);
		}
		defaultDataSource.saveFile(false);
	}
	
	
}
