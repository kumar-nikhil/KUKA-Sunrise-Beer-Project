package additionalFunction;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.CharBuffer;

import com.kuka.task.ITaskLogger;


public class TCPServer {
    private Thread rT;
    private ServerSocket server_socket;
    private Socket socket;
    private static BufferedReader in;
    private static PrintWriter out;
	boolean end_flag = false;
    static int port;
    String threadedMessage;
    public String getThreadedMessage() {
		return threadedMessage;
	}


	// loger
    private static ITaskLogger logger;
    
    
    public TCPServer ( ITaskLogger t ) {
    	port = 30000;
    	logger = t;
    }

    
	/**
	 * @param message : message to send to client
	 */
	public void send(String message) {
        try {
			logger.info("TCP Server [SEND] : \"" + message + "\"");
			out.println(message);   // sending message via outputstream
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}
	
	/**
	 * use this method once, and then use <b>getThreadedMessage()</b> method to get messages continuously.
	 */
	public void receiveWithThread() {
		threadedMessage = "";
		
		ReceiveRunnable rR = new ReceiveRunnable();
		rT = new Thread(rR);
		rT.start();
		
	}
	
	
	public String receiveWait() {
    	try {
    		// clear buffered stacks
    		while( socket.getInputStream().available() != 0 ) {
				socket.getInputStream().read();
			}
    		
    		logger.info("Waitiing for a message from Client...");
    		
    		String message;
			if ( (message = in.readLine()) != null ) { // receiving message via inputstream
//    		message = in.readLine();
				logger.info("TCP Server [RECEIVE] : \"" + message + "\"");
				return message;
			}
//			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    	
		return null;
	}

	public String receiveLines() {
		String content = "";
		
    	try {
    		// clear buffered stacks
    		while( socket.getInputStream().available() != 0 ) {
				socket.getInputStream().read();
			}
    		
    		logger.info("Waitiing for a message from Client...");
    		
    		do {
    			content += in.readLine();	// receiving message via inputstream
			} while (content.contains("END"));
    		
		} catch (IOException e) {
			e.printStackTrace();
		}
    	
		return content;
	}
	
	public void sendFile() throws Exception {
        // Specify the file
		File file = new File("E:\\data.xml");
		FileInputStream fis = new FileInputStream(file);
		BufferedInputStream bis = new BufferedInputStream(fis);

        //Get socket's output stream
        OutputStream os = socket.getOutputStream();
        
        //Read File Contents into contents array 
        byte[] contents;
        long fileLength = file.length(); 
        long current = 0;
        
        long start = System.nanoTime();
        while(current!=fileLength){ 
            int size = 10000;
            if(fileLength - current >= size)
                current += size;    
            else{ 
                size = (int)(fileLength - current); 
                current = fileLength;
            } 
            contents = new byte[size]; 
            bis.read(contents, 0, size); 
            os.write(contents);
            System.out.print("Sending file ... "+(current*100)/fileLength+"% complete!");
        }   
        
        os.flush(); 
	}
	
	public void receiveFile() throws Exception {
        byte[] contents = new byte[1000000];	// 1MB
        
        //Initialize the FileOutputStream to the output file's full path.
        FileOutputStream fos = new FileOutputStream("E:\\data.xml");
        BufferedOutputStream bos = new BufferedOutputStream(fos);
        InputStream is = socket.getInputStream();
        
        //No of bytes read in one read() call
        int bytesRead = 0;

        while( ( bytesRead=is.read(contents) )!=-1 ) {
        	bos.write(contents, 0, bytesRead);         	
        }
        
        bos.flush();
        System.out.println("File saved successfully!");
	}
	
	public void startComm() {
    	logger.info("Starting TCP communication");
    	 try{
         	server_socket = null;
 			int maxretry = 10;
 			
 			while ( maxretry-- > 0 ) {
 				try {
 					server_socket = new ServerSocket(port);
 					break;	// try succeed
 				} catch (Exception e) {
// 					e.printStackTrace();
 					if ( port > 30010 ) {
 						logger.info("All ports are being used");
 						e.printStackTrace();
 					}
 					logger.info("port " + port + " is in use");
 					port++;
 				} finally {	}
 			}	// end of while

 			logger.info("Server is ready on port [" + server_socket.getLocalPort() + "]" );
 			logger.info("TCP\tC : Wating for connection...");

 			socket = server_socket.accept();
 			logger.info("got a connection from [" + socket.getInetAddress() + "]\n");
 			out = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true);
 			in = new BufferedReader(new InputStreamReader(socket.getInputStream()));


         } catch (IOException e) {
             logger.info("TCP Server : Error" + e);
         }
    	 
	}
	
	public void endComm() {
        try {
        	logger.info("Ending TCP communication");
			socket.close();
			server_socket.close();
			end_flag = true;
			try {
				rT.interrupt();
			} catch (Exception e) {
			}				
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public boolean checkComm() {
		return socket.isConnected();
	}

    // socket thread
    public class ReceiveRunnable implements Runnable {
    	public String message;

        public String getMessage() {
			return message;
		}

		public void run() {
        	while ( !Thread.currentThread().isInterrupted() && !end_flag ) {
    			try {
					if ( (message = in.readLine()) != null ) { // receiving message via inputstream
						threadedMessage = message;
					}
				} catch (IOException e) {
//					e.printStackTrace();
				}
        	}
        }	// end of run()

    }
}
