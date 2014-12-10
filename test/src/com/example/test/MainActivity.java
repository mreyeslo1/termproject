package com.example.test;

import android.app.Activity;
import android.os.Bundle;
import android.os.StrictMode;

import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

//main 
public class MainActivity extends Activity {
	TextView txt1,txt2;
	//initialize all classes 
	String str=null;
	byte[] send_data = new byte[1024];
	byte[] receiveData = new byte[1024];
	String modifiedSentence;
	Button bt1,bt2,bt3,bt4,bt5,bt6;

	@Override
	public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        StrictMode.ThreadPolicy policy = new StrictMode.
        ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy); 
     
        setContentView(R.layout.activity_main);
        //text boxes 
        txt1   = (TextView)findViewById(R.id.textView1); 
        txt2   = (TextView)findViewById(R.id.textView9); 
        
        
        bt1 = (Button) findViewById(R.id.but1);
        bt2 = (Button) findViewById(R.id.but2);
        bt3 = (Button) findViewById(R.id.but3);
        bt4 = (Button) findViewById(R.id.but4);
        bt5 = (Button) findViewById(R.id.but5);
        bt6 = (Button) findViewById(R.id.but6);
        
        //function for classes 
        bt1.setOnClickListener(new View.OnClickListener(){             
        	public void onClick(View v) {   
        		//send string str to udp server 
        		str="forward\0";
				txt1.setText(str);
				try {
					client();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        	}  
 
         });
        bt2.setOnClickListener(new View.OnClickListener(){             
        	public void onClick(View v) {                 
        		str="left\0";
				txt1.setText(str);
				try {
					client();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        	}  
 
         });
        bt3.setOnClickListener(new View.OnClickListener(){             
        	public void onClick(View v) {                 
				
				
				str="right\0";
				txt1.setText(str);
				try {
					client();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        	}  
 
         });
        bt4.setOnClickListener(new View.OnClickListener(){             
        	public void onClick(View v) {                 
        		str="back\0";
				txt1.setText(str);
				try {
					client();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        	}  
 
         });
        
        bt5.setOnClickListener(new View.OnClickListener(){             
        	public void onClick(View v) {                 
        		str="close\0";
				txt1.setText(str);
				try {
					client();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        	}  
 
         });
        
        bt6.setOnClickListener(new View.OnClickListener(){             
        	public void onClick(View v) {                 
        		str="sensor\0";
				txt1.setText(str);
				try {
					client();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        	}  
 
         });

}
	

//functino that is responible for UDP communication 
public void client() throws IOException{
	  
	 //intialize sockets for UDP 
	 DatagramSocket client_socket = new DatagramSocket(8888);
    InetAddress IPAddress =  InetAddress.getByName("10.0.2.2"); 
	//loop will send the STR and recieve 
   while (true)
   {
   	send_data = str.getBytes();
//   	System.out.println("Type Something (q or Q to quit): ");
//send the string 
       DatagramPacket send_packet = new DatagramPacket(send_data,str.length(), IPAddress, 8888);
       client_socket.send(send_packet); 
       
		DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
		
		//recieves the string 
		client_socket.receive(receivePacket);
		modifiedSentence = new String(receivePacket.getData());
		//confrim that we recieved 
		System.out.println("FROM SERVER:" + modifiedSentence);
		//changes the text box 2
		txt2.setText(modifiedSentence);
	   client_socket.close();

	   

     }

   }       

}
