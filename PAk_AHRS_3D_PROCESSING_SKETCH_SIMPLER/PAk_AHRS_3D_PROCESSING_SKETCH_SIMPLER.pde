/*************************************************************************************
 * Test Sketch for TIVA AHRS v. 1.0
 * 9 Degree of Measurement Attitude and Heading Reference System
 * for SENSORHUB 
 *
 * Released under GNU GPL (General Public License) v3.0 
 * Written by Francisco SY (frasalyu at gmail.com)
 *
 * Infos, updates, bug reports and feedback:
 *     
 *************************************************************************************/

/*
  NOTE: There seems to be a bug with the serial library in the latest Processing
 versions 1.5 and 1.5.1: "WARNING: RXTX Version mismatch ...". The previous version
 1.2.1 works fine and is still available on the web.
 */

import processing.opengl.*;
import processing.serial.*;

// IF THE SKETCH CRASHES OR HANGS ON STARTUP, MAKE SURE YOU ARE USING THE RIGHT SERIAL PORT:
// 1. Have a look at the Processing console output of this sketch.
// 2. Look for the serial port list and find the port you need 
// 3. Set your port name here:
//final static int SERIAL_PORT_NUM = 2;
final String serialPort = "COM15"; // replace this with your serial port. On windows you will need something like "COM1".

// 4. Try again.



final static int SERIAL_PORT_BAUD_RATE = 115200;

float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;
float yawOffset = 0.0f;



float [] q = new float [4];
float [] acc = new float [3];
float [] gyro = new float [3];
float [] magn = new float [3];
float [] Euler = new float [3];

String String_RX;
String string_roll;
String string_pitch;
String string_yaw;
String string_tmp;

boolean yaw_a_cero=false;
boolean pitch_a_cero=false;
boolean roll_a_cero=false;


PFont font;
Serial serial;

boolean synched = false;


float decodeFloat(String inString)
{
  byte [] inData = new byte[4];


  return Float.valueOf(inString).floatValue();
}

void drawArrow(float headWidthFactor, float headLengthFactor) {
  float headWidth = headWidthFactor * 200.0f;
  float headLength = headLengthFactor * 200.0f;

  pushMatrix();

  // Draw base
  translate(0, 0, -100);
  box(100, 100, 200);

  // Draw pointer
  translate(-headWidth/2, -50, -100);
  beginShape(QUAD_STRIP);
  vertex(0, 0, 0);
  vertex(0, 100, 0);
  vertex(headWidth, 0, 0);
  vertex(headWidth, 100, 0);
  vertex(headWidth/2, 0, -headLength);
  vertex(headWidth/2, 100, -headLength);
  vertex(0, 0, 0);
  vertex(0, 100, 0);
  endShape();
  beginShape(TRIANGLES);
  vertex(0, 0, 0);
  vertex(headWidth, 0, 0);
  vertex(headWidth/2, 0, -headLength);
  vertex(0, 100, 0);
  vertex(headWidth, 100, 0);
  vertex(headWidth/2, 100, -headLength);
  endShape();

  popMatrix();
}

void drawBoard() {
  pushMatrix();

  if (yaw_a_cero)
    yaw=0;

  rotateY(radians(yaw - yawOffset));
  rotateX(-radians(pitch));
  rotateZ(radians(roll)); 

  // Board body
  fill(255, 0, 0);
  box(250, 20, 400);

  pushMatrix();
  translate(0, 10, 0);
  fill(0, 0, 255);
  box(250, 10, 200);
  popMatrix();

  // Forward-arrow
  pushMatrix();
  translate(0, 0, -200);
  scale(0.5f, 0.2f, 0.25f);
  fill(0, 255, 0);
  drawArrow(1.0f, 2.0f);
  popMatrix();

  popMatrix();
}

// Skip incoming serial stream data until token is found
boolean readToken(Serial serial, String token) {
  // Wait until enough bytes are available
  if (serial.available() < token.length())
    return false;

  // Check if incoming bytes match token
  for (int i = 0; i < token.length (); i++) {
    if (serial.read() != token.charAt(i))
      return false;
  }

  return true;
}

// Global setup
void setup() {
  // Setup graphics
  //size(640, 480, OPENGL);
  size(1280, 720, P3D);
  smooth();
  noStroke();
  frameRate(50);

  // Load font
  // font = loadFont("Univers-66.vlw");
  //textFont(font);

  font = loadFont("CourierNew36.vlw");
  textFont(font, 20);

  // Setup serial port I/O
  println("AVAILABLE SERIAL PORTS:");
  println(Serial.list());
  //String portName = Serial.list()[SERIAL_PORT_NUM];
  println();
  println("HAVE A LOOK AT THE LIST ABOVE AND SET THE RIGHT SERIAL PORT NUMBER IN THE CODE!");
  //println("  -> Using port " + SERIAL_PORT_NUM + ": " + portName);
  //serial = new Serial(this, portName, SERIAL_PORT_BAUD_RATE);


  serial = new Serial(this, serialPort, SERIAL_PORT_BAUD_RATE);
}

void setupTiva() {
  println("Trying to setup and synch ...");


  delay(1000);  // 1 seconds should be enough


  // Synch 
  serial.clear();  // Clear input buffer up to here
}



void draw() {
  // Reset scene
  background(0);
  lights();

  // Sync with Tiva
  if (!synched) {
    textAlign(CENTER);
    fill(255);
    text("Connecting to SENSORHUB...", width/2, height/2, -200);

    if (frameCount == 2)
      setupTiva();  // Set ouput params and request synch token
    else if (frameCount > 2)

      synched = readToken(serial, "\n");  // Look for synch token
    return;
  }

  // Read angles from serial port
  while (serial.available () >= 15) {


    //String_RX = serial.readStringUntil('!');
    String_RX = serial.readStringUntil('\n');
    if (String_RX != null) {
      println(String_RX);


      ///////////////////////////
      if (String_RX != null && String_RX.length() > 0) {
        String [] inputStringArr = split(String_RX, ",");
        if (inputStringArr.length >= 15) { // q1,q2,q3,q4,\r\n so we have 5 elements

          q[0] = decodeFloat(inputStringArr[0]);
          q[1] = decodeFloat(inputStringArr[1]);
          q[2] = decodeFloat(inputStringArr[2]);
          q[3] = decodeFloat(inputStringArr[3]);
          //println("xpos:"+xPos);
          acc[0] = decodeFloat(inputStringArr[4]);
          acc[1] = decodeFloat(inputStringArr[5]);
          acc[2] = decodeFloat(inputStringArr[6]);
          gyro[0] = decodeFloat(inputStringArr[7]);
          gyro[1] = decodeFloat(inputStringArr[8]);
          gyro[2] = decodeFloat(inputStringArr[9]);
          magn[0] = decodeFloat(inputStringArr[10]);
          magn[1] = decodeFloat(inputStringArr[11]);    
          magn[2] = decodeFloat(inputStringArr[12]);  
          roll = decodeFloat(inputStringArr[13]);
          pitch = decodeFloat(inputStringArr[14]);
          yaw=  decodeFloat(inputStringArr[15]);
        }
      }
      /////////////////////////////
    }
  }

  // Draw board
  pushMatrix();
  translate(width/2, height/2, -350);
  drawBoard();
  popMatrix();

  textFont(font, 20);
  fill(255);
  textAlign(LEFT);

  // Output info text
  text("Point MPU9250 towards screen and press 'a' to align, 'y' to clear yaw", 10, 25);


  text("Q: \n" + nf(q[0], 3, 3) + "\n" +  nf(q[1], 3, 3) + " \n" +  nf(q[2], 3, 3) + "\n" +  nf(q[3], 3, 3), 10, 40);
  //text("Euler Angles:\nYaw (psi)  : " + nfp(degrees(Euler[0]),3,2) + "\nPitch (theta): " + nfp(degrees(Euler[1]),3,2) + "\nRoll (phi)  : " + nfp(degrees(Euler[2]),3,2), 200, 20);
  text("Acc:\n" + nfp(acc[0], 3, 3) + "\n" + nfp(acc[1], 3, 3) + "\n" + nfp(acc[2], 3, 3) + "\n", 20, 140);
  text("Gyro:\n" + nfp(gyro[0], 3, 3) + "\n" + nfp(gyro[1], 3, 3) + "\n" + nfp(gyro[2], 3, 3) + "\n", 20, 230);
  text("Magn:\n" + nfp(magn[0], 3, 3) + "\n" + nfp(magn[1], 3, 3) + "\n" + nfp(magn[2], 3, 3) + "\n", 20, 320);



  // Output angles
  pushMatrix();
  translate(10, height - 10);
  textAlign(LEFT);
  text("Yaw: " + ((int) yaw), 0, 0);
  text("Pitch: " + ((int) pitch), 150, 0);
  text("Roll: " + ((int) roll), 300, 0);
  text("Yaw-offset: " + ((int) (yaw-yawOffset)), 450, 0);
  popMatrix();
}

void keyPressed() {
  switch (key) {
  case '0':  // a
    // serial.write("xx0");
    break;
  case '1':  
    // serial.write("xx1");
    break;

  case 'r':  
    roll_a_cero=!roll_a_cero;
    break;

  case 'p':  
    pitch_a_cero=!pitch_a_cero;
    break;

  case 'y':  
    yaw_a_cero=!yaw_a_cero;
    break;

  case 'a':  // Align screen
    yawOffset = yaw;
  }
}



