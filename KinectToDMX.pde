import org.openkinect.processing.*;
import processing.net.*;

Kinect2 kinect2;

Client client;
String input;
float send[];

PVector loc;
int[] data;
Boolean[] tfArray;
Boolean[] edgeArray;
int closeThresh = 0;
int farThresh = 4500;
int topCrop = 0;
int bottomCrop = 424;
int leftCrop = 0;
int rightCrop = 512;
int preCenX = 250;
int preCenY = 250;
float smFactor = 0.08;
int distX = 50;
int distY = 70;
float avgD = 2400;
float lastD = 2400;

float prevXmax = width;
float prevYmax = height;
float prevXmin = 0;
float prevYmin = 0;

Boolean imgToggle = false;
Boolean operationToggle = false;
boolean personSwitch = false;

//to store the values for the crop and thresholds
PrintWriter output;


void setup(){
  size(512,424);
  client = new Client(this, "169.254.238.25", 5050);
  kinect2 = new Kinect2(this);
  kinect2.initDepth();
  kinect2.initDevice();
  
  readSettings();
  loc = new PVector(width/2, height/2);
  tfArray = new Boolean[kinect2.depthWidth * kinect2.depthHeight];
  edgeArray = new Boolean[kinect2.depthWidth * kinect2.depthHeight];
  for(int i = 0; i < tfArray.length; i++){
    tfArray[i] = false; 
    edgeArray[i] = false;
  }
}

void draw(){
  PImage visual = createImage(kinect2.depthWidth, kinect2.depthHeight, RGB);
  data = kinect2.getRawDepth();
  visual.loadPixels();
  int count = 0;
  
  for(int x = 0; x < kinect2.depthWidth; x++){
   for(int y = 0; y < kinect2.depthHeight; y++){
     int value = data[x + y * kinect2.depthWidth];
     if(value > closeThresh && value < farThresh & x > leftCrop && x < rightCrop && y > topCrop && y < bottomCrop){
       visual.pixels[x + y * kinect2.depthWidth] = color(255, 0, 0);
       avgD += value;
       tfArray[x + y * kinect2.depthWidth] = true;
       count++;
     }else{
       tfArray[x + y * kinect2.depthWidth] = false; 
     }
   }
  } 
  
  for(int x = 0; x < kinect2.depthWidth; x++){
   for(int y = 0; y < kinect2.depthHeight; y++){
     color c = visual.pixels[x + y * visual.width];
     if(tfArray[x + y * kinect2.depthWidth] && int(abs(x - preCenX)) > distX && int(abs(y - preCenY)) > distY){
       visual.pixels[x + y * kinect2.depthWidth] = color(0,0,0);
     }
   }
  }
  
  
  int sum = 1;
  int centerX = 0;
  int centerY = 0;
  for(int x = 0; x < kinect2.depthWidth; x++){
   for(int y = 0; y < kinect2.depthHeight; y++){
     int value = data[x + y * kinect2.depthWidth];
     if(visual.pixels[x + y * visual.width] == color(255,0, 0)){
       centerX += x;
       centerY += y;
       sum++;
     }
   }
  }
  if(sum > 1000){
  centerX = centerX / sum;
  centerY = centerY / sum;
  preCenX += (centerX - preCenX) * smFactor;
  preCenY += (centerY - preCenY) * smFactor;
  avgD = avgD / sum;
  personSwitch = true;
  }else{
    personSwitch = false;
    avgD = lastD;
  }
  
  for( int x = 0; x < kinect2.depthWidth; x++){
   for(int y = 0; y < kinect2.depthHeight; y++){
     int edgeCount = 0;
     if(x > 3 && x < kinect2.depthWidth - 3 && y > 3 && y < kinect2.depthHeight - 3){
    for(int i = -3; i <=3; i++){
     for(int j = -3; j <= 3; j++){
       if(tfArray[(x + i) + (y + j) * kinect2.depthWidth]){
         edgeCount++;
       }
     }
     if(edgeCount > 20 && edgeCount < 35){
      edgeArray[x + y * kinect2.depthWidth] = true; 
     }else{
      edgeArray[x + y * kinect2.depthWidth] = false; 
     }
    }
    }
   }
  }
  
  float xmin = width + 1;
  float xmax = -1;
  float ymin = height + 1;
  float ymax = -1;
  PImage edgeImg = createImage(kinect2.depthWidth, kinect2.depthHeight, RGB);
  edgeImg.loadPixels();
  for(int x = 0; x < kinect2.depthWidth; x++){
   for(int y = 0; y < kinect2.depthHeight; y++){
     if(edgeArray[x + y * kinect2.depthWidth]){
       if(x < xmin){
        xmin = x; 
        ymin = y; 
       }
       if(x > xmax){
        xmax = x;
        ymax = y;
       }
       edgeImg.pixels[x + y * kinect2.depthWidth] = color(255);
     }
   }
  }
  edgeImg.updatePixels();
  if(xmin > preCenX - 150){
    prevXmin += (xmin - prevXmin) * smFactor * 2;
    prevYmin += (ymin - prevYmin) * smFactor * 2;
  }if(personSwitch == false){
    prevYmin += (height/2 - prevYmin) * smFactor * 2;
  }
  if(xmax < preCenX + 150){
   prevXmax += (xmax - prevXmax) * smFactor * 2;
   prevYmax += (ymax - prevYmax) * smFactor * 2;
  }if(personSwitch == false){
   prevYmax += (height/2 - prevYmax) * smFactor * 2; 
  }
  
  visual.updatePixels();
  if(imgToggle){
  image(visual, 0, 0);
  }else{
    image(edgeImg, 0, 0);
  }
  ellipseMode(CENTER);
  stroke(255);
  noFill();
  ellipse(prevXmin, prevYmin, 30, 30);
  ellipse(prevXmax, prevYmax, 30, 30);
  ellipse(preCenX, preCenY, 35, 150);
  float temp;
  if(personSwitch){
   temp = 1;
  }else{
   temp = 0;
  }
  client.write(preCenX + "," + preCenY + "," + avgD + "," + prevXmin + "," + prevYmin + "," + prevXmax + "," + prevYmax + "," + temp);
}

void readSettings(){
  String[] settings = loadStrings("settings.txt");
  closeThresh = int(settings[0]);
  farThresh = int(settings[1]);
  topCrop = int(settings[2]);
  bottomCrop = int(settings[3]);
  leftCrop = int(settings[4]);
  rightCrop = int(settings[5]);
}

void keyPressed(){
 if(key == 'p'){
   output = createWriter("settings.txt");
   output.println(closeThresh);
   output.println(farThresh);
   output.println(topCrop);
   output.println(bottomCrop);
   output.println(leftCrop);
   output.println(rightCrop);
   output.flush();
   output.close();
 }

   if(key == 'l'){
    if(operationToggle == true){
     operationToggle = false;
    }else{
     operationToggle = true; 
    }
   }
  if(key == 's'){
    if(operationToggle == true){
    closeThresh += 10;
    }else{
     closeThresh -= 10; 
    }
  }
  if(key == 'w'){
    if(operationToggle == true){
    farThresh += 10;
    }else{
     farThresh -= 10; 
    }
  }
  if(key == 'a'){
   if(operationToggle == true){
    leftCrop += 10;
    }else{
     leftCrop -= 10; 
    } 
  }
  if(key == 'd'){
   if(operationToggle == true){
    rightCrop += 10;
    }else{
     rightCrop -= 10; 
    } 
  }
  if(key == 'u'){
    if(operationToggle == true){
    topCrop += 10;
    }else{
     topCrop -= 10; 
    }
  }
  if(key == 'j'){
    if(operationToggle == true){
    bottomCrop += 10;
    }else{
     bottomCrop -= 10;
    }
  }
  if(key == 'i'){
    if(imgToggle == true){
     imgToggle = false; 
    }else{
     imgToggle = true; 
    }
    
  }
}