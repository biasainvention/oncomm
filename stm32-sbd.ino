#include "StringSplitter.h"
#include <TinyGPS.h>
TinyGPS gps;

 #include <EEPROM.h>
// Absolute min and max eeprom addresses. Actual values are hardware-dependent.
const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 512;

boolean eeprom_is_addr_ok(int addr) {
  return ((addr >= EEPROM_MIN_ADDR) && (addr <= EEPROM_MAX_ADDR));
}

boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes){
  int i;
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }
  for (i = 0; i < numBytes; i++) {
    EEPROM.write(startAddr + i, array[i]);}
    return true;
}

int atx,atrty;
#define maxRetry 20

#define adcB PA7

//time tmx;

double vbat;

unsigned long t1,ctr,tb,cb;
String strLoc="1224.45555,3456.77888";
String strSBD="1224.45555,3456.77888";
String strGPS="==";
bool newData = false;
int myy;
byte myy8,mmm,mdd,mhh,mnn,mss,msec;
byte vbat8;
byte sbdwb[32];
word csum;
byte fSend;

float flat, flon;
unsigned long age;
unsigned long dlat,dlon;
byte speed,heading;
unsigned int mins;
unsigned int rpm1;
unsigned int rpm2;

byte io1,io2,io3;

#define ledp PB15

unsigned int itv,itvm;
int gctr;

String strRpm=">99999,99999<";

int initsbd(){
  atrty=0;
  atx=0;
  while(atx<=0)
  {
    Serial.println("sending AT...");
    Serial2.println("AT");
    String atstr=".......................";
    if(Serial2.available())atstr=Serial2.readString();
    atx=atstr.indexOf("OK");
    atrty++;
    if(atrty>10){
      Serial.println("AT timeout");
      return 0;
    }
    digitalWrite(PC13,HIGH);
    delay(500);
    digitalWrite(PC13,LOW);
    delay(500);
  }
  Serial2.println("AT&K0");
  Serial.println("AT&K0");
  delay(1000);
  Serial2.println("AT*R1");
  Serial.println("AT*R1");
  delay(1000);
  Serial2.println("AT+SBDMTA=1");
  Serial.println("AT+SBDMTA=1");
  delay(1000);
  Serial2.println("AT&W0");
  Serial.println("AT&W0");
  delay(1000);
  Serial2.println("AT&Y0");
  Serial.println("AT&Y0");
  delay(1000);
  Serial2.println("AT+CIER=1,1,1");
  Serial.println("AT+CIER=1,1,1");
  delay(1000);
  Serial.println("Init OK");
  
}

int sendsbd(byte ln){
  digitalWrite(ledp,LOW);
  Serial2.setTimeout(5000);
  Serial2.println("AT*R1");
  delay(1000);
  Serial2.println("AT+SBDD0");
  delay(1000);
  Serial2.println("AT+CIER=0,0,0");
  delay(1000);
  if(ln){
      Serial.println("Send Binary...");
      Serial2.print("AT+SBDWB="+String(ln)+"\r");
      delay(1000);
      csum=0;
      for(int e=0;e<ln;e++){
        Serial2.write(sbdwb[e]);
        if(sbdwb[e]<0x0a)Serial.print('0');
        Serial.print(sbdwb[e],HEX);
        Serial.print(" ");
        csum+=sbdwb[e];
      }
      Serial2.write(csum >> 8);
      Serial.print(": ");
      Serial.print(csum >> 8,HEX);
      Serial.print(" ");
      Serial2.write(csum & 0xff);
      Serial.print(csum & 0xff,HEX);
  }
  else{
      Serial.println("Send Text...");
      Serial2.println("AT+SBDWT="+strSBD);
  }
  delay(1000);
  
  byte resp=0;
  if(Serial2.available()){
    resp=1;
    Serial.println(Serial2.readString());  
  }
  if(resp==0){
    Serial.println("modem timeout...");
  }
  atx=0;
  atrty=0;
  Serial2.setTimeout(500);
  while(atx<=0){
    Serial.flush();
    Serial2.println("AT+SBDIX");
    Serial.println("-------------------------");
    Serial.println("Init Transmit...("+String(atrty)+")");
    unsigned long tx=millis();
    String atstr="No Data ............................................................................................................................";
    unsigned int axt=25; 
    unsigned long ts1;
    ts1=millis();
    atstr="";
    while(axt){
      if(millis()-ts1>1000){
        ts1=millis();
        axt--;
        digitalWrite(PC13,HIGH);
        Serial.print(".");
        digitalWrite(ledp,LOW);
        delay(100);
        digitalWrite(ledp,LOW);
        digitalWrite(PC13,HIGH);
      }
      if(Serial2.available()){
        char cx = Serial2.read();
        //Serial.write(cx);
        atstr=atstr+cx;
      }
      if(atstr.indexOf("OK")>=0)break;
    }
    Serial.println();
    Serial.println("RESPONSE: ("+String(millis()-tx)+" ms)");
    Serial.println(">>");
    Serial.println(atstr);
    Serial.println("<<");
    atrty++;
    if(atrty>maxRetry){
      Serial.println("SBDI timeout, Tranmission Failed !!!");
      return 0;
    }
    if(atstr.indexOf("+SBDIX: 0,")>=0){Serial.println("Transferred successfully.");atx=1;}
    if(atstr.indexOf("+SBDIX: 1,")>=0){Serial.println("Transferred successfully, MT message too big.");atx=1;}
    if(atstr.indexOf("+SBDIX: 2,")>=0){Serial.println("Transferred successfully, location not accepted.");atx=1;}
    if(atstr.indexOf("+SBDIX: 32")>=0)Serial.println("No network service, unable to initiate call.");
    if(atstr.indexOf("+SBDIX: 33")>=0)Serial.println("Antenna fault, unable to initiate call.");
    if(atstr.indexOf("+SBDIX: 34")>=0)Serial.println("Radio is disabled.");
    if(atstr.indexOf("+SBDIX: 35")>=0)Serial.println("Transceiver is busy.");
    if(atstr.indexOf("+SBDIX: 18")>=0)Serial.println("Connection lost (RF drop).");
    Serial.println();
    Serial.println();
  }
  Serial.println("Transmit OK");
  delay(1000);
//  Serial2.println("AT*R0");
//  delay(1000);
  Serial2.println("AT+SBDD0");
  delay(1000);
  Serial2.println("AT+CIER=1,1,1");
  delay(1000);
}
int sendLoc(){
  sendsbd(15);
}

void setup() {
  pinMode(PC13,OUTPUT);
  digitalWrite(PC13,HIGH);
  delay(2000);
  itv=3600;
  Serial.begin(4800);
  Serial2.begin(19200);
  Serial3.begin(9600);
  Serial.setTimeout(500);
  Serial2.setTimeout(500);
  Serial3.setTimeout(500);
  digitalWrite(PA0,HIGH);
  pinMode(PA0,OUTPUT);
  digitalWrite(ledp,HIGH);
  pinMode(ledp,OUTPUT);
  delay(500);
  Serial.println("OnComm 9602 system init...");
  delay(500);
  Serial.println("EEPROM read:");
  Serial.print(EEPROM.read(10),HEX);
  Serial.print(" ");
  Serial.println(EEPROM.read(11),HEX);
  itv=(EEPROM.read(10)<<16) + EEPROM.read(11);
  if(itv>86400)itv=3600;
  Serial.print("Interval:");
  Serial.println(itv+String(" Seconds"));
  initsbd();
  delay(500);
}

void loop() {
  int adc1 = analogRead(adcB);
  
  vbat=35.5*adc1/4095;
  vbat8= round(vbat);
  if(digitalRead(PB4)==0)io1=1;else io1=0;
  
  if(t1>millis())t1=millis();
  if(millis()-t1>1000){
    t1=millis();
    if(digitalRead(ledp)==0)digitalWrite(ledp,HIGH);
    else digitalWrite(ledp,LOW);
    ctr++;
    Serial.print("----------------------");
    Serial.println(String(ctr)+" of "+String(itv));
    Serial.println("Vbat="+String(vbat)+"V ("+String(adc1)+")");
    if(newData){
      newData=0;
      
      gps.f_get_position(&flat, &flon, &age);
      speed=gps.f_speed_knots(); 
      heading=gps.f_course()*0.708333333;
      gps.crack_datetime(&myy,&mmm,&mdd,&mhh,&mnn,&mss,&msec,&age);
      myy8=myy-2000;
      //setTime(mhh,mnn,mss,mdd,mmm,myy);
      
      mins=mnn+(mhh*60);
      itvm = mins%(itv/60);
      
      Serial.print(flat,6);
      Serial.print(",");
      Serial.println(flon,6);
      Serial.print(String(speed)+"knot");
      Serial.print(",");
      Serial.println(String(heading)+"deg");
      Serial.print(String(rpm1)+"Hz");
      Serial.print(",");
      Serial.println(String(rpm2)+"Hz");
      Serial.print(myy);
      Serial.print("/");
      Serial.print(mmm);
      Serial.print("/");
      Serial.print(mdd);
      Serial.print(" ");
      Serial.print(mhh);
      Serial.print(":");
      Serial.print(mnn);
      Serial.print(":");
      Serial.print(mss);
      Serial.print(" ");
      Serial.print(mins);
      Serial.print("/");
      Serial.print(itv/60);
      Serial.print(" ");
      Serial.println(itvm);
      
      dlat=(flat*100000+9000000)*0.9320675;
      dlon=(flon*100000+18000000)*0.46603375;
      sbdwb[0]=(dlon&0xff0000)>>16;
      sbdwb[1]=(dlon&0x00ff00)>>8;
      sbdwb[2]=(dlon&0x0000ff);
      sbdwb[3]=(dlat&0xff0000)>>16;
      sbdwb[4]=(dlat&0x00ff00)>>8;
      sbdwb[5]=(dlat&0x0000ff);
      sbdwb[6]=(speed & 0x3f) | ((myy8 & 0x60)<<1);
      sbdwb[7]=heading;
      sbdwb[8]=rpm1&0xff;
      sbdwb[9]=rpm2&0xff;
      sbdwb[10]=(rpm1>>8) | ((rpm1>>5) & 0b00111000);  
      sbdwb[11]=vbat8 & 0b00011111;  
      if(io1) sbdwb[11] |= 0x80;
      if(io2) sbdwb[11] |= 0x40;
      if(io3) sbdwb[11] |= 0x20;
      sbdwb[12]=mins & 0xff;
      sbdwb[13]=(mins>>8) | (mdd<<2) |(mmm<<7);
      sbdwb[14]=(mmm>>1) | (myy8<<3);
      csum=0;
      for(int e=0;e<15;e++){
        csum += sbdwb[e];
        if(sbdwb[e]<0x0a)Serial.print('0');
        Serial.print(sbdwb[e],HEX);
        Serial.print(" ");   
      }
      Serial.print("* ");
      if(csum>>8<0x0a)Serial.print('0');
      Serial.print(csum>>8,HEX);
      Serial.print(":");
      if((csum & 0xff)<0x0a)Serial.print('0');
      Serial.println(csum & 0xff,HEX);      
      if(itvm==0 && fSend==0){
        fSend=1;
        ctr=0;
        Serial.println("Send Location...");
        sendLoc();
      }
      if(itvm) fSend=0;
    }
  }
  if(ctr>=itv){
    ctr=0;
  }
  if(Serial.available()){
    String sstr=Serial.readString();
    Serial.println(sstr);
    if(sstr[0]==0x0A){
      Serial.println("SOH: RPM");
      StringSplitter *splitter = new StringSplitter(sstr, ',', 2);
      String item1 = splitter->getItemAtIndex(0);
      String item2 = splitter->getItemAtIndex(1);
      rpm1=item1.toInt();
      rpm2=item2.toInt();
      Serial.println("RPM1="+String(rpm1));
      Serial.println("RPM2="+String(rpm2));
    }
    if(sstr.indexOf("*TIMER#")>=0){
      StringSplitter *splitter = new StringSplitter(sstr, '#', 3);
      if(splitter->getItemCount()>1){
        String item = splitter->getItemAtIndex(1);
        Serial.println("TIMER SET: "+item+" seconds");
        itv=item.toInt();
        EEPROM.write(10,itv >> 16);
        EEPROM.write(11,itv);
      }
    }
    if(sstr.indexOf("*BRIDGE#")>=0){
      int ctrB=20;
      byte s10;
      byte sdx[100];
      int sdix;
      char cdx;
      StringSplitter *splitter = new StringSplitter(sstr, '#', 3);
      if(splitter->getItemCount()>1){
        String item = splitter->getItemAtIndex(1);
        ctrB = item.toInt();
        Serial.println("Fixed bridging timeout: "+String(ctrB)+" seconds");
      }
      Serial.println("Enter bridge mode for "+String(ctrB)+" seconds...");
      tb=millis();
      cb=0;
      while(cb<ctrB){
        if(millis()-tb>1000){
          tb=millis();
          cb++;
        }
        if(Serial.available()){
          cb=0;
          cdx=Serial.read();
          Serial2.write(cdx);
          if(cdx=='*'){
            sdix=0;
          }
          if(cdx=='#'){
            if(sdx[0]=='*' && sdx[1]=='E'&& sdx[2]=='N'&& sdx[3]=='D' ){
              cb=ctrB+1;
            }
          }
          sdx[sdix]=cdx;
          sdix++;
          if(sdix>=99)sdix=0;
        }
        if(Serial2.available())Serial.write(Serial2.read());
//        if(cb%10==0 && s10==0){
//          Serial.println(">>>bridge time: "+String(cb)+" s#");
//          s10=1;
//        }
//        if(cb%10!=0)s10=0;
      }
      Serial.println("Bridge mode disengaged...");
    }
    if(sstr.indexOf("*SEND#")>=0){
      Serial.println(">Send Data");
      sendLoc();
    }
    if(sstr.indexOf("*SENDT#")>=0){
      StringSplitter *splitter = new StringSplitter(sstr, '#', 3);
      if(splitter->getItemCount()>1){
        String item = splitter->getItemAtIndex(1);
        Serial.println(">Send instant message:");
        Serial.println(item);
        strSBD=item;
        sendsbd(0);
      }
    }
    if(sstr.indexOf("*SENDB#")>=0){
      //*SENDB#33 44 DE 23 DD FE FF 00 23
      StringSplitter *splitter = new StringSplitter(sstr, '#', 3);
      if(splitter->getItemCount()>1){
        String item = splitter->getItemAtIndex(1);
        Serial.println(">Send instant binary:");
        char stbx[16];
        int dti,sbdc;
        byte dtx;
        sbdc=0;
        for(int ix=0;ix<item.length()+1;ix++){
          stbx[dti++] = item[ix];
          if(item[ix]==' ' || item[ix]==0){
             dtx=strtol(&stbx[0], (char **) '\0',16);
             Serial.println("data:"+String(dtx));
             dti=0;
             sbdwb[sbdc++]=dtx;
             //sbdc++;
          }
        }
        Serial.println(String(sbdc)+ " bytes send");
        //strSBD=item;
        sendsbd(sbdc);
      }
    }
  }
  if(Serial2.available())Serial.write(Serial2.read());
  while(Serial3.available()){
    char cg;
    cg=Serial3.read();
    //Serial.print(cg);
    if (gps.encode(cg)) newData = true;
  }
}
