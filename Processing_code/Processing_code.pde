//  This is processing side of code for your custom ESP8266 based multirotor flight control board  
//  to be used together with the processing sketch 
//  the config tested was - my laptop was AP and ESP8266 was acting as the station
 
//  I have tried to make flying a quadcopter much similar to playing a game -
//  use 'W' and 'S' to pitch forward and backward respectively
//  use 'A' and 'D' to roll left and right respectively
//  use 'Q' and 'E' to yaw counter-clockwise and clockwise respectively
//  use Space to jump (increase throttle here)
//  use Shift to Crouch (decrease throttle here)
//  use 'R' to increase rate of change of throttle by Space and Shift

//  this sketch stores 2 types of data in 2 files - PID values and trim data
//  at every start, it loads the data into the sketch and in case the values change, it rewrites new values into the files

//  this skech also can display the filtered values of accel & gyro from sensors and also motor values and battery voltage  



/*
PITCH - 'w s' dev changed by ' up down '
ROLL - 'a d'  dev changed by ' left right '
YAW - ' q e ' 
THROTTLE - 'space'  dev changed by ' + - '
*/

import processing.net.*;
//int ab=3;
Client c;
String ip2 = "10.42.3.204";
//String ip2 = "10.42.0.210";

//static boolean uplink = false; 
static boolean uplink = true; 

//String ip2 = "10.42.0.1";

//String a = "";
int port = 300;
int aa = 0;
int dev = 0;
  int Pdef = 45, Pdev = 0,Rdef = 45,Rdev = 0,Ydef = 45,Ydev = 0,Tdef = 0,Tdev = 0;

int trimP = 0, trimR = 0, trimY = 0;
boolean w = false, a = false,s = false,d = false,q = false, e = false, sp = false, r = false, t = false, p = false, Gpid = false;
boolean i = false, j = false,k = false,l = false,u = false, o = false, f = false, ce = false, m = false, v = false;

boolean trimMode = false, trimSave = false, last_v = false, y = false;
boolean up = false, dn  = false, le = false, ri = false;
boolean sh  = false, al = false, ct  = false, ca = false, rec = false;
boolean[] num = {false, false, false, false, false, false, false, false, false, false };
boolean pl = false, mi = false, arm = false;
int  throttle  =  0 , yaw = 90 , pitch = 90 , roll = 90, arm_ch = 0 , thro = 0;
//double throttle = 0 ;
int count = 0;
boolean pidMode = false, save = false, disarmed = false, dot = false, last_dot = false;
int kpR , kiR , kdR;
int kpP , kiP , kdP;
int kpY , kiY , kdY;

int gkpR , gkiR , gkdR;
int gkpP , gkiP , gkdP;
int gkpY , gkiY , gkdY;

double dkpR , dkiR , dkdR;
double dkpP , dkiP , dkdP;
double dkpY , dkiY , dkdY;
String Mode = "nor";
int []amax = {0,0,0} ; int []aoff = {0,0,0} ; int []aval = {0,0,0} ; int []amin = {0,0,0} ;
int lastsend = 0; 
int vol = 0;
int []hh = {0,0,0};   int []gg = {0,0,0};   int []mo = {0,0,0,0};   int []rp = {0,0}; 

byte pMode = 0;
int [][]la_mo = new int [30][4];

long y_tim = 0;
void load_PID()
{
 try 
   {
      String []ab = loadStrings("pid_values.txt");

      String bb = ab[0].substring(ab[0].indexOf('q') + 1 , ab[0].indexOf('r') );
      String cc = ab[0].substring(ab[0].indexOf('r') + 1 , ab[0].indexOf('s') );
      String dd = ab[0].substring(ab[0].indexOf('s') + 1 , ab[0].indexOf('t') );

      String ee = ab[1].substring(ab[1].indexOf('t') + 1 , ab[1].indexOf('u') );
      String ff = ab[1].substring(ab[1].indexOf('u') + 1 , ab[1].indexOf('v') );
      String gg = ab[1].substring(ab[1].indexOf('v') + 1 , ab[1].indexOf('w') );
              
      String hh = ab[2].substring(ab[2].indexOf('w') + 1 , ab[2].indexOf('x') );
      String ii = ab[2].substring(ab[2].indexOf('x') + 1 , ab[2].indexOf('y') );
      String jj = ab[2].substring(ab[2].indexOf('y') + 1 , ab[2].indexOf('z') );


      kpP = Integer.parseInt(bb);   kiP = Integer.parseInt(cc);   kdP = Integer.parseInt(dd);
      kpR = Integer.parseInt(ee);   kiR = Integer.parseInt(ff);   kdR = Integer.parseInt(gg);
      kpY = Integer.parseInt(hh);   kiY = Integer.parseInt(ii);   kdY = Integer.parseInt(jj);
   }
catch (Exception e)
   {
      kpP = 0 ; kiP = 0 ; kdP = 0 ;
      kpR = 0 ; kiR = 0 ; kdR = 0 ;
      kpY = 0 ; kiY = 0 ; kdY = 0 ;
   }
              dkpP = kpP*4 ; dkiP = kiP*4 ; dkdP = kdP*4 ;
              dkpR = kpR*4 ; dkiR = kiR*4 ; dkdR = kdR*4 ;
              dkpY = kpY*4 ; dkiY = kiY*4 ; dkdY = kdY*4 ;

}
void load_trims()
{
  try {
        String []tr = loadStrings("trims.txt");
        String aa = tr[0].substring(tr[0].indexOf('p') + 1 , tr[0].indexOf('q') );
        String bb = tr[1].substring(tr[1].indexOf('q') + 1 , tr[1].indexOf('r') );
        String cc = tr[2].substring(tr[2].indexOf('r') + 1 , tr[2].indexOf('s') );
            
        trimP = Integer.parseInt(aa); trimR= Integer.parseInt(bb); trimY = Integer.parseInt(cc);

      }
  catch (Exception e)
      {
         trimP = 0; trimR = 0; trimY = 0; 
      }

}
void setup()
{
 frameRate(50); 
 size(640,480);
 throttle  =  0 ;
 for (int aq = 0; aq<30; aq++)
 {
    for (int bq = 0; bq<4; bq++)
      {    la_mo[aq][bq] = 0;   }
 }
//   c = new Client(this, ip2, port ); // Replace with your server's IP and port
load_PID();
load_trims();
  surface.setAlwaysOnTop(true);

}

void draw()
{
  Pdef = 3; Rdef = 3; Ydef = 3; Tdef = 45;
  
//  int bb = 127;
  aa = 127; dev = 0; background(0, 100, 0);
  //int dev2 = 0;s
//  int[] num1 = {0,0,0,0,0,0,0,0,0,0};
  
//  for ( int i=0 ; i<10; i++) num1[i] = (num[i]) ? 1 : 0 ;

//  W S A D Q E - are the main controls as mentioned at the top
//  moreover, we have I K J L U O as secondary controls.. 
//  these do not work without the main controls but adjust the sensitivity of main controls 
//  layout of secondary keys is same as primary for better ergonomics

  if (!trimMode)
  {
  if (i) {if (w) Pdev += 3 ; if(s) Pdev -= 3; Pdev = constrain(Pdev, 0,60); i = false;}
  if (k) {if (s) Pdev += 3 ; if(w) Pdev -= 3; Pdev = constrain(Pdev, 0,60); k = false;}
  if (j) {if (a) Rdev += 3 ; if(d) Rdev -= 3; Rdev = constrain(Rdev, 0,60); j = false;}
  if (l) {if (d) Rdev += 3 ; if(a) Rdev -= 3; Rdev = constrain(Rdev, 0,60); l = false;}
  if (u) {if (q) Ydev += 3 ; if(e) Ydev -= 3; Ydev = constrain(Ydev, 0,60); u = false;}
  if (o) {if (e) Ydev += 3 ; if(q) Ydev -= 3; Ydev = constrain(Ydev, 0,60); o = false;}
  }



//  this is for adjusting the trims -
//  even when all motors are at same speeds, the quad might tilt to a side...
//  mostly due to little errors in weight balanicng or thrust
//  by changing values of trims, we give an offset to motors to counteract the above mentioned problem
// to change the value of trim for a particular control ex. WS, look up its secondary control.. i.s. IK
// press h  for trm mode to begin 
 
 else 
 {
  if (i) {trimP -= 1; trimP = constrain(trimP, -20, 20); i = false;}
  if (k) {trimP += 1; trimP = constrain(trimP, -20, 20); k = false;}
  if (j) {trimR -= 1; trimR = constrain(trimR, -20, 20); j = false;}
  if (l) {trimR += 1; trimR = constrain(trimR, -20, 20); l = false;}
  if (u) {trimY -= 1; trimY = constrain(trimY, -20, 20); u = false;}
  if (o) {trimY += 1; trimY = constrain(trimY, -20, 20); o = false;} 
  
  
  if (trimSave) 
      {
        String []trr = {"p" + trimP + "q", "q" + trimR + "r", "r" + trimY + "s", };
        saveStrings("trims.txt", trr); trimSave =   false; trimMode = false; 
      }
 }

  //    7    8    9   kpY    kiY    kdY
  //    4    5    6   kpR    kiR    kdR
  //    1    2    3   kpP    kiP    kdP
  // 0 WILL TURN PID MODE ON OFF, . will save the values on the ESP 
  // as well as in the text file in the folder... 
  
  if (pidMode && disarmed)
  {
  if (num[1]) {  if (pl) dkpP += 1;  if (mi) dkpP -= 1;  }
  if (num[2]) {  if (pl) dkiP += 1;  if (mi) dkiP -= 1;  }
  if (num[3]) {  if (pl) dkdP += 1;  if (mi) dkdP -= 1;  }
  
  if (num[4]) {  if (pl) dkpR += 1;  if (mi) dkpR -= 1;  }
  if (num[5]) {  if (pl) dkiR += 1;  if (mi) dkiR -= 1;  }
  if (num[6]) {  if (pl) dkdR += 1;  if (mi) dkdR -= 1;  }
  
  if (num[7]) {  if (pl) dkpY += 1;  if (mi) dkpY -= 1;  }
  if (num[8]) {  if (pl) dkiY += 1;  if (mi) dkiY -= 1;  }
  if (num[9]) {  if (pl) dkdY += 1;  if (mi) dkdY -= 1;  }
  
  kpP = ((int)(dkpP/4));  kiP = ((int)(dkiP/4));  kdP = ((int)(dkdP/4));
  kpR = ((int)(dkpR/4));  kiR = ((int)(dkiR/4));  kdR = ((int)(dkdR/4));
  kpY = ((int)(dkpY/4));  kiY = ((int)(dkiY/4));  kdY = ((int)(dkdY/4));
    
  kpP = constrain(kpP, 0 , 999);
  kiP = constrain(kiP, 0 , 999);
  kdP = constrain(kdP, 0 , 999);
  
  kpR = constrain(kpR, 0 , 999);
  kiR = constrain(kiR, 0 , 999);
  kdR = constrain(kdR, 0 , 999);
  
  kpY = constrain(kpY, 0 , 999);
  kiY = constrain(kiY, 0 , 999);
  kdY = constrain(kdY, 0 , 999);
  
  //pMq300r300s300t300u300v300w300x300y300z
  String pidv = "pMq", pids = "pNq";
  
  String pidp = (kpP) + "r" + (kiP) + "s" + (kdP) + "t"  ;  
  String pidr = (kpR) + "u" + (kiR) + "v" + (kdR) + "w"  ;  
  String pidy = (kpY) + "x" + (kiY) + "y" + (kdY) + "z"  ;
  
  String pid = "";
  if (dot) save = true;
  String []ppp = {"q" + pidp, "t" + pidr, "w" + pidy};
  if (!save) pid = pidv + pidp + pidr + pidy + "\n";
      else { pid = pids + pidp + pidr + pidy + "\n"; saveStrings("pid_values.txt", ppp); }
    
  
  if (last_dot && !dot ) {save = false; pidMode = false; last_dot = false;}// println("balle");
  last_dot = dot;
  if (uplink)  
      {
       try { c.output.write(pid.getBytes()); }//println("\t good\t" + c.active()+ "\t" + millis() ); }
       catch (Exception ee) { println("bad\t" + millis());   c = new Client(this, ip2, port );}
       }
  print(pid +  millis()/1000 + "  " );
  text("PID", 540, 450  ); 
  text(". : save", 400, 450  );
  text("0 : exit ", 470, 450  );
  
  }
  else if (!pidMode && ce && disarmed)
  {
   String cal = "Calib\n"; 
     text("v : save", 400, 450  );

   if (v) cal = "CSave\n";   
   if (last_v && !v) {cal = "CSave\n"; ce = false; }

 if (millis() - lastsend > 50){    print(cal +  millis()/1000 + "  " ); delay(6);

if (uplink){
            try { c.output.write(cal.getBytes()); }//println("\t good\t" + c.active()+ "\t" + millis() ); }
            catch (Exception ee) { println("bad\t" + millis());   c = new Client(this, ip2, port );}
           }
lastsend = millis();}
  
    
     text("Calib", 320, 420  );last_v = v ;
           
       // 980/16384
/*
    String a1 = (int)(amin[0] ) + "   " + (int)(aval[0] ) + "   " + (int)(amax[0] ) ;
    String a2 = (int)(amin[1] ) + "   " + (int)(aval[1] ) + "   " + (int)(amax[1] ) ;
    String a3 = (int)(amin[2] ) + "   " + (int)(aval[2] ) + "   " + (int)(amax[2] ) ;
    String a4 = (int)(aoff[0] ) + "   " + (int)(aoff[1] ) + "   " + (int)(aoff[2] ) ;
*/
    text((int)(amin[0]), 100, 100); text((int)(aval[0]), 150, 100); text((int)(amax[0]), 200, 100); 
    text((int)(amin[1]), 100, 125); text((int)(aval[1]), 150, 125); text((int)(amax[1]), 200, 125); 
    text((int)(amin[2]), 100, 150); text((int)(aval[2]), 150, 150); text((int)(amax[2]), 200, 150);
    
    text((int)(aoff[0]), 100, 200); text((int)(aoff[1]), 150, 200); text((int)(aoff[2]), 200, 200);
  }
  
  else if (p && disarmed)
  {
    if (uplink)
          {
            String gpid = "GPID";
            try { c.output.write(gpid.getBytes()); }//println("\t good\t" + c.active()+ "\t" + millis() ); }
            catch (Exception ee) { println("bad\t" + millis());   c = new Client(this, ip2, port );}
//            long tim_net = millis();
//            while(c.available() == 0 &&  millis() - tim_net < 500) { delay(3); text("getting PID values", ); }
//            if(c.available() > 0) ;
           //if (millis() - lastsend > 50){    print( +  millis()/1000 + "  " ); delay(6);

          }
  }
  else
  {
  pitch  = 90 + trimP ;
  roll   = 90 + trimR ;
  yaw    = 90 + trimY ;
  
  float thr_mul = 1;
  
  //if (al) {arm = true; }
  //if (f) {if(arm) arm = false; else arm = true;}
  if (arm)
  {
  if(w) pitch -=  (Pdef + Pdev); 
  if(s) pitch +=  (Pdef + Pdev);
  if(a) roll  -=  (Rdef + Rdev); 
  if(d) roll  +=  (Rdef + Rdev);
  if(q) yaw   -=  (Ydef + Ydev);
  if(e) yaw   +=  (Ydef + Ydev);
  if (r) thr_mul = 4;    if (ce) thr_mul = 0.25;
  if (t) thro -= 10;  if (thro < 5) thro = 5;
  if (sp){ thro += (1*thr_mul) ; thro = constrain(thro, 20, 700); }
  if (sh){ thro -= (1*thr_mul) ; thro = constrain(thro, 20, 700); }
throttle = thro/4;
  arm_ch = 120;
  background(100,0,0);
  count = 0; disarmed = false;
  }
  
  else 
  {
//    background(0,0,0);
    pitch =  90 + trimP; roll = 90 + trimR ; yaw = 90 + trimY; 
   throttle = 5 ; thro = 20;
    arm_ch = 60;
    if (count<20) count++;
  }
  
  pitch = constrain(pitch, 30 , 150);  roll = constrain(roll, 30 , 150);  yaw = constrain(yaw, 30 , 150);
   int []ch = {90,90,5,90,90,5};
   ch[2] = throttle; ch[5] = arm_ch;  ch[0] = pitch; ch[4] = roll; ch[1] = yaw;
   //channel 5 is arm channel... 
   
   
  for ( int tt = 0 ; tt < 6 ; tt++) ch[tt] = constrain(ch[tt] , 5, 175); 
String mSend1 = "A";
String mSend2 = "A";

     if (Mode == "acc" ) mSend2 = "H" ;
else if (Mode == "gyr" ) mSend2 = "G" ;
else if (Mode == "mot" ) mSend2 = "I" ;
else if (Mode == "ang" ) mSend2 = "J" ;


if (pMode != 0 && pMode != 1 && pMode != 2 && pMode != 3 ) pMode = 0;

     if (pMode == 1 ) mSend1 = "B";
else if (pMode == 2 ) mSend1 = "C";
else if (pMode == 3 ) mSend1 = "D";

  String tosend = "a" + mSend1 + mSend2 + "b" +  ch[0] + "c" +  ch[1] + "d" +  ch[2] + "e" +  ch[3] + "f" +  ch[4] + "g" +  ch[5] + "h\n" ;

  
  if (uplink){
              try { c.output.write(tosend.getBytes()); }//println("\t good\t" + c.active()+ "\t" + millis() ); }
              catch (Exception ee) { println("bad\t" + millis());   c = new Client(this, ip2, port );}
             }
    
    if (count >= 19) disarmed = true;
    delay(3);
  print("   " + tosend +  millis()/1000 + "  " );
   
 }
 
 // String Recieving and processing part
 if(uplink){
   String inn = c.readStringUntil('\n'); if (inn != null) print(inn);
    try {           
         
       if (inn.indexOf("vo") != -1)
          {
           try
            {
             String vo = inn.substring(inn.indexOf("vo") + 2, inn.indexOf('l') );
             vol   = Integer.parseInt(vo);
            }
            catch(Exception ee)
             { vol = 0;}
         }
        
       else if (inn.indexOf("hh") != -1)
           {              
            try
            {
             String vo = inn.substring(inn.indexOf("hh") + 2, inn.indexOf('i') );
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );
             String v3 = inn.substring(inn.indexOf('k') + 1, inn.indexOf('l') );
//             629795 
             vol   = Integer.parseInt(vo);
             hh[0] = Integer.parseInt(v1);
             hh[1] = Integer.parseInt(v2);
             hh[2] = Integer.parseInt(v3);
             
             print(" ac ");
            }
             catch(Exception ee)
             {hh[0] = 0;  hh[1] = 0;  hh[2] = 0; print("h-> " + inn); vol = 0;}

          }
  
         else if (inn.indexOf("gg") != -1)
           {
            try
            {
             String vo = inn.substring(inn.indexOf("gg") + 2, inn.indexOf('i') );              
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );
             String v3 = inn.substring(inn.indexOf('k') + 1, inn.indexOf('l') );
//             629795   
             vol   = Integer.parseInt(vo);
             gg[0] = Integer.parseInt(v1);
             gg[1] = Integer.parseInt(v2);
             gg[2] = Integer.parseInt(v3);
             
             print(" gy ");
             }
             catch(Exception ee)
             {gg[0] = 0;  gg[1] = 0;  gg[2] = 0; print("g-> " + inn); vol = 0;}
           }
  
        else if (inn.indexOf("nn") != -1)
           {
            try
            {   
             String vo = inn.substring(inn.indexOf("nn") + 2, inn.indexOf('i') );
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );
             String v3 = inn.substring(inn.indexOf('k') + 1, inn.indexOf('l') );
             String v4 = inn.substring(inn.indexOf('l') + 1, inn.indexOf('m') );

//             629795   
             vol   = Integer.parseInt(vo);
             mo[0] = Integer.parseInt(v1);
             mo[1] = Integer.parseInt(v2);
             mo[2] = Integer.parseInt(v3);
             mo[3] = Integer.parseInt(v4);
             
             print(" mot ");
            }
             catch(Exception ee)
             {mo[0] = 0;  mo[1] = 0;  mo[2] = 0;  mo[3] = 0; print("mo-> " + inn); vol = 0; }
           }
   
        else if (inn.indexOf("pr") != -1)
           {
            try
            {
             String vo = inn.substring(inn.indexOf("pr") + 2, inn.indexOf('i') );              
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );

//             629795   
             vol   = Integer.parseInt(vo);
             rp[0] = Integer.parseInt(v1);
             rp[1] = Integer.parseInt(v2);
             //double r1 = (double)rp[0] /
             print(" ang ");
            }
             catch(Exception ee)
             {rp[0] = 0;  rp[1] = 0; print("pr-> " + inn); vol = 0; }
           }
           
   //Calib wala        
         if (inn.indexOf("cu") != -1)
          {
           try
           {
             String vo = inn.substring(inn.indexOf("vv") + 2, inn.indexOf('i') );
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );
             String v3 = inn.substring(inn.indexOf('k') + 1, inn.indexOf('l') );
             
             vol     = Integer.parseInt(vo);
             aval[0] = Integer.parseInt(v1);
             aval[1] = Integer.parseInt(v2);
             aval[2] = Integer.parseInt(v3);
             
             print("  1  ");
            }
             catch(Exception ee)
             {aval[0] = 0;  aval[1] = 0;  aval[2] = 0; print("1-> " + inn); vol = 0;} 
           }

        else  if (inn.indexOf("ma") != -1)
           {
            try
            {
             String vo = inn.substring(inn.indexOf("vv") + 2, inn.indexOf('i') );
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );
             String v3 = inn.substring(inn.indexOf('k') + 1, inn.indexOf('l') );
             
             vol     = Integer.parseInt(vo);
             amax[0] = Integer.parseInt(v1);
             amax[1] = Integer.parseInt(v2);
             amax[2] = Integer.parseInt(v3);
             
             print("  2  ");
            }
             catch(Exception ee)
             {aval[0] = 0;  aval[1] = 0;  aval[2] = 0; print("2-> " + inn); vol = 0;}
           }

       else if (inn.indexOf("mm") != -1)
           {
            try
            { 
             String vo = inn.substring(inn.indexOf("vv") + 2, inn.indexOf('i') );
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );
             String v3 = inn.substring(inn.indexOf('k') + 1, inn.indexOf('l') );
             
             vol     = Integer.parseInt(vo);
             amin[0] = Integer.parseInt(v1);
             amin[1] = Integer.parseInt(v2);
             amin[2] = Integer.parseInt(v3);
             
             print("  3  ");
            }
             catch(Exception ee)
             {aval[0] = 0;  aval[1] = 0;  aval[2] = 0; print("3-> " + inn); vol = 0;}
           }
       else if (inn.indexOf("oo") != -1)
           {
            try
            {
             String vo = inn.substring(inn.indexOf("vv") + 2, inn.indexOf('i') );
             String v1 = inn.substring(inn.indexOf('i') + 1, inn.indexOf('j') );
             String v2 = inn.substring(inn.indexOf('j') + 1, inn.indexOf('k') );
             String v3 = inn.substring(inn.indexOf('k') + 1, inn.indexOf('l') );
             
             vol     = Integer.parseInt(vo);
             aoff[0] = Integer.parseInt(v1);
             aoff[1] = Integer.parseInt(v2);
             aoff[2] = Integer.parseInt(v3);
             
              print("  4  ");
             }
             catch(Exception ee)
             {aoff[0] = 0;  aoff[1] = 0;  aoff[2] = 0; print("4-> " +inn); vol = 0;}
            }
            
       else if (inn.indexOf("Kq") != -1)
           {
 try 
     {
      String ab = inn;

      String bb = ab.substring(ab.indexOf('q') + 1 , ab.indexOf('r') );
      String cc = ab.substring(ab.indexOf('r') + 1 , ab.indexOf('s') );
      String dd = ab.substring(ab.indexOf('s') + 1 , ab.indexOf('t') );

      String ee = ab.substring(ab.indexOf('t') + 1 , ab.indexOf('u') );
      String ff = ab.substring(ab.indexOf('u') + 1 , ab.indexOf('v') );
      String gg = ab.substring(ab.indexOf('v') + 1 , ab.indexOf('w') );
              
      String hh = ab.substring(ab.indexOf('w') + 1 , ab.indexOf('x') );
      String ii = ab.substring(ab.indexOf('x') + 1 , ab.indexOf('y') );
      String jj = ab.substring(ab.indexOf('y') + 1 , ab.indexOf('z') );


      gkpP = Integer.parseInt(bb);   gkiP = Integer.parseInt(cc);   gkdP = Integer.parseInt(dd);
      gkpR = Integer.parseInt(ee);   gkiR = Integer.parseInt(ff);   gkdR = Integer.parseInt(gg);
      gkpY = Integer.parseInt(hh);   gkiY = Integer.parseInt(ii);   gkdY = Integer.parseInt(jj);
   }
catch (Exception e)
   {
      gkpP = 0 ; gkiP = 0 ; gkdP = 0 ;
      gkpR = 0 ; gkiR = 0 ; gkdR = 0 ;
      gkpY = 0 ; gkiY = 0 ; gkdY = 0 ;
   }

              dkpP = kpP*4 ; dkiP = kiP*4 ; dkdP = kdP*4 ;
              dkpR = kpR*4 ; dkiR = kiR*4 ; dkdR = kdR*4 ;
              dkpY = kpY*4 ; dkiY = kiY*4 ; dkdY = kdY*4 ;           

           }
   
 } catch(Exception ee)
             {  }// print("ex-> " + inn);}
  } // if uplink
 
 
 
 
 
 
 
 
 
 
 
 
  // Display -
  
  //left vertical slider   -   pitch control indicator
  //horizontal  slider     -   roll control indicator
  //curved slider          -   yaw control indicator
  //right vertical slider  -   pitch control indicator
  
  

  
  stroke(255,255,255);strokeWeight(2); 
  noFill();arc(135,460,90,90,-PI,0);
  //translate(250, 415); rotate(-yaw*PI/180);
  fill(255,255,255);ellipse(135 + 45*(sin((yaw-90)*PI/180) ), 460 - 45*abs(sin((yaw)*PI/180 ) ),5,10);
  //rotate(+yaw*PI/180);  translate(0, 0);

  line(40, 460, 40, 370); ellipse(40, 415 +pitch/2-45, 10,5);
  line(90, 370, 180, 370); ellipse(135 +roll/2-45 , 370, 5,10);
  line(600, 460, 600, 370); ellipse(600, 415 -throttle/2+45, 10,5);
  
  text("t=" + trimP, 5, 415); text("t=" + trimY, 96, 470); text("t=" + trimR, 93, 360); 
  text("s=" + Pdev/3, 5, 440); text("s=" + Ydev/3, 140, 470); text("s=" + Rdev/3, 147, 360); 
  text(pitch, 35, 360); text(roll, 200, 375); text(yaw, 125, 445);
  text(throttle, 595, 360);
  
  text( "kpP", 500, 425 ); text( "kiP", 527, 425 ); text( "kdP", 550, 425 );
  text( "kpR", 500, 405 ); text( "kiR", 527, 405 ); text( "kdR", 550, 405 );
  text( "kpY", 500, 385 ); text( "kiY", 527, 385 ); text( "kdY", 550, 385 );
  text( kpP, 386, 425 ); text( kiP, 423, 425 ); text( kdP, 460, 425 );
  text( kpR, 386, 405 ); text( kiR, 423, 405 ); text( kdR, 460, 405 );
  text( kpY, 386, 385 ); text( kiY, 423, 385 ); text( kdY, 460, 385 );
  
  if (p)
    {  
    text( gkpP, 386, 325 ); text( gkiP, 423, 325 ); text( gkdP, 460, 325 );
    text( gkpR, 386, 305 ); text( gkiR, 423, 305 ); text( gkdR, 460, 305 );
    text( gkpY, 386, 285 ); text( gkiY, 423, 285 ); text( gkdY, 460, 285 );  
    }
  
if (Mode == "gyr") { text((int)(gg[0]), 80, 175); text((int)(gg[1]), 150, 175); text((int)(gg[2]), 220, 175);  text("gyro", 290, 175);text("/ : mot " ,320, 450);}
if (Mode == "acc") { text((int)(hh[0]), 80, 175); text((int)(hh[1]), 150, 175); text((int)(hh[2]), 220, 175);  text("acco", 290, 175);text("/ : gyr " ,320, 450);}
if (Mode == "ang") { text(((float)rp[0]/100), 80, 175); text(((float)rp[1]/100), 150, 175); text("angl", 220, 175);text("/ : nor " ,320, 450);}
if (Mode == "mot") { fill(255,255,255);
    
            int []la_mot = {0,0,0,0};
for (int aq = 0; aq<30; aq++)
 {
    for (int bq = 0; bq<4; bq++)
      {
         la_mot[bq] += la_mo[aq][bq];      
      }
 }            
             fill(255,255,255);
             if ((la_mot[3]/30) < mo[3]) fill(255,255,0); else if ((la_mot[3]/30) > mo[3]) fill(0,255,255); text((int)(mo[3]), 80 , 175); fill(255,255,255);
             if ((la_mot[0]/30) < mo[0]) fill(255,255,0); else if ((la_mot[0]/30) > mo[0]) fill(0,255,255); text((int)(mo[0]), 150, 175); fill(255,255,255);
             if ((la_mot[2]/30) < mo[2]) fill(255,255,0); else if ((la_mot[2]/30) > mo[2]) fill(0,255,255); text((int)(mo[2]), 80 , 225); fill(255,255,255);
             if ((la_mot[1]/30) < mo[1]) fill(255,255,0); else if ((la_mot[1]/30) > mo[1]) fill(0,255,255); text((int)(mo[1]), 150, 225); fill(255,255,255);
             text("mot", 220, 200);text("/ : ang " ,320, 450);
          // above lines are indicator of changing of quad speed - 
          // the color of motor values change according to whether the speed of quad has increased or decreased over an average of last 30 values
for (int aq = 0; aq<30; aq++)
 {
    for (int bq = 0; bq<4; bq++)
      {
         if (aq == 29) {la_mo[aq][bq] = mo[bq];}
         else la_mo[aq][bq] = la_mo[aq+1][bq];      
      }
 }
                   
                   }

if (Mode == "nor") { text("/ : acc" ,320, 450);}


  if (!arm)    text("Disarmed" ,280, 380); else text("Armed",280, 380);
  if(trimMode) text("n : save",245, 450); else text("h : trim",245, 450);
  
  if(!ce && !pidMode)     text("c : calib", 400, 450  );
  if(!pidMode)      text("0 : pid", 500, 450  );
  
   text("m : change pid mode", 240 , 475);
   text("y : save Frame",410, 475);
if(pMode == 0) text("G mode" , 250 , 420 );
if(pMode == 1) text("A mode" , 250 , 420 );
if(pMode == 2) text("G + A " , 250 , 420 );
if(pMode == 3) text("G & A " , 250 , 420 );


//  fill(0,0,0);
  stroke(0,0,0);
  ellipse(135,370,3,7); ellipse(40,415,7,3); ellipse(135,415,3,7);
  
  
  if (rec) {  println("Reconnecting...\t"); delay(5);c.stop();  delay(5); 
try {c = new Client(this, ip2, port );} catch (Exception ee) {;}rec = false;delay(5);}
    
     text("r : * throt", 550, 475  );
     text("V = ", 560 , 20 );
     text(vol   ,590 , 20 );
     
     if (p && Gpid)
        {           

         kpP = gkpP;  kiP = gkiP;  kdP = gkdP;
         kpR = gkpR;  kiR = gkiR;  kdR = gkdR;
         kpY = gkpY;  kiY = gkiY;  kdY = gkdY;
          
         dkpP = kpP*4 ; dkiP = kiP*4 ; dkdP = kdP*4 ;
         dkpR = kpR*4 ; dkiR = kiR*4 ; dkdR = kdR*4 ;
         dkpY = kpY*4 ; dkiY = kiY*4 ; dkdY = kdY*4 ; 
         p = false; Gpid = false; 
         
           String pidp = (kpP) + "r" + (kiP) + "s" + (kdP) + "t"  ;  
           String pidr = (kpR) + "u" + (kiR) + "v" + (kdR) + "w"  ;  
           String pidy = (kpY) + "x" + (kiY) + "y" + (kdY) + "z"  ;
           String []ppp = {"q" + pidp, "t" + pidr, "w" + pidy};
  
            saveStrings("pid_values.txt", ppp); 
        }
     
  if (y) 
   {
     String da = year() + "-" + month() + "-" + day() + "-" + hour() +":" + minute() + ":" + second() ;  
     saveFrame("output/" + da + ".png"); y = false; println("recorded at " + (millis()/1000)) ; y_tim = millis();
   }
if (millis() - y_tim < 1500 && y_tim > 0 ) text("saved..", 280 ,400);
  
  // aAb180b180d180e180f180g180h
  
  /*
   * throttle (R Slider)= ch[2] ; arm(L Slider) = ch[5] ;
   * pitch (Left Y) = ch[0] ; roll ( Left X ) = ch[4] ;
   * yaw ( Right X ) = ch[1] ; nil ( Right Y ) = ch[3] ;
   */
      
  //    ee.printStackTrace();
  // maybe remove break in switch : enable multiple at once
  
  //println((yaw-90) + "\t" + (pitch-90) + "\t" + (roll-90) + "\t" + (throttle) );   //+ "\t"  + 50*sin((yaw - 90)*PI/180) +  "\t"  + sin(45*PI/180)  + "\t"  + sin(90*PI/180)   );
  }
  
  void keyPressed()
  {
    
   /*
   if (keyCode == CONTROL) { ct = true; println("ct"); }
    else if (keyCode == SHIFT){sh = true; println("sh");}
      else if (keyCode == ALT){al = true; println("sl");}
  */
      switch (keyCode)
  {
    
   case LEFT  : {le = true;} break; case RIGHT : {ri = true;} break;
   case UP    : {up = true;} break; case DOWN  : {dn = true;} break;
   case ALT   : {al = true;} break; case SHIFT : {sh = true;} break;
   
   case CONTROL : {ct = true;} break; 
  }

  
     switch (key)
     {
      case 'w' : { w = true;} break;    case 'W' : { w = true;} break;
      case 's' : { s = true;} break;    case 'S' : { s = true;} break;      
      case 'a' : { a = true;} break;    case 'A' : { a = true;} break;
      case 'd' : { d = true;} break;    case 'D' : { d = true;} break;
      case 'q' : { q = true;} break;    case 'Q' : { q = true;} break;
      case 'e' : { e = true;} break;    case 'E' : { e = true;} break;
      case ' ' : {sp = true;}break;     case '`' : {rec = true;}break;
      case 'r' : { r = true;} break;    case 'R' : { r = true;} break;
      case 't' : { t = true;} break;    case 'T' : { t = true;} break;
      
      case 'm' : { pMode++; if(pMode > 3 || pMode < 0) pMode = 0; } break;   
      case 'M' : { pMode++; if(pMode > 3 || pMode < 0) pMode = 0; } break;
    
      case 'v' : { v = true;} break;    case 'V' : { v = true;} break;
      case 'c' : {if(!ce) ce = true; else ce = false;} break;   
      case 'C' : {if(!ce) ce = true; else ce = false;} break;

      case 'p' : { if (!p) p = true; else p = false;} break;
      case 'P' : { if (!p) p = true; else p = false;} break;
      case '=' : { Gpid = true;} break;    

      case 'f' : { f = true; if(arm) arm = false; else arm = true;} break;    
      case 'F' : { f = true; if(arm) arm = false; else arm = true;} break;
  
      case 'h' : { if(trimMode) trimMode = false; else trimMode = true;} break;   
      case 'H' : { if(trimMode) trimMode = false; else trimMode = true;} break;
      case 'n' : { trimSave = true;} break; case 'N' : { trimSave = true;} break;
      case '/' : { if (Mode == "nor") Mode = "acc"; else if (Mode == "acc") Mode = "gyr";
              else if (Mode == "gyr") Mode = "mot"; else if (Mode == "mot") Mode = "ang";
              else if (Mode == "ang") Mode = "nor"; else Mode = "nor";} break;
      
      case 'i' : { i = true;} break;    case 'I' : { i = true;} break;
      case 'k' : { k = true;} break;    case 'K' : { k = true;} break;      
      case 'j' : { j = true;} break;    case 'J' : { j = true;} break;
      case 'l' : { l = true;} break;    case 'L' : { l = true;} break;
      case 'u' : { u = true;} break;    case 'U' : { u = true;} break;
      case 'o' : { o = true;} break;    case 'O' : { o = true;} break;
  

    case '0' : {num[0] = true; if (pidMode) pidMode = false; else pidMode = true; } break;    
    case '1' : {num[1] = true;} break;
    case '2' : {num[2] = true;} break;    case '3' : {num[3] = true;} break;
    case '4' : {num[4] = true;} break;    case '5' : {num[5] = true;} break;
    case '6' : {num[6] = true;} break;    case '7' : {num[7] = true;} break;
    case '8' : {num[8] = true;} break;    case '9' : {num[9] = true;} break;
    
    case '+' : {pl  = true;} break;    case '-' : {mi = true;} break;    
    case '.' : {dot = true;} break;
    case 'y' : {y   = true;} break;    case 'Y' : {y   = true;} break;
   }
  
  
}

void keyReleased()
{

         switch (keyCode)
{
  
 case LEFT  : {le = false;} break; case RIGHT : {ri = false;} break;
 case DOWN  : {dn = false;} break; case UP    : {up = false;} break;
 case ALT   : {al = false;} break; case SHIFT : {sh = false;} break;
 
 case CONTROL : {ct = false;} break; 
}
     
     
     
     switch (key)
   {
    case 'w' : { w = false;} break;    case 'W' : { w = false;} break;
    case 's' : { s = false;} break;    case 'S' : { s = false;} break;      
    case 'a' : { a = false;} break;    case 'A' : { a = false;} break;
    case 'd' : { d = false;} break;    case 'D' : { d = false;} break;
    case 'q' : { q = false;} break;    case 'Q' : { q = false;} break;
    case 'e' : { e = false;} break;    case 'E' : { e = false;} break;
    case ' ' : {sp = false;} break;
    case 'f' : { f = false;} break;    case 'F' : { f = false;} break;
    case 'r' : { r = false;} break;    case 'R' : { r = false;} break;
    case 't' : { t = false;} break;    case 'T' : { t = false;} break;
    case 'v' : { v = false;} break;    case 'V' : { v = false;} break;
    case '=' : { Gpid = false;} break;   

//    case 'c' : {ce = false;} break;    case 'C' : {ce = false;} break;



    case '0' : {num[0] = false;} break;    
    case '1' : {num[1] = false;} break;
    case '2' : {num[2] = false;} break;    case '3' : {num[3] = false;} break;
    case '4' : {num[4] = false;} break;    case '5' : {num[5] = false;} break;
    case '6' : {num[6] = false;} break;    case '7' : {num[7] = false;} break;
    case '8' : {num[8] = false;} break;    case '9' : {num[9] = false;} break;
    
    case '+' : { pl = false;} break;    case '-' : {mi = false;} break;
    case '.' : {dot = false;} break;
    case 'y' : {y   = false;} break;    case 'Y' : {y   = false;} break;


   }
}




//if (ct) {dev = 40;}
//  if (sh) {dev = 80;}
//  if (al) {dev = 120;}
  
//if (w) {aa =  bb + dev + dev2; ground(aa,aa,0);}
//if (s) {aa =  bb - dev + dev2; background(aa,0,aa);}

//  println(aa + "\t" + dev2);
//  if (pl) println("plus");
//  if (mi) println("minus");
//  boolean capsLocked = getLockingKeyState(KeyEvent.CAPS_LOCK);






 /*
   if (key == 'w' || key == 'W') {  w = true;  }
   if (key == 's' || key == 'S') {  s = true;  } 
   if (key == 'a' || key == 'S') {  s = true;  }
   if (key == 'd' || key == 'S') {  s = true;  }
  
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   if (key == 's' || key == 'S') {  s = true;  }
   */
   
  /*
   if (keyCode == CONTROL) { ct = false; println("ct");}
  else if (keyCode == SHIFT){sh = false; println("sh");}
  else if (keyCode == ALT){al = false; println("al");}
  /* 
 if (key == 'w' || key == 'W') {  w = false;  }  
 if (key == 's' || key == 'S') {  s = false;  } 
 */
       //  dev2 = ( (num1[5])*5 + (num1[6]*5) + (num1[7]*5)+ (num1[8]*5) + (num1[9]*5) ) - 
 //        ( (num1[0]*5) + (num1[1]*5) + (num1[2]*5)+ (num1[3]*5) + (num1[4]*5) ) ;