//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // hapus // untuk mendapatkan banyak serial output, tetapi resiko proteksi juga berkurang
#define VERSION               (2)  // banyak versi yang digunakan untuk mengtranslasi
#define BAUD                  (115200)  // Kecepatan Komunikasi Arduino
#define MAX_BUFFERFER         (100)  // Jumlah pesan yang dapat arduino simpan
#define STEPS_PER_PUTARAN     (200)  // banyak step untuk 1 putaran penuh (cek model motor stepper)
#define STEPS_PER_MM          (STEPS_PER_PUTARAN*16/2)  // Leadscrew digunakan M8 jadi (400*16)/1.25
#define MAX_KEC_stepper       (1000000)// kecepatan maksimum untuk stepper untuk motor stepper
#define MIN_KEC_stepper       (1)//kecepatan minimal untuk stepper motor stepper
#define NUM_AXIS              (3)// Jumlah Axis/Sumbu yang digunakan
#define SPINDLE               (1) // Spindle yang dipasang
#include "HardwareSerial.h"
#include <EEPROM.h>

//---------------------------------------------------------------------------------
// Struktur   
//for line()
typedef struct {  
  long delta;  
  long absdelta;
  long over; 
} Axis;

typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;

typedef struct{
  int kecepatan;
} Spindle;

//------------------------------------------------------------------------------
Axis a[NUM_AXIS];  // untuk garis()
Axis atemp;
Motor motors[NUM_AXIS];
Spindle kecepatan;

char serialBuffer[MAX_BUFFERFER]; // Buffer tempat penyimpanan pesan 
int sofar;  // banyak jumlah karakter dalam buffer

// kecepatan 
float fr=0;   // fr : feedrate / jumlah stepper untuk bahasa manusia
long step_delay;  //untuk pembacaan mesin
float posx,posy,posz,pe;  // posisi untuk tiap axis

// konfigurasi 
char mode_abs=1;  // mode pergerakan Absolut (lihat perintah dalam G-Code)
long line_Angka=0;

//------------------------------------------------------------------------------
// Metode
// waktu yang dibutuhkan untuk menunggu
void jeda(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  
}

//perintah untuk perhitungan  kecepatan stepper pada motor stepper
void kecepatan_stepper(float nfr) {
  if(fr==nfr) return;  
  if(nfr>MAX_KEC_stepper || nfr<MIN_KEC_stepper) { 
   // Serial.print(F("Kecepatan stepper Harus Lebih dari "));
 //   Serial.print(MIN_KEC_stepper);
//    Serial.print(F("Langkah Lebih dari dan Kurang dari "));
//    Serial.print(MAX_KEC_stepper);
 //   Serial.println(F("Langkah."));
    return;
  }
  step_delay = MAX_KEC_stepper/nfr;
  fr=nfr;
}

void position(float nposx,float nposy,float nposz,float npe) {
  posx=nposx;
  posy=nposy;
  posz=nposz;
}

void onestep(int motor) {
#ifdef VERBOSE
  char *letter="XYZ";
  Serial.print(letter[]);
#endif  
  digitalWrite(motors[motor].step_pin,HIGH);
  digitalWrite(motors[motor].step_pin,LOW);
}
//------------------------------------------------------------------------------------
//penggunaan algoritma garis bresenham untuk membuat step pergerakan dua motor
void line(float xbaru,float ybaru,float zbaru,float newe) {
  a[0].delta = (xbaru-posx)*STEPS_PER_MM;
  a[1].delta = (ybaru-posy)*STEPS_PER_MM;
  a[2].delta = (zbaru-posz)*STEPS_PER_MM;
  
  long i,j,maxsteps=0;

  for(i=0;i<NUM_AXIS;++i) {
    a[i].absdelta = abs(a[i].delta);
    if( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;
    digitalWrite(motors[i].dir_pin,a[i].delta>0?HIGH:LOW);
  }
  for(i=0;i<NUM_AXIS;++i) {
    a[i].over= maxsteps/2;
  } 
  long dt = MAX_KEC_stepper/8000;
  long aksel = 1;
  long langkah_utk_aksel = dt - step_delay;
  if(langkah_utk_aksel > maxsteps/2 ) 
    langkah_utk_aksel = maxsteps/2; 
  long langkah_utk_lambat = maxsteps - langkah_utk_aksel;
 // Serial.print("MULAI ");
 // Serial.println(dt);
//  Serial.print("ATAS ");
//  Serial.println(step_delay);
  
 // Serial.print("laju sampai ");
//  Serial.println(langkah_utk_aksel);  
//  Serial.print("lambat setelah ");
//  Serial.println(langkah_utk_lambat);  
 // Serial.print("total ");
 // Serial.println(maxsteps);  
#ifdef VERBOSE
  //Serial.println(F("Start >"));
#endif

  for( i=0; i<maxsteps; ++i ) {
    for(j=0;j<NUM_AXIS;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= maxsteps) {
        a[j].over -= maxsteps;    
        digitalWrite(motors[j].step_pin,HIGH);
        digitalWrite(motors[j].step_pin,LOW);
      }
    }
    if(i<langkah_utk_aksel) {
      dt -= aksel;
    }
    if(i>=langkah_utk_lambat) {
      dt += aksel;
    }
    delayMicroseconds(dt);
  }
  
#ifdef VERBOSE
  //Serial.println(F("< Selesai."));
#endif
  position(xbaru,ybaru,zbaru,newe);
  dimana();
}
//------------------------------------------------------------------------------------
// sudut kembali dx/dy dari 0..2PI
static float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}

/**
 * pencarian untuk karaker /code/ pada serial buffer lalu membaca float untuk kelanjutannya
 * 
 */
  float parseAngka(char kode,float val) {
    char *petunjuk=serialBuffer;  // dimulai pada serialBuffer
    while((long)petunjuk > 1 && (*petunjuk) && (long)petunjuk < (long)serialBuffer+sofar) { 
      if(*petunjuk==kode) {  //jika char pada petunjuk = kode yang dicari
        return atof(petunjuk+1);  // mengisi fungsi parseAngka dengan merubah alpha numerical character ke floating number 
      }
      petunjuk=strchr(petunjuk,' ')+1; // akan mencari char pada spasi selanjutnya, jika char pada posisi petunjuk tidak = kode yang dicari  
    }
    return val;  // pada akhiran tidak ditemukan sesuatu, akan balik ke val bawaan
  }
/**
 * penulisan string diikuti dengan float pada serial l  ine
 */
void output(const char *kode,float val) {
  Serial.print(kode);
  Serial.println(val);
}
/**
 * perintah untuk petunjuk posisi axis, kecepatan stepper, mode absolut dan incremental.
 */
void dimana() {
  output("X",posx);
  output("Y",posy);
  output("Z",posz);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 
/**
 * Info bantuan yang ditampilkan pada Serial Monitor
 */
void bantuan() {
  Serial.print(F("CNC Router Rizqy "));
  Serial.println(VERSION);
  Serial.println(F("List Perintah:"));
  Serial.println(F("G00/G01 [X/Y/Z/E(Langkah)] [F(kecepatan stepper)]; - gerakan lurus"));
  Serial.println(F("G04 P[detik]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/E(steps)]; - merubah posisi logika"));
  Serial.println(F("M3 Untuk Menyalakan Spindle"));
  Serial.println(F("M5 Untuk Mematikan Spindle"));
  Serial.println(F("M18; - mematikan semua motor"));
  Serial.println(F("M100; - memunculkan pesan bantuan"));
  Serial.println(F("M114; - memunculkan posisi axis dan kecepatan stepper"));
  Serial.println(F("S [1-10] Untuk Menyesuaikan Kecepatan Spindle"));
  Serial.println(F("Semua perintah harus diakhiri dengan pembuatan baris baru."));
}
/**
 *  Proses Case pada setiap Perintah
 *  setiap Case memiliki Char Awal yang berbeda walau angka case sama, untuk menghindari tabrakan pembacaan perintah
 */
void processCommand() { // Perintah yang dapat digunakan
  int prh = parseAngka('G',-1);
  switch(prh) {
  case  0: // Gerakan Cepat pada Axis
  case  1: { // perintah gerak garis
    kecepatan_stepper(parseAngka('F',fr));
    line( parseAngka('X',(mode_abs?posx:0)) + (mode_abs?0:posx),
          parseAngka('Y',(mode_abs?posy:0)) + (mode_abs?0:posy),
          parseAngka('Z',(mode_abs?posz:0)) + (mode_abs?0:posz),
          parseAngka('E',(mode_abs?pe:0)) + (mode_abs?0:pe) );
    break;
    }
  case  2:
  case  4:  jeda(parseAngka('P',0)*1000);  break;  
  case 90:  mode_abs=1;  break;  //mode absolut untuk gerakan axis
  case 91:  mode_abs=0;  break;   //mode relatif untuk gerakan axis
  case 92:  //set point posisi logika
    position( parseAngka('X',0),
              parseAngka('Y',0),
              parseAngka('Z',0),
              parseAngka('E',0) );
    break;
  default:  break;
  }
  
  prh = parseAngka('M',-1);
  switch(prh) {
  case   5:  spindle_disable(); break; // Mematikan Motor Spindle
  case  17:  motor_enable();  break; //menyalakan semua motor kembali
  case  18:  motor_disable();  break; //mematikan seluruh motor
  case 100:  bantuan();  break;
  case 114:  dimana();  break;
  default:  break;
  }
  prh = parseAngka('S',-1);
  switch(prh){
  case   1: spindle_1(); break; 
  case   2: spindle_2(); break;
  case   3: spindle_3(); break;
  case   4: spindle_4(); break;
  case   5: spindle_5(); break;
  case   6: spindle_6(); break;
  case   7: spindle_7(); break;
  case   8: spindle_8(); break;
  case   9: spindle_9(); break;
  case  10: spindle_10(); break;
  }
}

void ready() {
  sofar=0;  //menghapus isi serialBuffer
  Serial.print(F(">"));  //perintah menandakan sudah siap menerima input
}


//------------------------------------------------------------------------------------
/* *
 * Jalur Pin yang dipakai, lihat Jalur pcb wiring RAMPS 1.4 untuk info lebih jelas
 */
void motor_setup() {
//Pin Motor Axis X
  motors[0].step_pin=54;
  motors[0].dir_pin=55;
  motors[0].enable_pin=38;
  motors[0].limit_switch_pin=3;
//Pin Motor Axis Y
  motors[1].step_pin=60;
  motors[1].dir_pin=61;
  motors[1].enable_pin=56;
  motors[1].limit_switch_pin=14;
//Pin Motor Axis Z
  motors[2].step_pin=46;
  motors[2].dir_pin=48;
  motors[2].enable_pin=62;
  motors[2].limit_switch_pin=18;

  int i;
  for(i=0;i<NUM_AXIS;++i) {     
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);
  }
}


void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIS;++i) {  
    digitalWrite(motors[i].enable_pin,LOW);
  }
}


void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIS;++i) {  
    digitalWrite(motors[i].enable_pin,HIGH);
  }
}

void endstop(){
  int lswitch_x = 3;
  int lswitch_y = 14;
  int lswitch_z = 34;
  
  pinMode(lswitch_x, INPUT);
  pinMode(lswitch_y, INPUT);
  pinMode(lswitch_z, INPUT);
}

int spindle_pin = 9;
void spindle_1(){
    int kecepatan  = 23;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_2(){
    int kecepatan  = 45;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_3(){
    int kecepatan  = 68;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_4(){
    int kecepatan  = 90;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_5(){
    int kecepatan  = 113;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_6(){
    int kecepatan  = 135;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_7(){
    int kecepatan  = 158;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_8(){
    int kecepatan  = 180;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_9(){
    int kecepatan  = 203;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}
void spindle_10(){
    int kecepatan  = 225;
    analogWrite(spindle_pin, kecepatan);
    delayMicroseconds(1);
}

void spindle_disable(){
  analogWrite(spindle_pin, LOW);
  delayMicroseconds(2);
}


void setup() {
  Serial.begin(BAUD); // kecepatan komunikasi (lihat pada #define)

  motor_setup();
  motor_enable();
  spindle_disable();
  endstop(); 
  dimana();  // untuk debugging 
  bantuan(); // Say Hello to My Little friend
  position(0,0,0,0);  // set posisi koordinat mulai
  kecepatan_stepper(3000); // kecepatan dasar pada setiap motor stepper
  ready();
}



void loop() {
   //standby untuk menyimak perintah pada serial command
  while(Serial.available() > 0) {
  //  delay (20);
    char c=Serial.read();  
    Serial.print(c);  
    if(sofar<MAX_BUFFERFER-1) serialBuffer[sofar++]=c;
    if(c=='\n') {
         //seluruh pesan sudah diterima buffer
      serialBuffer[sofar]=0;
//      Serial.print("Serial.available");
//      Serial.println(Serial.available());
 //     Serial.print("MAX_BUFFERFER");
//      Serial.println(MAX_BUFFERFER);
 //     Serial.print("sofar");
 //     Serial.println(sofar);  
      Serial.print(F("\r\n")); 
      processCommand();  
      Serial.println("@"); 
      ready();
    }
}
}
