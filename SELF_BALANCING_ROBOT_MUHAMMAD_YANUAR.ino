/* 
Nama : Muhammad Yanuar
NRP  : 2040221068
*/

#include <PID_v1.h> //library PID
#include <LMotorController.h> // library kontrol motor
#include "I2Cdev.h" // library komunikasi serial 
#include "MPU6050_6Axis_MotionApps20.h" // library mpu6050

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE 
#include "Wire.h"
#endif
#define MIN_ABS_SPEED 20 //nilai minimum untuk menjaga keseimbangan robot

MPU6050 mpu;

// kontrol/status pada MPU
bool dmpReady = false; // menandai apakah DMP (Digital Motion Processor) sudah diinisialisasi dengan sukses atau belum.
uint8_t mpuIntStatus; // menyimpan byte status interupsi aktual dari MPU
uint8_t devStatus; // mengembalikan status setelah setiap operasi perangkat (0 = sukses, !0 = error)
uint16_t packetSize; // menyimpan ukuran paket DMP yang diharapkan (standarnya adalah 42 byte)
uint16_t fifoCount; // hitung semua byte saat ini di FIFO (First-In-First-Out) buffer pada sensor MPU.
uint8_t fifoBuffer[64]; // buffer penyimpanan FIFO

// mengukur data inersia dan menghitung nilai yaw pitch roll
Quaternion q; // [w, x, y, z] format reperentasi rotasi 4 dimensi bisa diubah 3 dimensi
VectorFloat gravity; // [x, y, z] menyimpan data percepatan gravitasi. u/ hitung orientasi absolut
float ypr[3]; // [yaw, pitch, roll] tipe float yang digunakan untuk menyimpan ypr dalam bentuk array

//PID
double originalSetpoint = 180; //menyimpan nilai setpoint awal satuan derajat
double setpoint = originalSetpoint; //menyimpan nilai setpoint yang sedang digunakan
double movingAngleOffset = 0.1; //untuk menyimoan nilai offset sudut,yakni seberapa jauh dari setpointawal (0.1 derajat)
double input, output; //menyimpan nilai input (aktual dari posisi sudut) dan output (untuk mengatur posisi sudut)

/* deklarasi variabel dengan sistem PID sesuaikan nilai-nilai ini agar sesuai dengan desain Anda sendiri*/
double Kp =  60 ; // terlalu sedikit KP akan membuat robot jatuh, jika terlalu tinggi membuat robot berosilasi (koef, proporsional)
double Kd = 2.0 ; // untuk mengurangi osilasi, diatur setelah nilai kp (koef. differensial)
double Ki = 279 ; // nilai nya disesuaikan dengan kp dan kd (koef. integral)
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); //&input alamat variabel PID, direct = arah kontrol (posisi/kecepatan)
double motorSpeedFactorLeft = 1; //ini menunjukan bahwa motor kiri dikontrol dengan kecepatan penuh
double motorSpeedFactorRight = 1; //begitu juga motor kanan (faktor kecepatan motor kanan)

// PENGENDALI MOTOR
int ENA = 5; //pin pwm untuk mengatur kecepatan motor (bisa kiri atau kanan)
int IN1 = 6; //pin untuk mengatur arah putaran motor (ini bisa diatur dari mekaniknya)
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight); 
//motor controller digunakan untuk mengontrol kecepatan dan arah putaran motor menggunakan sinyal PWM

volatile bool mpuInterrupt = false; // digunakan untuk tidak melakukan optimasi pada variabel karena nilainya dapat berubah"
void dmpDataReady() //callback function yang akan dipanggil setelah data dari mpu 6050 siap digunakan
{
mpuInterrupt = true; //ketika data siap digunakan maka fungsi akan menetapkan nilai true
}

void setup()
// inisialisasi interface komunikasi I2C 
{
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE //merupakan preprocessor directive untuk mengecek apakah nilainya sama
 Wire.begin(); //jika kedua nilai sama maka baris ini akan memulai eksekusi dengan library wire dg protokol I2C
 TWBR = 24; // mengatur nilai prescaler untuk I2C,nilai 24 akan menghasilkan kecepatan komunikasi 200kHz(jika CPU 8MHz)
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE //jika nilai tidak sama maka preprocessor akan dibaca kembali
 Fastwire::setup(400, true);//jika nilai I2CImple sama dengan Builtin maka inisiasi dilakukan, akan aktifkan pul up resistor internal pada pin SDA dan SCL
 #endif

 mpu.initialize(); // inisiasi sensor mpu dengan fungsi initialize
 devStatus = mpu.dmpInitialize(); //insiasi mpu6050 dengan DMP (Digital Motion Processing) hasil disimpan di devstatus
 
 // (kalibrasi manual) berikan offset gyro Anda sendiri di sini, diskalakan untuk sensitivitas min
 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1688); // 1688 default untuk chip pengujian saya
  if (devStatus == 0)//variabel ini menyimpan hasil inisialisasi DMP pada MPU6050 jika nilai devstatus = 0 maka perintah dijalankan
 {
  mpu.setDMPEnabled(true);//untuk mengaktifkan fitur DMP pada sensor MPU, true = diaktifkan

 // aktifkan deteksi interupsi Arduino
 attachInterrupt(0, dmpDataReady, RISING);//atur interupsi pada pin D2 dg fungsi attach, dipicu oleh sinyal Rising, maka
   mpuIntStatus = mpu.getIntStatus();
 // setel flag DMP Ready kita sehingga fungsi loop() utama mengetahui bahwa tidak apa-apa untuk menggunakannya
 dmpReady = true; //jika DMP siap diambil maka akselerometer akan dijalankan

// dapatkan ukuran paket DMP yang diharapkan untuk perbandingan nanti
 packetSize = mpu.dmpGetFIFOPacketSize();
 
//menyiapkan PID
 pid.SetMode(AUTOMATIC); //m
 pid.SetSampleTime(10); //Ambil data erorr PID tiap 10 milisecon
 pid.SetOutputLimits(-150, 150);  // atur kecepatan minimum dan maksimum pwm motor
 }
 else
 {
 // KESALAHAN!
 // 1 = pemuatan memori awal gagal
 // 2 = Pembaruan konfigurasi DMP gagal
 // (kalau mau pecah, biasanya kodenya 1)
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}
void loop() //ini adalah fungsi yang berulang dari pembacaan data FIFO
{
 // jika pemrograman gagal, jangan mencoba melakukan apapun
 if (!dmpReady) return;
 // tunggu interupsi MPU atau paket tambahan tersedia
 while (!mpuInterrupt && fifoCount < packetSize)
 {
 //tidak ada data mpu - melakukan kalkulasi PID dan output ke motor 
 pid.Compute(); //untuk membandingkan nilai aktual dan variabel dengan perhitungan erorr
 motorController.move(output, MIN_ABS_SPEED); //gerak motor dg kecepatan berdasarkan nilai pid yang diterima dan akan menjalankan output dengan minimum speed 
 }
 // setel ulang flag interupsi dan dapatkan byte INT_STATUS
 mpuInterrupt = false; //apakah telah terjadi interrupt dari mpu
 mpuIntStatus = mpu.getIntStatus(); //kode ini berfungsi untuk memanggil mpu.getstatus apakah data yang diterima FIFO ada perubahan atau tidak
 // dapatkan hitungan FIFO saat ini
 fifoCount = mpu.getFIFOCount();//;akukan perhitungan FIFO
// periksa luapan (ini tidak boleh terjadi kecuali kode kita terlalu tidak efisien)
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)// operasi pengujian dengan logika or dan periksa apakah fifocount = 1024. jika buffer FIFO telah hampir penuh maka program harus segera membaca  
 {
// atur ulang agar kita dapat melanjutkan dengan bersih
 mpu.resetFIFO(); //jika FIFO overflow maka program mpu harus segera mereset agar program berjalan kembali 
 Serial.println(F("FIFO overflow!")); // program ini menampilkan FIFO OVERFLOW
  }
 else if (mpuIntStatus & 0x02)//kode ini menandakan jika tidak terjadi overflow dan data dari sensor siap dibaca oleh program
 {
 // jika tidak, periksa interupsi siap data DMP (ini harus sering terjadi)
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();//untuk menunggu hingga data yang cukup banyak tersedia di buffer FIFO sebelum data tersebut dibaca dan diproses oleh program. 
 // membaca paket dari FIFO
 mpu.getFIFOBytes(fifoBuffer, packetSize);//membaca data dari FIFO sebanyak packetsize dan menyimpan ke fifobuffer
// lacak hitungan FIFO di sini jika ada > 1 paket tersedia
// (ini memungkinkan kita segera membaca lebih lanjut tanpa menunggu interupsi)
 fifoCount -= packetSize; //mengurangi jumlah data yg tersedia di fifocount dengan ukuran paket data di packetsize
 mpu.dmpGetQuaternion(&q, fifoBuffer); //digunakan untuk menghitung rotasi matematis dan lebih akurat dari program matriks
 mpu.dmpGetGravity(&gravity, &q); //menghitung nilai gravitasi lokal pada sensor MPU-6050 berdasarkan nilai quaternion
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //nilai ini digunakan untuk hitung yaw pitch roll dan grafitasi dari data mpu6050
 input = ypr[1] * 180/M_PI + 180; 
 // Kode ini mengambil nilai Pitch dari hasil perhitungan pada langkah sebelumnya (ypr[1]) dan mengkonversinya ke dalam satuan derajat. Nilai Pitch ini kemudian dijadikan sebagai input bagi program untuk dikalkulasikan menjadi output yang sesuai dengan kebutuhan program.
 }
}