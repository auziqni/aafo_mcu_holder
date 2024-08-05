//---------------------------------------------library pemograman/header files
#include <ESP32Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
// teknisee
//-----------------------------------------------------------------------------------------------
#include <WiFi.h>          // import wifi library
#include <HTTPClient.h>    // import http library
#include <ArduinoJson.h>   // import arduino json library
#include <FirebaseESP32.h> // import firebase library

// main static global variable
String norekam = "rokam";
// const char *ssid = "teknisee";
// const char *password = "myteknisee";
const char *ssid = "Gei part 3";
const char *password = "SIMIONE19";

#define FIREBASE_HOST "https://aafo-9b7b2-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "lWOv4jbAHmdJHW3Hi6bvqenkFnOVos0psAeNUCR2"

// main dynamcic global variable
int timersession, timersecond;
String jsonReqPayload = "";
float pitch;
int fsr1value, fsr2value;
bool sessionRead = false;
int timessessionRead = 5, bacaSession = 0;

// global varialbe firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//-----------------------------------------------------------------------------------------------

//-------------------------------------------- deklarasi servo dan pin
Servo myServo;
#define SERVO_PIN 2     // definisi pin untuk servo
#define INTERRUPT_PIN 5 // defini pin untuk interrupt MPU6050
#define FSR_1_PIN 35    // pin FSR 1
#define FSR_2_PIN 32    // pin FSR 2

//-------------------------------------------- Array Posisi dan Durasi
int poss[] = {0, -5, 0, 5, 0, 20, 10, 0, 0};                    // array yang menyimpan sudut target untuk servo
int duration[] = {150, 150, 150, 350, 250, 150, 150, 150, 150}; // array yang menyimpan durasi perpindahan dari satu sudut ke sudut berikutnya

//-------------------------------------------- MPU6050 offsets
int mpu_offset[6] = {-2778, -838, -1234, 789, 93, -3}; // array yang menyimpan nilai offset untuk mpu

//--------------------------------------------MPU6050 variables
MPU6050 mpu; // membuat objek MPU

bool dmpReady = false;  // menyimpan status kesiapan DMP
uint8_t mpuIntStatus;   // menyimpan status interrupt MPU6050
uint8_t devStatus;      // menyimpan status inisialisasi
uint16_t packetSize;    // menyimpan ukuran paket FIFO DMP
uint16_t fifoCount;     // menyimpan jumlah data dalam FIFO
uint8_t fifoBuffer[64]; // Buffer untuk menyimpan data FIFO

Quaternion q;                       // menyimpan data quartenion dari MPU
VectorFloat gravity;                // menyimpan vektor gravitasi dari MPU
volatile bool mpuInterrupt = false; // Flag untuk interrupt MPU
float ypr[3];                       // menyimpan nilai yaw, pitch, dan roll

// teknisee function
//-----------------------------------------------------------------------------------------------

void reconnectWifi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("WIFI NOT CONNECTED");
        Serial.print("Connecting to ");
        Serial.println(ssid);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
    }
}

void initWifi()
{
    WiFi.begin(ssid, password);

    reconnectWifi();

    Serial.println("");
    Serial.print("wifi connected. ");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("");
}

void initFirebase()
{
    config.database_url = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    Firebase.begin(&config, &auth);
}

bool writeToFirebase(const char *path, const String &value, const String &typedata)
{
    bool success = false;

    if (typedata == "int")
    {
        success = Firebase.setInt(fbdo, path, value.toInt());
    }
    else if (typedata == "float")
    {
        success = Firebase.setFloat(fbdo, path, value.toFloat());
    }
    else if (typedata == "bool")
    {
        success = Firebase.setBool(fbdo, path, value.equalsIgnoreCase("true"));
    }
    else if (typedata == "string")
    {
        success = Firebase.setString(fbdo, path, value);
    }
    else
    {
        // Tipe data tidak dikenal
        Serial.println("Tipe data tidak dikenal");
        return false;
    }

    if (!success)
    {
        Serial.println("Gagal menulis data ke Firebase");
        Serial.println(fbdo.errorReason());
        return false;
    }

    return success;
}

void writeFirebase()
{
    if (Firebase.ready())
    {
        writeToFirebase("/data1/beratBelakang", String(fsr2value), "float");
        writeToFirebase("/data1/beratDepan", String(fsr1value), "float");
        writeToFirebase("/data1/sudut", String(pitch), "float");
    }
    else
    {
        Serial.println("Firebase not ready");
    }
}

String generateRandomString()
{
    String result = "";
    String characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

    for (int i = 0; i < 5; i++)
    {
        int randomIndex = random(0, characters.length());
        result += characters[randomIndex];
    }

    return result;
}

void readFirebase()
{
    if (Firebase.ready())
    {
        Firebase.getBool(fbdo, "data1/sessionRead") && (sessionRead = fbdo.boolData());
        if (sessionRead)
        {
            norekam = generateRandomString();
            bacaSession = timessessionRead;
            writeToFirebase("/data1/sessionRead", String(false), "bool");
        }
    }
    else
    {
        Serial.println("Firebase not ready");
    }
}

void data2json()
{
    jsonReqPayload = "";
    StaticJsonDocument<200> D2W;

    D2W["norekam"] = norekam;
    D2W["sudut"] = pitch;
    D2W["beratDepan"] = fsr1value;
    D2W["beratBelakang"] = fsr2value;

    serializeJson(D2W, jsonReqPayload);
    // Serial.println(jsonReqPayload);
}

void send2psql()
{
    HTTPClient http;

    http.begin("https://www.aafo.teknisee.store/api/postpengukuran");
    http.addHeader("Content-Type", "application/json; charset=UTF-8");
    int httpCode = http.POST(jsonReqPayload);

    if (httpCode > 0)
    {
        // file found at server
        if (httpCode == HTTP_CODE_OK)
        {
            String payload = http.getString();
            Serial.println(payload);
        }
        else
        {
            // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTP] GET... code: %d\n", httpCode);

            send2psql();
        }
        delay(3000);
    }
    else
    {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        send2psql();
        delay(3000);
    }

    http.end();
}

// teknisee funtion end
//-----------------------------------------------------------------------------------------------

//------------------------------------------ fungsi interrupt
void dmpDataReady() // fungsi interrupt yang dipanggil saat data DMP siap
{
    mpuInterrupt = true; // mengatur variabel menjadi true
}

// ----------------------------------------- koneksi MPU6050
void MPU_Connect()
{
    Serial.println(F("Initializing I2C devices..."));                                                           // menampilkan pesan di serial monitor bahwa perangkat i2c sedang diinisialisasi
    mpu.initialize();                                                                                           // menginisialisasi MPU6050
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); // menguji koneksi ke MPU dan menampilkan pesan di serial monitor apakah koneksi berhasil atau gagal

    Serial.println(F("Initializing DMP...")); // menampilkan pesan di serial monitoring bahwa DMP sedang diinisialisasi
    devStatus = mpu.dmpInitialize();          // menginisialisasi DMP dan menyimpan status inisialisasi dalam 'devStatus'

    mpu.setXAccelOffset(mpu_offset[0]); // menetapkan offset akselerometer sumbu X dari array 'mpu_offset'
    mpu.setYAccelOffset(mpu_offset[1]); // menetapkan offset akselerometer sumbu Y dari array 'mpu_offset'
    mpu.setZAccelOffset(mpu_offset[2]); // menetapkan offset akselerometer sumbu Z dari array 'mpu_offset'
    mpu.setXGyroOffset(mpu_offset[3]);  // menetapkan offset gyroscope sumbu X dari array 'mpu_offset'
    mpu.setYGyroOffset(mpu_offset[4]);  // menetapkan offset gyroscope sumbu Y dari array 'mpu_offset'
    mpu.setZGyroOffset(mpu_offset[5]);  // menetapkan offset gyroscope sumbu Z dari array 'mpu_offset'

    if (devStatus == 0) // memeriksa apakah inisialisasi DMP berhasil
    {
        mpu.CalibrateAccel(10);   // mengkalibrasi akselerometer dengan 10 sampel
        mpu.CalibrateGyro(10);    // mengkalibrasi gyroscope dengan 10 sampel
        mpu.PrintActiveOffsets(); // mencetak nilai offset aktif ke serial monitor

        Serial.println(F("Enabling DMP...")); // menampilkan pesan di serial monitor bahwa DMP sedang diaktifkan
        mpu.setDMPEnabled(true);              // mengaktifkan DMP

        Serial.print(F("Enabling interrupt detection (Arduino external interrupt ")); // menampilkan pesan di serial monitor bahwa deteksi interrupt sedang diaktifkan
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));                           // menampilkan nomor interrupt yang digunakan oleh INTERRUPT_PIN di serial monitor.
        Serial.println(F(")..."));                                                    // Menampilkan penutupan pesan sebelumnya di serial monitor.
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);  // Mengatur interrupt pada INTERRUPT_PIN sehingga fungsi dmpDataReady dipanggil saat sinyal naik (RISING).
        mpuIntStatus = mpu.getIntStatus();                                            // Mendapatkan status interrupt dari MPU6050.

        Serial.println(F("DMP ready! Waiting for first interrupt...")); // Menampilkan pesan bahwa DMP siap dan menunggu interrupt pertama di serial monitor.
        dmpReady = true;                                                // Mengatur status DMP menjadi siap.

        packetSize = mpu.dmpGetFIFOPacketSize(); // Mendapatkan ukuran paket data dari FIFO buffer.
    }
    else // jika inisialisasi DMP gagal
    {
        Serial.print(F("DMP Initialization failed (code ")); // Menampilkan pesan bahwa inisialisasi DMP gagal di serial monitor.
        Serial.print(devStatus);                             // Menampilkan kode status kesalahan inisialisasi.
        Serial.println(F(")"));                              // Menutup pesan status kesalahan.
    }
}

//-----------------------------------------durasi servo bergerak dari satu posisi ke posisi lain
int getDuration(int duration) // Fungsi untuk mendapatkan durasi perpindahan servo.
{
    if (duration < 3) // Jika durasi kurang dari 3, maka:
    {
        return 1; // Kembalikan 1.
    }
    int increment = 3;                //  Inisialisasi increment dengan nilai 3.
    while (duration % increment != 0) // Selama durasi tidak habis dibagi increment:
    {
        increment += 1; // tambahkan increment dengan 1
    }
    return increment; // kembalikan nilai increment
}

void printData(float servoAngle) // mencetak nilai
{
    Serial.print("fsr1value = ");
    Serial.print(fsr1value);
    Serial.print("\tfsr2value = ");
    Serial.print(fsr2value);
    Serial.print("\tpitch = ");
    Serial.print(pitch);
    Serial.print("\tServoAngle = ");
    Serial.println(90 + (int)servoAngle);
}

//----------------------------------------------- fungsi setup
void setup()
{
    Wire.begin();          // inisialisasi komunikasi I2C
    Wire.setClock(400000); // Mengatur kecepatan komunikasi i2c menjadi 400 kHz
    initWifi();            // it must connect to wifi, unless it will not continue to the next line
    initFirebase();

    Serial.begin(9600); // memulai komunikasi serial dengan kecepatan 9600 bps

    //------------------------------------------------Pasangkan servo ke pin GPIO
    myServo.attach(SERVO_PIN); // menghubungkan servo ke pin 2
    myServo.write(90);         // mengatur posisi awal servo ke 90° (posisi tengah)

    //------------------------------------------------MPU initialization
    while (!dmpReady) // Loop ini terus berjalan sampai dmpReady diatur menjadi true.
    {
        MPU_Connect(); // inisialisasi mpu
    }
}

void loop()
{
    //----------------------------------------- set the initial servo angle
    float servoAngle = poss[0];                              // Mengatur sudut awal servo dengan nilai awal dari array poss.
    for (int i = 0; i < sizeof(poss) / sizeof(poss[0]); i++) // loop akan berjalan dari i = 0 hingga i = 8 (9 iterasi), mengakses setiap elemen dalam array poss.
    {
        // ------------------------------------- set the values for servo angle and position
        int increment = getDuration(duration[i]);                                             // Menghitung jumlah langkah yang dibutuhkan untuk mencapai posisi berikutnya, menggunakan fungsi getDuration(duration[i]).
        int nextPosition = (i != sizeof(poss) / sizeof(poss[0]) - 1) ? poss[i + 1] : poss[0]; // menentukan posisi berikutnya
        float anggelAdd = (float)(nextPosition - poss[i]) / increment;                        // Menghitung selisih antara posisi berikutnya dan posisi saat ini

        //-------------------------------------- move the servo to the next position
        for (int j = 0; j < duration[i]; j += duration[i] / increment) // servo bergerak secara bertahap dari satu posisi ke posisi berikutnya dalam durasi yang ditentukan.
        {
            // --------------------------------- Read the FSR values
            fsr1value = analogRead(FSR_1_PIN);
            fsr2value = analogRead(FSR_2_PIN);

            // --------------------------------- Read the MPU6050 pitch values
            if (!dmpReady)
                return;
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
            {
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                // -------------------------------- pitch from the MPU6050
                pitch = (ypr[1] * 180 / M_PI); // Mengkonversi nilai pitch dari radian ke derajat.
            }

            // ----------------------------------- mengatur sudut servo
            servoAngle += anggelAdd; // Menambahkan nilai anggelAdd ke servoAngle. anggelAdd adalah jumlah kecil perubahan sudut yang dihitung untuk setiap langkah.

            // ------------------------------------ menulis sudut ke servo
            myServo.write(90 + (int)servoAngle); // Menulis sudut baru ke servo. Sudut dasar adalah 90 derajat, dan servoAngle ditambahkan untuk menentukan sudut akhir.
            delay(300);

            if (millis() - timersecond > 1000)
            {
                timersecond = millis();
                writeFirebase();
            }

            if (millis() - timersession > 10000)
            {
                timersession = millis();
                // call function
                readFirebase();
                data2json();

                if (bacaSession > 0)
                {
                    send2psql();
                    bacaSession--;
                }
            }
        }

        printData(servoAngle); // mencetak data sensor dan sudut servo

        // Serial.print("servoAngle = ");
        // Serial.println(90 + (int)servoAngle);

        delay(300);
    }
}
