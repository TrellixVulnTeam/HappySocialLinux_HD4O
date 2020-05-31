
#include "Adafruit_BME280.h"
#include <IRremote.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
//#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "time.h"

const String ssid     = "kinositakinenjigyoudan";
const String password = "kinositakinenjigyoudan1975";


// Minimal class to replace std::vector
template<typename Data>
class Vector
{
    size_t d_size; // Stores no. of actually stored objects
    size_t d_capacity; // Stores allocated capacity
    Data *d_data; // Stores data
  public:
    Vector() : d_size(0), d_capacity(0), d_data(0) {}; // Default constructor
    Vector(Vector const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0)
    {
      d_data = (Data *)malloc(d_capacity * sizeof(Data));
      memcpy(d_data, other.d_data, d_size * sizeof(Data));
    }; // Copy constuctor
    ~Vector()
    {
      free(d_data);
    }; // Destructor
    Vector &operator=(Vector const &other)
    {
      free(d_data);

      d_size = other.d_size;
      d_capacity = other.d_capacity;
      d_data = (Data *)malloc(d_capacity * sizeof(Data));
      memcpy(d_data, other.d_data, d_size * sizeof(Data));
      return *this;
    }; // Needed for memory management
    void push_back(Data const &x)
    {
      if (d_capacity == d_size) resize();
      d_data[d_size++] = x;
    }; // Adds new value. If needed, allocates more space
    size_t size() const
    {
      return d_size;
    }; // Size getter
    Data const &operator[](size_t idx) const
    {
      return d_data[idx];
    }; // Const getter
    Data &operator[](size_t idx)
    {
      return d_data[idx];
    }; // Changeable getter
    Data getSum()
    {
      Data sum = 0;
      for (int i = 0; i < size(); i++)
      {
        sum += d_data[i];
      }
      return sum;
    };

    float getMean()
    {
      Data sum = getSum();
      return float(sum) / size();
    };

    float getVar()
    {
      float mean = getMean();
      float var = 0.0;
      for (int i = 0; i < size(); i++)
      {
        var += ((d_data[i] - mean) * (d_data[i] - mean));
      }
      var = var / size();
      return var;
    };
    float getStd()
    {
      float var = getVar();
      return sqrt(var);
    };
  private:
    void resize()
    {
      d_capacity = d_capacity ? d_capacity * 2 : 1;
      Data *newdata = (Data *)malloc(d_capacity * sizeof(Data));
      memcpy(newdata, d_data, d_size * sizeof(Data));
      free(d_data);
      d_data = newdata;
    };// Allocates double the old space
};

class Sensor
{
  public:
    String name;
    virtual float getData() = 0;
    virtual void process() = 0;
    virtual void reset() = 0;
};

class PIRSensor: public Sensor
{
  public:
    int data = 0;
    int pin;
    PIRSensor(int x)
    {
      name = "pir";
      pin = x;
      pinMode(pin, INPUT);
    }
    int readData() {
      return digitalRead(pin);
    }
    void process() {
      int newData = readData();
      if (newData == 1)
      {
        data  = 1;
      }
    }
    void reset()
    {
      data = 0;
    }
    float getData()
    {
      return float(data);
    }
};

class IRSensor: public Sensor
{
  public:
    int data = 0;
    int pin;
    IRrecv *irrecv;
    decode_results results;
    IRSensor(int x)
    {
      name = "ir";
      pin = x;
      init();
    }
    void init() {
      irrecv = new IRrecv(pin);
      irrecv->enableIRIn();//start the receiver
      pinMode(pin, INPUT_PULLUP);
    }
    int readData() {
      if (irrecv->decode(&results))
      {
        irrecv->resume(); //receiver the next value
        return 1;
      }
      return 0;
    }
    void process() {
      int newData = readData();
      if (newData == 1)
      {
        data  = 1;
      }
    }
    void reset()
    {
      data = 0;
    }
    float getData()
    {
      return float(data);
    }
};

class PhotoSensor: public Sensor
{
  public:
    Vector<int> *vec;
    int pin;
    PhotoSensor(int x)
    {
      name = "photo";
      pin = x;
      //      vec = new Vector<int>;
      pinMode(pin, INPUT);
    }
    int readData()
    {
      return analogRead(pin);
    }
    void process()
    {
    }
    void reset()
    {
      //      delete vec;
      //      vec = new Vector<int>;
    }
    float getData()
    {
      return readData();
    }
};

class MicSensor: public Sensor
{
  public:
    Vector<int> *vec;
    int pin;
    MicSensor(int x)
    {
      name = "mic";
      pin = x;
      vec = new Vector<int>;
    }
    int readData()
    {
      return analogRead(pin);
    }
    void process()
    {
      int newData = readData();
      vec->push_back(newData);
    }
    void reset()
    {
      delete vec;
      vec = new Vector<int>;
    }
    float getData()
    {
      return vec->getVar();
    }
};

class BME
{
  public:
    int address;
    Adafruit_BME280 *bmeSensor;
    int sdi;
    int sck;
    BME(int x, int y, int z)
    {
      address = x;
      sdi = y;
      sck = z;
      init();
    }
    void init()
    {
      bmeSensor = new Adafruit_BME280(sdi, sck);
      bool status = bmeSensor->begin(address);
      if (!status)
      {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
      }
    }
    float getTemp()
    {
      return bmeSensor->readTemperature();
    }
    float getHumid()
    {
      return bmeSensor->readHumidity();
    }
    float getPressure()
    {
      return bmeSensor->readPressure() / 100.0F;
    }
};

class TempSensor: public Sensor
{
  public:
    Vector<float> *vec;
    BME *bme;
    TempSensor(BME *_bme)
    {
      name = "temp";
      bme = _bme;
    }
    float readData()
    {
      return bme->getTemp();
    }
    void process()
    {
    }
    void reset()
    {
      //      delete vec;
      //      vec = new Vector<float>;
    }
    float getData()
    {
      return readData();
    }
};

class HumidSensor: public Sensor
{
  public:
    Vector<float> *vec;
    BME *bme;
    HumidSensor(BME *_bme)
    {
      bme = _bme;
      name = "humid";
      //      vec = new Vector<float>;
    }
    float readData()
    {
      return bme->getHumid();
    }
    void process()
    {
    }
    void reset()
    {
      //      delete vec;
      //      vec = new Vector<float>;
    }
    float getData()
    {
      return readData();
    }
};

class PressureSensor: public Sensor
{
  public:
    Vector<float> *vec;
    BME *bme;
    PressureSensor(BME *_bme)
    {
      bme = _bme;
      name = "pressure";
      //      vec = new Vector<float>;
    }
    float readData()
    {
      return bme->getPressure();
    }
    void process()
    {
    }
    void reset()
    {
      //      delete vec;
      //      vec = new Vector<float>;
    }
    float getData()
    {
      return readData();
    }
};

class LED
{
  public:
    int pin;
    LED(int x)
    {
      pin = x;
      init();
    }
    void init()
    {
      ledcSetup(0, 12800, 8);
      ledcAttachPin(pin, 0);
    }
    void turnOff()
    {
      ledcWrite(0, 0);
    }
    void turnOn(int x)
    {
      ledcWrite(0, x);
    }
};

class Device
{
  public:
    String ssid;
    String password;
    bool wifiConnected = false;

    String mqttServer;
    int mqttPort;
    bool mqttConnected = false;

    String mqttUsername;
    String mqttPassword;

    WiFiClientSecure *wifiClient;
    PubSubClient *mqttClient;

    LED *led;
    String macAdd = "";

    Vector<Sensor*> sensors;

    unsigned long sendStep = 6000;

    Device()
    {
    }
    void init(String _ssid, String _password, String _mqttServer, int _mqttPort, String _mqttUsername, String _mqttPassword, int ledPin)
    {
      ssid = _ssid;
      password = _password;
      mqttServer = _mqttServer;
      mqttPort = _mqttPort;
      mqttUsername = _mqttUsername;
      mqttPassword = _mqttPassword;
      led = new LED(ledPin);

      wifiClient = new WiFiClientSecure();
      mqttClient = new PubSubClient(*wifiClient);

      updateLED();
      connectWifi();
      updateLED();
      connectMqtt();
      updateLED();
    }

    void updateLED()
    {
      if (wifiConnected && mqttConnected)
      {
        led->turnOn(255);
      }
      else if (wifiConnected && !mqttConnected)
      {
        led->turnOn(50);
      }
      else
      {
        led->turnOff();
      }
    }

    void connectMqtt()
    {
      mqttConnected = false;
      mqttClient->setServer(mqttServer.c_str(), mqttPort);
      while (!mqttClient->connected())
      {
        Serial.println(macAdd);
        Serial.println("Connecting to MQTT...");
        if (mqttClient->connect(macAdd.c_str(), mqttUsername.c_str(), mqttPassword.c_str()))
        {
          Serial.println("connected");
          mqttConnected = true;
          updateLED();
        } else {
          Serial.print("Failed. Error state=");
          Serial.print(mqttClient->state());
          Serial.println("");
        }
        delay(1000);
        randomSeed(micros());
      }
    }

    void connectWifi()
    {
      wifiConnected = false;
      Serial.print("Connecting to ");
      Serial.println(ssid);
      WiFi.begin(ssid.c_str(), password.c_str());
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
      }
      macAdd = WiFi.macAddress();
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Mac Address: ");
      Serial.println(macAdd);
      wifiConnected = true;
    }
    void addSensor(Sensor *sensor)
    {
      sensors.push_back(sensor);
    }
    void addSensors(Sensor* sensors[], int s)
    {
      for (int i = 0; i < s; i++)
      {
        addSensor(sensors[i]);
      }
    }
    void collectData()
    {
      for (int i = 0; i < sensors.size(); i++)
      {
        sensors[i]->process();
      }
    }
    void reset()
    {
      for (int i = 0; i < sensors.size(); i++)
      {
        sensors[i]->reset();
      }
    }
    void sendData() {
      StaticJsonBuffer<500> JSONbuffer;
      JsonObject& JSONencoder = JSONbuffer.createObject();
      for (int i = 0; i < sensors.size(); i++)
      {
        JSONencoder[sensors[i]->name] = sensors[i]->getData();
      }
      JSONencoder["token"] = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VyX25hbWUiOiJsb25ncGhhbSIsInVzZXJfaWQiOjIsImlhdCI6MTU4OTcwOTUyMX0.kMaJh-meTBf5gmOqFgPZAj7Xpox7QFdnjGWPuRwGN4A";
      JSONencoder["device_id"] = macAdd;
      char JSONmessageBuffer[500];
      JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

      Serial.print("Sensor data: ");
      Serial.println(JSONmessageBuffer);


      Serial.println("Sending message to MQTT topic..");
      Serial.println(JSONmessageBuffer);
      Serial.println(macAdd);
      while (!mqttClient->publish("ESP32", JSONmessageBuffer))
      {
        Serial.println("Error sending message");
        Serial.println("Server disconnected, retrying");
        mqttConnected = false;
        updateLED();
        connectMqtt();
        Serial.println("Server reconnected successfully");
      }
      Serial.println("Success sending message");
      mqttClient->loop();
      Serial.println("-------------");
    }
};

//phototransistor
const int photoPin = 39;
//pir sensor
const int pirPin = 36;
//mic sensor
const int micPin = 32;
//remote sensor
int irPin = 34;
//BME
const int sck = 25;
const int sdi = 26;
const int bme_address = 0x76;
// led
const int ledPin = 12;

const String mqttServer = "fc.happysocial.net";
const int mqttPort = 8883;

const String mqttUsername = "esp32";
const String mqttPassword = "fchappy0419";

Device *esp;
BME *bme;
unsigned long startTime;
unsigned long nextCollectTime;
unsigned long nextSendTime;

unsigned long collectStep = 60;
unsigned long sendStep = 6000;
void setup()
{
   WiFi.mode(WIFI_STA);

  Serial.begin(115200);
  esp = new Device();
  esp->init(ssid, password, mqttServer, mqttPort, mqttUsername, mqttPassword, ledPin);
  bme = new BME(bme_address, sdi, sck);
  Sensor* sensors[] = {new PhotoSensor(photoPin), new MicSensor(micPin), new PIRSensor(pirPin), new IRSensor(irPin), new TempSensor(bme), new HumidSensor(bme), new PressureSensor(bme)};
  esp->addSensors(sensors, 7);
  unsigned long now = millis();
  startTime = now;
  nextCollectTime = now + collectStep;
  nextSendTime = now + sendStep;
}

void loop()
{
  unsigned long currentTime = millis();
  if ((currentTime - startTime) > (nextCollectTime - startTime))
  {
    esp->collectData();
    nextCollectTime += collectStep;
  }
  if ((currentTime - startTime) > (nextSendTime - startTime))
  {
    esp->sendData();
    esp->reset();
    nextSendTime += sendStep;
  }
}
