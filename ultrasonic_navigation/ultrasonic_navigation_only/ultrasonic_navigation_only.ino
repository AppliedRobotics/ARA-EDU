#include <HardwareSerial.h>
#include <math.h>
#include <ESPAsyncWebServer.h>

struct Vector4 {
  float x;
  float y;
  float z;
  float r;
};

struct Vector3 {
  float x;
  float y;
  float z;
};

#define MAX_X 2500
#define MAX_Y 3000
#define MAX_Z 2400
#define ka 1
#define kb 0

#define DXL_SERIAL Serial

const char *ssid = "ESP_US_TOF";
const char *password = "12345678";
AsyncWebServer server(80);

char position_config[500];
char message2[100];
char send_coords[100] = {0};

static unsigned long previousMillis = 0;
const long interval = 200;
static uint8_t ack_en = 0;
static uint8_t count = 0;
static uint8_t dlay = 0;

Vector3 position;
uint32_t measure_count[4];
uint32_t measure_time[4];
Vector4 p[4];

float vectorLength(Vector3 v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector4 intersectionLength(Vector4 c1, Vector4 c2, uint8_t zflag = 0) {
  Vector3 distance = {c2.x - c1.x, c2.y - c1.y, c2.z - c1.z};
  float d = vectorLength(distance);

  float x,y,z;

  if (d > c1.r + c2.r || d < abs(c1.r - c2.r)) {
    Vector3 r1, r2;
    float k = c1.r/d;
    r1.x = c1.x + (float) k * (c2.x-c1.x); 
    r1.y = c1.y + (float) k * (c2.y-c1.y);
    r1.z = c1.z + (float) k * (c2.z-c1.z);
    k = c2.r/d;
    r2.x = c2.x + k * (c1.x-c2.x); 
    r2.y = c2.y + k * (c1.y-c2.y); 
    r2.z = c2.z + k * (c1.z-c2.z);

    return {(r1.x+r2.x)/2,(r1.y+r2.y)/2,(r1.z+r2.z)/2, 0};
  }

  float a = (c1.r * c1.r - c2.r * c2.r + d * d) / (2 * d);
  float h = sqrt(c1.r * c1.r - a * a);

  Vector3 point = {c1.x + a * (c2.x - c1.x) / d, c1.y + a * (c2.y - c1.y) / d, c1.z + a * (c2.z - c1.z) / d};

  Vector3 intersection1 = {point.x + h * (c2.y - c1.y) / d, point.y - h * (c2.x - c1.x) / d, point.z - h};
  Vector3 intersection2 = {point.x - h * (c2.y - c1.y) / d, point.y + h * (c2.x - c1.x) / d, point.z - h};

  x = (intersection1.x + intersection2.x) / 2;
  y = (intersection1.y + intersection2.y) / 2;

  z = MAX_Z;
  if (zflag == 1)
  {
      z = intersection1.z;
  }


  Vector4 tmp = {x, y, z, h};
  return tmp;
}

void setup() {
  DXL_SERIAL.begin(115200, SERIAL_8N1, 3, 1);
  DXL_SERIAL.println("start");

  log_i("Configuring access point...");
  
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", send_coords);
  });
  server.begin();

  log_i("Server started");

  p[0] = {0    , 0    , MAX_Z, 3000}; // Положение передатчиков по углам
  p[1] = {MAX_X, 0    , MAX_Z, 3000};
  p[2] = {0    , MAX_Y, MAX_Z, 3000};
  p[3] = {MAX_X, MAX_Y, MAX_Z, 3000};
}

uint64_t timer = 0;
void loop() {
  if (DXL_SERIAL.available())
  {
    char head[4];
    head[0] = DXL_SERIAL.read();
    head[1] = DXL_SERIAL.read();
    head[2] = DXL_SERIAL.read();
    head[3] = DXL_SERIAL.read();
    if (strncmp(head, "DATA", 4) == 0)
    {
      String packet = DXL_SERIAL.readStringUntil('\n');
      int num1, num2, num3;
      sscanf(packet.c_str(), " %d %d %d", &num1, &num2, &num3);
      Serial.println("packet = " + packet);
      if (num1 < 4)
      {     
        p[num1].r = num2*ka + kb;

        Vector4 r[4];
        r[0] = intersectionLength(p[0], p[1], 0);
        r[1] = intersectionLength(p[0], p[2], 0);
        r[2] = intersectionLength(p[2], p[3], 0);
        r[3] = intersectionLength(p[1], p[3], 0);
        position.x = (r[0].x + r[2].x)/2;
        position.y = (r[1].y + r[3].y)/2;
        Vector4 rcr[2];
        rcr[0] = intersectionLength(r[0], r[2], 1);
        rcr[1] = intersectionLength(r[1], r[3], 1);
        position.z = (rcr[0].z + rcr[1].z)/2;
        position.x = constrain(position.x, 0, MAX_X);
        position.y = constrain(position.y, 0, MAX_Y);
        position.z = constrain(position.z, 0, MAX_Z);

        memset(send_coords, 0, 100);
        if (!isnan(position.x))
          sprintf(send_coords, "%f %f %f %d %d %d", position.x, position.y, position.z, num1, num2, num3);
      }
    }
  }
    
}

