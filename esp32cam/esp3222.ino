#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>

const char* ssid = "Redmi";  // ใส่ชื่อ Wi-Fi ของคุณ
const char* password = "1111122222";  // ใส่รหัสผ่าน Wi-Fi ของคุณ

#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM    32
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    0
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      21
  #define Y4_GPIO_NUM      19
  #define Y3_GPIO_NUM      18
  #define Y2_GPIO_NUM      5
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22
#endif

WiFiServer server(8080);  // ตั้งค่าให้เซิร์ฟเวอร์ฟังที่พอร์ต 8080

void startCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;  // ส่งข้อมูลแบบ JPEG
    config.frame_size = FRAMESIZE_VGA;     // ขนาดของภาพ 640x480
    config.jpeg_quality = 12;              // คุณภาพของ JPEG (1-63), 12 = คุณภาพปานกลาง
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
}

void sendVideoData(WiFiClient client) {
    while (client.connected()) {
        camera_fb_t *fb = esp_camera_fb_get();  // ดึงภาพจากกล้อง
        if (fb) {
            client.write((const char *)fb->buf, fb->len);  // ส่งข้อมูลภาพผ่าน socket
            esp_camera_fb_return(fb);  // คืนพื้นที่หน่วยความจำให้กับ framebuffer
        }
        delay(30);  // หน่วงเวลาการส่งข้อมูลทุกๆ 30 ms เพื่อให้ส่งแบบเรียลไทม์
    }
    client.stop();
    Serial.println("Client disconnected");
}

void cameraTask(void *pvParameters) {
    server.begin();
    Serial.println("Socket server started on port 8080");

    while (true) {
        WiFiClient client = server.available();  // รอการเชื่อมต่อจาก client
        if (client) {
            Serial.println("Client connected");
            sendVideoData(client);  // ส่งข้อมูลวิดีโอไปยัง client
        }
        delay(100);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);  // เชื่อมต่อ Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    if (!MDNS.begin("esp32cam")) {  // เริ่มต้นการทำงานของ mDNS เพื่อให้เรียกกล้องได้ง่ายขึ้น
        Serial.println("Error setting up MDNS responder!");
        return;
    }
    Serial.println("mDNS responder started");
    Serial.println("Access the device at http://esp32cam.local");

    startCamera();

    // สร้าง task สำหรับการทำงานกล้อง
    xTaskCreate(cameraTask, "Camera Task", 8192, NULL, 1, NULL);
}

void loop() {
  // Loop ว่าง เพราะการทำงานหลักอยู่ใน setup()
}
