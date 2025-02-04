/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-display-web-server/
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <ESPAsyncWebServer.h>
#include <StringArray.h>
#include <SPIFFS.h>
#include <FS.h>
#include <AsyncUDP.h>
#include "ov2640_settings.h"
#include "sccb.h"
#include "ov2640.h"
#include "ov2640_regs.h"

#include "driver/i2c.h"

#define WRITE_BIT               I2C_MASTER_WRITE      /*!< I2C master write */
#define ACK_CHECK_EN            0x1                   /*!< I2C master will check ack from slave*/
static int sccb_i2c_port = 1;

// Replace with your network credentials
const char* ssid = "QHub-Protoshed";
const char* password = "Q@nt04511";
//const char* ssid = "TELUS2519";
//const char* password = "ymkknfbhz3";

// // Set your Static IP address
// IPAddress local_IP(10, 1, 1, 53);
// // Set your Gateway IP address
// IPAddress gateway(10, 1, 1, 1);
// IPAddress subnet(255, 255, 0, 0);

camera_config_t config; // OV2640 camera module

AsyncWebServer server(80); // Create AsyncWebServer object on port 80
AsyncUDP udp;// Create UDP object

boolean takeNewPhoto = false; // Variable that determines if a picture will be taken or notv

const int BUTTON_PIN = 12; // The number of the pushbutton pin
int lastState = LOW; // The previous state from the input pin
int currentState; // The current reading from the input pin

// unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
// unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Webpage format
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { text-align:center; }
    .vert { margin-bottom: 10%; }
    .hori{ margin-bottom: 0%; }
  </style>
</head>
<body>
  <div id="container">
    <p>
      <button onclick="rotatePhoto();">ROTATE</button>
      <button onclick="capturePhoto()">CAPTURE PHOTO</button>
      <button onclick="location.reload();">REFRESH PAGE</button>
    </p>
  </div>
  <div><img src="saved-photo" id="photo" width="70%"></div>
</body>
<script>
  var deg = 0;
  function capturePhoto() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/capture", true);
    xhr.send();
  }
  function rotatePhoto() {
    var img = document.getElementById("photo");
    deg += 90;
    if(isOdd(deg/90)){ document.getElementById("container").className = "vert"; }
    else{ document.getElementById("container").className = "hori"; }
    img.style.transform = "rotate(" + deg + "deg)";
  }
  function Download(url) {
    document.getElementById('photo').src = url;
  }
  function isOdd(n) { return Math.abs(n % 2) == 1; }
</script>
</html>)rawliteral";

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  // // Configures static IP address
  // if (!WiFi.config(local_IP, gateway, subnet)) {
  //   Serial.println("STA Failed to configure");
  // }

  // Connect to Local Wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  // Mount SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }

  // Print ESP32 Local IP Address
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());

  //Initialize the camera  
  Serial.print("Initializing the camera module...");
  configInitCamera();
  Serial.println("Ok!");

 // Initialize IP Config button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("html layout sent");
    request->send_P(200, "text/html", index_html);
  });

  server.on("/capture", HTTP_GET, [](AsyncWebServerRequest * request) {
    takeNewPhoto = true;
    Serial.println("recieved capture request");
    request->send_P(200, "text/plain", "Taking Photo");
  });

  server.on("/res-up", HTTP_GET, [](AsyncWebServerRequest * request) {
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;
    //res = s->set_framesize(s, (framesize_t)13); // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    res = s->set_quality(s, 10); //10-63 lower number means higher quality

    Serial.println("raising resolution");
    request->send_P(200, "text/plain", "raising resolution");
  });

  server.on("/res-down", HTTP_GET, [](AsyncWebServerRequest * request) {
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;
    //res = s->set_framesize(s, (framesize_t)); // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    res = s->set_quality(s, 25); //10-63 lower number means higher quality

    Serial.println("lowering resolution");
    request->send_P(200, "text/plain", "lowering resolution");
  });

  server.on("/saved-photo", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("recieved save photo request");
    request->send(SPIFFS, FILE_PHOTO, "image/jpg", false);
  });

  // server.on("/alignment-start", HTTP_GET, [](AsyncWebServerRequest * request) {
  //   sensor_t * s = esp_camera_sensor_get()
  //   config.frame_size = FRAMESIZE_QVGA;
  //   config.jpeg_quality = 50;
  //   config.fb_count = 2;
  //   request->send_P(200, "text/plain", "Starting Alignment");
  // });

  //   server.on("/alignment-end", HTTP_GET, [](AsyncWebServerRequest * request) {
  //   config.frame_size = FRAMESIZE_XGA;
  //   config.jpeg_quality = 10;
  //   config.fb_count = 2;
  //   request->send_P(200, "text/plain", "Ending Alignment");
  // });

  // Start server
  server.begin();

}

void loop() {
  if (takeNewPhoto) 
  {
    capturePhotoSaveSpiffs();
    takeNewPhoto = false;
  }

  currentState = digitalRead(BUTTON_PIN); //check if button is pressed

  if (lastState == HIGH && currentState == LOW)
  {
    IPAddress localIP = WiFi.localIP(); // Obtain ESP32 IP address
    String message = localIP.toString(); // Create broadcast message
    udp.broadcastTo(message.c_str(), 1234); // Send broadcast message
    Serial.println("Broadcast message sent!");
  }
  //save the last state
  lastState = currentState;
  delay(10);
}

// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) 
{
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSaveSpiffs( void ) 
{
  camera_fb_t * fb = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly

  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }

    // Photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);

    // Insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
      
    }
    // Close the file
    file.close();
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
    if(ok){
      Serial.println("SPIFFS good");
    }
  } while ( !ok );
}

void configInitCamera()
{
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming

  //Select lower framesize if the camera doesn't support PSRAM
  // if(psramFound()){
  config.frame_size = FRAMESIZE_SXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 10; //10-63 lower number means higher quality
  // config.jpeg_quality = 15;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // } else {
  //   config.frame_size = FRAMESIZE_SVGA;
  //   config.jpeg_quality = 12;
  //   config.fb_count = 1;
  // }

  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  

  sensor_t * s = esp_camera_sensor_get();

  //unsigned char enable_effect = 255;
  // unsigned char enable_effect= 1(saturation and hue enable)+
  // 2(aditional saturation enable)+
  // 4(brighnes and contrast enable)+
  // 8(spital effect1 enable green -> blue)+
  // 16(spital effect1 enable gray -> read)+
  // 32(remove (UV) in YUV color system)+
  // 64(enable negative effect)+
  // 128(remove (Y) in YUV color system);

  // uint8_t ADDR = 0xFF;
  // uint8_t DATA = 0x00;
  // Serial.println(ADDR);
  // Serial.println(DATA);
  // Serial.println(s->slv_addr);
  //int temp = 0;
//  temp = write_reg(s, 0x00, 0xFF, 0x00);
//  temp = WRITE_REG_OR_RETURN(0, 0xFF, 0x00);

  // temp = SCCB_Write(s->slv_addr, 0xFF, 0x00);
   s->set_brightness(s, 0);     // -2 to 2

  // temp = SCCB_Write(s->slv_addr, 0xFF, 0x00);
   s->set_contrast(s, 0);       // -2 to 2

  // temp = SCCB_Write(s->slv_addr, 0xFF, 0x00);
   s->set_saturation(s, 0);     // -2 to 2

  // temp = SCCB_Write(s->slv_addr, 0xFF, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7c, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x04);
  // temp = SCCB_Write(s->slv_addr, 0x7c, 0x09);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x00);

  // temp = SCCB_Write(s->slv_addr, 0xFF, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7c, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x07);
  
  // temp = SCCB_Write(s->slv_addr, 0xFF, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7c, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x02);
  // temp = SCCB_Write(s->slv_addr, 0x7c, 0x03);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x28);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x28);

  // temp = SCCB_Write(s->slv_addr, 0xFF, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7c, 0x00);
  // temp = SCCB_Write(s->slv_addr, 0x7d, 0x07);

}

// int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
// {
//     esp_err_t ret = ESP_FAIL;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     if(ret != ESP_OK) {
//         ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
//     }
//     return ret == ESP_OK ? 0 : -1;
// }
