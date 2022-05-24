#include <Servo.h>


#include "esp_camera.h"
#include "fd_forward.h"
#include "fb_gfx.h"
#include <WiFi.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

#include "camera_pins.h"

const char* ssid = "ESP32-CAM Access Point";
const char* password = "faceTime";

void startCameraServer();

mtmn_config_t mtmn_config = {0};
int detections = 0;

Servo upperEyelid1;
Servo lowerEyelid2;
Servo rightUD3;
Servo rightLR4;
Servo leftUD5;
Servo leftLR6;
// U Eyelid, L Eylid, Right up/down, Right left/right, Left up/down, Left left/right
Servo servos[6] = {upperEyelid1, lowerEyelid2, rightUD3, rightLR4, leftUD5, leftLR6};
int pins[6] = {2, 4, 12, 13, 14, 15};

// Define initial positions for each servo
int startpos[6] = {140, 60, 85, 87, 85, 100};
int currpos[6] = {140, 60, 85, 87, 85, 100};

// Define bounds of movement
int minBound[6] = {105, 30, 70, 70, 110, 80}; // Down, right, or closed is LOW
int maxBound[6] = {140, 60, 110, 105, 60, 120}; // Up, left, or open is HIGH

void blink(int dt) {
  int uOpen = maxBound[0];
  int lOpen = maxBound[1];
  int uClosed = minBound[0];
  int lClosed = minBound[1];

  int uInt = uOpen - uClosed;
  int lInt = lOpen - lClosed;
  int numsteps = min(uInt, lInt);

  int uStart = uOpen;
  int lStart = lOpen;

  for (int i = 0; i < numsteps; i++) {
    currpos[0] = uStart - (i * uInt) / numsteps;
    servos[0].write(currpos[0]);
    //Serial.println(currpos[0]);
    currpos[1] = lStart - (i * lInt) / numsteps;
    servos[1].write(currpos[1]);
    //Serial.println(currpos[1]);
    delay(5);
  }

  delay(dt);

  for (int i = numsteps; i > 0; i--) {
    currpos[0] = uStart - (i * uInt) / numsteps;
    servos[0].write(currpos[0]);
    //Serial.println(currpos[0]);
    currpos[1] = lStart - (i * lInt) / numsteps;
    servos[1].write(currpos[1]);
    //Serial.println(currpos[1]);
    delay(5);
  }
}

void moveEye(float xPercent, float yPercent) {
  int lLeft = maxBound[5];
  int lRight = minBound[5];
  int rLeft = maxBound[3];
  int rRight = minBound[3];

    int ltargetX = lLeft - int(xPercent*float(lLeft-lRight));
    int rtargetX = rLeft - int(xPercent*float(rLeft-rRight));
//  int ltargetX = lRight + int(xPercent * float(lLeft - lRight));
//  int rtargetX = rRight + int(xPercent * float(rLeft - rRight));

  int lUp = maxBound[4];
  int lDown = minBound[4];
  int rUp = maxBound[2];
  int rDown = minBound[2];

  //  int ltargetY = yPercent*(lUp-lDown)+lDown;
  //  int rtargetY = yPercent*(rUp-rDown)+rDown;
  int ltargetY = lUp - yPercent * (lUp - lDown);
  int rtargetY = rUp + yPercent * (rDown - rUp);


  int leftLRint = ltargetX - currpos[5];
  int rightLRint = rtargetX - currpos[3];
  int xnumsteps = min(abs(leftLRint), abs(rightLRint));
  int lLRstart = currpos[5];
  int rLRstart = currpos[3];

  int leftUDint = ltargetY - currpos[4];
  int rightUDint = rtargetY - currpos[2];
  int ynumsteps = min(abs(leftUDint), abs(rightUDint));
  int lUDstart = currpos[4];
  int rUDstart = currpos[2];

  //Serial.print("LTarget Y: ");
  //Serial.println(ltargetY);
  //Serial.print("RTarget Y: ");
  //Serial.println(rtargetY);

  int numsteps = min(xnumsteps, ynumsteps);

  for (int i = 0; i < numsteps; i++) {
    // Left Eye Left/Right
    currpos[5] = lLRstart + (i * leftLRint) / numsteps;
    servos[5].write(currpos[5]);
    // Serial.print("L Left/Right: ");
    //Serial.println(currpos[5]);
    // Right Eye Left/Right
    currpos[3] = rLRstart + (i * rightLRint) / numsteps;
    servos[3].write(currpos[3]);
    //Serial.print("R Left/Right: ");
    //Serial.println(currpos[3]);
    // Left Eye Up/Down
    currpos[4] = lUDstart + (i * leftUDint) / numsteps;
    servos[4].write(currpos[4]);
    //Serial.print("L Up/Down: ");
    //Serial.println(currpos[4]);
    // Right Eye Up/Down
    currpos[2] = rUDstart + (i * rightUDint) / numsteps;
    servos[2].write(currpos[2]);
    //Serial.print("L Up/Down: ");
    //Serial.println(currpos[2]);
    delay(30);
  }
}


void setup() {
  for (int i = 0; i < 6; i++) {
    servos[i].attach(pins[i], i + 2);
    servos[i].write(startpos[i]);
  }

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pixel_format = PIXFORMAT_JPEG;
  //

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  //
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.softAP(ssid, password);

  startCameraServer();

  Serial.print("Camera Ready! Use 192.168.4.1 to connect");
  //Serial.print(WiFi.localIP());
  //Serial.println("' to connect");
  mtmn_config = mtmn_init_config();
}

void printCoords(int coord_one, int coord_two, int faceID) {
  Serial.print("Face ");
  Serial.print(faceID);
  Serial.print(" found at x: ");
  Serial.print(coord_one);
  Serial.print(" y: ");
  Serial.println(coord_two);
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str) {
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

double minX = 60; //mess with this value
double minY = 40; //mess with this value
double maxX = 260; //mess with this value
double maxY = 130;//mess with this value

double dividerX = maxX - minX;
double dividerY = maxY - minY;
float percentX, percentY;
void moveEyesToFace(int positionX, int positionY) {
  if (positionX > maxX) {
    positionX = maxX;
  }
  if (positionX < minX) {
    positionX = minX;
  }
  if (positionY > maxY) {
    positionY = maxY;
  }
  if (positionY < minY) {
    positionY = minY;
  }
  percentX = (positionX - minX) / dividerX;
  percentY = (positionY - minY) / dividerY;
  moveEye(percentX, percentY);

}
int x, y, w, h, center_x, center_y;
float timeSinceStart, lastTimeSinceStart;
void loop() {
    // put your main code here, to run repeatedly:
    //test.write(90);
    timeSinceStart = float(esp_timer_get_time()) / 1000000;
    if (timeSinceStart - lastTimeSinceStart > 7) {
      lastTimeSinceStart = timeSinceStart;
      blink(75);
    }
    camera_fb_t * frame;
    frame = esp_camera_fb_get();
    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, frame->width, frame->height, 3);
    fmt2rgb888(frame->buf, frame->len, frame->format, image_matrix->item);
  
    esp_camera_fb_return(frame);
  
    box_array_t *boxes = face_detect(image_matrix, &mtmn_config);
    //boxes[i] gets info for face i, we only wanna do 1 face at a time.
    if (boxes != NULL) {
      x = (int)boxes->box[0].box_p[0];
      y = (int)boxes->box[0].box_p[1];
      w = (int)boxes->box[0].box_p[2] - x + 1;
      h = (int)boxes->box[0].box_p[3] - y + 1;
      center_x = x + w/2;
      center_y = y + h/2;
      moveEyesToFace(center_x, center_y);
      dl_lib_free(boxes->score);
      dl_lib_free(boxes->box);
      dl_lib_free(boxes->landmark);
      dl_lib_free(boxes);
    }
    //printCoords(center_x, center_y, 0);
    printCoords(center_x, center_y, 0);
    moveEyesToFace(center_x, center_y);
    dl_matrix3du_free(image_matrix);
  /*for(int i = 0; i < 6; i++){
    Serial.print(currpos[i]);
    Serial.print(",");
    }
    Serial.print("\n");
  */

//  if (Serial.available()) {
//    int temp = Serial.parseInt();
//    if (temp != 0) {
//      float targetpos = float(temp) / 100;
//      Serial.println(targetpos);
//      moveEye(targetpos, targetpos);
//    }
//  }




}
/*static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id)
  camera_fb_t * frame;
  frame = esp_camera_fb_get();
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, frame->width, frame->height, 3);
  fmt2rgb888(frame->buf, frame->len, frame->format, image_matrix->item);

  esp_camera_fb_return(frame);

  box_array_t *boxes = face_detect(image_matrix, &mtmn_config);
  //boxes[i] gets info for face i, we only wanna do 1 face at a time.
  if (boxes != NULL) {
    x = (int)boxes->box[0].box_p[0];
    y = (int)boxes->box[0].box_p[1];
    w = (int)boxes->box[0].box_p[2] - x + 1;
    h = (int)boxes->box[0].box_p[3] - y + 1;
    center_x = x + w/2;
    center_y = y + h/2;
    moveEyesToFace(center_x, center_y);
    dl_lib_free(boxes->score);
    dl_lib_free(boxes->box);
    dl_lib_free(boxes->landmark);
    dl_lib_free(boxes);
  }
  //printCoords(center_x, center_y, 0);

  dl_matrix3du_free(image_matrix);
  }
*/
