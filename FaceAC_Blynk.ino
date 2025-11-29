#define BLYNK_TEMPLATE_ID "TMPL6m2BEQ7pC"
#define BLYNK_TEMPLATE_NAME "ESP32 CAM"
#define BLYNK_AUTH_TOKEN "xajmeBy1Gf9Ji93mQPj9ScSW_2neCwH6"

// #define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

#include "esp_http_server.h"
#include "Arduino.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Nora";
//char pass[] = "Nora1911";
char ssid[] = "\xE2\x8B\x88";
char pass[] = "Nora#1911";


// Blynk virtual pin aliases
#define VP_STATUS V1
#define VP_VIDEOURL V2
#define VP_NAME_IN V3
#define VP_ENROLL V4
#define VP_DETECT V5
#define VP_RECOG V6
#define VP_DELALL V7
#define VP_SERVO V8
#define VP_LED V9
#define VP_LIST V10
#define VP_STREAM V11
#define VP_MENU V12
#define VP_DELSEL V13

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

// Select camera model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Camera frame buffer
camera_fb_t *fb = NULL;

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;

// Face detection confi
static inline mtmn_config_t app_mtmn_config() {
  mtmn_config_t mtmn_config = { 0 };
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

// State machine
typedef enum {
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state = START_STREAM;

// Enrollment name storage
typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

// Global mutable buffer
#define NAME_BUF_SIZE 64
char global_name_buf[NAME_BUF_SIZE];

// Servo and LED pins
const int servoPin = 12;  // SG90 control pin (GPIO12)
const int ledPin = 4;     // flash LED (GPIO4)

// Servo object
Servo myservo;

// Door timing
unsigned long door_opened_millis = 0;
unsigned long interval = 5000;  // milliseconds to keep door open
bool door_is_open = false;

// For last-detection timestamp (rate-limiting messages)
long last_detected_millis = 0;
long last_sent_no_face_millis = 0;
const long NO_FACE_MSG_INTERVAL = 3000;

// Enrollment timing control
long last_enroll_sample_millis = 0;
const long ENROLL_SAMPLE_INTERVAL = 500;  // minimum 500ms between enrollment samples
bool enrollment_in_progress = false;
int enrollment_samples_taken = 0;

// Recognition timing control (prevent servo spam)
long last_recognition_action_millis = 0;
const long RECOGNITION_ACTION_INTERVAL = 3000;  // minimum 3 seconds between door opens

int v12_selected_index = 0;  // 0..7 as user requested; 0 -> none

// Forward declarations for functions
void app_facenet_main();
void read_initial_face_list();
//int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id);
void send_enrolled_list_to_blynk();
String get_enrolled_name_by_index(int idx);  // 0-based
void delete_selected_user_by_index(int idx);
void openServo();
void closeServo();
void update_video_url_to_blynk();
void startCameraServer();
void app_httpserver_init();

char *getMutableCStr(const String &str) {
  strncpy(global_name_buf, str.c_str(), NAME_BUF_SIZE);
  global_name_buf[NAME_BUF_SIZE - 1] = '\0';  // Ensure null termination
  return global_name_buf;
}

BLYNK_CONNECTED() {
  update_video_url_to_blynk();
  // send initial status
  Blynk.virtualWrite(VP_STATUS, "Blynk Connected. STREAMING");
}

// V3 name input
BLYNK_WRITE(VP_NAME_IN) {
  String name = param.asStr();
  if (name.length() >= (ENROLL_NAME_LEN - 1)) {
    name = name.substring(0, ENROLL_NAME_LEN - 2);
  }
  name.toCharArray(st_name.enroll_name, ENROLL_NAME_LEN);
  Blynk.virtualWrite(VP_STATUS, String("Name input set: ") + st_name.enroll_name);
}

// V4 enroll button (push)
BLYNK_WRITE(VP_ENROLL) {
  int v = param.asInt();
  if (v) {  // only on press
    if (strlen(st_name.enroll_name) == 0) {
      Blynk.virtualWrite(VP_STATUS, "Enroll blocked: name is empty (set V3 first)");
      return;
    }
    g_state = START_ENROLL;
    enrollment_in_progress = true;
    enrollment_samples_taken = 0;
    Blynk.virtualWrite(VP_STATUS, String("Enrollment started for: ") + st_name.enroll_name);
  }
}

// V5 start detect
BLYNK_WRITE(VP_DETECT) {
  int v = param.asInt();
  if (v) {
    g_state = START_DETECT;
    Blynk.virtualWrite(VP_STATUS, "DETECTING");
  }
}

// V6 start recognition
BLYNK_WRITE(VP_RECOG) {
  int v = param.asInt();
  if (v) {
    g_state = START_RECOGNITION;
    Blynk.virtualWrite(VP_STATUS, "RECOGNISING");
  }
}

// V7 delete all
BLYNK_WRITE(VP_DELALL) {
  int v = param.asInt();
  if (v) {
    delete_face_all_in_flash_with_name(&st_face_list);
    read_initial_face_list();  // refresh
    send_enrolled_list_to_blynk();
    Blynk.virtualWrite(VP_STATUS, "All faces deleted");
  }
}

// V8 servo manual slider (0..90)
BLYNK_WRITE(VP_SERVO) {
  int angle = param.asInt();
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  myservo.write(angle);
  if (angle > 0) {
    door_is_open = true;
    door_opened_millis = millis();
  } else {
    door_is_open = false;
  }
  Blynk.virtualWrite(VP_STATUS, String("Servo moved to ") + angle + "°");
}

// V9 LED on/off
BLYNK_WRITE(VP_LED) {
  int v = param.asInt();
  digitalWrite(ledPin, v ? HIGH : LOW);
  Blynk.virtualWrite(VP_STATUS, String("LED ") + (v ? "ON" : "OFF"));
}

// V11 stream only (no detect / no recognize)
BLYNK_WRITE(VP_STREAM) {
  int v = param.asInt();
  if (v) {
    g_state = START_STREAM;
    Blynk.virtualWrite(VP_STATUS, "STREAMING ONLY (no detect/recognise)");
  }
}

// V12 menu select user index (0..7)
BLYNK_WRITE(VP_MENU) {
  int v = param.asInt();
  v12_selected_index = v;  // direct store; app uses 0..7
  Blynk.virtualWrite(VP_STATUS, String("Selected menu index = ") + v12_selected_index);
}

// V13 delete selected user
BLYNK_WRITE(VP_DELSEL) {
  int v = param.asInt();
  if (!v) return;
  if (v12_selected_index < 0 || v12_selected_index > FACE_ID_SAVE_NUMBER) {
    Blynk.virtualWrite(VP_STATUS, "Delete selected: invalid index (choose 0..6)");
    return;
  }
  int idx0 = v12_selected_index;  // convert to 0-based
  String name = get_enrolled_name_by_index(idx0);
  if (name.length() == 0) {
    Blynk.virtualWrite(VP_STATUS, "No user in selected slot");
    return;
  }
  delete_face_id_in_flash_with_name(&st_face_list, getMutableCStr(name));
  read_initial_face_list();
  send_enrolled_list_to_blynk();
  Blynk.virtualWrite(VP_STATUS, String("Deleted user: ") + name);
}

void update_video_url_to_blynk() {
  IPAddress ip = WiFi.localIP();
  if (ip) {
    String streamURL = String("http://") + ip.toString() + ":81/stream";  // keep port 81 if your camera server uses it
    Blynk.setProperty(VP_VIDEOURL, "url", streamURL);
    Blynk.virtualWrite(VP_STATUS, String("Video URL set to: ") + streamURL);
  } else {
    Blynk.virtualWrite(VP_STATUS, "Could not set Video URL: no IP");
  }
}

void send_enrolled_list_to_blynk() {
  // Update VP_LIST with comma-separated user names
  if (st_face_list.count == 0) {
    Blynk.virtualWrite(VP_LIST, "No users");
  } else {
    face_id_node *node = st_face_list.head;
    String out = "";
    for (int i = 0; i < st_face_list.count && node != NULL; ++i) {
      if (i) out += ", ";
      out += String(node->id_name);
      node = node->next;
    }
    Blynk.virtualWrite(VP_LIST, out);
  }

  // Update VP_MENU labels with enrolled user names
  // Build menu items:
  BlynkParamAllocated items(10);
  items.add("Select user...");

  // Fill in enrolled users
  face_id_node *node = st_face_list.head;
  for (int i = 0; i < FACE_ID_SAVE_NUMBER; i++) {
    if (i < st_face_list.count && node != NULL) {
      items.add(String(i + 1) + ". " + String(node->id_name));
      node = node->next;
    } else {
      items.add("Empty");
    }
  }

  // Set the menu property with all labels
  Blynk.setProperty(VP_MENU, "labels", items);
}

String get_enrolled_name_by_index(int idx) {
  if (idx < 0 || idx >= st_face_list.count) return "";
  face_id_node *node = st_face_list.head;
  for (int i = 0; i < idx && node != NULL; ++i) node = node->next;
  if (node) return String(node->id_name);
  return "";
}

void delete_selected_user_by_index(int idx) {
  String name = get_enrolled_name_by_index(idx);
  if (name.length() > 0) {
    delete_face_id_in_flash_with_name(&st_face_list, getMutableCStr(name));
    read_initial_face_list();
    send_enrolled_list_to_blynk();
    Blynk.virtualWrite(VP_STATUS, String("Deleted user ") + name);
  } else {
    Blynk.virtualWrite(VP_STATUS, "No user found for that index");
  }
}

void openServo() {
  myservo.write(90);  // open
  door_is_open = true;
  door_opened_millis = millis();
  // Blynk.virtualWrite(VP_STATUS, "Servo OPEN (90°)");
  Blynk.virtualWrite(VP_SERVO, 90);
}

void closeServo() {
  myservo.write(0);  // closed
  door_is_open = false;
  Blynk.virtualWrite(VP_STATUS, "Servo CLOSED (0°)");
  Blynk.virtualWrite(VP_SERVO, 0);
}

httpd_handle_t stream_httpd = NULL;
static const char *STREAM_TAG = "camera_stream";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char *part_buf[64];
  static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  static const char *_STREAM_BOUNDARY = "\r\n--frame\r\n";
  static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGE(STREAM_TAG, "Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &fb->buf, &fb->len);
      if (!jpeg_converted) {
        ESP_LOGE(STREAM_TAG, "JPEG compression failed");
        esp_camera_fb_return(fb);
        res = ESP_FAIL;
        break;
      }
    }

    size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);

    if (res != ESP_OK) break;
  }

  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;  // Blynk video widget default port

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  ESP_LOGI(STREAM_TAG, "Starting camera stream server on port %d", config.server_port);

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    ESP_LOGI(STREAM_TAG, "MJPEG stream ready at http://<ESP_IP>:81/stream");
  } else {
    ESP_LOGE(STREAM_TAG, "Failed to start stream server");
  }
}

// read face list from flash and init the in-memory list
void read_initial_face_list() {
  read_face_id_from_flash_with_name(&st_face_list);
}

// Enrollment wrapper
static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id) {
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  return left_sample_face;
}

// App main initialization for FaceNet
void app_facenet_main() {
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_initial_face_list();
  send_enrolled_list_to_blynk();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting Blynk + FaceNet sketch...");

  // servo and LED init
  // Allocate timer 3 for servo (camera uses timer 0)
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  // SG90 servo: use wider pulse range (500-2400µs) for full 180° range
  myservo.attach(servoPin, 500, 2400);
  myservo.write(0);  // start closed
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // camera config
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
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    // fail early
    while (true) {
      delay(1000);
    }
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  // WiFi + Blynk init
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected:");
  Serial.println(WiFi.localIP());

  // start camera streaming server (keeps /stream route)
  startCameraServer();

  // FaceNet init
  app_facenet_main();

  // Blynk begin (Blynk.Cloud)
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud");
  // BLYNK_CONNECTED callback will set video URL

  Serial.println("Setup complete. Streaming available at:");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
}

// ---------- Main Loop ----------
void loop() {
  Blynk.run();  // handle blynk events

  // Close servo automatically after 'interval' ms if open
  if (door_is_open && millis() - door_opened_millis > interval) {
    closeServo();
  }

  // Always capture frames so video stream is alive and face detection can run if requested.
  fb = esp_camera_fb_get();
  if (!fb) {
    esp_camera_fb_return(fb);
    Blynk.virtualWrite(VP_STATUS, "Camera capture failed");
    return;
  }

  // Convert and prepare image buffer for detection (use same image size as used by your face_detect)
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  if (image_matrix == NULL) {
    esp_camera_fb_return(fb);
    Blynk.virtualWrite(VP_STATUS, "img matrix alloc failed");
    return;
  }
  http_img_process_result out_res = { 0 };
  out_res.image = image_matrix->item;

  // Only do face detection/recognition if state demands it
  if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION) {
    out_res.net_boxes = NULL;
    out_res.face_id = NULL;

    fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);
    out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

    if (out_res.net_boxes) {
      if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {
        out_res.face_id = get_face_id(aligned_face);
        last_detected_millis = millis();

        if (g_state == START_DETECT) {
          Blynk.virtualWrite(VP_STATUS, "FACE DETECTED");
        }

        if (g_state == START_ENROLL && enrollment_in_progress) {
          // Rate-limit enrollment samples to prevent rapid-fire captures
          if (millis() - last_enroll_sample_millis >= ENROLL_SAMPLE_INTERVAL) {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            last_enroll_sample_millis = millis();
            enrollment_samples_taken++;

            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE %d/%d FOR %s", enrollment_samples_taken, ENROLL_CONFIRM_TIMES, st_name.enroll_name);
            Blynk.virtualWrite(VP_STATUS, enrolling_message);

            if (left_sample_face == 0 || enrollment_samples_taken >= ENROLL_CONFIRM_TIMES) {
              // Enrollment complete - immediately stop enrollment
              enrollment_in_progress = false;
              g_state = START_STREAM;

              char captured_message[64];
              sprintf(captured_message, "ENROLLMENT COMPLETE FOR %s", st_face_list.tail->id_name);
              Blynk.virtualWrite(VP_STATUS, captured_message);
              // refresh list to V10
              send_enrolled_list_to_blynk();

              st_name.enroll_name[0] = '\0';
              Blynk.virtualWrite(VP_NAME_IN, st_name.enroll_name);
              Blynk.virtualWrite(VP_ENROLL, 0);
              enrollment_samples_taken = 0;
            }
          }
        }

        if (g_state == START_RECOGNITION && (st_face_list.count > 0)) {
          face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
          if (f) {
            // Only open door if enough time has passed since last recognition action
            if (millis() - last_recognition_action_millis >= RECOGNITION_ACTION_INTERVAL) {
              char recognised_message[64];
              sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
              Blynk.virtualWrite(VP_STATUS, recognised_message);
              // open servo (instead of old relay)
              openServo();
              last_recognition_action_millis = millis();
            }
          } else {
            // Rate-limit "not recognised" messages too
            if (millis() - last_sent_no_face_millis > NO_FACE_MSG_INTERVAL) {
              Blynk.virtualWrite(VP_STATUS, "FACE NOT RECOGNISED");
              last_sent_no_face_millis = millis();
            }
          }
        }
        if (out_res.face_id) dl_matrix3d_free(out_res.face_id);
      }
    } else {
      // no net boxes
      if (g_state != START_DETECT) {
        // If not in detect mode, we don't need to spam NO FACE
      } else {
        if (millis() - last_sent_no_face_millis > NO_FACE_MSG_INTERVAL) {
          Blynk.virtualWrite(VP_STATUS, "NO FACE DETECTED");
          last_sent_no_face_millis = millis();
        }
      }
    }
  }

  // Return frame buffer and free intermediate
  esp_camera_fb_return(fb);
  if (image_matrix) dl_matrix3du_free(image_matrix);

  // Periodically ensure enrolled list is pushed to V10 (defensive)
  static unsigned long last_list_sent = 0;
  if (millis() - last_list_sent > 5000) {
    send_enrolled_list_to_blynk();
    last_list_sent = millis();
  }

  fb = NULL;
}
