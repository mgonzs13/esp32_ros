// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "WiFi.h"
#include <Adafruit_NeoPixel.h>
#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

#include <esp32_msgs/msg/wifi.h>
#include <esp32_msgs/msg/wifi_array.h>
#include <esp32_msgs/srv/set_led_color.h>

// Macros for error handling and task execution
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      return false;                                                            \
    }                                                                          \
  }

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

// Pin and LED configuration
#define LED_PIN 48
#define NUM_LEDS 1

// Global variables and micro-ROS entities
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_parameter_server_t param_server;
rclc_executor_t executor;

rcl_publisher_t wifi_publisher;
rcl_timer_t wifi_timer;
rcl_service_t set_led_color_service;

esp32_msgs__msg__WifiArray wifi_msg;
esp32_msgs__srv__SetLedColor_Request set_led_color_req;
esp32_msgs__srv__SetLedColor_Response set_led_color_res;

int64_t max_wifis = 32;
Adafruit_NeoPixel strip =
    Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Define possible states for the micro-ROS connection
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Set the color of the LED using RGB values
void set_led_color(int r, int g, int b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

// Callback function for the "set_led_color" service
void set_led_color_service_callback(const void *req, void *res) {
  auto *req_in = (esp32_msgs__srv__SetLedColor_Request *)req;
  auto *res_in = (esp32_msgs__srv__SetLedColor_Response *)res;

  // Set the LED to the requested color
  set_led_color(req_in->color.r, req_in->color.g, req_in->color.b);
  res_in->success = true;
}

// Scan for available Wi-Fi networks and return the result
esp32_msgs__msg__Wifi *get_scanned_networks(size_t *result_size) {
  // Allocate memory for storing scanned Wi-Fi information
  esp32_msgs__msg__Wifi *result = (esp32_msgs__msg__Wifi *)malloc(
      max_wifis * sizeof(esp32_msgs__msg__Wifi));
  *result_size = 0;

  if (result == nullptr) {
    return nullptr; // Memory allocation failed
  }

  // Perform the network scan
  int n = WiFi.scanNetworks();
  if (n == -1) {
    free(result);
    return nullptr; // Scan failed
  }

  for (int i = 0; i < n && i < max_wifis; ++i) {
    esp32_msgs__msg__Wifi msg;
    msg.nr = i + 1;
    msg.rssi = WiFi.RSSI(i);
    msg.channel = WiFi.channel(i);

    String ssid = WiFi.SSID(i);
    String encryption;

    // Determine encryption type
    switch (WiFi.encryptionType(i)) {
    case WIFI_AUTH_OPEN:
      encryption = "open";
      break;
    case WIFI_AUTH_WEP:
      encryption = "WEP";
      break;
    case WIFI_AUTH_WPA_PSK:
      encryption = "WPA";
      break;
    case WIFI_AUTH_WPA2_PSK:
      encryption = "WPA2";
      break;
    case WIFI_AUTH_WPA_WPA2_PSK:
      encryption = "WPA+WPA2";
      break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
      encryption = "WPA2-EAP";
      break;
    case WIFI_AUTH_WPA3_PSK:
      encryption = "WPA3";
      break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
      encryption = "WPA2+WPA3";
      break;
    case WIFI_AUTH_WAPI_PSK:
      encryption = "WAPI";
      break;
    default:
      encryption = "unknown";
    }

    // Allocate memory for SSID and encryption type
    msg.ssid.data = (char *)malloc(ssid.length() + 1);
    msg.encryption.data = (char *)malloc(encryption.length() + 1);

    if (msg.ssid.data == nullptr || msg.encryption.data == nullptr) {
      free(msg.ssid.data);
      free(msg.encryption.data);
      continue;
    }

    // Copy SSID and encryption type into message fields
    strncpy(msg.ssid.data, ssid.c_str(), ssid.length() + 1);
    msg.ssid.size = ssid.length();
    msg.ssid.capacity = ssid.length() + 1;

    strncpy(msg.encryption.data, encryption.c_str(), encryption.length() + 1);
    msg.encryption.size = encryption.length();
    msg.encryption.capacity = encryption.length() + 1;

    result[*result_size] = msg;
    (*result_size)++;
  }

  WiFi.scanDelete(); // Free resources allocated by the scan
  return result;
}

// Timer callback to publish scanned networks
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer == NULL)
    return;

  size_t num_networks = 0;
  esp32_msgs__msg__Wifi *networks = get_scanned_networks(&num_networks);

  if (networks == nullptr)
    return; // No networks scanned

  wifi_msg.wifis.data = networks;
  wifi_msg.wifis.capacity = max_wifis;
  wifi_msg.wifis.size = num_networks;

  // Publish the scanned Wi-Fi networks
  rcl_publish(&wifi_publisher, &wifi_msg, NULL);

  // Free the allocated memory after publishing
  for (size_t i = 0; i < num_networks; i++) {
    free(networks[i].ssid.data);
    free(networks[i].encryption.data);
  }
  free(networks);
}

// Initialize micro-ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Initialize support and node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // Initialize parameter server
  RCCHECK(rclc_parameter_server_init_default(&param_server, &node));

  // Initialize Wi-Fi publisher
  RCCHECK(rclc_publisher_init_default(
      &wifi_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(esp32_msgs, msg, WifiArray), "wifi_scan"));

  // Initialize Wi-Fi scan timer
  const unsigned int timer_timeout = 1000; // 1-second interval
  RCCHECK(rclc_timer_init_default(&wifi_timer, &support,
                                  RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Initialize "set_led_color" service
  set_led_color(0, 0, 0); // Turn off LED initially
  RCCHECK(rclc_service_init_default(
      &set_led_color_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(esp32_msgs, srv, SetLedColor),
      "/set_led_color"));

  // Initialize executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context,
                             RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 2,
                             &allocator));
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, NULL));
  RCCHECK(rclc_executor_add_timer(&executor, &wifi_timer));
  RCCHECK(rclc_executor_add_service(&executor, &set_led_color_service,
                                    &set_led_color_req, &set_led_color_res,
                                    set_led_color_service_callback));

  // Initialize parameters
  RCCHECK(rclc_add_parameter(&param_server, "max_wifis", RCLC_PARAMETER_INT));
  RCCHECK(rclc_parameter_set_int(&param_server, "max_wifis", max_wifis));
  RCCHECK(rclc_add_parameter_description(
      &param_server, "max_wifis", "Maximum number of wifis to scan", ""));
  RCCHECK(rclc_parameter_get_int(&param_server, "max_wifis", &max_wifis));

  return true;
}

// Destroy micro-ROS entities
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&wifi_publisher, &node);
  rcl_timer_fini(&wifi_timer);
  rcl_service_fini(&set_led_color_service, &node);
  rclc_parameter_server_fini(&param_server, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  set_led_color(0, 0, 0); // Turn off LED on cleanup
}

// Arduino setup function
void setup() {
  WiFi.mode(WIFI_STA); // Set Wi-Fi to station mode
  WiFi.disconnect();   // Disconnect from any previously connected networks

  strip.begin();
  strip.show(); // Initialize LED strip

  set_microros_transports(); // Initialize micro-ROS transports
  state = WAITING_AGENT;
}

// Arduino loop function
void loop() {
  switch (state) {
  case WAITING_AGENT:
    // Check if the micro-ROS agent is available every 500 ms
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    // Attempt to create entities when the agent is available
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    }
    break;
  case AGENT_CONNECTED:
    // Maintain connection and spin executor
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;
  case AGENT_DISCONNECTED:
    // Destroy entities and return to waiting state
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }
}
