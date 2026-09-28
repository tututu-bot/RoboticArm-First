/**
 * 开热点：ESP32 自己当路由器，手机连上它就能遥控，不依赖任何外部网络
 * 分7个步骤：
 * 1. 初始化小存储（NVS）：这里只是初始化nvs，并没做任何操作。为的是以后存储数据使用
 * 2. 初始化网络接口：相当于装上网络驱动，告诉esp32我要使用网卡，否则无法使用网络
 * 3. 创建默认事件循环：创建一个广播站，注册过的事件回调可以在这个广播站中发出
 * 4. 创建默认 WiFi AP 接口：AP = Access Point，让 ESP32 扮演路由器，自带 DHCP 服务器
 * 5. 初始化 WiFi 驱动：初始化配置网络驱动
 * 6. 注册事件回调：有设备连上/断开时触发，串口立刻看得见，出问题时能定位到是哪一步
 * 7. 配置热点名密码并启动：手机搜到热点名，输密码连上，浏览器开 192.168.4.1
 */
#include <string.h>
#include "flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"      // MACSTR / MAC2STR（打印连上来的设备地址）
#include "wifi.h"
#define AP_SSID    "arm-esp32"  // 手机搜到的热点名
#define AP_PASS    "12345678"   // 至少 8 位：不足 8 位配不上 WPA2，热点会退回开放网络
#define AP_CHANNEL 1            // 2.4G 信道，1~13 随便挑，1 就够用
#define AP_MAX_STA 4            // 最多允许几台设备同时连

// 有设备连上热点：说明手机那边已经没问题了，剩下的都是页面的事
static void on_sta_connected(void *arg, esp_event_base_t base, int32_t id, void *data) {
    wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
    printf("设备已连上热点: " MACSTR "  遥控地址: http://192.168.4.1\n", MAC2STR(e->mac));
}
// 有设备断开：手机切走了、走远了、或者热点被关了，串口都能看到
static void on_sta_disconnected(void *arg, esp_event_base_t base, int32_t id, void *data) {
    wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
    printf("设备已断开: " MACSTR "\n", MAC2STR(e->mac));
}

//开启热点
void wifi_init(void) {
    nvs_init();                              // 1. 初始化nvs
    esp_netif_init();                        // 2. 初始化⽹络接⼝层
    esp_event_loop_create_default();         // 3. 事件循环（WiFi 事件靠它分发）
    esp_netif_create_default_wifi_ap();      // 4. 创建"当路由器"模式（AP = 热点，自带 DHCP 服务器）
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // 5. 初始化 WiFi 驱动（5.1 说的第 2 层）
    esp_wifi_init(&cfg);
    // 6. 注册事件回调：谁连上、谁断开，串口都要说话，别让它闷着
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED,    &on_sta_connected,    NULL);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &on_sta_disconnected, NULL);
    // 7. 配置热点名和密码并启动
    wifi_config_t wc = {0};
    strcpy((char *)wc.ap.ssid, AP_SSID);
    wc.ap.ssid_len = strlen(AP_SSID);        // 不写也行，但写明了更保险
    strcpy((char *)wc.ap.password, AP_PASS);
    wc.ap.channel = AP_CHANNEL;
    wc.ap.max_connection = AP_MAX_STA;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;     // 密码不足 8 位时这句配不上，会退回打开状态
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wc);
    esp_wifi_start();
    printf("热点已开启: %s\n手机连上后访问: http://192.168.4.1\n", AP_SSID);
}
