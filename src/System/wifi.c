/**
 * 连网：ESP32通过WI-FI名+密码的方式通过路由器验证，路由器分配给ESP32一个IP地址
 * 分7个步骤：
 * 1. 初始化小存储（NVS）：这里只是初始化nvs，并没做任何操作。为的是以后存储数据使用
 * 2. 初始化网络接口：相当于装上网络驱动，告诉esp32我要使用网卡，否则无法使用网络
 * 3. 创建默认事件循环：创建一个广播站，注册过的事件回调可以在这个广播站中发出
 * 4. 创建默认 WiFi STA 接口：确定使用STA模式
 * 5. 初始化 WiFi 驱动：初始化配置网络驱动
 * 6. 注册事件回调：将事件注册进入广播站（获取ip地址事件），以便在事件发生时触发回调函数
 * 7. 配置账号密码并启动连接：通过Wi-Fi账号密码连接到路由器，得到ip地址（得到ip后触发事件回调，打印ip地址）
 */
#include <string.h>               // strcpy 需要
#include "nvs_flash.h"            // WiFi 需要的⼩存储（存配置）
#include "esp_netif.h"            // ⽹络接⼝层
#include "esp_wifi.h"             // WiFi 驱动
#include "esp_event.h"            // 事件循环（WiFi 事件分发）
#include "wifi.h"                 // ⾃⼰的头⽂件：让"声明(wifi.h)和定义(wifi.c)"保持⼀致
#define WIFI_SSID  "2405-1"    // ★ 改成你的 2.4G WiFi
#define WIFI_PASS  "13802211121lxl"  // ★ 改成你的密码
// ★ 固定 IP（想恢复 DHCP 就把下面的 1 改成 0）
#define STATIC_IP_ENABLE 1
#define STATIC_IP   "192.168.2.20"   // ★ 你要的固定 IP
#define STATIC_GW   "192.168.2.1"    // ★ 网关，跟路由器后台一致
#define STATIC_NM   "255.255.255.0"  // ★ 子网掩码，一般就是这个
// 拿到 IP 的回调：连上后⾃动打印地址
static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
    printf("连上 WiFi！遥控地址: http://" IPSTR "\n", IP2STR(&event->ip_info.ip));
}
void wifi_init(void) {
    esp_err_t e = nvs_flash_init();          // 1. 初始化⼩存储（WiFi 参数⽤）
    if (e != ESP_OK) printf("nvs_flash_init err 0x%x\n", e);
    esp_netif_init();                        // 2. 初始化⽹络接⼝层
    esp_event_loop_create_default();         // 3. 事件循环（WiFi 事件靠它分发）
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();     // 4. 创建"连路由器"模式（STA = 站点）
    if (STATIC_IP_ENABLE) {                    // 4.1 配固定 IP：先停 DHCP，再写死地址
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta));
        esp_netif_ip_info_t ip = {0};
        ip.ip.addr       = esp_ip4addr_aton(STATIC_IP);
        ip.gw.addr       = esp_ip4addr_aton(STATIC_GW);
        ip.netmask.addr  = esp_ip4addr_aton(STATIC_NM);
        ESP_ERROR_CHECK(esp_netif_set_ip_info(sta, &ip));
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // 5. 初始化 WiFi 驱动（5.1 说的第 2 层）
    esp_wifi_init(&cfg);
    // 6. 注册事件回调：拿到 IP 时打印地址
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL);
    // 7. 配置账号密码并启动连接
    wifi_config_t wc = {0};
    strcpy((char *)wc.sta.ssid, WIFI_SSID);
    strcpy((char *)wc.sta.password, WIFI_PASS);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
    esp_wifi_connect();
}
// 📌  看起来⽐ Arduino 的 WiFi.begin() ⻓，但每⾏职责明确——ESP-IDF ⻛格就是"显式"。
//    第 8 章语⾳、第 9 章视觉会原样复⽤ wifi_init()。