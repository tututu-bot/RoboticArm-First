#pragma once // 防止头文件被重复包含
// 开热点：ESP32 自己当路由器，手机/电脑连上 "arm-esp32" 后浏览器访问 http://192.168.4.1
// 不依赖路由器、不依赖手机热点，插电即用（热点名密码在 wifi.c 顶部配置区改）
void wifi_init(void);
