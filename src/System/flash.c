#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "flash.h"

static nvs_handle_t h;

char traj_list[TRAJ_LIST_MAX];

static int nvs_ready = 0;

void nvs_init(){
    if (nvs_ready)                       // 重复调用直接返回：wifi_init 和 web_start 都会调
        return;
    esp_err_t e = nvs_flash_init();
    // 分区写满 / 旧固件留下的版本号：官方推荐先擦再建，否则这块 NVS 永远打不开
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND){
        printf("nvs 需要重建 0x%x\n", e);
        e = nvs_flash_erase();
        if (e == ESP_OK)
            e = nvs_flash_init();
    }
    if (e != ESP_OK){
        printf("nvs_flash_init err 0x%x\n", e);
        return;                          // 失败不置位，下次调用还能重试
    }
    nvs_ready = 1;
}

void trajs_load(){
    traj_list[0] = '\0';
    esp_err_t e = nvs_open("traj", NVS_READONLY, &h);
    if (e != ESP_OK){
        printf("traj_load: nvs_open err 0x%x\n", e);
        return;
    }
    size_t len = sizeof(traj_list);
    e = nvs_get_str(h, TRAJ_NVS_KEY, traj_list, &len);
    if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND)
        printf("traj_load: nvs_get err 0x%x\n", e);
    nvs_close(h);
}

void traj_save(){
    esp_err_t e = nvs_open("traj", NVS_READWRITE, &h);
    if (e != ESP_OK){
        printf("traj_save: nvs_open err 0x%x\n", e);
        return;
    }
    e = nvs_set_str(h, TRAJ_NVS_KEY, traj_list);
    if (e != ESP_OK)
        printf("traj_save: nvs_set err 0x%x\n", e);
    e = nvs_commit(h);
    if (e != ESP_OK)
        printf("traj_save: nvs_commit err 0x%x\n", e);
    nvs_close(h);
}
//删除某条轨迹
void traj_remove_line(const char *name){
    char out[TRAJ_LIST_MAX];
    out[0] = '\0';
    const char *line = traj_list;
    int first = 1;
    while (*line){
        const char *nl = strchr(line, '\n');
        size_t len = nl ? (size_t)(nl - line) : strlen(line);
        size_t field = strcspn(line, ";");
        int keep = !(field == strlen(name) && strncmp(line, name, field) == 0);
        if (keep){
            if (!first)
                strcat(out, "\n");
            strncat(out, line, len);
            first = 0;
        }
        if (!nl)
            break;
        line = nl + 1;
    }
    strcpy(traj_list, out);
}
//获取轨迹的所有点位
int traj_get_points(const char *name, char *dst, size_t dst_size){
    trajs_load();
    const char *line = traj_list;
    while (*line){
        const char *nl = strchr(line, '\n');
        size_t len = nl ? (size_t)(nl - line) : strlen(line);
        size_t field = strcspn(line, ";");
        if (field == strlen(name) && strncmp(line, name, field) == 0){
            const char *p = line + field;
            if (*p == ';')
                p++;
            size_t pl = len - (size_t)(p - line);
            if (pl < dst_size){
                memcpy(dst, p, pl);
                dst[pl] = '\0';
                return 1;
            }
            return 0;
        }
        if (!nl)
            break;
        line = nl + 1;
    }
    return 0;
}