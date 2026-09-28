/**
 * 轨道nvs存储:
 * traj(key)->所有轨道信息traj_list(value)，用一个字符串表达
 * traj_list中，有多条轨迹
 * "\n" —— 分隔轨迹（一条轨迹 = 一行）
 * ";" —— 分隔名字↔点位、以及点位↔点位
 * "," —— 分隔一个点位里的 6 个角度
 */
#include <stdio.h>
#include <string.h>
#include "esp_http_server.h"
#include "freertos/queue.h"
#include "Hardware/servo.h"
#include "Hardware/bus_servo.h"
#include "flash.h"
#include "web.h"

#define TRAJ_BUF_SIZE 1024
static int traj_speed = 60;
static char traj_buf[TRAJ_BUF_SIZE];
static volatile int traj_running = 0;

static void run_traj(const char *s);

// 回放中（含刚入队、还没开跑的）：这期间只放行读接口，其余一律回 busy
static int traj_busy(void){
    return traj_running || uxQueueMessagesWaiting(cmd_queue) > 0;
}

void traj_execute(void) { run_traj(traj_buf); }


//轨迹回放
static void run_traj(const char *s){
    traj_running = 1;
    int pt_no = 0;
    const char *p = s;
    while (*p){
        int v[6];
        if (sscanf(p, "%d,%d,%d,%d,%d,%d",&v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) < 6)
            break;
        int target[6];
        int max_delta = 0;
        for (int i = 0; i < 6; i++){
            target[i] = clamp_angle(i, v[i]);
            //变化幅度最大的joint决定本次点位移动的耗时
            int d = target[i] - current_angle[i];
            if (d < 0)
                d = -d;
            if (d > max_delta)
                max_delta = d;
        }
        int spend_time = (max_delta * 1000) / traj_speed;
        if (spend_time < 60)
            spend_time = 60;
        int resp = move_to(target, spend_time);
        if (resp == 1){
            pt_no++;
            const char *semi = strchr(p, ';');
            if (!semi)break;
            p = semi + 1;
            vTaskDelay(pdMS_TO_TICKS(200));
        }else{
            printf("轨迹回放【点位%d】失败",(pt_no+1));
            move_to(POSE_HOME,2000);
            break;
        }
    }
    traj_running = 0;
}

extern const unsigned char index_html[];
extern const unsigned int  index_html_len;

//加载前端页面
static esp_err_t page_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)index_html, index_html_len);
    return ESP_OK;
}

static void url_decode(char *dst, const char *src, size_t dst_size)
{
    size_t i = 0, j = 0;
    while (src[i] && j < dst_size - 1)
    {
        if (src[i] == '%' && src[i + 1] && src[i + 2])
        {
            int hi = (src[i + 1] >= '0' && src[i + 1] <= '9')   ? src[i + 1] - '0'
                     : (src[i + 1] >= 'A' && src[i + 1] <= 'F') ? src[i + 1] - 'A' + 10
                     : (src[i + 1] >= 'a' && src[i + 1] <= 'f') ? src[i + 1] - 'a' + 10
                                                                : -1;
            int lo = (src[i + 2] >= '0' && src[i + 2] <= '9')   ? src[i + 2] - '0'
                     : (src[i + 2] >= 'A' && src[i + 2] <= 'F') ? src[i + 2] - 'A' + 10
                     : (src[i + 2] >= 'a' && src[i + 2] <= 'f') ? src[i + 2] - 'a' + 10
                                                                : -1;
            if (hi >= 0 && lo >= 0)
            {
                dst[j++] = (char)((hi << 4) | lo);
                i += 3;
                continue;
            }
        }
        dst[j++] = src[i++];
    }
    dst[j] = '\0';
}

// /pose?a=90,90,90,90,90,90：滑条绝对值，6 个角度一次下发（clamp_angle 兜限位）
static esp_err_t pose_handler(httpd_req_t *req){
    printf("-----------pose_handler------------");
    if (traj_busy()){ // 回放中不接手动调角度，避免和轨迹抢舵机
        httpd_resp_send(req, "busy", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    char buf[128];
    char val[128] = "";
    int v[SERVO_COUNT];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK &&
        httpd_query_key_value(buf, "a", val, sizeof(val)) == ESP_OK){
        url_decode(val, val, sizeof(val));
        if (sscanf(val, "%d,%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == SERVO_COUNT){
            for (int i = 0; i < SERVO_COUNT; i++){
                current_angle[i] = clamp_angle(i, v[i]);
            }
            int resp = move_to(current_angle,200);
            if (resp == 1){
                httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
    }
    httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

//读取当前返回前端
static esp_err_t state_handler(httpd_req_t *req){
    char buf[64];
    int resp = read_current_state(current_angle);
    if (resp == 1){
        //读取成功；末位多带一个回放标志（1=回放中），前端据此锁住滑条
        snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d",
        current_angle[0], current_angle[1], current_angle[2],
        current_angle[3], current_angle[4], current_angle[5],
        traj_busy());
        httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    return ESP_OK;
    // else{
    //     //读取失败处理
    //     httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN);
    //     return ESP_FAIL;
    // }
}
    
   

//获取所有轨道信息返回前端
static esp_err_t traj_handler(httpd_req_t *req){
    trajs_load();
    httpd_resp_send(req, traj_list, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

//添加轨道
static esp_err_t traj_add_handler(httpd_req_t *req){
    if (traj_busy()){
        httpd_resp_send(req, "busy", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    char buf[2048];              // 和数据里 HTTPD_MAX_URI_LEN 一样大（超了 httpd 直接拒收）
    char name[256] = "";
    char pts[1024] = "";
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK &&
        httpd_query_key_value(buf, "n", name, sizeof(name)) == ESP_OK &&
        httpd_query_key_value(buf, "p", pts, sizeof(pts)) == ESP_OK)
    {
        url_decode(name, name, sizeof(name));
        url_decode(pts, pts, sizeof(pts));
        for (char *q = name; *q; q++)
            if (*q == ';' || *q == '\n' || *q == '\r')
                *q = '_';
        if (!name[0] || !pts[0]){
            httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
        trajs_load();
        traj_remove_line(name);//覆盖同名轨道
        int add = (int)strlen(name) + 1 + (int)strlen(pts); // 新行 "名称;点位"
        if ((int)strlen(traj_list) + (traj_list[0] ? 1 : 0) + add >= TRAJ_LIST_MAX){
            httpd_resp_send(req, "full", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
        if (traj_list[0])
            strcat(traj_list, "\n");//字符串拼接
        strcat(traj_list, name);
        strcat(traj_list, ";");
        strcat(traj_list, pts);
        traj_save();
        httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t traj_del_handler(httpd_req_t *req)
{
    if (traj_busy()){
        httpd_resp_send(req, "busy", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    char buf[1024];
    char name[256] = "";
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK &&
        httpd_query_key_value(buf, "n", name, sizeof(name)) == ESP_OK)
    {
        url_decode(name, name, sizeof(name));
        if (name[0])
        {
            trajs_load();
            traj_remove_line(name);
            traj_save();
            httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t traj_play_handler(httpd_req_t *req)
{
    char buf[512];
    char name[256] = "";
    //回放中，或者已经有一个回放排在队列里等着跑
    if (traj_busy()){
        httpd_resp_send(req, "busy", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK &&
        httpd_query_key_value(buf, "n", name, sizeof(name)) == ESP_OK)
    {
        url_decode(name, name, sizeof(name));
        //轨迹回放任务放入队列中
        if (strlen(name) > 0 && traj_get_points(name, traj_buf, sizeof(traj_buf))){
            char t = 'T';
            if (xQueueSend(cmd_queue, &t, 0) == pdTRUE){
                httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
    }
    httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN); // 没这个名字 / 点位串超长 / 入队失败
    return ESP_OK;
}

void web_start(void){
    nvs_init();
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    httpd_handle_t server = NULL;
    httpd_start(&server, &cfg);
    const httpd_uri_t routes[] = {
        {.uri = "/",          .method = HTTP_GET, .handler = page_handler},
        {.uri = "/state",     .method = HTTP_GET, .handler = state_handler},
        {.uri = "/pose",      .method = HTTP_GET, .handler = pose_handler},
        {.uri = "/traj",      .method = HTTP_GET, .handler = traj_handler},
        {.uri = "/traj_add",  .method = HTTP_GET, .handler = traj_add_handler},
        {.uri = "/traj_del",  .method = HTTP_GET, .handler = traj_del_handler},
        {.uri = "/traj_play", .method = HTTP_GET, .handler = traj_play_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++){
        esp_err_t e = httpd_register_uri_handler(server, &routes[i]);
        if (e != ESP_OK)
            printf("web_start: 注册 %s 失败 0x%x\n", routes[i].uri, e);
    }
}
