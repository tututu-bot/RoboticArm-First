#include <stdio.h>           // printf
#include <string.h>          // strncpy / strchr
#include "esp_http_server.h" // ⽹⻚服务器（5.1 说的第 4 层组件）
#include "freertos/queue.h"  // 队列（第 3 章）
#include "../Hardware/servo.h"           // ⾃制库：指令解析要控制舵机
#include "nvs.h"             // 轨迹存开发板 flash（断电不丢）
#include "web.h"             // ⾃⼰的头⽂件（让声明和定义⼀致）
// 遥控⾯板：机械臂控制（5 路滑条实时控制舵机⻆度）
static const char *REMOTE_PAGE =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>机械臂遥控</title></head>"
    "<body style='text-align:center;font-size:18px;max-width:480px;margin:auto'>"
    "<style>.row{display:flex;align-items:center;margin:10px 0}"
    "label{width:48px;text-align:left}span{width:42px;color:#333}"
    "input[type=range]{flex:1;margin:0 8px}"
    "input[type=text]{width:190px;padding:6px;font-size:16px;margin:6px 0}"
    "button{margin:4px;padding:6px 12px;font-size:15px}"
    ".sec{background:#f7f7f7;border-radius:8px;padding:10px;margin:12px 0;text-align:left}"
    ".pt{font-size:13px;color:#555;margin:2px 0}</style>"
    "<h3>机械臂控制（0-180°）</h3>"
    "<div class='row'><label>底座</label><input type='range' id='s0' min='0' max='180' value='90' oninput='move(0)'><span id='v0'>90</span></div>"
    "<div class='row'><label>大臂</label><input type='range' id='s1' min='0' max='180' value='120' oninput='move(1)'><span id='v1'>120</span></div>"
    "<div class='row'><label>小臂</label><input type='range' id='s2' min='0' max='180' value='45' oninput='move(2)'><span id='v2'>45</span></div>"
    "<div class='row'><label>手腕</label><input type='range' id='s3' min='0' max='180' value='10' oninput='move(3)'><span id='v3'>10</span></div>"
    "<div class='row'><label>夹爪</label><input type='range' id='s4' min='0' max='180' value='180' oninput='move(4)'><span id='v4'>180</span></div>"
    "<div id='status' style='font-size:14px;color:#999;margin-top:10px'></div>"
    // ---- 轨迹录制：拖滑条摆好机械臂，点「记录当前点」存一个点位 ----
    "<div class='sec'><b>轨迹录制</b><br>"
    "<input type='text' id='tname' placeholder='轨迹名称（≤16字）'>"
    "<br><button onclick='recordPoint()'>记录当前点</button>"
    "<button onclick='clearPts()'>清空</button>"
    "<button onclick='genTraj()'>生成轨迹并保存</button>"
    "<div id='ptlist'></div></div>"
    // ---- 轨迹管理：列出开发板保存的轨迹，可播放 / 停止 / 删除 ----
    "<div class='sec'><b>轨迹管理（共 <span id='tcount'>0</span> 条）</b><br>"
    "回放速度: <input type='number' id='tspeed' value='60' min='10' max='180' style='width:70px'> 度/秒<br>"
    "<div id='trajlist'></div></div>"
    "<script>"
    "var pts=[],trajNames=[],timer=null;"
    // 拖动滑条：角度数值立即刷新，30ms 防抖后把 5 个角度一起发给舵机
    "function show(i){document.getElementById('v'+i).textContent=document.getElementById('s'+i).value;}"
    "function move(i){show(i);if(timer)clearTimeout(timer);timer=setTimeout(function(){"
    "var a=[];for(var j=0;j<5;j++)a.push(document.getElementById('s'+j).value);"
    "fetch('/cmd?c='+encodeURIComponent('A='+a.join(',')))"
    ".then(function(){document.getElementById('status').textContent='已发送: '+a.join(',');})"
    ".catch(function(){document.getElementById('status').textContent='发送失败!';});},30);}"
    // 打开页面时从 /state 读当前角度，让滑条对齐舵机真实状态
    "function loadState(){fetch('/state').then(function(r){return r.text();}).then(function(t){"
    "var v=t.split(',');if(v.length>=5){for(var i=0;i<5;i++){"
    "document.getElementById('s'+i).value=v[i];"
    "document.getElementById('v'+i).textContent=v[i];}}}).catch(function(){});}"
    // ---- 轨迹录制：把当前滑条角度存成一个点位 ----
    "function renderPts(){var d=document.getElementById('ptlist');d.innerHTML='';"
    "for(var i=0;i<pts.length;i++)d.innerHTML+=\"<div class='pt'>点\"+(i+1)+\": \"+pts[i].join(',')+\"</div>\";}"
    "function recordPoint(){if(pts.length>=40){document.getElementById('status').textContent='最多40个点!';return;}"
    "var a=[];for(var j=0;j<5;j++)a.push(document.getElementById('s'+j).value);"
    "pts.push(a);renderPts();document.getElementById('status').textContent='已记录点'+(pts.length);}"
    "function clearPts(){pts=[];renderPts();document.getElementById('status').textContent='已清空点位';}"
    "function genTraj(){var name=document.getElementById('tname').value.replace(/[;\"\\n\\r<>&]/g,'').trim().slice(0,16);"
    "if(!pts.length){document.getElementById('status').textContent='请先记录点位!';return;}"
    "if(!name){document.getElementById('status').textContent='请输入轨迹名称!';return;}"
    "var p=[];for(var i=0;i<pts.length;i++)p.push(pts[i].join(','));"
    "fetch('/traj_add?n='+encodeURIComponent(name)+'&p='+p.join(';'))"
    ".then(function(r){return r.text();}).then(function(t){"
    "if(t==='ok'){document.getElementById('status').textContent='已保存: '+name;"
    "pts=[];renderPts();document.getElementById('tname').value='';loadTraj();}"
    "else if(t==='full')document.getElementById('status').textContent='存储已满，请先删除旧轨迹!';"
    "else document.getElementById('status').textContent='保存失败!';})"
    ".catch(function(){document.getElementById('status').textContent='保存失败!';});}"
    // ---- 轨迹管理：从开发板列出轨迹，支持播放 / 停止 / 删除 ----
    "function loadTraj(){trajNames=[];"
    "fetch('/traj').then(function(r){return r.text();}).then(function(t){"
    "var d=document.getElementById('trajlist');d.innerHTML='';"
    "var lines=t.split(/\\r?\\n/);var cnt=0;"
    "for(var i=0;i<lines.length;i++){var line=lines[i];if(!line)continue;"
    "var parts=line.split(';');trajNames.push(parts[0]);"
    "var esc=parts[0].replace(/[<>&]/g,'');cnt++;"
    "d.innerHTML+=\"<div>\"+esc+\"（\"+(parts.length-1)+\"点） <button onclick='useTraj(\"+i+\")'>使用</button> <button onclick='stopTraj()'>停止</button> <button onclick='delTraj(\"+i+\")'>删除</button></div>\";}"
    "document.getElementById('tcount').textContent=cnt;}).catch(function(){});}"
    "function useTraj(i){var sp=document.getElementById('tspeed').value;"
    "fetch('/cmd?c=V='+sp).then(function(){fetch('/cmd?c='+encodeURIComponent('U='+trajNames[i]))"
    ".then(function(){document.getElementById('status').textContent='正在播放轨迹...';}).catch(function(){});});}"
    "function stopTraj(){fetch('/cmd?c='+encodeURIComponent('S'))"
    ".then(function(){document.getElementById('status').textContent='已停止';}).catch(function(){});}"
    "function delTraj(i){fetch('/traj_del?n='+encodeURIComponent(trajNames[i]))"
    ".then(function(){loadTraj();document.getElementById('status').textContent='已删除';}).catch(function(){});}"
    "loadState();loadTraj();"
    "</script></body></html>";
// ===== 轨迹相关全局 =====
#define TRAJ_BUF_SIZE 768            // 40 点轨迹约 560 字节，给足余量
static int traj_speed = 60;          // 回放速度：度/秒（V= 指令可调，10~180）
static char traj_buf[TRAJ_BUF_SIZE]; // 轨迹数据："90,120,45,10,180;80,110,..."
static volatile int stop_traj = 0;   // 停⽌标志（S 指令置 1）
// 执⾏⼀条轨迹（内部函数，仅供 traj_execute 调⽤）
static void run_traj(const char *s);
// 给 exec_task 的⼊⼝：执⾏缓冲⾥的轨迹（web.h ⾥声明的就是它）
void traj_execute(void) { run_traj(traj_buf); }

// ===== 轨迹存储（NVS，断电不丢）=====
#define TRAJ_NVS_KEY "traj_list"      // NVS 里存整个列表的 key
#define TRAJ_LIST_MAX 3500            // 列表总长上限（NVS 字符串 4000 内留余量）
static char traj_list[TRAJ_LIST_MAX]; // 列表缓存：一行一条轨迹

// 从 NVS 把整个列表读进缓存
static void traj_load(void)
{
    nvs_handle_t h;// NVS 句柄（所有NVS操作都需要句柄才能进行操作）
    traj_list[0] = '\0';
    // 打开 NVS namescape为 "traj" 只读
    esp_err_t e = nvs_open("traj", NVS_READONLY, &h);
    if (e != ESP_OK)
    {
        printf("traj_load: nvs_open err 0x%x\n", e);
        return;
    }
    size_t len = sizeof(traj_list);
    e = nvs_get_str(h, TRAJ_NVS_KEY, traj_list, &len);
    if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND)
        printf("traj_load: nvs_get err 0x%x\n", e);
    nvs_close(h);
}
// 把缓存写回 NVS
static void traj_save(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open("traj", NVS_READWRITE, &h);
    if (e != ESP_OK)
    {
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
// 删掉列表中所有名字为 name 的行（删除 / 同名覆盖共用）
static void traj_remove_line(const char *name)
{
    char out[TRAJ_LIST_MAX];
    out[0] = '\0';
    const char *line = traj_list;
    int first = 1;
    while (*line)
    {
        const char *nl = strchr(line, '\n');
        size_t len = nl ? (size_t)(nl - line) : strlen(line);
        size_t field = strcspn(line, ";");
        int keep = !(field == strlen(name) && strncmp(line, name, field) == 0);
        if (keep)
        {
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
// 按名字找轨迹，把点位串拷进 dst；找到返回 1
static int traj_get_points(const char *name, char *dst, size_t dst_size)
{
    traj_load();
    const char *line = traj_list;
    while (*line)
    {
        const char *nl = strchr(line, '\n');
        size_t len = nl ? (size_t)(nl - line) : strlen(line);
        size_t field = strcspn(line, ";");
        if (field == strlen(name) && strncmp(line, name, field) == 0)
        {
            const char *p = line + field;
            if (*p == ';')
                p++;
            size_t pl = len - (size_t)(p - line);
            if (pl < dst_size)
            {
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
// 执⾏⼀条轨迹：逐点到位，每段按 traj_speed 匀速平滑过渡（复用 move_to）
static void run_traj(const char *s)
{
    stop_traj = 0;
    // ★ 播放前：先把该轨迹所有点位的舵机角度打到串口（记录值，方便核对）
    {
        const char *q = s;
        int n = 0;
        while (*q)
        {
            int v[5];
            if (sscanf(q, "%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4]) < 5)
                break; // 解析失败就停（数据截断时也能把已解析的点打印出来）
            n++;
            printf("[轨迹] 起始点位%d: %d %d %d %d %d\n",
                   n, v[0], v[1], v[2], v[3], v[4]);
            const char *semi = strchr(q, ';'); // 跳到下⼀个点位
            if (!semi)
                break;
            q = semi + 1;
        }
        printf("[轨迹] 共%d个点位，开始回放\n", n);
    }
    int pt_no = 0;   // 当前完成到第几个点位（从 1 数起）
    const char *p = s;
    while (*p && !stop_traj)
    {
        int v[5];
        if (sscanf(p, "%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4]) < 5)
            break; // 解析失败（数据被截断）就退出
        int target[SERVO_COUNT];
        int max_delta = 0;
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            target[i] = clamp_angle(i, v[i]); // 轨迹点同样过限位（防倾覆兜底）
            int d = target[i] - current_angle[i];
            if (d < 0)
                d = -d;
            if (d > max_delta)
                max_delta = d;
        }
        int duration = (max_delta * 1000) / traj_speed; // 匀速走到该点的时间
        if (duration < 60)
            duration = 60;
        move_to(target, duration); // ★ 平滑限速过渡（内部再限位）
        if (stop_traj)
            break;
        // ★ 一个点位到位后，串口打印当前状态的 5 路舵机角度
        pt_no++;
        printf("[轨迹] 点位%d完成 -> 舵机角度: %d %d %d %d %d\n",
               pt_no,
               current_angle[0], current_angle[1], current_angle[2],
               current_angle[3], current_angle[4]);
        const char *semi = strchr(p, ';'); // 找下⼀个点的分号
        if (!semi)
            break; // 没有分号 = 最后⼀个点
        p = semi + 1;
        vTaskDelay(pdMS_TO_TICKS(200)); // 到位稍停（可调）
    }
}
// 指令解析：即时指令直接执⾏；⻓任务指令（G/H/T）在 cmd_handler ⾥⼊队
void handle_command(const char *cmd)
{
    // ---- 滑动条绝对值：A=90,120,45,10,180 ----
    if (strncmp(cmd, "A=", 2) == 0)
    {
        int v[5];
        if (sscanf(cmd + 2, "%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4]) == 5)
        {
            printf("滑条指令收到: %d,%d,%d,%d,%d -> 限位后: %d,%d,%d,%d,%d\n",
                   v[0], v[1], v[2], v[3], v[4],
                   clamp_angle(0, v[0]), clamp_angle(1, v[1]), clamp_angle(2, v[2]),
                   clamp_angle(3, v[3]), clamp_angle(4, v[4]));
            for (int i = 0; i < SERVO_COUNT; i++)
            {
                current_angle[i] = clamp_angle(i, v[i]); // 滑条拖到危险区？限位弹回
                servo_write(servo_channels[i], current_angle[i]);
            }
        }
        return;
    }
    // ---- 回放速度：V=60 表示每秒 60 度（10~180）----
    if (strncmp(cmd, "V=", 2) == 0)
    {
        int sp;
        if (sscanf(cmd + 2, "%d", &sp) == 1 && sp >= 10 && sp <= 180)
            traj_speed = sp;
        return;
    }
    const int STEP = 2; // 微调步进（调试⽤，界⾯上已收起）
    switch (cmd[0])
    {
    // ---- 机械臂区微调（第 2 章的 clamp_angle 限位⾃动保护防倾覆）----
    case 'q':
        current_angle[0] = clamp_angle(0, current_angle[0] + STEP);
        break;
    case 'a':
        current_angle[0] = clamp_angle(0, current_angle[0] - STEP);
        break;
    case 'w':
        current_angle[1] = clamp_angle(1, current_angle[1] + STEP);
        break;
    case 's':
        current_angle[1] = clamp_angle(1, current_angle[1] - STEP);
        break;
    case 'e':
        current_angle[2] = clamp_angle(2, current_angle[2] + STEP);
        break;
    case 'd':
        current_angle[2] = clamp_angle(2, current_angle[2] - STEP);
        break;
    case 'r':
        current_angle[3] = clamp_angle(3, current_angle[3] + STEP);
        break;
    case 'f':
        current_angle[3] = clamp_angle(3, current_angle[3] - STEP);
        break;
    case 't':
        current_angle[GRIPPER_IDX] = clamp_angle(GRIPPER_IDX, current_angle[GRIPPER_IDX] + STEP);
        break;
    case 'g':
        current_angle[GRIPPER_IDX] = clamp_angle(GRIPPER_IDX, current_angle[GRIPPER_IDX] - STEP);
        break;
    case 'S':
        stop_traj = 1;
        return; // 停⽌当前轨迹（下个点前停下）
    case 'p':
        printf("⻆度: %d %d %d %d %d\n", current_angle[0], current_angle[1],
               current_angle[2], current_angle[3], current_angle[4]);
        return;
    // ---- ⻋控区（第 7 章替换为真实现，协议不变）----
    case 'N':
    case 'B':
    case 'L':
    case 'R':
    case 'X':
    case 'P':
        printf("⼩⻋未安装（第 7 章启⽤）: %c\n", cmd[0]);
        return;
    default:
        return;
    }
    // ⻆度类指令统⼀写出（限位已经在 clamp_angle ⾥）
    for (int i = 0; i < SERVO_COUNT; i++)
        servo_write(servo_channels[i], current_angle[i]);
}
// ---- web.c 第三块：HTTP 路由（web.c 私有，不暴露）----
// 首页：返回遥控面板
static esp_err_t page_handler(httpd_req_t *req)
{
    httpd_resp_send(req, REMOTE_PAGE, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// /cmd?c=...：解析指令（整串）→ 即时指令直接处理 / 长任务入队（网页立即返回）
// httpd_query_key_value 拿到的值是"百分号编码"的（%3D=% %2C=,），这里手动解码
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

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char buf[1024]; // 轨迹数据可能较长，给足空间
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
        char cmd[768];
        if (httpd_query_key_value(buf, "c", cmd, sizeof(cmd)) == ESP_OK)
        {
            url_decode(cmd, cmd, sizeof(cmd)); // ★ 手动百分号解码（%3D->= %2C->,）
            if (cmd[0] == 'T')
            {                                                     // 轨迹：数据存全局，T 入队
                strncpy(traj_buf, cmd + 2, sizeof(traj_buf) - 1); // 去掉 "T="
                traj_buf[sizeof(traj_buf) - 1] = '\0';
                char t = 'T';
                xQueueSend(cmd_queue, &t, 0);
            }
            else if (cmd[0] == 'U')
            { // 按名称回放在板轨迹：查 NVS 拿点位后 T 入队
                if (traj_get_points(cmd + 2, traj_buf, sizeof(traj_buf)))
                {
                    char t = 'T';
                    xQueueSend(cmd_queue, &t, 0);
                }
            }
            else if (cmd[0] == 'G' || cmd[0] == 'H')
            { // 其他长任务入队
                char first = cmd[0];
                xQueueSend(cmd_queue, &first, 0);
            }
            else
            { // 即时指令（A= / 微调 / S）直接处理
                handle_command(cmd);
            }
        }
    }
    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN); // 网页立即收到"ok"
    return ESP_OK;
}

// /state：返回 5 路舵机当前⻆度，浏览器加载页面时对齐滑条
static esp_err_t state_handler(httpd_req_t *req)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d",
             current_angle[0], current_angle[1], current_angle[2],
             current_angle[3], current_angle[4]);
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// /test：舵机自检——5 个舵机依次扫一个来回，验证 httpd 任务里 servo_write 是否生效
static esp_err_t test_handler(httpd_req_t *req)
{
    httpd_resp_send(req, "sweeping...", HTTPD_RESP_USE_STRLEN);
    int save[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++)
        save[i] = current_angle[i];
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        printf("测试舵机 %d (GPIO %d)\n", i, servo_gpios[i]);
        servo_write(servo_channels[i], clamp_angle(i, 40));
        vTaskDelay(pdMS_TO_TICKS(300));
        servo_write(servo_channels[i], clamp_angle(i, 140));
        vTaskDelay(pdMS_TO_TICKS(300));
        servo_write(servo_channels[i], save[i]);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    return ESP_OK;
}

// /traj：返回整张轨迹表（一行一条：名称;点位1;点位2;...）
static esp_err_t traj_handler(httpd_req_t *req)
{
    traj_load();
    httpd_resp_send(req, traj_list, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// /traj_add?n=名称&p=点位串：保存一条轨迹（同名覆盖），写入 NVS
static esp_err_t traj_add_handler(httpd_req_t *req)
{
    char buf[1600];
    char name[256] = "";
    char pts[768] = "";
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK &&
        httpd_query_key_value(buf, "n", name, sizeof(name)) == ESP_OK &&
        httpd_query_key_value(buf, "p", pts, sizeof(pts)) == ESP_OK)
    {
        url_decode(name, name, sizeof(name));
        url_decode(pts, pts, sizeof(pts));
        for (char *q = name; *q; q++) // 名称里不允许出现分隔符
            if (*q == ';' || *q == '\n' || *q == '\r')
                *q = '_';
        if (!name[0] || !pts[0])
        {
            httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
        traj_load();
        traj_remove_line(name);                             // 同名覆盖旧轨迹
        int add = (int)strlen(name) + 1 + (int)strlen(pts); // 新行 "名称;点位"
        if ((int)strlen(traj_list) + (traj_list[0] ? 1 : 0) + add >= TRAJ_LIST_MAX)
        {
            httpd_resp_send(req, "full", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
        if (traj_list[0])
            strcat(traj_list, "\n");
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

// /traj_del?n=名称：按名称删除一条轨迹
static esp_err_t traj_del_handler(httpd_req_t *req)
{
    char buf[1024];
    char name[256] = "";
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK &&
        httpd_query_key_value(buf, "n", name, sizeof(name)) == ESP_OK)
    {
        url_decode(name, name, sizeof(name));
        if (name[0])
        {
            traj_load();
            traj_remove_line(name);
            traj_save();
            httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    httpd_resp_send(req, "err", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void web_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG(); // 默认配置（端口 80）
    cfg.stack_size = 8192;                       // 轨迹接口的本地缓冲大，任务栈给足
    httpd_handle_t server = NULL;
    httpd_start(&server, &cfg); // 启动服务器
    // 注册两个"路由"：访问 / 返回面板，访问 /cmd 处理指令
    httpd_uri_t page = {.uri = "/", .method = HTTP_GET, .handler = page_handler};
    httpd_uri_t cmd = {.uri = "/cmd", .method = HTTP_GET, .handler = cmd_handler};
    httpd_uri_t state = {.uri = "/state", .method = HTTP_GET, .handler = state_handler};
    httpd_uri_t test = {.uri = "/test", .method = HTTP_GET, .handler = test_handler};
    httpd_uri_t traj = {.uri = "/traj", .method = HTTP_GET, .handler = traj_handler};
    httpd_uri_t traj_add = {.uri = "/traj_add", .method = HTTP_GET, .handler = traj_add_handler};
    httpd_uri_t traj_del = {.uri = "/traj_del", .method = HTTP_GET, .handler = traj_del_handler};
    httpd_register_uri_handler(server, &page);
    httpd_register_uri_handler(server, &cmd);
    httpd_register_uri_handler(server, &state);
    httpd_register_uri_handler(server, &test);
    httpd_register_uri_handler(server, &traj);
    httpd_register_uri_handler(server, &traj_add);
    httpd_register_uri_handler(server, &traj_del);
    printf("网页遥控已启动\n");
}
