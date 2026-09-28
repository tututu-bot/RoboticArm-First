#pragma once

#include <stddef.h>

#define TRAJ_NVS_KEY "traj_list"
#define TRAJ_LIST_MAX 3500
extern char traj_list[TRAJ_LIST_MAX];

void nvs_init();
void trajs_load();
void traj_remove_line(const char *name);
int traj_get_points(const char *name, char *dst, size_t dst_size);
void traj_save();

