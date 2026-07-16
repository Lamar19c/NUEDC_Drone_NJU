/**
 * uwb_parser.c — $DIST 协议解析器 (实现)
 */
#include "uwb_parser.h"
#include <stdlib.h>   /* atof */
#include <string.h>   /* strchr */

/* ---- diagnostic ---- */
char g_raw_line[64];
int  g_has_raw_line = 0;
int  g_line_ok = 0, g_line_bad = 0;

void uwb_parser_init(struct UWB_Parser *p) {
    p->buffer_idx = 0;
    p->buffer[0] = '\0';
    for (int i = 0; i < 8; i++) {
        p->distances[i]      = -1.0f;
        p->last_update_ms[i] = 0;
    }
}

/**
 * 解析 $DIST,M1,S<id>,<distance_meters>
 * 返回 1 = 完成一行解析, 0 = 继续接收
 */
static int uwb_parser_parse_line(struct UWB_Parser *p, const char *line,
                                  uint32_t now_ms) {
    if (line[0] != '$' || line[1] != 'D' || line[2] != 'I' ||
        line[3] != 'S' || line[4] != 'T' || line[5] != ',') {
        if (line[0] != '\0') {
            int i;
            for (i = 0; line[i] && i < 63; i++) g_raw_line[i] = line[i];
            g_raw_line[i] = '\0';
            g_has_raw_line = 1;
        }
        g_line_bad++;
        return 0;
    }
    g_line_ok++;

    const char *ptr = line + 6;  /* skip "$DIST," */

    /* field 1: M1 (skip) */
    const char *comma = strchr(ptr, ',');
    if (!comma) return 0;
    ptr = comma + 1;

    /* field 2: S<id> */
    if (*ptr != 'S') return 0;
    ptr++;
    int slave_id = 0;
    while (*ptr >= '0' && *ptr <= '9') {
        slave_id = slave_id * 10 + (*ptr - '0');
        ptr++;
    }
    if (slave_id < 1 || slave_id > 8) return 0;
    if (*ptr != ',') return 0;
    ptr++;

    /* field 3: distance */
    float dist = (float)atof(ptr);
    if (dist <= 0.0f || dist >= 50.0f) {
        p->distances[slave_id - 1] = -1.0f;
        return 1;
    }

    p->distances[slave_id - 1]      = dist;
    p->last_update_ms[slave_id - 1] = now_ms;
    return 1;
}

int uwb_parser_feed(struct UWB_Parser *p, char c, uint32_t now_ms) {
    if (c == '\n' || c == '\r') {
        if (p->buffer_idx > 0) {
            p->buffer[p->buffer_idx] = '\0';
            p->buffer_idx = 0;
            return uwb_parser_parse_line(p, p->buffer, now_ms);
        }
        return 0;
    }
    if (p->buffer_idx < (int)(sizeof(p->buffer) - 1)) {
        p->buffer[p->buffer_idx++] = c;
    }
    return 0;
}

int uwb_parser_valid_count(struct UWB_Parser *p, uint32_t now_ms) {
    int cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (p->distances[i] >= 0.0f && p->distances[i] < 50.0f
            && (now_ms - p->last_update_ms[i]) < 500) {
            cnt++;
        }
    }
    return cnt;
}

int uwb_parser_get_valid_distances(struct UWB_Parser *p, float out[],
                                    int max_anchors, uint32_t now_ms) {
    int cnt = 0;
    for (int i = 0; i < max_anchors; i++) {
        if (p->distances[i] >= 0.0f && p->distances[i] < 50.0f
            && (now_ms - p->last_update_ms[i]) < 500) {
            out[i] = p->distances[i];
            cnt++;
        } else {
            out[i] = -1.0f;
        }
    }
    return cnt;
}
