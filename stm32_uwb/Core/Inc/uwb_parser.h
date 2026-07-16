/**
 * uwb_parser.h — $DIST 协议解析器 (独立模块)
 */
#ifndef UWB_PARSER_H
#define UWB_PARSER_H

#include <stdint.h>

struct UWB_Parser {
    char     buffer[64];
    int      buffer_idx;
    float    distances[8];      /* S1..S8, -1.0f = 未收到 */
    uint32_t last_update_ms[8]; /* per-anchor last update timestamp (ms) */
};

void uwb_parser_init(struct UWB_Parser *p);
int  uwb_parser_feed(struct UWB_Parser *p, char c, uint32_t now_ms);
int  uwb_parser_valid_count(struct UWB_Parser *p, uint32_t now_ms);
int  uwb_parser_get_valid_distances(struct UWB_Parser *p, float out[],
                                     int max_anchors, uint32_t now_ms);

extern char g_raw_line[64];
extern int  g_has_raw_line;
extern int  g_line_ok, g_line_bad;

#endif
