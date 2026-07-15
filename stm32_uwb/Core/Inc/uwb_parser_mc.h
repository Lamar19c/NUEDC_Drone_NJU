/**
 * TREK1000 mc 协议解析器 — 参考实现
 *
 * 保存自: fb0b52f (2026-07-14 回退)
 * 用途: 迁移到 TREK1000 UWB 模块时替换 $DIST 解析
 *
 * 集成方法:
 *   1. 将 uwb_parser_parse_mc() 复制到 uwb_solver.c
 *   2. 将 uwb_parser_feed() 中的路由逻辑替换 (mc 优先, $DIST fallback)
 *   3. 不需要改动其他文件
 */

/* ========================================================================
 * TREK1000 mc 协议解析
 *
 * mc 帧格式:
 *   mc <valid_hex> <S1_mm_hex> <S2_mm_hex> <S3_mm_hex> <S4_mm_hex>
 *      <lcount_hex> <rnum_hex> <time_hex> t<tag>:a<anchor>\r\n
 *
 * 示例:
 *   mc 0f 0000064a 000005f2 00000680 00000610 000a 01 000001f4 t0:0
 *
 * 所有数值为 HEX 编码，距离单位 mm。
 * ======================================================================== */

static int uwb_parser_parse_mc(struct UWB_Parser *p, const char *line,
                                uint32_t now_ms) {
    /* validate frame header "mc " */
    if (line[0] != 'm' || line[1] != 'c' || line[2] != ' ') return 0;

    const char *ptr = line + 3;
    char *end;

    /* field 1: valid_mask (1 hex char, bitmask: bit0=S1 .. bit3=S4) */
    int mask = (int)strtol(ptr, &end, 16);
    if (ptr == end) return 0;
    ptr = end;

    /* fields 2-5: 4 distance values (HEX 8-digit, unit: mm) */
    uint32_t dists[4] = {0};
    for (int i = 0; i < 4; i++) {
        while (*ptr == ' ') ptr++;
        if (*ptr < '0') return 0;
        dists[i] = (uint32_t)strtol(ptr, &end, 16);
        if (ptr == end) return 0;
        ptr = end;
    }

    /* store valid distances: mm → m */
    for (int i = 0; i < 4; i++) {
        if (mask & (1 << i)) {
            float d = (float)dists[i] / 1000.0f;
            if (d > 0.0f && d < 50.0f) {
                p->distances[i]      = d;
                p->last_update_ms[i] = now_ms;
            }
        }
    }

    g_parser_line_ok++;
    return 1;
}

/* ========================================================================
 * 修改后的 uwb_parser_feed — mc + $DIST 双协议路由
 *
 * 替换原有的 uwb_parser_feed 即可同时支持两种格式:
 *   - mc 帧优先匹配 (TREK1000)
 *   - $DIST 帧回退匹配 (JZM01)
 * ======================================================================== */

int uwb_parser_feed(struct UWB_Parser *p, char c, uint32_t now_ms) {
    if (c == '\n' || c == '\r') {
        if (p->buffer_idx > 0) {
            p->buffer[p->buffer_idx] = '\0';
            p->buffer_idx = 0;

            /* ─── TREK1000 migration: route mc frames ─── */
            if (p->buffer[0] == 'm' && p->buffer[1] == 'c')
                return uwb_parser_parse_mc(p, p->buffer, now_ms);

            /* fallback: JZM01 $DIST */
            if (p->buffer[0] == '$')
                return uwb_parser_parse_line(p, p->buffer, now_ms);

            return 0;
        }
        return 0;
    }
    if (p->buffer_idx < (int)(sizeof(p->buffer) - 1)) {
        p->buffer[p->buffer_idx++] = c;
    }
    return 0;
}
