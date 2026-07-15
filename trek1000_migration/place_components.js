/**
 * TREK1000 原理图自动绘制 — 方案 B: DWM1000/BU01 + STM32F103RCT6
 *
 * 用法:
 *   1. 打开 EasyEDA Pro 桌面版
 *   2. 新建一个原理图工程，打开空白原理图页
 *   3. Ctrl+Shift+I → Console
 *   4. 粘贴本脚本全部代码，回车执行
 *   5. 等待 20-30 秒，元件逐个放置
 *
 * 如果元件找不到，脚本会打印提示，手动从库中拖入即可。
 */

(async function() {
    const SCALE = 10;  // EasyEDA 单位: 1 = 10mil

    function pos(x, y) { return [x * SCALE, y * SCALE]; }

    // =========================================================
    // 元件定义: [Designator, 搜索关键词, x, y, rotation]
    // =========================================================
    const components = [

        // ── MCU: STM32F103RCT6 ──
        ["U5", "STM32F103RCT6", 400, 400, 0],

        // ── DWM1000 模块 ──
        ["U1", "DWM1000", 900, 400, 0],

        // ── LDO ──
        ["U3", "TPS73601DBVR", 200, 250, 0],

        // ── Micro USB ──
        ["CN1", "MICRO USB B", 100, 100, 0],

        // ── 12MHz 晶振 ──
        ["X2", "12MHz 3225", 400, 200, 0],

        // ── 0.5A 自恢复保险丝 ──
        ["F1", "0.5A PTC 0805", 150, 150, 0],

        // ── SWD 排针 2x4 ──
        ["P1", "Header 2x4", 100, 500, 0],

        // ── UART 输出排针 ──
        ["J1", "Header 1x4", 100, 650, 0],

        // ── 电容 ──
        ["C25", "10uF TAN A", 180, 200, 90],       // 钽电容 LDO IN
        ["C26", "1uF 0402", 240, 250, 90],           // LDO OUT
        ["C2", "10uF 0805", 850, 480, 90],           // DWM1000 VDDBAT
        ["C32", "4.7uF 0603", 880, 480, 90],         // DWM1000 VDD1V8
        ["C35", "4.7uF 0603", 910, 480, 90],         // DWM1000 VDD1V8
        ["C19", "0.1uF 0402", 940, 480, 90],         // DWM1000 去耦
        ["C20", "0.1uF 0402", 970, 480, 90],         // DWM1000 去耦
        ["C34", "0.1uF 0402", 1000, 480, 90],        // DWM1000 去耦
        ["C31", "0.1uF 0402", 1030, 480, 90],        // DWM1000 去耦
        ["C3", "10pF 0402", 370, 160, 0],            // STM32 OSC_IN
        ["C10", "10pF 0402", 430, 160, 0],           // STM32 OSC_OUT

        // ── 电阻 ──
        ["R10", "10k 0402", 340, 480, 0],            // NRST 上拉
        ["R13", "10k 0402", 340, 510, 0],            // BOOT0 上拉
        ["R11", "10k 0402", 500, 500, 0],            // DW_IRQ 上拉

        // ── LED 指示灯 ──
        ["D1", "LED RED 0603", 750, 550, 0],
        ["D2", "LED RED 0603", 750, 590, 0],
        ["D3", "LED RED 0603", 750, 630, 0],
        ["D4", "LED RED 0603", 750, 670, 0],

        // ── LED 限流电阻 ──
        ["R_LED1", "270 0402", 700, 550, 0],
        ["R_LED2", "270 0402", 700, 590, 0],
        ["R_LED3", "270 0402", 700, 630, 0],
        ["R_LED4", "270 0402", 700, 670, 0],
    ];

    // =========================================================
    // 搜索并放置元件
    // =========================================================
    let placed = 0;
    let failed = [];

    for (const [designator, keyword, cx, cy, rot] of components) {
        try {
            // 搜索库
            const results = await eda.lib_Device.search(keyword, 0, 5);
            if (!results || results.length === 0) {
                failed.push([designator, keyword, "not found in library"]);
                continue;
            }
            const dev = results[0];  // 取第一个搜索结果
            const [x, y] = pos(cx, cy);

            // 放置
            await eda.sch_PrimitiveComponent.create(
                { libraryUuid: dev.libraryUuid, uuid: dev.uuid },
                x, y, "", rot, false, true, true
            );

            // 修改位号 (需要选中后才能改)
            // 位号会在放置后自动分配，手动调一下即可

            placed++;
        } catch (e) {
            failed.push([designator, keyword, e.message || String(e)]);
        }
    }

    // =========================================================
    // 报告
    // =========================================================
    console.log(`=== TREK1000 Schematic Generation ===`);
    console.log(`Placed: ${placed} / ${components.length}`);
    if (failed.length > 0) {
        console.log(`Failed (${failed.length} items):`);
        for (const [des, kw, reason] of failed) {
            console.log(`  ${des} (${kw}): ${reason}`);
        }
        console.log(`\nPlease place these manually from EasyEDA library.`);
    }
    console.log(`\nNext steps:`);
    console.log(`  1. Adjust designators (U?, C?, R?, etc.)`);
    console.log(`  2. Draw wires per netlist in trek1000_migration/HARDWARE.md`);
    console.log(`  3. Add net labels (VDD3V3, GND, DW_SCK, etc.)`);
    console.log(`=========================================`);

})();
