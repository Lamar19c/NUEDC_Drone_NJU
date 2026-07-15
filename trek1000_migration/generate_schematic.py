#!/usr/bin/env python3
"""
TREK1000 原理图自动生成器 — 方案 B: DWM1000/BU01 模块 + STM32F103RCT6

读取结构化连线网表，输出 EasyEDA Pro 原理图 JSON 文件。
在 EasyEDA Pro 中: 文件 → 导入 → 选择生成的 .json 文件。

用法:
  python generate_schematic.py                    # 生成方案B
  python generate_schematic.py --variant A        # 生成方案A (裸DW1000)
  python generate_schematic.py --variant B        # 生成方案B (DWM1000模块, 默认)
"""

import json
import uuid
import argparse
from collections import OrderedDict

# ============================================================
# EasyEDA 内置库元件 UUID (基础元件, 所有版本通用)
# 用户导入后如果提示找不到元件，在 EasyEDA 库中搜索替换即可
# ============================================================

# MCU: STM32F103RCT6 LQFP64
STM32_UUID = "STM32F103RCT6_LQFP64"

# DWM1000/BU01 模块 (24pin 邮票孔)
DWM1000_UUID = "DWM1000_MODULE"

# 电源
TPS73601_UUID = "TPS73601DBVR_SOT23-5"
USB_MICRO_UUID = "USB_MICRO_B"

# 无源元件 (使用 EasyEDA 内置基础库)
RES_0402_UUID = "RES_0402"
CAP_0402_UUID = "CAP_0402"
CAP_0603_UUID = "CAP_0603"
CAP_0805_UUID = "CAP_0805"
CAP_TAN_A_UUID = "CAP_TAN_A"
LED_0603_UUID = "LED_0603"
XTAL_3225_UUID = "XTAL_3225"
IND_0402_UUID = "IND_0402"
FUSE_0805_UUID = "FUSE_0805"
HEADER_4X2_UUID = "HDR_2X4"
SW_DIP_4P_UUID = "SW_DIP_4P"

GRID = 100  # EasyEDA 网格单位 (1 = 10mil)


def gen_uid(prefix="gge"):
    """生成 EasyEDA 风格的唯一 ID"""
    return f"{prefix}{uuid.uuid4().hex[:8]}"


def make_dochead():
    return {
        "type": "DOCHEAD",
        "docType": "SCH_PAGE",
        "uuid": gen_uid("sch"),
        "client": "python-generator"
    }


def make_canvas():
    return {
        "type": "CANVAS",
        "ticket": 1,
        "originX": 0,
        "originY": 0
    }


def make_component(uid, x, y, rotation=0, mirror=False, part_id=""):
    """创建原理图元件"""
    return {
        "type": "COMPONENT",
        "id": uid,
        "ticket": 1,
        "partId": part_id,
        "groupId": 0,
        "positionX": x,
        "positionY": y,
        "rotation": rotation,
        "isMirror": mirror,
        "data": {}
    }


def make_wire(uid, points, net_name=""):
    """创建导线"""
    return {
        "type": "WIRE",
        "id": uid,
        "ticket": 1,
        "partId": "",
        "groupId": 0,
        "locked": False,
        "zIndex": 0.235,
        "dots": points,
        "strokeColor": None,
        "strokeStyle": 0,
        "fillColor": "",
        "strokeWidth": None,
        "fillStyle": 1,
        "netName": net_name
    }


def make_netlabel(uid, x, y, net_name, rotation=0):
    """创建网络标签"""
    return {
        "type": "NETLABEL",
        "id": uid,
        "ticket": 1,
        "partId": "",
        "groupId": 0,
        "positionX": x,
        "positionY": y,
        "rotation": rotation,
        "netName": net_name
    }


def make_attr(parent_id, key, value, x=0, y=0):
    """创建元件属性"""
    return {
        "type": "ATTR",
        "id": gen_uid("attr"),
        "ticket": 1,
        "partId": "",
        "groupId": 0,
        "locked": True,
        "zIndex": 0.1,
        "parentId": parent_id,
        "key": key,
        "value": value,
        "keyVisible": True,
        "valueVisible": True,
        "positionX": x,
        "positionY": y,
        "rotation": 0,
        "color": None, "fillColor": None,
        "fontFamily": None, "fontSize": None,
        "strikeout": None, "underline": None,
        "italic": None, "fontWeight": None,
        "vAlign": 0, "hAlign": 2
    }


def generate_schematic_b(variant="B"):
    """生成 方案B: DWM1000 + RCT6 原理图"""

    elements = OrderedDict()

    # ── 文档头 ──
    dh = make_dochead()
    elements["dochead"] = dh

    # ── 画布 ──
    cv = make_canvas()
    elements["canvas"] = cv

    items = []  # 按顺序添加图元

    # ═══════════════════════════════════════════════════
    # 页1: MCU + 周边 (左侧)
    # ═══════════════════════════════════════════════════

    # U? MCU: STM32F103RCT6 LQFP64 @ (400, 400)
    mcu_uid = gen_uid("mcu")
    items.append(make_component(mcu_uid, 400, 400, 0))
    items.append(make_attr(mcu_uid, "Designator", "U5"))
    items.append(make_attr(mcu_uid, "Device", STM32_UUID))
    items.append(make_attr(mcu_uid, "Footprint", "LQFP64_10X10"))

    # U? DWM1000 模块 @ (900, 400)
    dwm_uid = gen_uid("dwm")
    items.append(make_component(dwm_uid, 900, 400, 0))
    items.append(make_attr(dwm_uid, "Designator", "U1"))
    items.append(make_attr(dwm_uid, "Device", DWM1000_UUID))
    items.append(make_attr(dwm_uid, "Footprint", "DWM1000_24PIN"))

    # U? LDO TPS73601 @ (200, 200)
    ldo_uid = gen_uid("ldo")
    items.append(make_component(ldo_uid, 200, 200, 0))
    items.append(make_attr(ldo_uid, "Designator", "U3"))
    items.append(make_attr(ldo_uid, "Device", TPS73601_UUID))
    items.append(make_attr(ldo_uid, "Footprint", "SOT-23-5"))

    # CN? Micro USB @ (100, 100)
    usb_uid = gen_uid("usb")
    items.append(make_component(usb_uid, 100, 100, 0))
    items.append(make_attr(usb_uid, "Designator", "CN1"))
    items.append(make_attr(usb_uid, "Device", USB_MICRO_UUID))
    items.append(make_attr(usb_uid, "Footprint", "MICRO_USB_B"))

    # X? 12MHz STM32 HSE 晶振 @ (400, 200)
    xtal_uid = gen_uid("xtal")
    items.append(make_component(xtal_uid, 400, 200, 0))
    items.append(make_attr(xtal_uid, "Designator", "X2"))
    items.append(make_attr(xtal_uid, "Device", XTAL_3225_UUID))
    items.append(make_attr(xtal_uid, "Footprint", "XTAL_3225"))

    # ═══════════════════════════════════════════════════
    # 页2: SPI 通信线 (MCU ↔ DWM1000)
    # ═══════════════════════════════════════════════════

    spi_wires = [
        ("DW_SCK",  (500, 340), (850, 340)),    # PA5 → DWM1000 SCK
        ("DW_MISO", (500, 360), (850, 360)),    # PA6 ← DWM1000 MISO
        ("DW_MOSI", (500, 380), (850, 380)),    # PA7 → DWM1000 MOSI
        ("DW_NSS",  (500, 400), (850, 400)),    # PA4 → DWM1000 CS
        ("DW_RSTn", (500, 420), (850, 420)),    # PC5 → DWM1000 RSTn
        ("DW_IRQn", (500, 440), (850, 440)),    # PB0 ← DWM1000 IRQ
    ]

    for net, (x1, y1), (x2, y2) in spi_wires:
        w_uid = gen_uid("wire")
        items.append(make_wire(w_uid, [[x1, y1, x2, y2]], net))
        # 网络标签
        lbl_uid = gen_uid("lbl")
        items.append(make_netlabel(lbl_uid, (x1+x2)//2, (y1+y2)//2, net))

    # ═══════════════════════════════════════════════════
    # 页3: 电源网络
    # ═══════════════════════════════════════════════════

    # USB 5V → LDO → 3.3V
    power_wires = [
        ("VCC5V_USB", (150, 100), (150, 180)),
        ("VCC5V_IN",  (150, 180), (150, 220)),
        ("VDD3V3_A",  (250, 220), (350, 220)),
    ]

    for net, (x1, y1), (x2, y2) in power_wires:
        w_uid = gen_uid("wire")
        items.append(make_wire(w_uid, [[x1, y1, x2, y2]], net))
        lbl_uid = gen_uid("lbl")
        items.append(make_netlabel(lbl_uid, (x1+x2)//2, (y1+y2)//2, net))

    # Fuse: VCC5V_USB → VCC5V_IN
    fuse_uid = gen_uid("fuse")
    items.append(make_component(fuse_uid, 150, 150, 90))
    items.append(make_attr(fuse_uid, "Designator", "F1"))
    items.append(make_attr(fuse_uid, "Device", FUSE_0805_UUID))
    items.append(make_attr(fuse_uid, "Value", "0.5A"))

    # ═══════════════════════════════════════════════════
    # 页4: LDO 外围电容
    # ═══════════════════════════════════════════════════

    caps_power = [
        ("C25", CAP_TAN_A_UUID, (200, 250), 90, "10µF/16V", "VCC5V_IN"),
        ("C26", CAP_0402_UUID,   (250, 250), 90, "1µF", "VDD3V3_A"),
    ]

    for des, dev, (cx, cy), rot, val, net in caps_power:
        c_uid = gen_uid("cap")
        items.append(make_component(c_uid, cx, cy, rot))
        items.append(make_attr(c_uid, "Designator", des))
        items.append(make_attr(c_uid, "Device", dev))
        items.append(make_attr(c_uid, "Value", val))

    # ═══════════════════════════════════════════════════
    # 页5: 晶振匹配电容
    # ═══════════════════════════════════════════════════

    xtal_caps = [
        ("C3",  "10pF", (380, 160), 0, "OSC_IN"),
        ("C10", "10pF", (420, 160), 0, "OSC_OUT"),
    ]

    for des, val, (cx, cy), rot, net in xtal_caps:
        c_uid = gen_uid("cap")
        items.append(make_component(c_uid, cx, cy, rot))
        items.append(make_attr(c_uid, "Designator", des))
        items.append(make_attr(c_uid, "Device", CAP_0402_UUID))
        items.append(make_attr(c_uid, "Value", val))

    # ═══════════════════════════════════════════════════
    # 页6: 上拉电阻
    # ═══════════════════════════════════════════════════

    pullups = [
        ("R10", "10kΩ", (350, 300), 0, "NRST"),
        ("R13", "10kΩ", (350, 330), 0, "BOOT0"),
        ("R11", "10kΩ", (500, 460), 0, "DW_IRQn"),
    ]

    for des, val, (rx, ry), rot, net in pullups:
        r_uid = gen_uid("res")
        items.append(make_component(r_uid, rx, ry, rot))
        items.append(make_attr(r_uid, "Designator", des))
        items.append(make_attr(r_uid, "Device", RES_0402_UUID))
        items.append(make_attr(r_uid, "Value", val))

    # ═══════════════════════════════════════════════════
    # 页7: LED ×4
    # ═══════════════════════════════════════════════════

    for i in range(4):
        ly = 550 + i * 40
        led_uid = gen_uid("led")
        items.append(make_component(led_uid, 750, ly, 0))
        items.append(make_attr(led_uid, "Designator", f"D{i+1}"))
        items.append(make_attr(led_uid, "Device", LED_0603_UUID))
        items.append(make_attr(led_uid, "Value", "RED"))

        r_uid = gen_uid("res")
        items.append(make_component(r_uid, 700, ly, 0))
        items.append(make_attr(r_uid, "Designator", f"R_LED{i+1}"))
        items.append(make_attr(r_uid, "Device", RES_0402_UUID))
        items.append(make_attr(r_uid, "Value", "270Ω"))

    # ═══════════════════════════════════════════════════
    # 页8: 去耦电容 (DWM1000 周边)
    # ═══════════════════════════════════════════════════

    decaps = [
        ("C19", "0.1µF", "VDDBAT"),
        ("C20", "0.1µF", "VDD3V3"),
        ("C34", "0.1µF", "VDDIO"),
        ("C31", "0.1µF", "VDDDIG"),
    ]

    for i, (des, val, net) in enumerate(decaps):
        cx, cy = 900, 500 + i * 30
        c_uid = gen_uid("cap")
        items.append(make_component(c_uid, cx, cy, 90))
        items.append(make_attr(c_uid, "Designator", des))
        items.append(make_attr(c_uid, "Device", CAP_0402_UUID))
        items.append(make_attr(c_uid, "Value", val))

    # ═══════════════════════════════════════════════════
    # 页9: SWD 调试接口 (4x2 排针)
    # ═══════════════════════════════════════════════════

    swd_uid = gen_uid("swd")
    items.append(make_component(swd_uid, 100, 500, 0))
    items.append(make_attr(swd_uid, "Designator", "P1"))
    items.append(make_attr(swd_uid, "Device", HEADER_4X2_UUID))

    swd_wires = [
        ("SWDIO",  (140, 500), (350, 500)),   # PA13
        ("SWCLK",  (140, 520), (350, 520)),   # PA14
        ("NRST",   (140, 540), (350, 540)),   # NRST
    ]

    for net, (x1, y1), (x2, y2) in swd_wires:
        w_uid = gen_uid("wire")
        items.append(make_wire(w_uid, [[x1, y1, x2, y2]], net))
        lbl_uid = gen_uid("lbl")
        items.append(make_netlabel(lbl_uid, (x1+x2)//2, (y1+y2)//2, net))

    # ═══════════════════════════════════════════════════
    # 页10: UART 输出 (Tag → stm32_uwb)
    # ═══════════════════════════════════════════════════

    uart_uid = gen_uid("uart")
    items.append(make_component(uart_uid, 100, 650, 0))
    items.append(make_attr(uart_uid, "Designator", "J_UART"))
    items.append(make_attr(uart_uid, "Value", "UART OUT"))

    uart_wire = make_wire(gen_uid("wire"), [[140, 650], [350, 650]], "UART1_TX")
    items.append(uart_wire)

    # ═══════════════════════════════════════════════════
    # 组装最终 JSON
    # ═══════════════════════════════════════════════════

    output = {
        "head": {
            "docType": "SCH_PAGE",
            "uuid": dh["uuid"],
            "client": dh["client"]
        },
        "canvas": {
            "originX": cv["originX"],
            "originY": cv["originY"]
        },
        "items": items,
        "itemOrder": list(range(len(items)))
    }

    return output


def generate_schematic_a():
    """生成 方案A: 裸 DW1000 + RCT6 (含完整RF链路)"""
    items = []

    # 与方案B相同的基础部分，额外增加:
    # - U? DW1000 QFN48 裸片
    # - U? HHM1595A1 Balun
    # - X? 38.4MHz XTAL
    # - SMA 母座
    # - 0402 RF 匹配电容 (1.2pF~27pF)
    # - LXDC2HL_18A
    # - 1.8V LDO

    # TODO: 完整 A 方案 RF 链路
    output = {
        "head": {"docType": "SCH_PAGE", "uuid": gen_uid("sch")},
        "canvas": {"originX": 0, "originY": 0},
        "items": items,
        "itemOrder": []
    }
    return output


def main():
    parser = argparse.ArgumentParser(description="TREK1000 原理图生成器")
    parser.add_argument("--variant", "-v", default="B", choices=["A", "B"],
                        help="方案 A: 裸DW1000 / B: DWM1000模块 (默认)")
    parser.add_argument("--output", "-o", default=None,
                        help="输出文件路径")
    args = parser.parse_args()

    if args.variant == "A":
        schematic = generate_schematic_a()
        default_name = "trek1000_sch_dw1000_bare.json"
    else:
        schematic = generate_schematic_b()
        default_name = "trek1000_sch_dwm1000.json"

    out_path = args.output or default_name

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(schematic, f, ensure_ascii=False, indent=2)

    print(f"Schematic generated: {out_path}")
    print(f"   Component count: {len(schematic['items'])}")
    print()
    print("Import steps:")
    print("   1. Open EasyEDA Pro -> New Schematic")
    print("   2. Press F12 -> Console")
    print("   3. Paste the following:")
    print()
    print('      const fs = require("fs");')
    print(f'      const data = fs.readFileSync("{out_path}", "utf-8");')
    print('      const sch = JSON.parse(data);')
    print('      sch.items.forEach(item => {')
    print('        api("createShape", item);')
    print('      });')
    print()
    print("Note: After import, map component symbols to EasyEDA library manually.")


if __name__ == "__main__":
    main()
