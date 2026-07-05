/**
 * uwb_solver.h — UWB 三边测量求解器 + 距离滤波器 + 位置中值滤波器
 *
 * 移植自 Python 版 uwb.py 的 UWBSolver + filter_distance + SerialReader
 * 使用中值滤波替代扩展卡尔曼滤波 (EKF) 对位置输出进行平滑
 */

#ifndef UWB_SOLVER_H
#define UWB_SOLVER_H

#include "uwb_config.h"

// ============================================================================
// 1. 距离滤波器 (滑动窗口 + 中位数 + 跳变限幅)
// ============================================================================

class DistanceFilter {
public:
  DistanceFilter() { reset(); }

  void reset() {
    _windowCount = 0;
  }

  /**
   * 对一帧测距数据做滤波
   * @param raw      原始测距 [d1, d2, d3, d4, ...] (数组就地修改)
   * @param count    锚点数量
   */
  void filter(float raw[], int count) {
    // 将有效数据入窗口
    if (_windowCount < DIST_WINDOW_SIZE) {
      // 窗口未满，存储完整帧
      for (int i = 0; i < count; i++) {
        _window[_windowCount][i] = raw[i];
      }
      _windowCount++;
      if (_windowCount < 3) return;  // 窗口太小，不做滤波
    } else {
      // 窗口已满，淘汰最旧帧
      for (int w = 0; w < DIST_WINDOW_SIZE - 1; w++) {
        for (int i = 0; i < count; i++) {
          _window[w][i] = _window[w + 1][i];
        }
      }
      for (int i = 0; i < count; i++) {
        _window[DIST_WINDOW_SIZE - 1][i] = raw[i];
      }
    }

    // 逐锚点做中位数滤波
    for (int i = 0; i < count; i++) {
      if (raw[i] <= 0.0f || raw[i] >= 50.0f) continue;  // 无效值不处理

      // 收集该锚点的历史值
      float hist[DIST_WINDOW_SIZE];
      int histCnt = 0;
      for (int w = 0; w < _windowCount; w++) {
        float v = _window[w][i];
        if (v > 0.0f && v < 50.0f) {
          hist[histCnt++] = v;
        }
      }
      if (histCnt < 2) continue;

      // 中位数
      float med = median(hist, histCnt);

      // 跳变限幅
      if (fabsf(raw[i] - med) > MAX_DIST_JUMP) {
        raw[i] = med;
      }
    }
  }

private:
  float _window[DIST_WINDOW_SIZE][8];  // 最多支持 8 个锚点
  int _windowCount;

  static float median(float arr[], int n) {
    float sorted[DIST_WINDOW_SIZE];
    for (int i = 0; i < n; i++) sorted[i] = arr[i];
    for (int i = 0; i < n - 1; i++) {
      for (int j = 0; j < n - i - 1; j++) {
        if (sorted[j] > sorted[j + 1]) {
          float tmp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = tmp;
        }
      }
    }
    if (n % 2 == 1) return sorted[n / 2];
    return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0f;
  }
};


// ============================================================================
// 2. 位置中值滤波器 (对解算后的 x,y,z 做平滑)
// ============================================================================

class PositionFilter {
public:
  PositionFilter() { reset(); }

  void reset() {
    _count = 0;
  }

  /**
   * 对解算后的位置做中值滤波
   * @param x, y, z  解算后的坐标 (就地修改)
   */
  void filter(float &x, float &y, float &z) {
    // 入窗口
    if (_count < POS_WINDOW_SIZE) {
      _wx[_count] = x;
      _wy[_count] = y;
      _wz[_count] = z;
      _count++;
    } else {
      // 移位并存入新值
      for (int i = 0; i < POS_WINDOW_SIZE - 1; i++) {
        _wx[i] = _wx[i + 1];
        _wy[i] = _wy[i + 1];
        _wz[i] = _wz[i + 1];
      }
      _wx[POS_WINDOW_SIZE - 1] = x;
      _wy[POS_WINDOW_SIZE - 1] = y;
      _wz[POS_WINDOW_SIZE - 1] = z;
    }

    // 窗口>=3 时应用中值滤波
    if (_count >= 3) {
      x = median(_wx, _count);
      y = median(_wy, _count);
      z = median(_wz, _count);
    }
  }

private:
  static const int POS_WINDOW_SIZE = 8;
  float _wx[POS_WINDOW_SIZE], _wy[POS_WINDOW_SIZE], _wz[POS_WINDOW_SIZE];
  int _count;

  static float median(float arr[], int n) {
    float sorted[POS_WINDOW_SIZE];
    for (int i = 0; i < n; i++) sorted[i] = arr[i];
    for (int i = 0; i < n - 1; i++) {
      for (int j = 0; j < n - i - 1; j++) {
        if (sorted[j] > sorted[j + 1]) {
          float tmp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = tmp;
        }
      }
    }
    if (n % 2 == 1) return sorted[n / 2];
    return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0f;
  }
};


// ============================================================================
// 3. UWB 串口协议解析器
// ============================================================================

class UWB_Parser {
public:
  UWB_Parser() { reset(); }

  void reset() {
    _bufferIdx = 0;
    _buffer[0] = '\0';
    for (int i = 0; i < 8; i++) _distances[i] = -1.0f;  // -1 = 未收到
  }

  /**
   * 喂入新收到的字符
   * @return true 如果解析到完整一行 $DIST
   */
  bool feed(char c) {
    if (c == '\n' || c == '\r') {
      if (_bufferIdx > 0) {
        _buffer[_bufferIdx] = '\0';
        _bufferIdx = 0;
        return parseLine(_buffer);
      }
      return false;
    }
    if (_bufferIdx < (int)(sizeof(_buffer) - 1)) {
      _buffer[_bufferIdx++] = c;
    }
    return false;
  }

  /**
   * 获取当前周期的测距数组
   * 收集到 >=3 个锚点后应将此数组传给求解器，然后调用 clearDistances()
   * 无效距离返回 -1.0f
   */
  void getDistances(float out[], int maxAnchors) {
    for (int i = 0; i < maxAnchors; i++) {
      out[i] = _distances[i];  // -1.0f = 未收到
    }
  }

  /**
   * 有效锚点数量
   */
  int validCount() {
    int cnt = 0;
    for (int i = 0; i < 8; i++) {
      if (_distances[i] >= 0.0f && _distances[i] < 50.0f) cnt++;
    }
    return cnt;
  }

  /**
   * 清空本周期缓存
   */
  void clearDistances() {
    for (int i = 0; i < 8; i++) _distances[i] = -1.0f;
  }

private:
  char _buffer[64];
  int _bufferIdx;
  float _distances[8];  // S1..S8, -1 = 未收到

  /**
   * 解析 $DIST,M1,S<id>,<distance_meters>
   */
  bool parseLine(const char* line) {
    // 检查前缀 "$DIST,"
    if (line[0] != '$' || line[1] != 'D' || line[2] != 'I' ||
        line[3] != 'S' || line[4] != 'T' || line[5] != ',') {
      return false;
    }

    // 解析字段: $DIST,M1,S3,1.234
    const char* p = line + 6;  // 跳过 "$DIST,"

    // 字段1: M1 (跳过)
    const char* comma1 = strchr(p, ',');
    if (!comma1) return false;
    p = comma1 + 1;

    // 字段2: S<id>
    if (*p != 'S') return false;
    p++;
    int slaveId = 0;
    while (*p >= '0' && *p <= '9') {
      slaveId = slaveId * 10 + (*p - '0');
      p++;
    }
    if (slaveId < 1 || slaveId > 8) return false;
    if (*p != ',') return false;
    p++;

    // 字段3: 距离值
    float dist = atof(p);
    if (dist <= 0.0f || dist >= 50.0f) {
      // 距离超出 UWB 有效范围 (0~50m), 标记为无效
      _distances[slaveId - 1] = -1.0f;
      return true;
    }

    _distances[slaveId - 1] = dist;
    return true;
  }
};


// ============================================================================
// 4. 三边测量求解器 (中值滤波替代 EKF)
// ============================================================================

class UWBSolver {
public:
  UWBSolver(const float anchors[][3], int anchorCount)
    : _anchors(anchors), _anchorCount(anchorCount)
  {
    _lastT = 0;
    _mode2D = false;
    _lastX = _lastY = _lastZ = 0;
    _hasLastPos = false;
  }

  /**
   * 求解位置 (中值滤波平滑)
   * @param distances  测距数组 (按 S1..Sn 顺序, 不可用的设为 NAN)
   * @param nowMs      当前时间 (millis())
   * @param outX, outY, outZ, outVx, outVy, outVz  输出
   * @return true 如果解算成功
   */
  bool solve(const float distances[], unsigned long nowMs,
             float &outX, float &outY, float &outZ,
             float &outVx, float &outVy, float &outVz)
  {
    // 收集有效锚点 (无效距离 = -1.0f)
    int validIdx[8];
    int validCnt = 0;
    for (int i = 0; i < _anchorCount; i++) {
      if (distances[i] > 0.0f && distances[i] < 50.0f) {
        validIdx[validCnt++] = i;
      }
    }
    if (validCnt < 3) return false;

    // 计算 dt
    float dt = 0.1f;
    if (_lastT > 0) {
      dt = (nowMs - _lastT) / 1000.0f;
    }
    _lastT = nowMs;
    if (dt < 0.02f) dt = 0.02f;
    if (dt > 2.0f) dt = 2.0f;

    // 保存上一帧位置 (用于速度计算)
    float prevX = _lastX, prevY = _lastY, prevZ = _lastZ;
    bool hadPrev = _hasLastPos;

    if (validCnt == 3) {
      if (!solve2D(validIdx, distances, outX, outY, outZ)) return false;
    } else {
      if (!solve3D(validIdx, distances, validCnt, outX, outY, outZ)) return false;
    }

    // 中值滤波平滑
    _posFilter.filter(outX, outY, outZ);

    // 保存滤波后位置
    _lastX = outX; _lastY = outY; _lastZ = outZ;
    _hasLastPos = true;

    // 速度: 有限差分
    if (hadPrev && dt > 0.001f) {
      outVx = (outX - prevX) / dt;
      outVy = (outY - prevY) / dt;
      outVz = (outZ - prevZ) / dt;
    } else {
      outVx = outVy = outVz = 0;
    }

    return true;
  }

private:
  const float (*_anchors)[3];
  int _anchorCount;
  unsigned long _lastT;
  bool _mode2D;
  PositionFilter _posFilter;
  float _lastX, _lastY, _lastZ;
  bool _hasLastPos;

  /**
   * 3D 求解 (4+ 锚点) — 加权最小二乘
   */
  bool solve3D(const int idx[], const float dist[], int n,
               float &outX, float &outY, float &outZ)
  {
    _mode2D = false;

    // 以第一个锚点为参考
    int i0 = idx[0];
    float p0x = _anchors[i0][0], p0y = _anchors[i0][1], p0z = _anchors[i0][2];
    float d0 = dist[i0];
    float p0_norm_sq = p0x * p0x + p0y * p0y + p0z * p0z;

    int m = n - 1;  // 方程数
    // 构建 A^T A (3x3) 和 A^T b (3x1) 通过加权正规方程
    float ATA[9] = {0};
    float ATb[3] = {0};

    for (int j = 0; j < m; j++) {
      int ij = idx[j + 1];
      float px = _anchors[ij][0], py = _anchors[ij][1], pz = _anchors[ij][2];
      float dj = dist[ij];

      float w = 1.0f / (dj + 0.1f);  // 距离越近权重越高

      // A 的第 j 行 = 2 * (p_j - p0)
      float a0 = 2.0f * (px - p0x) * w;
      float a1 = 2.0f * (py - p0y) * w;
      float a2 = 2.0f * (pz - p0z) * w;

      // b 的第 j 个元素
      float pj_norm_sq = px * px + py * py + pz * pz;
      float bj = (d0 * d0 - dj * dj - p0_norm_sq + pj_norm_sq) * w;

      // 累加 A^T A 和 A^T b
      ATA[0] += a0 * a0;  ATA[1] += a0 * a1;  ATA[2] += a0 * a2;
      ATA[3] += a1 * a0;  ATA[4] += a1 * a1;  ATA[5] += a1 * a2;
      ATA[6] += a2 * a0;  ATA[7] += a2 * a1;  ATA[8] += a2 * a2;

      ATb[0] += a0 * bj;
      ATb[1] += a1 * bj;
      ATb[2] += a2 * bj;
    }

    // 求解 ATA * result = ATb
    float result[3];
    if (!solve3x3(result, ATA, ATb)) return false;

    // 合法性检查
    for (int i = 0; i < 3; i++) {
      if (fabsf(result[i]) > 100.0f) return false;  // 解算失败
    }

    // 坐标范围约束
    outX = fmaxf(POS_CLAMP_MIN, fminf(POS_CLAMP_MAX, result[0]));
    outY = fmaxf(POS_CLAMP_MIN, fminf(POS_CLAMP_MAX, result[1]));
    outZ = fmaxf(POS_CLAMP_MIN, fminf(POS_CLAMP_MAX, result[2]));
    return true;
  }

  /**
   * 2D 求解 (3 锚点降级) — Z 固定
   */
  bool solve2D(const int idx[], const float dist[],
               float &outX, float &outY, float &outZ)
  {
    _mode2D = true;
    float fixed_z = DEFAULT_HEIGHT;

    int i0 = idx[0], i1 = idx[1], i2 = idx[2];
    float p0x = _anchors[i0][0], p0y = _anchors[i0][1], p0z = _anchors[i0][2];
    float p1x = _anchors[i1][0], p1y = _anchors[i1][1], p1z = _anchors[i1][2];
    float p2x = _anchors[i2][0], p2y = _anchors[i2][1], p2z = _anchors[i2][2];

    float d0 = dist[i0], d1 = dist[i1], d2 = dist[i2];
    float p0_norm_sq = p0x * p0x + p0y * p0y + p0z * p0z;

    // 构建加权 A (2x2) 和 b (2x1)
    float w1 = 1.0f / (d1 + 0.1f);
    float w2 = 1.0f / (d2 + 0.1f);

    float A00 = 2.0f * (p1x - p0x) * w1;
    float A01 = 2.0f * (p1y - p0y) * w1;
    float A10 = 2.0f * (p2x - p0x) * w2;
    float A11 = 2.0f * (p2y - p0y) * w2;

    float p1_norm_sq = p1x * p1x + p1y * p1y + p1z * p1z;
    float p2_norm_sq = p2x * p2x + p2y * p2y + p2z * p2z;

    float b0 = (d0 * d0 - d1 * d1 - p0_norm_sq + p1_norm_sq
                - 2.0f * (p1z - p0z) * fixed_z) * w1;
    float b1 = (d0 * d0 - d2 * d2 - p0_norm_sq + p2_norm_sq
                - 2.0f * (p2z - p0z) * fixed_z) * w2;

    // ATA = A^T A (2x2), ATb = A^T b (2x1)
    float ATA0 = A00 * A00 + A10 * A10;
    float ATA1 = A00 * A01 + A10 * A11;
    float ATA2 = ATA1;  // 对称
    float ATA3 = A01 * A01 + A11 * A11;

    float ATb0 = A00 * b0 + A10 * b1;
    float ATb1 = A01 * b0 + A11 * b1;

    // 求解 2x2
    float det = ATA0 * ATA3 - ATA1 * ATA2;
    if (fabsf(det) < 1e-10f) return false;

    float inv_det = 1.0f / det;
    float result_x = ( ATA3 * ATb0 - ATA1 * ATb1) * inv_det;
    float result_y = (-ATA2 * ATb0 + ATA0 * ATb1) * inv_det;

    if (fabsf(result_x) > 100.0f || fabsf(result_y) > 100.0f) return false;

    outX = fmaxf(POS_CLAMP_MIN, fminf(POS_CLAMP_MAX, result_x));
    outY = fmaxf(POS_CLAMP_MIN, fminf(POS_CLAMP_MAX, result_y));
    outZ = fixed_z;
    return true;
  }

  /**
   * 3x3 线性方程组求解 (克拉默法则)
   */
  static bool solve3x3(float out[3], const float A[9], const float b[3]) {
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);

    if (fabsf(det) < 1e-10f) return false;

    float inv_det = 1.0f / det;

    // Cramer's rule: x_i = det(A_i) / det(A)
    out[0] = (b[0] * (A[4] * A[8] - A[5] * A[7])
           -  A[1] * (b[1] * A[8] - A[5] * b[2])
           +  A[2] * (b[1] * A[7] - A[4] * b[2])) * inv_det;

    out[1] = (A[0] * (b[1] * A[8] - A[5] * b[2])
           -  b[0] * (A[3] * A[8] - A[5] * A[6])
           +  A[2] * (A[3] * b[2] - b[1] * A[6])) * inv_det;

    out[2] = (A[0] * (A[4] * b[2] - b[1] * A[7])
           -  A[1] * (A[3] * b[2] - b[1] * A[6])
           +  b[0] * (A[3] * A[7] - A[4] * A[6])) * inv_det;

    return true;
  }
};

#endif // UWB_SOLVER_H
