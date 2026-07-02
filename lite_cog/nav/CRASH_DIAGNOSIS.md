# hdl_localization 崩溃诊断记录

## 崩溃现象

- **信号**: SIGSEGV
- **Fault Address**: `0x1a8`（稳定复现，多次崩溃相同地址）
- **崩溃位置**: `Eigen::PartialPivLU<MatrixXf>::compute()` — 矩阵求逆时的 LU 分解
- **时机**: 第一次 `ukf->correct(observation)` 调用内部，`expected_measurement_cov.inverse()` 时

## 诊断时间线

| 时间 | 事件 |
|------|------|
| 1778839704.124 | 点云回调开始，globalmap 尚未到达 |
| 1778839705.004 | globalmap 到达 |
| 1778839705.027 | 第一个 points_callback 进入 predict 前 (cov norm=0.04) |
| 1778839705.028 | correct: align 前 (cov norm=0.04) |
| 1778839705.126 | correct: align 完成 (收敛), ukf->correct 前 (cov norm=0.04, hasNaN=0) |
| 1778839708.948 | Bond broken（心跳超时检测到崩溃，~3.8s 后） |

## 修改的代码

### 1. 已应用的修复

**`apps/hdl_localization_nodelet.cpp` line 307-310**: globalmap_callback 中 `registration->setInputTarget(globalmap)` 加了 mutex 锁。

```cpp
// 修改前
registration->setInputTarget(globalmap);

// 修改后
{
  std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
  registration->setInputTarget(globalmap);
}
```

**原因**: globalmap_callback 在 `nh` 线程，points_callback 在 `mt_nh` 线程，两者共享 `registration` 对象。`setInputTarget` 会重建体素网格，`align` 会读取体素网格，无锁并发访问是数据竞争。

**效果**: 未解决崩溃，崩溃推迟了约 4 秒但仍发生。

### 2. 诊断日志（当前代码已添加但未测试）

**`include/hdl_localization/pose_estimator.hpp`**: 添加 `cov_norm()` 方法声明。

**`src/hdl_localization/pose_estimator.cpp`**:
- 添加 `cov_norm()` 实现
- `correct()` 中添加 ROS_INFO: align 前后、ukf->correct 前后
- `correct()` 中添加 try-catch 包裹 ukf->correct
- `predict(control)` 中添加 NaN 检测

**`apps/hdl_localization_nodelet.cpp`**: points_callback 中添加 cov norm 打印。

### 3. 编译问题（未解决）

`NODELET_ERROR(tfError.c_str())` 在 `-DCMAKE_BUILD_TYPE=RelWithDebInfo` 下触发 `-Werror=format-security`。当前使用默认编译选项可通过（仅 warning）。

## Fault Address 分析

`0x1a8` = 424 字节偏移。典型含义：**NULL 指针 + 424 字节偏移处的成员访问**。

即某对象的 `this` 为 NULL，访问其偏移 424 字节处的成员。该对象可能是 `PartialPivLU` 内部的矩阵存储。

## 可能根因（按可能性排序）

1. **UKF 协方差数值问题**: `ensurePositiveFinite()` 被完全禁用（函数第一行就 `return`），协方差可能变为非正定，导致 Cholesky/LU 分解失败。
2. **`downsample_filter` 并发**: `downsample()` 在 mutex 之外使用共享的 `pcl::VoxelGrid`，多线程并发调用可能破坏堆内存，间接导致 Eigen 内部分配失败。
3. **NDT 内部状态**: 虽然 mutex 保护了 `setInputTarget` 和 `align`，但 NDT OpenMP 并行 (`#pragma omp parallel for`) 可能引入内部竞争。
4. **`globalmap` shared_ptr 竞争**: 第 305 行写、第 200 行读，C++ 标准下是未定义行为（但 x86 下通常不会直接导致此崩溃）。

## 下一步

1. ~~修复编译问题后测试带诊断日志的版本，确认崩溃精确位置~~ ✅ 编译通过，已部署运行（2026-06-15 21:00）
2. ~~如果崩溃在 `ukf->correct` 内部，修复 `ensurePositiveFinite`~~ ✅ 已加固（2026-06-15 21:05）
3. ~~将 `downsample()` 移入 mutex 保护范围~~ ✅ 已在 `pose_estimator_mutex` 内（line 238-240）

## 2026-06-15 第二次加固（21:05）

诊断日志显示系统当前运行稳定（converged=1, hasNaN=0, cov norm ~3.9-4.4）。为防止未来在退化场景下崩溃，应用了以下防御性修复：

### Fix 1: `ensurePositiveFinite()` 加固 (`unscented_kalman_filter.hpp`)

- 特征值分解前强制对称化：`cov = 0.5 * (cov + cov.transpose())`
- 添加 `solver.info() != Eigen::Success` 检查，失败时重置为单位矩阵
- 防止数值漂移破坏 `SelfAdjointEigenSolver` 导致崩溃

### Fix 2: `expected_measurement_cov` 求逆保护 (`unscented_kalman_filter.hpp`)

- 在 `kalman_gain = sigma * expected_measurement_cov.inverse()` 前调用 `ensurePositiveFinite(expected_measurement_cov)`
- 这直接防止了原始崩溃（`PartialPivLU::compute()` at 0x1a8）

### Fix 3: 协方差融合求逆保护 (`pose_estimator.cpp`)

- 新增 `safeInverse()` 静态辅助函数，使用特征值正则化求安全逆
- `imu_cov.inverse()` → `safeInverse(imu_cov)`
- `odom_cov.inverse()` → `safeInverse(odom_cov)`
- `(inv_imu_cov + inv_odom_cov).inverse()` → `safeInverse(inv_imu_cov + inv_odom_cov)`
- 防止 IMU/odom 协方差奇异时融合公式崩溃
