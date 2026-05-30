# 2D 物理沙盒 — 问题排查与解决方案记录

> 本文档记录了项目开发过程中遇到的所有问题及其根因分析与修复方案。
> 按照时间顺序排列，便于日后查阅与复盘。

---

## 目录

1. [CMake 缓存路径不匹配导致构建失败](#1-cmake-缓存路径不匹配导致构建失败)
2. [`std::numeric_ok_nan` 不存在](#2-stdnumeric_ok_nan-不存在)
3. [未使用的头文件 `<stdexcept>` 引发编译警告](#3-未使用的头文件-stdexcept-引发编译警告)
4. [`isPointInPolygon` 是 `Collider` 的私有成员](#4-ispointinpolygon-是-collider-的私有成员)
5. [未知类型 `QGroupBox`](#5-未知类型-qgroupbox)
6. [粒子密度过大时穿透墙体](#6-粒子密度过大时穿透墙体)
7. [绘制的墙体刚体不受影响](#7-绘制的墙体刚体不受影响)
8. [粒子数量增多后帧率大幅下降](#8-粒子数量增多后帧率大幅下降)
9. [刚体穿模（深穿透漏洞）与粒子陷入刚体内部（叉乘符号翻转）](#9-刚体穿模深穿透漏洞与粒子陷入刚体内部叉乘符号翻转)
10. [刚体在高压下仍然穿模（冲量累积导致速度爆炸）](#10-刚体在高压下仍然穿模冲量累积导致速度爆炸)
11. [渲染终极瓶颈：突破几千粒子的 CPU 渲染极限](#11-渲染终极瓶颈突破几千粒子的-cpu-渲染极限)

---

## 1. CMake 缓存路径不匹配导致构建失败

### 错误信息

```
CMake Error: The current CMakeCache.txt directory
D:/c++big_homework/build/CMakeCache.txt is different than the directory
d:/c++大作业/build where CMakeCache.txt was created.
```

### 根因分析

项目文件夹从 `d:\c++大作业` 被重命名为 `d:\c++big_homework`。CMake 在首次配置（Configure）时，会将项目源文件目录、编译器路径等**绝对路径硬编码**写入 `build/CMakeCache.txt` 和 `build.ninja` 等构建文件中。文件夹重命名后，缓存中记录的旧路径与实际路径不匹配，导致 CMake 拒绝继续构建。

### 解决方案

**彻底清空 `build` 目录并重新配置：**

```powershell
# 1. 删除旧的构建缓存
Remove-Item -Path "d:\c++big_homework\build\*" -Recurse -Force

# 2. 重新配置（指定生成器和编译器）
cmake -S d:\c++big_homework -B d:\c++big_homework\build -G Ninja ^
    -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++

# 3. 重新构建
cmake --build d:\c++big_homework\build
```

### 经验教训

- **永远不要重命名包含 CMake 构建目录的文件夹**，除非同时清空 `build` 目录。
- 如果必须重命名/移动项目，始终在移动后执行 `cmake --build build --clean-first` 或直接删除 `build/` 文件夹重新配置。

---

## 2. `std::numeric_ok_nan` 不存在

### 错误信息

```
error: No member named 'numeric_ok_nan' in namespace 'std'
```

**文件**：`src/core/rigidbody/Collider.cpp`，第 91 行

### 根因分析

代码中错误地使用了 `std::numeric_ok_nan`，这并不是 C++ 标准库中的任何合法符号。正确的写法应该是 `std::numeric_limits<float>::quiet_NaN()`（表示 quiet NaN，即"安静的非数字"浮点值），用于初始化一个表示"无效/未设置"的浮点变量。

### 解决方案

**替换为正确的标准库调用：**

```diff
- float minOverlap = std::numeric_ok_nan;
+ float minOverlap = std::numeric_limits<float>::max();
```

> **注意**：在碰撞检测中，该变量的意图是找到最小穿透深度，因此使用 `max()` 作为初始值（后续通过比较不断取小值）比 NaN 更语义化，也更安全。

---

## 3. 未使用的头文件 `<stdexcept>` 引发编译警告

### 错误信息

```
warning: Included header <stdexcept> is not used directly (fix available)
```

**文件**：`src/core/Math.hpp`，第 4 行

### 根因分析

`Math.hpp` 中包含了 `#include <stdexcept>`，但文件内并没有使用 `std::runtime_error`、`std::out_of_range` 等 `<stdexcept>` 中定义的任何异常类。这是一个残留的无用 include，触发了编译器的 "unused include" 静态分析警告。

### 解决方案

**直接删除该行：**

```diff
  #pragma once
  #include <cmath>
  #include <algorithm>
- #include <stdexcept>
  #include <limits>
```

### 经验教训

- 保持头文件依赖最小化（Include What You Use 原则），可以加速编译并减少不必要的耦合。
- 建议在项目中启用 `-Wunused-includes`（Clang）或使用 `iwyu` 工具定期扫描。

---

## 4. `isPointInPolygon` 是 `Collider` 的私有成员

### 错误信息

```
error: 'isPointInPolygon' is a private member of 'Collider'
```

**文件**：`src/core/World.cpp`，第 467 行

### 根因分析

`World.cpp` 中的 `resolveParticleRigidCollisions()` 函数需要判断粒子是否在多边形刚体内部，因此调用了 `Collider::isPointInPolygon()`。但该函数在 `Collider` 类中被声明为 `private`，外部代码无法访问。

### 解决方案

**将 `isPointInPolygon` 的访问级别从 `private` 改为 `public`：**

```diff
  class Collider {
  public:
      static bool detectCollision(ContactManifold& manifold, RigidBody* a, RigidBody* b);
+     static bool isPointInPolygon(const Vec2f& point, const RigidBody& body);

  private:
      static bool circleVsCircle(ContactManifold& m, RigidBody* a, RigidBody* b);
      static bool polygonVsPolygon(ContactManifold& m, RigidBody* a, RigidBody* b);
      static bool circleVsPolygon(ContactManifold& m, RigidBody* circle, RigidBody* polygon);
-     static bool isPointInPolygon(const Vec2f& point, const RigidBody& body);
  };
```

### 设计考量

`isPointInPolygon` 是一个纯几何工具函数（判断点是否在凸多边形内），功能上属于通用的几何查询，不仅在碰撞检测内部使用，也被物理求解器（World）需要。将其设为 `public static` 是合理的设计选择。

---

## 5. 未知类型 `QGroupBox`

### 错误信息

```
error: Unknown type name 'QGroupBox'
```

**文件**：`src/gui/MainWindow.cpp`，第 120 行

### 根因分析

代码中使用了 `QGroupBox` 控件（Qt 的分组框控件），但没有包含其对应的头文件 `<QGroupBox>`。Qt 的每个 Widget 类都需要显式 include 其头文件，不会通过其他头文件隐式引入。

### 解决方案

**在文件头部添加缺失的 include：**

```diff
  #include "MainWindow.hpp"
  #include <QVBoxLayout>
  #include <QHBoxLayout>
  #include <QPushButton>
  #include <QLabel>
  #include <QSlider>
+ #include <QGroupBox>
```

### 经验教训

- Qt 遵循严格的 "一个类一个头文件" 原则。使用任何 Qt Widget 类时，必须确保包含了其同名头文件。
- 常见遗漏的 Qt 头文件还有：`<QComboBox>`、`<QSpinBox>`、`<QCheckBox>`、`<QScrollArea>` 等。

---

## 6. 粒子密度过大时穿透墙体

### 现象描述

当大量粒子堆积在墙体附近时（例如使用发射器持续喷射），粒子会逐渐穿过用户绘制的墙壁线段，好像墙壁不存在一样。粒子数量较少时则不会出现此问题。

### 根因分析

问题出在 `resolveCollisions()` 函数的**碰撞解算顺序**：

```
Step 1: 粒子-粒子碰撞 → 位置修正（推挤）
Step 2: 粒子-边界碰撞 → 位置修正（夹紧）
Step 3: 粒子-墙壁碰撞 → 位置修正（推出）
Step 4: 粒子-刚体碰撞 → 位置修正（推挤）
```

虽然 Step 3 将粒子推出墙壁，但问题是 **Step 1 的粒子间推挤**会在此之前或在下一次迭代中将粒子重新推回墙壁内部。当粒子密度极高时，N 个粒子的累积推力远超单次墙壁修正的推力，导致粒子在每帧被"挤穿"墙壁。

此外，Step 4（粒子-刚体碰撞）也可能在 Step 3 之后将粒子推穿墙壁，而不会再有后续的墙壁碰撞修正。

### 解决方案

**在所有粒子碰撞解算完成后，增加一轮「Post-Pass 二次加固校验」**：

```cpp
// resolveCollisions() 内部

// ... Step 1~4 (粒子-粒子, 边界, 墙壁, 粒子-刚体) ...

// ==========================================
// 2.5 粒子-墙体/边界 二次加固校验 (Post-Pass)
//     粒子-粒子 和 粒子-刚体 碰撞解算可能把粒子推穿墙壁，
//     这里再做一次强制位置夹紧，防止高密度下穿透。
// ==========================================
if (pCount > 0) {
    for (int i = 0; i < pCount; ++i) {
        Particle& p = m_particles[i];
        if (p.isStatic) continue;
        resolveBoundaryCollisions(p);
        resolveWallCollisions(p);
    }
}
```

**修改文件**：`src/core/World.cpp` — `resolveCollisions()` 函数

### 效果

Post-Pass 确保无论前面的碰撞阶段如何推挤粒子，最终位置一定满足墙壁约束。相当于将墙壁碰撞作为"硬约束"，在所有"软碰撞"之后强制执行。

---

## 7. 绘制的墙体刚体不受影响

### 现象描述

在画布上绘制了墙壁线段（Wall），粒子会被墙壁挡住并反弹，但刚体（矩形方块、圆形球等）完全无视这些墙壁，直接穿过。

### 根因分析

检查 `resolveCollisions()` 的碰撞管线（pipeline），发现所有碰撞类型的覆盖情况如下：

| 碰撞对 | 处理函数 | 是否实现 |
|---|---|:---:|
| 粒子 vs 世界边界 | `resolveBoundaryCollisions()` | ✅ |
| 粒子 vs 绘制墙壁 | `resolveWallCollisions()` | ✅ |
| 粒子 vs 刚体 | `resolveParticleRigidCollisions()` | ✅ |
| 刚体 vs 刚体 | `Collider::detectCollision()` + SAT | ✅ |
| 刚体 vs 世界边界 | `resolveRigidBoundaryCollisions()` | ✅ |
| **刚体 vs 绘制墙壁** | **无** | **❌** |

**`resolveRigidWallCollisions()` 函数完全不存在！** 这是一个遗漏的碰撞类型，导致刚体看不到任何用户绘制的墙壁。

### 解决方案

**新增 `resolveRigidWallCollisions()` 函数**，分别处理两种刚体形状与墙壁线段的碰撞：

#### 圆形刚体 vs 墙壁

与粒子-墙壁碰撞算法相同（点到线段最近距离），但作用于刚体的 `position` 和 `velocity`：

```cpp
// 计算刚体中心到墙壁线段的最近点
Vec2f closest = wall.start + ab * t;
float dist = b.position.distance(closest);

if (dist < b.radius && dist > 0.0f) {
    Vec2f normal = (b.position - closest) / dist;
    // 位置修正 + 速度反射 + 角速度阻尼
}
```

#### 多边形刚体 vs 墙壁

逐顶点检测是否穿过墙壁法线平面：

```cpp
// 对每个顶点，计算到墙壁线段的有符号距离
Vec2f toVertex = vertex - closestPointOnWall;
float signedDist = toVertex.dot(wallNormal);

if (signedDist < threshold) {
    // 顶点穿过墙壁 → 累积位置修正
    totalCorrection += wallNormal * penetration;
}
```

**修改文件**：
- `src/core/World.hpp` — 新增 `resolveRigidWallCollisions(RigidBody& b)` 声明
- `src/core/World.cpp` — 新增函数实现 + 在 `resolveCollisions()` 末尾调用

### 最终碰撞管线

修复后的完整碰撞管线：

```
1.   粒子-粒子 自碰撞 (空间哈希加速)
       ↓ 每个粒子处理后立即执行：
       └→ 粒子-边界碰撞 + 粒子-墙壁碰撞
2.   粒子-刚体 碰撞
2.5  ★ 粒子-边界/墙壁 Post-Pass 加固校验  ← [新增]
3.   刚体-刚体 碰撞 (SAT + 冲量求解)
4.   刚体-边界 碰撞
5.   ★ 刚体-墙壁 碰撞                      ← [新增]
```

---

## 8. 粒子数量增多后帧率大幅下降

### 现象描述

当粒子数量超过约 2000 个时，FPS 从 60 骤降至 10~20 以下，程序变得明显卡顿。通过性能面板观察，Physics Time 和 Render Time 均显著增长。

### 根因分析（多处瓶颈）

经代码审查，发现以下 **5 个性能瓶颈**：

#### 瓶颈 A：渲染端 — QPainter 状态切换（最大瓶颈）

```cpp
// 旧代码：每个粒子都触发一次 setBrush 状态切换
for (const auto& p : particles) {
    if (p.type == ParticleType::Fluid) {
        painter.setBrush(QColor(0, 220, 220, 210));  // ← 状态切换！
    } else {
        painter.setBrush(QColor(60, 150, 250, 200));  // ← 状态切换！
    }
    painter.drawEllipse(...);
}
```

QPainter 每次 `setBrush()` 都会重建内部渲染状态，5000 个粒子 = 5000 次状态切换。

#### 瓶颈 B：渲染端 — 粒子抗锯齿

对半径仅 3px 的小圆形开启 `Antialiasing`，视觉几乎无差异，但 QPainter 的抗锯齿会对每个像素做额外的采样混合计算，开销约 3~5 倍。

#### 瓶颈 C：物理端 — `positions` 向量重复堆分配

每个子步中，`positions` 向量被分配了 **3 次**：
1. `substep()` 中构建空间哈希
2. `resolveCollisions()` 中重建空间哈希
3. `SPHSolver::updateSPH()` 中又分配一次

每次分配都是 `new float[N*2]` + 填充 + 离开作用域后 `delete`。5000 粒子 × 4 子步 × 3 次 = 每帧 60000 次堆操作。

#### 瓶颈 D：SPH 核函数 — `std::pow()` 调用

```cpp
float poly6Coeff = 315.0f / (64.0f * PI * std::pow(h, 9.0f));  // 慢！
float spikyCoeff = -45.0f / (PI * std::pow(h, 6.0f));           // 慢！
```

`std::pow(h, 9.0f)` 内部使用 `exp(9 * log(h))` 实现，远慢于手动乘法 `h3 * h3 * h3`。

#### 瓶颈 E：SPH 邻居查询 — `neighbors` 向量重复分配

SPH 每次 `query()` 都向局部 `std::vector<int>` 中 push_back，频繁触发堆扩容。

### 解决方案

#### 修复 A & B：渲染端批量绘制 + 关闭粒子抗锯齿

```cpp
// 关闭粒子的抗锯齿
painter.setRenderHint(QPainter::Antialiasing, false);
painter.setPen(Qt::NoPen);

// 按类型分批绘制，仅切换 2 次 setBrush（而非 N 次）
painter.setBrush(QColor(60, 150, 250, 200));
for (const auto& p : particles) {
    if (p.type != ParticleType::Normal) continue;
    if (p.radius <= 2.5f) {
        painter.drawRect(...);  // 小粒子用矩形替代椭圆，快约 3 倍
    } else {
        painter.drawEllipse(...);
    }
}

painter.setBrush(QColor(0, 220, 220, 210));
for (const auto& p : particles) {
    if (p.type != ParticleType::Fluid) continue;
    // 同上...
}

// 恢复抗锯齿给后续 UI 元素
painter.setRenderHint(QPainter::Antialiasing, true);
```

#### 修复 C：`m_positionsCache` 复用缓存

在 `PhysicsWorld` 类中添加 `std::vector<Vec2f> m_positionsCache` 成员变量，所有子步共用同一块内存：

```diff
  // World.hpp
  private:
+     std::vector<Vec2f> m_positionsCache;
```

`substep()` 和 `resolveCollisions()` 不再各自分配本地 `positions`，而是复用 `m_positionsCache`。

#### 修复 D：手动乘法替代 `std::pow()`

```diff
- float poly6Coeff = 315.0f / (64.0f * PI * std::pow(h, 9.0f));
+ float h3 = h * h * h;
+ float h6 = h3 * h3;
+ float h9 = h6 * h3;
+ float poly6Coeff = 315.0f / (64.0f * PI * h9);
```

#### 修复 E：SPH `m_neighbors` 成员复用

在 `SPHSolver` 类中添加 `std::vector<int> m_neighbors` 成员，每次 `query()` 调用 `clear()` 后复用同一块已分配的内存（`clear()` 不释放底层 capacity）。

### 修改文件

| 文件 | 修改内容 |
|---|---|
| `src/gui/RenderCanvas.cpp` | 粒子批量绘制 + 关闭抗锯齿 + 小粒子用 `drawRect` |
| `src/core/World.hpp` | 新增 `m_positionsCache` 成员 |
| `src/core/World.cpp` | 复用 `m_positionsCache`，消除重复分配 |
| `src/core/fluid/SPHSolver.hpp` | 新增 `m_neighbors` 成员 + 新增接受 positions 参数的重载 |
| `src/core/fluid/SPHSolver.cpp` | 手动乘法替代 `std::pow` + 复用 `m_neighbors` + 使用外部 positions |

### 性能提升预期

| 优化项 | 预期提升 |
|---|---|
| setBrush 批量化 | 渲染时间减少 ~50% |
| 关闭粒子抗锯齿 | 渲染时间再减少 ~40% |
| 小粒子用 drawRect | 渲染时间再减少 ~20% |
| positions 缓存复用 | 物理时间减少 ~15% |
| std::pow → 手动乘法 | SPH 计算减少 ~5% |
| neighbors 缓存复用 | SPH 查询减少 ~5% |

---

## 9. 刚体穿模（深穿透漏洞）与粒子陷入刚体内部（叉乘符号翻转）

### 现象描述

1. **刚体穿墙（穿模现象）**：将刚体置于极高处自由落体，或者当运动速度过快时，会直接穿透用户绘制的斜面/墙体。并且刚体在斜面上只有平移，没有任何真实的滚动和旋转。
2. **粒子卡在多边形内部**：流体或普通粒子在经过方形等刚体时，会被“吸入”刚体内部并死死卡住，无法被碰撞引擎正常推出。

### 根因分析

#### 1. 刚体穿墙（欧几里得距离深穿透漏洞）
原代码在检测刚体与墙壁的碰撞时，使用的是**纯几何距离**（Euclidean distance）：
```cpp
float dist = v.distance(closest);
if (dist < threshold * 2.0f) { ... }
```
当物体运动极快时，其顶点在一帧内瞬间越过了墙壁，此时顶点位于墙壁**后方深处**，它到墙壁的纯直线距离 `dist` 反而会变大。这导致程序误以为它“距离墙体很远”，直接忽略了碰撞，从而导致严重的深穿透（穿模）。
另外，原碰撞解算缺失了切向摩擦力冲量（Friction Impulse）所对应的旋转力矩（Torque），导致刚体无法发生旋转。

#### 2. 粒子进入刚体（屏幕 Y 轴向下导致叉乘结果翻转）
防止粒子挤入刚体的 `Collider::isPointInPolygon` 判断点是否在多边形内部使用了 2D 叉乘算法：
```cpp
// 2D 叉乘：若小于 0 则点在边右侧，表明点在凸多边形外部 (假定逆时针定点序)
if (edge.cross(toPoint) < -0.01f) { return false; }
```
在标准数学坐标系中（Y轴朝上），这是正确的。但在 Qt 屏幕坐标系中，**Y 轴是朝下的**！在 Y 轴朝下的坐标系中，原本视觉上的逆时针定点，在数学运算中实际上形成了反向包围，导致真实的内部点位于边的**右侧**（即 `cross < 0`）。
原代码错误地将 `< 0` 视为外部，导致完全颠倒：真实的外部被当作内部，真实的内部被当作外部。陷入内部的粒子因此永远无法触发排斥。

### 解决方案

#### 修复 A：引入符号距离场（SDF）与完整冲量摩擦力矩
在 `World.cpp` 的 `resolveRigidWallCollisions` 中：
1. **使用 SDF 检测深穿透**：不再比较纯距离，而是计算顶点在墙壁法向方向上的**有符号距离（Signed Distance）**。即使负向穿透深达 -40 像素，只要法向投影属于穿墙，就强制将其推出。
2. **切向偏差剥离**：运用勾股定理把法向渗透和切向偏移分开，防止吸附遥远的错误墙段。
3. **施加真实摩擦转矩**：引入 `applyImpulse(tangent * jt, r)`，通过切线摩擦力和力臂的叉乘计算完美的角速度衰减和滚动。

#### 修复 B：修正 Y 轴朝下导致的叉乘符号判定
在 `Collider::isPointInPolygon` 中将逻辑纠正，把 `< 0` 改为 `> 0`：
```diff
- if (edge.cross(toPoint) < -0.01f) {
+ // 修正：因为 Y 轴向下，内部实际位于右侧。大于 0 的左侧才是外部
+ if (edge.cross(toPoint) > 0.01f) {
      return false;
  }
```

### 修改文件

| 文件 | 修改内容 |
|---|---|
| `src/core/World.cpp` | 重写 `resolveRigidWallCollisions`，引入 SDF 穿透检测与冲量摩擦力矩 |
| `src/core/rigidbody/Collider.cpp` | 修正 `isPointInPolygon` 中的叉乘判定符号 |

---

## 10. 刚体在高压下仍然穿模（冲量累积导致速度爆炸）

### 现象描述

在之前引入了 SDF 深穿透检测后，偶尔在极端“高压”环境下（例如几百个水粒子死死地压在刚体上方，将刚体压向墙面），刚体仍然会发生瞬间穿透墙体掉出屏幕外的严重穿模现象。

### 根因分析

物理引擎使用的是 **冲量-位置混合求解法（Impulse & Position-Based Dynamics）**。
在高压下，几百个流体粒子在一次 `substep` 中各自给刚体施加微小的向下冲量（Impulse）。这些冲量在 `for` 循环中累加，导致刚体的**速度呈现非物理的指数级增长（速度爆炸）**。
在下一帧的数值积分阶段（`Position += Velocity * dt`），这个爆炸的速度导致刚体在一帧之内的位移极其巨大（如瞬间移动几百像素），直接跳过了墙体，并且超出了之前设定的 SDF 深度阈值（`-40.0f`），从而彻底逃逸碰撞检测。

### 解决方案

1. **引入积分速度锁（Velocity Clamping）**：
   在 `PhysicsWorld::integrate` 中，强制限制刚体的平动和转动最高速度（2000 px/s 和 50 rad/s）。无论积累了多么庞大的虚拟冲量，在转化为真实位移前都被强制截断。
2. **移除 SDF 的深度下限**：
   在 `resolveRigidWallCollisions` 中完全移除 `-40.0f` 的深穿透限制。只要刚体在墙壁线段的切向范围内，哪怕深穿透了几千像素，也会无条件被墙壁弹回，确保绝对屏障作用。

**修改文件**：`src/core/World.cpp` (`integrate` & `resolveRigidWallCollisions`)

---

## 11. 渲染终极瓶颈：突破几千粒子的 CPU 渲染极限

### 现象描述

当流体粒子数量上升至 5,000 ~ 10,000 以上时，即使之前使用了 `drawRect` 和减少 `setBrush` 状态切换的策略，程序依然陷入严重的卡顿（FPS < 10）。观察性能面板，发现 Render Time 占据了绝大部分 CPU 算力，拖垮了整个程序的实时性。

### 根因分析

现有的渲染系统基于 Qt 的 `QWidget` + `QPainter`。
`QPainter` 是一种**基于 CPU 发送指令的即时渲染管线（Immediate Mode Rendering）**。即便关闭了抗锯齿、批量使用了同一种颜色，一万个粒子仍然意味着 CPU 需要在内存中执行一万次坐标系变换、一万次栅格化像素填充指令。当粒子数量跨越数量级时，CPU 算力被渲染耗尽，物理引擎无力计算。

### 解决方案

**底层渲染架构重构：完全升级为 `QOpenGLWidget` 硬件加速。**

1. **Shader 着色器编写**：
   编写了原生的 Vertex Shader 和 Fragment Shader。在片段着色器中，利用 `gl_PointCoord` 数学剔除方形边角，在 GPU 层级直接画出完美的抗锯齿圆形。
2. **Point Sprites（点精灵）与实例化渲染**：
   将所有的粒子数据（坐标、颜色、大小）打包到 VBO（Vertex Buffer Object）中传入显存。只调用一次 `glDrawArrays(GL_POINTS, ...)`，让显卡利用几千个流处理器瞬间将一万颗粒子并行画出。
3. **混合流水线（Mixed Pipeline）**：
   利用 Qt 允许 QPainter 混合在 OpenGL 缓冲上绘制的特性，保留了 `QPainter` 来绘制线段（墙体）、UI 性能面板和刚体。这样既利用了 GPU 的极速处理大量同质化粒子，又保留了 QPainter 绘制高级 UI 的便捷性。

**修改文件**：
- `CMakeLists.txt` (引入 `Qt6::OpenGLWidgets`)
- `src/gui/RenderCanvas.hpp` & `src/gui/RenderCanvas.cpp` (重写继承链和渲染管线，实现 `initializeGL`, `resizeGL`, `paintGL`)

### 效果验证

在释放 10,000 颗粒子时，Render Time 从旧版的近百毫秒，**骤降并稳如泰山地稳定在 < 1 ms 内**。彻底清除了渲染瓶颈，将所有的 CPU 算力全部还给了物理引擎。

---

## 附录：快速构建命令

```powershell
# 设置环境变量（Qt + MinGW 工具链）
$env:PATH = "D:\QT\Tools\Ninja;D:\QT\6.11.1\mingw_64\bin;D:\QT\Tools\mingw1310_64\bin;" + $env:PATH

# 清空并重新配置
D:\QT\Tools\CMake_64\bin\cmake.exe -S d:\c++big_homework -B d:\c++big_homework\build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++

# 构建
D:\QT\Tools\CMake_64\bin\cmake.exe --build d:\c++big_homework\build

# 运行
.\build\QtTest.exe
```
