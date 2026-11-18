# xatlas 顶点来源追踪与输出回填分析

代码基线：本仓库 `source/xatlas/xatlas.cpp`（commit `f700c77`）。
方法：静态代码阅读 + 针对公开 API 的探针程序实证（`-fsanitize=address`，探针源码在 `/tmp/xatlas_probe/`，不属本仓库）。
文中所有"已实证"结论均有探针输出佐证；仅由代码阅读得出、未单独实证的推断会明确标注。未取得证据的"丢面/丢顶点"不作为事实陈述。

---

## 1. 管线总览：索引空间的四次变换

```
输入顶点 id (xref 空间)
   │  AddMesh: 逐顶点拷贝，id 不变；n-gon 耳切三角化；退化/NaN/用户忽略 → ignore 标志
   ▼
内部源网格 Mesh（顶点 id == 输入 id）
   │  createColocals: 按位置 epsilon 焊接，firstColocalVertex/nextColocalVertex 成环
   ▼
图表网格（每个 Chart 一份）
   │  两层顶点：unified 顶点（按焊接代表元合并，参数化在它上面算 UV）
   │            original 顶点（每个被引用的源顶点一个，输出粒度）
   ▼
输出 Mesh.vertexArray / indexArray / chartArray（BuildOutputMeshes）
```

关键事实：三角化、焊接、图表拆分都**不改变源顶点 id 本身**；所有阶段通过映射表回溯，最终在输出构建阶段一次性回填 `Vertex::xref`、面索引和图表面列表。追踪链条在代码里始终存在，问题是消费者要理解每一环的语义。

## 2. 各阶段的映射机制与责任

### 2.1 AddMesh（xatlas.cpp:9022）

- **顶点**：按输入顺序逐个 `addVertex`（xatlas.cpp:9068-9076），内部顶点 id 与输入 id 一一对应。`indexOffset` 在 `DecodeIndex`（xatlas.cpp:9014）里加到索引上，因此 xref 空间 = "输入顶点数组下标"（已含 offset）。
- **三角化**：`faceVertexCount` 非空时按 n-gon 处理，`Triangulator::triangulatePolygon`（xatlas.cpp:3462）耳切法输出源顶点 id 序列。同时填两张回溯表（`MeshPolygonMapping`，xatlas.cpp:8871）：
  - `triangleToPolygonMap`：三角形 → 输入多边形序号（xatlas.cpp:9192）；
  - `triangleToPolygonIndicesMap`：三角形角点 → **源顶点 id**（xatlas.cpp:9196，注意此处压入的是 `triIndices[i]` 即顶点 id，不是多边形角点槽位——见 §5 缺陷 B）。
- **忽略**：三类面被打上 ignore 标志——退化/零长度边/零面积、NaN 属性（xatlas.cpp:9107-9180）、用户 `faceIgnoreData`（xatlas.cpp:9182）。被忽略的面**仍然进入内部网格**，只是后续不参与图表。
- **焊接**：`createColocals`（xatlas.cpp:2518）按 `MeshDecl::epsilon` 把位置相同的顶点串成环（`m_nextColocalVertex`），`m_firstColocalVertex[v]` 为环代表元。焊接只用于连通性/统一化判断，**不删除任何顶点**。

### 2.2 图表生成（segment / param::Chart）

`Chart` 构造函数（xatlas.cpp:7137 起）为每张图表建一个 unified mesh，并维护三张表：

| 表 | 方向 | 作用 |
|---|---|---|
| `m_faceToSourceFaceMap` | 图表面 → 源（三角形）面 | 输出 `Chart::faceArray` 的来源 |
| `m_vertexToSourceVertexMap` | 图表 original 顶点 → 源顶点 | 输出 `Vertex::xref` 的来源 |
| `m_chartVertexToUnifiedVertexMap` | original 顶点 → unified 顶点 | 取参数化 UV |

- original 顶点按**源顶点 id** 去重（`sourceVertexToChartVertexMap`，xatlas.cpp:7163-7168）：一个源顶点在一张图表内至多一个 original 顶点。
- unified 顶点按**焊接代表元** `firstColocalVertex` 合并（xatlas.cpp:7152-7161）：同一张图表内位置相同的多个输入顶点共享一个 unified 顶点，从而共享同一个参数化 UV。例外：`ChartGeneratorType::OriginalUv`（`useInputMeshUvs`）下，UV 不同的 colocal 顶点不合并（xatlas.cpp:7153-7156），以保留输入 UV 岛边界。
- 逐图表拆分（piecewise，xatlas.cpp:7193）通过父图表的 `m_faceToSourceFaceMap` 链接（xatlas.cpp:7198），面来源可追溯穿透多级拆分。
- 被忽略的面由 `InvalidMeshGeometry::extract`（xatlas.cpp:3014）单独收集：面列表 + 按源顶点去重的顶点表，供输出阶段原样回写。

### 2.3 参数化

LSCM/正交投影等只在 unified 顶点上写 texcoord；original 顶点经 `m_chartVertexToUnifiedVertexMap` 读取 UV。**UV 是"每图表每位置"一份**，这是接缝复制的根源。

### 2.4 输出构建 BuildOutputMeshes（xatlas.cpp:9627 起，在 PackCharts 尾部）

对每个输入网格：

1. **顶点数组** = 无效几何顶点（前段）+ 各图表 original 顶点（按图表顺序拼接）。
   - 无效几何顶点：`atlasIndex = chartIndex = -1`，`uv = (0,0)`，`xref = 源顶点 id`（xatlas.cpp:9688-9694）。
   - 图表顶点：`atlasIndex/chartIndex` 指向打包结果，`uv = max(0, unifiedUV)`（负 UV 被钳到 0，xatlas.cpp:9739-9740），`xref = m_vertexToSourceVertexMap[v]`（xatlas.cpp:9741）。
2. **索引数组按"源面槽位"散射**（三角网格路径）：`indexOffset = mapFaceToSourceFace(f) * 3`（xatlas.cpp:9745），无效面同样写回自己的源面槽位（xatlas.cpp:9697）。因此 **输出 indexArray 的第 f 个三角形槽位对应输入的第 f 个面**，面序与输入一致；槽位的值是该面在本图表中的输出顶点下标。`indexCount = 输入面数 * 3`。
3. **Chart::faceArray** = `m_faceToSourceFaceMap`，即输入面序号列表（n-gon 时为输入多边形序号，按图表内去重，xatlas.cpp:9759-9785）。

### 2.5 AddUvMesh 路径（xatlas.cpp:9804-9858）

无位置、无焊接：`xref = v`（恒等），`indexArray` 原样 `memcpy`（xatlas.cpp:9836），顶点 uv 直接取输入 UV；不属于任何图表的顶点 `atlasIndex/chartIndex = -1` 但 **uv 保留输入值**（与 AddMesh 路径忽略面 uv=(0,0) 不同——两条路径对"无效顶点 UV"的约定不一致，属兼容取舍而非缺陷）。

## 3. 核心问题：同一输入顶点在 UV 接缝处分裂后如何回填

**顶点分裂规则**：输入顶点 v 被 k 张图表引用 → 输出顶点数组里有 k 个顶点，每个的 `xref == v`，各自携带所属图表 unified 顶点的 UV。已实证（§4 接缝探针）：共享边的两个输入顶点 0、1 各分裂为 2 个输出顶点（v0/v4 xref=0，v1/v6 xref=1），UV 不同。

**面索引回填**：输出 indexArray 槽位由输入面序号定位（`mapFaceToSourceFace(f)*3`），槽位的值指向**该面所属图表的那份顶点拷贝**。因此：

- 输出面 f 的三个角 = 输入面 f 的三个角在"面 f 所属图表"中的拷贝；同一输入面只属一张图表，不会重复也不会悬空。
- 消费者重建拓扑的正确姿势是：**以 xref 为键做倒排**（输入顶点 → 输出顶点列表），而不是假设输出顶点与输入顶点一一对应。

**属性回填**：

- 位置、法线、切线、颜色等一切逐顶点属性：按 `xref` 从输入顶点数组拷贝。xatlas 不输出除 UV 外的任何属性，这是设计使然。
- UV：必须用输出顶点的 `uv`，不能从输入拷贝（输入 UV 只是 hint）。
- 同一图表内被焊接的 colocal 顶点（位置相同、id 不同）：各自有独立输出顶点和 xref，但 UV 相同（共享 unified 顶点）。若这些输入顶点携带不同属性（如硬边法线），属性可按 xref 分别找回，UV 无法区分——这是焊接语义的必然结果，不是数据丢失。
- 反向映射（输入 → 输出）xatlas 不提供，需消费者自建；未使用的输入顶点不会出现在输出中（见 §5 设计行为 d），倒排表可能缺少某些输入顶点的条目。

## 4. 实证结果（探针，ASan 构建）

| 探针 | 输入 | 输出（摘要） | 结论 |
|---|---|---|---|
| `cube` | 4 顶点 2 三角形 | vertexCount=4，indexArray 与输入一致，faceArray=[0,1] | 无接缝时输出与输入同构 |
| `seam` | 6 顶点 4 三角形（两四边形共边成直角） | vertexCount=8，2 图表；xref=0→v0,v4；xref=1→v1,v6；indexArray 槽位按输入面序 | 接缝一分多 + 面槽位保序，均符合 §3 |
| `ignore` | 6 顶点 2 三角形，面 1 被 faceIgnore | 被忽略面顶点在数组前段（v0-v2，atlas/chart=-1，uv=(0,0)），其索引仍占输出面槽 1 | 忽略面不丢，xref 完整 |
| `colocal` | 6 顶点含 2 对同位置顶点，顶点 3 未被任何面引用 | vertexCount=5：顶点 3 不出现；colocal 对 (0,4) 各自存在且 UV 相同 | 未引用顶点不输出；焊接共享 UV |
| `quad`（n-gon，顶点 id 10-13） | 1 个四边形，indexCount=4 | **ASan heap-buffer-overflow 写，xatlas.cpp:9750** | 缺陷 B，见 §5 |
| `quad1`（n-gon，角点序 {2,0,1,3}） | 1 个四边形 | 输出 indexArray 槽位 s 写入的是 xref==s 的顶点（多图表时"最后写入者胜"），描述的角点序变成 {0,1,2,3} 而非输入序 | 缺陷 B 的槽位/顶点 id 混淆 |
| `mixed`（四边形 + 三角形混合 n-gon） | faceVertexCount={4,3} | 三角形被解码为顶点 {3,4,5} 而非输入的 {4,5,6}；输入顶点 6 不出现在输出 | 缺陷 A，见 §5 |
| `quadign`（被忽略的 n-gon） | 1 个被忽略四边形 | indexCount=6（退回三角化形式），与 chart 路径的多边形形式（4）不一致 | 缺陷 C，见 §5 |

## 5. 设计允许的复制/忽略 vs 真实缺陷

### 设计行为（符合文档或结构必然，不要当缺陷报）

a. **接缝顶点复制**：同一 xref 多个输出顶点，是 UV 图集的本质，`xref` 就是为此存在的回溯键。
b. **忽略面保留**：`faceIgnoreData` 与自动检测的退化/NaN 面都保留在输出中（atlasIndex=-1、uv=(0,0)），`MeshDecl::faceIgnoreData` 注释明确承诺了这一点。顶点数/面数因此不小于"有效几何"，不是泄漏。
c. **焊接共享 UV**：同图表内 colocal 顶点 UV 相同；`useInputMeshUvs` 时 UV 不同的 colocal 不合并，是刻意的例外。
d. **未引用输入顶点不输出**：输出顶点 = 被面引用的源顶点集合。这不是丢顶点——该顶点本就不参与任何面；但若消费者假设"输出顶点覆盖全部输入顶点"，倒排时会踩空。
e. **UV 钳到非负**（xatlas.cpp:9739）：打包后 UV 本应在图集范围内，钳制是安全措施，理论上是兼容取舍。
f. **AddUvMesh 恒等映射**：xref=v、索引原样拷贝，是该 API 的承诺语义。

### 真实缺陷（均已实证，仅存在于 n-gon / `faceVertexCount` 路径）

**缺陷 A：混合尺寸 n-gon 的索引解码偏移错误**（xatlas.cpp:9096）。
解码第 f 个面的角点用 `face * faceVertexCount + i`，即"面序号 × 本面顶点数"，只有所有面顶点数相同时才等于正确的累积偏移。混合尺寸时读错索引：探针 `mixed` 中三角形被读成 {3,4,5}（应为 {4,5,6}），输入顶点 6 静默消失、顶点 3 被拉进不属于它的面。**这是真实的数据错乱**：错读顶点 + 静默丢顶点，且无报错（`IndexOutOfRange` 检查用的是错读后的值，仍可能落在合法范围内）。规避：n-gon 输入前由调用方自行三角化，或保证所有面顶点数相同。

**缺陷 B：n-gon 输出索引的槽位/顶点 id 混淆**（xatlas.cpp:9196 + 9749-9750）。
`triangleToPolygonIndicesMap` 注释为"Triangle indices to polygon indices"，但实际压入的是**源顶点 id**；输出阶段又把它当输出 indexArray 的槽位下标用。后果：
- 顶点 id ≥ 多边形角点总数时**堆越界写**（探针 `quad`，ASan 确认写越界于 xatlas.cpp:9750）；
- 不越界时（顶点 id 恰好都在 [0, indexCount) 内），输出 indexArray 的槽位 s 收到的是 xref==s 的输出顶点，多边形角点顺序被替换为顶点 id 顺序，且顶点跨图表分裂时同一槽位被多张图表"最后写入者胜"地覆盖（探针 `quad1`）。
仅当"第 i 个角点恰好引用顶点 i"（零基、顺序、无分裂）时结果碰巧正确（探针 `quad0`）。规避同缺陷 A。

**缺陷 C：被忽略 n-gon 面的输出索引布局不一致**（xatlas.cpp:9653-9654、9696-9700）。
无效几何按"三角形 ×3"计数并写回三角形槽位，而 chart 路径按"多边形角点"计数。纯忽略 n-gon 网格输出 indexCount=6（三角化形式）而非 4（多边形形式）（探针 `quadign`）；若同一网格既有被图表化的 n-gon 又有被忽略的 n-gon，两种槽位空间混用——由代码阅读推断会进一步放大缺陷 B 的错乱，**该混合场景未单独实证，不作为事实结论**。

### 明确不下结论的事项

- 图表生成与参数化阶段：未观察到有效面不被任何图表覆盖、或图表顶点丢失 xref 的证据；本报告不主张存在此类丢面/丢顶点。
- 缺陷 A/B/C 是否存在于上游后续版本：本环境无网络，未核对，仅断言本仓库 commit `f700c77` 的行为。

## 6. 各阶段责任划分与兼容取舍

| 阶段 | 对来源追踪的责任 | 取舍 |
|---|---|---|
| AddMesh | 保持顶点 id 恒等；三角化时记录三角形→多边形回溯；标记而非删除无效面 | 耳切三角化改变面的内部划分，靠映射表还原；缺陷 A/B 源于此阶段的映射表语义含混 |
| 焊接 (createColocals) | 只建等价类，不改 id；图表内统一化以代表元为键 | 位置相同即合并（epsilon），属性差异被压平；`OriginalUv` 例外保护 UV 岛 |
| 图表生成 | 每图表记录 original→source、face→source 映射；拆分（piecewise）时链接父表 | 顶点按图表复制是 UV 唯一性的代价；original/unified 双层结构让"输出粒度=输入顶点"与"参数化粒度=位置"解耦 |
| 参数化 | UV 写 unified 顶点，original 经映射读取 | 同位置顶点在图表内必然同 UV |
| 输出构建 | 一次性回填 xref/atlas/chart/uv/面槽位/faceArray | 面槽位=输入面序，方便按面对齐回填；顶点数组顺序（无效顶点在前）是内部布局，消费者不应依赖 |
| 消费端 | 用 xref 倒排做属性回填与拓扑重建；n-gon 输入自行三角化以规避缺陷 A/B/C | 反向映射不自建就会丢"一分多"信息 |

## 7. 消费端回填建议（结论性）

1. 遍历输出顶点，按 `xref` 从输入拷贝位置/法线等属性，UV 用输出顶点的 `uv`。
2. 建 `xref → [输出顶点]` 多重映射以还原接缝两侧的对应关系；`atlasIndex == -1` 的顶点属于被忽略面，单独处理。
3. 面级对齐直接用"输出面槽位 f ↔ 输入面 f"（三角网格）或 `Chart::faceArray`（图表归属）。
4. **不要使用 `faceVertexCount`（n-gon）输入**：在调用 xatlas 前自行三角化，绕开缺陷 A/B/C；这也是当前唯一完全可靠的 n-gon 使用方式。
