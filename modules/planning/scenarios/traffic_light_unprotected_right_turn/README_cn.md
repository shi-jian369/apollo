# TrafficLightUnprotectedRightTurn 场景说明

## 1. 场景简介

`TrafficLightUnprotectedRightTurnScenario` 是 Apollo Planning 中处理交通灯路口无保护右转的场景插件。

该场景主要解决以下行为：

1. 车辆沿参考线接近交通灯路口。
2. 当前路径为右转路径，并且交通灯为红灯、黄灯或未知状态。
3. 车辆在停止线前停车。
4. 满足红灯右转条件后，解除交通灯停车墙。
5. 车辆低速蠕行观察路口内的动态障碍物。
6. 确认安全后完成右转并通过路口。

该场景与 `TrafficLightProtectedScenario` 的主要区别是：

| 场景 | 非绿灯时的行为 |
| ---- | ---- |
| `TrafficLightProtectedScenario` | 在停止线前等待，只有所有相关交通灯变为绿色后才进入路口。 |
| `TrafficLightUnprotectedRightTurnScenario` | 根据地图标志和配置决定是否允许红灯右转；允许时可进入蠕行和路口通过阶段，不需要等待绿灯。 |

## 2. 目录结构

```text
modules/planning/scenarios/traffic_light_unprotected_right_turn/
├── BUILD
├── README_cn.md
├── conf
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
├── context.h
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── traffic_light_unprotected_right_turn.proto
├── stage_creep.cc
├── stage_creep.h
├── stage_creep_test.cc
├── stage_intersection_cruise.cc
├── stage_intersection_cruise.h
├── stage_stop.cc
├── stage_stop.h
├── stage_stop_test.cc
├── traffic_light_unprotected_right_turn_scenario.cc
├── traffic_light_unprotected_right_turn_scenario.h
└── traffic_light_unprotected_right_turn_scenario_test.cc
```

## 3. 文件职责

| 文件 | 作用 |
| ---- | ---- |
| `traffic_light_unprotected_right_turn_scenario.cc/.h` | 定义场景插件，判断是否应从其他场景切换到无保护右转场景，并维护当前交通灯 overlap。 |
| `context.h` | 保存场景配置、当前交通灯 ID、停车等待起始时间和蠕行起始时间。 |
| `stage_stop.cc/.h` | 停止线前停车阶段，判断绿灯通行、红灯右转、禁止红灯右转等条件。 |
| `stage_creep.cc/.h` | 蠕行观察阶段，车辆低速向路口内部探行并判断动态障碍物风险。 |
| `stage_intersection_cruise.cc/.h` | 路口通过阶段，持续规划直到车辆驶出路口。 |
| `conf/scenario_conf.pb.txt` | 场景行为参数。 |
| `conf/pipeline.pb.txt` | 定义三个 stage 以及每个 stage 执行的 Planning Task。 |
| `proto/traffic_light_unprotected_right_turn.proto` | 定义 `ScenarioTrafficLightUnprotectedRightTurnConfig` 配置结构。 |
| `plugins.xml` | 注册 Scenario 和 Stage 插件。 |
| `BUILD` | 构建动态库和测试目标。 |

## 4. 场景状态机

该场景包含三个阶段：

```text
TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_STOP
                    |
                    v
TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP
                    |
                    v
TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE
                    |
                    v
               FinishScenario
```

在部分情况下会跳过 `CREEP`，直接从 `STOP` 进入 `INTERSECTION_CRUISE`：

```text
STOP -> INTERSECTION_CRUISE
```

## 5. 场景插件注册和加载

场景类定义为：

```cpp
class TrafficLightUnprotectedRightTurnScenario : public Scenario
```

插件注册在头文件中完成：

```cpp
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::TrafficLightUnprotectedRightTurnScenario, Scenario)
```

`plugins.xml` 注册以下插件：

| 插件类型 | 类名 |
| ---- | ---- |
| Scenario | `TrafficLightUnprotectedRightTurnScenario` |
| Stage | `TrafficLightUnprotectedRightTurnStageStop` |
| Stage | `TrafficLightUnprotectedRightTurnStageCreep` |
| Stage | `TrafficLightUnprotectedRightTurnStageIntersectionCruise` |

该场景还需要存在于 Public Road Planner 的场景列表中：

```proto
scenario {
  name: "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN"
  type: "TrafficLightUnprotectedRightTurnScenario"
}
```

场景管理器按配置顺序检查场景。当右转场景的 `IsTransferable()` 返回 `true` 后，场景管理器才会切换到本场景。

## 6. 场景进入条件

进入条件位于：

```cpp
TrafficLightUnprotectedRightTurnScenario::IsTransferable()
```

### 6.1 基础条件

必须满足：

1. 当前 Planning Command 包含 `lane_follow_command`。
2. 当前至少存在一条参考线。
3. 前方第一个相关 overlap 是交通灯，而不是 stop sign 或 yield sign。

如果首先遇到 stop sign 或 yield sign，函数会直接返回 `false`。

### 6.2 交通灯距离和颜色

代码会查找停止线位置相差不超过 2 米的同组交通灯：

```cpp
static constexpr double kTrafficLightGroupingMaxDist = 2.0;
```

对于同组交通灯，只有满足以下条件才认为需要进入交通灯场景：

```text
0 < 停止线与车辆前边缘的距离 <= start_traffic_light_scenario_distance
```

并且至少一个交通灯颜色不是：

```text
GREEN 或 BLACK
```

因此 `RED`、`YELLOW`、`UNKNOWN` 都可能触发本场景。

### 6.3 右转路径判断

当前代码通过以下方式判断停止线位置的路径是否右转：

```cpp
const auto& turn_type =
    reference_line_info.GetPathTurnType(traffic_sign_overlap->start_s);
if (turn_type != hdmap::Lane::RIGHT_TURN) {
  return false;
}
```

这是场景能否进入的关键条件。

如果车辆路线最终需要右转，但停止线位置仍被地图标记为直行车道，则本场景不会进入，`TrafficLightProtectedScenario` 可能会接管车辆并持续等待绿灯。

调试时应首先确认 DreamView 场景历史中是否出现：

```text
TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN
```

如果显示的是：

```text
TRAFFIC_LIGHT_PROTECTED
```

说明右转场景的 `IsTransferable()` 没有通过。

## 7. Enter 和 Exit

### 7.1 Enter

进入场景时，`Enter()` 会：

1. 查找前方第一个交通灯 overlap。
2. 查找与该交通灯停止线位置相差不超过 2 米的同组交通灯。
3. 把这些 ID 写入：

```text
planning_status.traffic_light.current_traffic_light_overlap_id
```

后续 STOP、CREEP 和 INTERSECTION_CRUISE 阶段通过这些 ID 获取当前交通灯位置。

### 7.2 Exit

退出场景时，`Exit()` 会清空 PlanningContext 中的交通灯状态：

```cpp
planning_status().traffic_light().Clear();
```

避免上一处路口的交通灯状态影响后续规划。

## 8. Context 运行时状态

`context.h` 定义：

```cpp
struct TrafficLightUnprotectedRightTurnContext : public ScenarioContext {
  ScenarioTrafficLightUnprotectedRightTurnConfig scenario_config;
  std::vector<std::string> current_traffic_light_overlap_ids;
  double stop_start_time = 0.0;
  double creep_start_time = 0.0;
};
```

| 字段 | 说明 |
| ---- | ---- |
| `scenario_config` | 从 `scenario_conf.pb.txt` 加载的场景参数。 |
| `current_traffic_light_overlap_ids` | 当前路口同组交通灯 overlap ID。 |
| `stop_start_time` | 红灯右转停车等待计时起点。 |
| `creep_start_time` | 蠕行阶段计时起点。 |

## 9. STOP 阶段

实现位于：

```text
stage_stop.cc
```

### 9.1 执行 Planning Task

STOP 阶段首先执行 `pipeline.pb.txt` 中配置的路径和速度任务：

```cpp
ExecuteTaskOnReferenceLine(planning_init_point, frame);
```

车辆实际的停止墙主要由通用 `TrafficLight` 交通规则产生，而不是由 `stage_stop.cc` 直接构造。

### 9.2 判断车辆是否到达停止线

代码计算：

```cpp
distance_adc_to_stop_line =
    current_traffic_light_overlap->start_s - adc_front_edge_s;
```

只有距离不大于 `max_valid_stop_distance` 时，才认为车辆已经处于有效停止区域。

### 9.3 绿灯处理

如果所有当前交通灯均为绿灯，并且车辆已接近停止线：

```cpp
return FinishStage(true);
```

随后直接进入：

```text
TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE
```

不会经过蠕行阶段。

### 9.4 红灯右转限制检查

`CheckTrafficLightNoRightTurnOnRed()` 会读取地图中的交通灯信息，并检查：

1. 是否存在 `NO_RIGHT_TURN_ON_RED` 标志。
2. 是否存在 `ARROW_RIGHT` 右箭头子灯。

只要存在其中之一，函数返回 `true`，表示红灯右转需要额外限制。

### 9.5 红灯右转放行

存在额外限制时，只有配置：

```proto
enable_right_turn_on_red: true
```

才会启动停车等待计时。

等待时长超过：

```proto
red_light_right_turn_stop_duration_sec
```

后，调用：

```cpp
FinishStage(false);
```

如果地图没有 `NO_RIGHT_TURN_ON_RED` 或 `ARROW_RIGHT`，代码会直接调用 `FinishStage(false)`，不等待绿灯。

### 9.6 FinishStage(false) 的关键作用

无保护模式结束 STOP 阶段时，会把当前交通灯 ID 写入：

```text
planning_status.traffic_light.done_traffic_light_overlap_id
```

通用 `TrafficLight` 规则读取到这些 ID 后，会跳过对应交通灯，不再生成红灯停车墙。

如果不写入 `done_traffic_light_overlap_id`，即使场景想继续右转，通用交通灯规则仍可能持续要求停车。

之后根据车辆速度决定下一阶段：

| 条件 | 下一阶段 |
| ---- | ---- |
| 当前速度大于 `max_adc_speed_before_creep` | 直接进入 `INTERSECTION_CRUISE`。 |
| 当前速度不大于 `max_adc_speed_before_creep` | 进入 `CREEP`。 |

## 10. CREEP 阶段

实现位于：

```text
stage_creep.cc
```

CREEP 阶段用于低速向前探行并观察路口内动态障碍物。

代码中特别注明：

```cpp
// note: don't check traffic light color while creeping on right turn
```

因此，进入 CREEP 阶段后不会继续等待红灯变绿。

### 10.1 蠕行停车墙

`ProcessCreep()` 会在交通灯 overlap 结束位置之后构造一个新的虚拟停车墙：

```text
creep_target_s = overlap_end_s + 4.0m
creep_finish_s = creep_target_s - 2.0m
```

车辆通过逐步向前移动来获得更好的路口视野。

### 10.2 蠕行完成条件

当车辆到达 creep finish 位置，或蠕行时间达到 `creep_timeout_sec` 后，代码会检查动态障碍物的 ST Boundary。

连续多个周期判断路口安全后，CREEP 阶段完成，进入：

```text
TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE
```

## 11. INTERSECTION_CRUISE 阶段

实现位于：

```text
stage_intersection_cruise.cc
```

该阶段持续执行路径和速度规划任务，并调用：

```cpp
CheckDone(*frame, injector_->planning_context(), false);
```

如果地图具有 junction overlap，车辆驶出 junction 后结束场景。

如果地图缺少 junction overlap，则使用交通灯 overlap 作为参考；车辆后边缘超过交通灯 overlap 结束位置 20 米后结束场景。

场景完成后，Planning 返回其他适合的场景，通常是 `LANE_FOLLOW`。

## 12. 配置参数说明

配置结构定义于：

```text
proto/traffic_light_unprotected_right_turn.proto
```

默认配置位于：

```text
conf/scenario_conf.pb.txt
```

| 参数 | 默认配置值 | 说明 |
| ---- | ---- | ---- |
| `start_traffic_light_scenario_distance` | `5.0` | 距离交通灯停止线多远开始尝试切入本场景。 |
| `enable_right_turn_on_red` | `false` | 存在禁止红灯右转标志或右箭头灯时，是否仍允许停车等待后右转。 |
| `max_valid_stop_distance` | `2.0` | 车辆距离停止线不超过该值时，认为车辆已进入有效停车区域。 |
| `min_pass_s_distance` | `3.0` | 车辆前边缘超过交通灯 overlap 结束位置一定距离后，可结束 STOP 阶段。 |
| `red_light_right_turn_stop_duration_sec` | `3.0` | 允许红灯右转前的停车等待时间。 |
| `creep_timeout_sec` | `10.0` | CREEP 阶段的超时时间。 |
| `max_adc_speed_before_creep` | `3.0` | STOP 阶段结束时，超过该速度则跳过 CREEP。 |
| `creep_stage_config.min_boundary_t` | `6.0` | 判断动态障碍物是否会在短时间内影响车辆。 |
| `creep_stage_config.ignore_max_st_min_t` | `0.1` | 忽略部分同向障碍物的 ST 时间阈值。 |
| `creep_stage_config.ignore_min_st_min_s` | `15.0` | 忽略部分远距离同向障碍物的 ST 距离阈值。 |

## 13. Pipeline 配置

`conf/pipeline.pb.txt` 为三个 stage 配置了类似的路径和速度任务：

| Task | 作用 |
| ---- | ---- |
| `LANE_FOLLOW_PATH` | 生成车道跟随路径。 |
| `LANE_BORROW_PATH` | 在满足条件时生成借道路径。 |
| `FALLBACK_PATH` | 路径规划失败时生成回退路径。 |
| `PATH_DECIDER` | 对障碍物生成路径方向决策。 |
| `RULE_BASED_STOP_DECIDER` | 根据规则生成停车决策。 |
| `ST_BOUNDS_DECIDER` | 构建速度规划 ST 边界。 |
| `SPEED_BOUNDS_PRIORI_DECIDER` | 生成初步速度边界。 |
| `SPEED_HEURISTIC_OPTIMIZER` | 生成启发式速度曲线。 |
| `SPEED_DECIDER` | 生成超车、跟车、停车等速度决策。 |
| `SPEED_BOUNDS_FINAL_DECIDER` | 生成最终速度边界。 |
| `PIECEWISE_JERK_SPEED` | 优化最终速度曲线。 |

## 14. 与 TrafficLight 交通规则的关系

本场景与以下目录中的通用交通灯规则协同工作：

```text
modules/planning/traffic_rules/traffic_light/
```

职责划分：

| 模块 | 职责 |
| ---- | ---- |
| `TrafficLight` 交通规则 | 非绿灯时在停止线前构造虚拟停车墙。 |
| `TrafficLightUnprotectedRightTurnScenario` | 判断红灯右转何时可以解除停车墙，并管理蠕行和路口通过阶段。 |

两者通过 `PlanningContext` 协作：

```text
current_traffic_light_overlap_id
done_traffic_light_overlap_id
```

场景进入时写入 `current_traffic_light_overlap_id`；允许红灯右转后写入 `done_traffic_light_overlap_id`，TrafficLight 规则随后跳过对应交通灯。

## 15. Profile 配置覆盖

默认模块配置位于：

```text
modules/planning/scenarios/traffic_light_unprotected_right_turn/conf/
```

赛事调参建议放入 profile：

```text
profiles/task1/modules/planning/scenarios/traffic_light_unprotected_right_turn/conf/
```

当前 `task1` 中的红灯右转配置示例：

```proto
start_traffic_light_scenario_distance: 8.0
enable_right_turn_on_red: true
max_valid_stop_distance: 2.0
min_pass_s_distance: 3.0
red_light_right_turn_stop_duration_sec: 1.0
creep_timeout_sec: 10.0
max_adc_speed_before_creep: 3.0
creep_stage_config {
    min_boundary_t: 6.0
    ignore_max_st_min_t: 0.1
    ignore_min_st_min_s: 15.0
}
```

应用 profile：

```bash
aem profile use task1
```

参数修改通常不需要重新编译，但需要重新应用 profile 并重启 Planning。

修改 `.cc`、`.h` 或 proto 后需要重新编译 Planning。

## 16. 红灯右转场景调试

### 16.1 车辆一直等待红灯

首先观察 DreamView 场景历史。

如果当前场景是：

```text
TRAFFIC_LIGHT_PROTECTED / APPROACH
```

说明本右转场景没有成功进入。常见原因：

1. 停止线位置的 `GetPathTurnType()` 不是 `RIGHT_TURN`。
2. 车辆距离停止线超过 `start_traffic_light_scenario_distance`。
3. 前方首先遇到 stop sign 或 yield sign。
4. 场景没有配置到 Public Road Planner 场景列表中。

如果当前场景是：

```text
TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN / STOP
```

但车辆一直不右转，检查：

1. `enable_right_turn_on_red` 是否为 `true`。
2. `red_light_right_turn_stop_duration_sec` 是否过大。
3. 车辆是否已经进入 `max_valid_stop_distance` 范围。
4. profile 配置是否已通过 `aem profile use task1` 应用。

### 16.2 路线最终右转，但场景进入 Protected

当前代码只检查交通灯停止线位置：

```cpp
reference_line_info.GetPathTurnType(traffic_sign_overlap->start_s)
```

如果停止线位置仍属于直行车道，但停止线之后路线才进入右转车道，判断结果可能不是 `RIGHT_TURN`。

可考虑将场景入口判断改为：从停止线开始向前扫描一段参考线，只要后续存在 `RIGHT_TURN`，就允许进入无保护右转场景。

修改时需要谨慎，避免将真正的直行路线误识别为右转路线。

### 16.3 建议关注的日志

场景成功切入：

```text
switch scenario from LANE_FOLLOW to TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN
```

阶段执行：

```text
stage: Stop
stage: Creep
stage: IntersectionCruise
```

地图红灯右转限制检查：

```text
Stop reason when right turn: NO_RIGHT_TURN_ON_RED
Stop reason when right turn:: ARROW_RIGHT
Stop when right turn: no stop
```

如果日志显示切入：

```text
TRAFFIC_LIGHT_PROTECTED
```

应优先排查右转场景的进入条件，而不是继续调整红灯停车距离。

## 17. 构建与测试

场景动态库目标：

```text
libtraffic_light_unprotected_right_turn_scenario.so
```

修改源码后可编译：

```bash
buildtool build -p modules/planning/scenarios/traffic_light_unprotected_right_turn
```

目录中的测试主要验证 Scenario 和 Stage 插件可以正常初始化：

```text
traffic_light_unprotected_right_turn_scenario_test.cc
stage_stop_test.cc
stage_creep_test.cc
```

这些测试未完整覆盖实际红灯右转行为，因此修改场景进入条件或阶段切换逻辑后，还需要在 Scenario Sim 中进行验证。

## 18. 总结

`TrafficLightUnprotectedRightTurnScenario` 的核心不是等待红灯变绿，而是在满足条件后解除红灯停车墙，通过蠕行观察完成无保护右转。

理解该场景时需要重点关注：

1. `IsTransferable()` 能否正确识别右转路线。
2. `enable_right_turn_on_red` 是否允许红灯右转。
3. STOP 阶段是否写入 `done_traffic_light_overlap_id`。
4. CREEP 阶段进入后不再检查交通灯颜色。
5. DreamView 中实际切入的是右转场景还是 Protected 场景。
