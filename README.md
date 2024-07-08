# flight_control

Хранит программы связанные с автопилотом. 

- autopilot.cpp - программа, которая отрабатывает функции автопилота:
    - Отработка миссий по точкам;
    - Перевод координат в указанные системы отсчета;

## Требования к заполняемой миссии

### Сегменты

Для сегмента миссии должны быть заполнены:
- `mode.mode` - drone_msgs::FlightControlMode - режим выполнения сегмента
- `completion_condition` - условие выполнения сегмента


### Точки

Для точки должны быть заполнены:
- `completion_policy` - drone_msgs::Goal::POLICY_* условие выполнения точки
- `policy_distance_threshold` и/или `policy_orientation_threshold` и `policy_timer_time` - порог выполнения точки по условию. Заполняютсе те, которые соответствуют текущему условию

Пример заполнения объекта миссии, который отправляется в топик для новой миссиии автопилота:

```cpp 
drone_msgs::Mission mission;
drone_msgs::MissionSegment segment;

drone_msgs::Goal new_goal;
new_goal.completion_policy = drone_msgs::Goal::POLICY_DISTANCE_REACHED;
new_goal.policy_distance_threshold = 0.2;
new_goal.pose.coordinates_type = drone_msgs::DronePose::LOCAL;
new_goal.pose.point.x = 0;
new_goal.pose.point.y = 0;
new_goal.pose.point.z = 1;

segment.trajectory.waypoints.push_back(new_goal);
segment.completion_condition = drone_msgs::MissionSegment::LAST_POINT_ARRIVAL;
segment.mode.mode = drone_msgs::FlightControlMode::GO_ALONG_TRAJECTORY;
mission.segments.push_back(segment);

mission_pub.publish(mission);
```