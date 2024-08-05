# Object information message format

# 物標 ID
ObjectId id
# 情報取得時刻
TimestampIts time
# 物標種別
ObjectClass[] object_class
# 存在信頼度
ExistenceConfidence existency

# 位置と移動
# 物標位置
Location object_location
# 物標参照位置
ReferencePoint ref_point
# Direction of movement
WGS84Angle direction
# 速さ
Speed speed
# 回転速度
YawRate yaw_rate
# 前後加速度
Acceleration acceleration

# Target status/attributes
# Direction of target
WGS84Angle orientation
# 物標のサイズ
ObjectSize size
# 物標の色
ObjectColor color
# 車両の状態等
# シフトポジション
ShiftPosition shift_position
# 前輪舵角
SteeringAngle steering_angle_front
# 後輪舵角
SteeringAngle steering_angle_rear
# ブレーキ状態
BrakeState brake_state
# 補助ブレーキ状態
AuxiliaryBrakeState auxiliary_brake_state
# アクセルペダル開度
ThrottlePosition throttle_position
# 灯火の状態
ExteriorLights exterior_lights
# 各種のシステムの作動状態
ControlSystemStates control_system_states
# 車両用途種別
VehicleRole vehicle_role
# 車両用途種別毎の状態
VehicleExtendedInformation vehicle_extended_info
# 牽引車両
ObjectId towing_vehicle
# 情報源のリスト
ObjectId[] information_source_list
