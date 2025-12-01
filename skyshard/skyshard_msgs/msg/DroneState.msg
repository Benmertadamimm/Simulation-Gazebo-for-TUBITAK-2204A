# skyshard_msgs/msg/DroneState.msg
# Bir drone'un anlık durumunu temsil eder.

int32 id                          # Drone'un benzersiz ID'si
geometry_msgs/Point position      # Drone'un (x, y, z) konumu
geometry_msgs/Vector3 velocity    # Drone'un (vx, vy, vz) hızı
float32 battery_level             # Pil seviyesi (0.0 - 1.0 arası)
bool is_main_drone               # Ana (Main) drone olup olmadığı
