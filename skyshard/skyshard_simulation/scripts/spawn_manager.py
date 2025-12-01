#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from skyshard_msgs.msg import DroneState
from geometry_msgs.msg import Point, Vector3
from gazebo_msgs.srv import SpawnEntity
import os
import random
# import xacro # Kaldırıldı, SDF direkt kullanılıyor.

class DroneSimulator(Node):
    def __init__(self, drone_id, initial_pos):
        super().__init__(f'drone_{drone_id}_sim')
        self.drone_id = drone_id
        self.position = initial_pos
        self.velocity = Vector3()
        self.publisher = self.create_publisher(DroneState, '/skyshard/drone_state', 10)
        
        # Basit hareket ve durum güncelleme zamanlayıcısı (50ms)
        self.timer = self.create_timer(0.05, self.update_state)

    def update_state(self):
        # Basit rastgele hareket ekle (Router'ın Prophet skorunu test etmek için hareketlilik gerekli)
        self.position.x += random.uniform(-0.02, 0.02)
        self.position.y += random.uniform(-0.02, 0.02)
        
        # Z ekseninde sabit kalma eğilimi
        self.position.z = max(5.0, 10.0 + 2 * (self.drone_id % 3) + random.uniform(-0.5, 0.5))

        msg = DroneState()
        msg.id = self.drone_id
        msg.position = self.position
        msg.velocity = self.velocity # Basitleştirilmiş, sadece hareket yayınlıyoruz
        msg.battery_level = 1.0 - (self.drone_id * 0.01) # Farklı batarya seviyeleri
        msg.is_main_drone = (self.drone_id == 1)

        self.publisher.publish(msg)


class SpawnManager(Node):
    def __init__(self):
        super().__init__('spawn_manager')
        
        self.declare_parameter('num_drones', 5)
        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        
        # Gazebo servisine erişim için client
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        
        # Drone listesi
        self.drone_nodes = []
        
        self.initial_positions = [
            Point(x=10.0, y=0.0, z=10.0),
            Point(x=-10.0, y=10.0, z=10.0),
            Point(x=0.0, y=-10.0, z=12.0),
            Point(x=20.0, y=-5.0, z=11.0),
            Point(x=-15.0, y=-15.0, z=13.0),
            Point(x=5.0, y=15.0, z=14.0)
        ]
        
        self.spawn_models()

    def spawn_models(self):
        self.get_logger().info('Drone modelleri Gazebo\'ya yukleniyor...')
        
        # Gazebo servisini bekle
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gazebo spawn_entity servisi hazir degil.')
            return

        for i in range(1, self.num_drones + 1):
            if i > len(self.initial_positions):
                pos = Point(x=float(i * 5), y=float(i * 5), z=10.0)
            else:
                pos = self.initial_positions[i - 1]
            
            self.spawn_drone(i, pos)
            
            # Her drone için durumu yayınlayacak simülatör düğümünü oluştur
            self.drone_nodes.append(DroneSimulator(i, pos))
            self.get_logger().info(f'Drone {i} (ID: {i}) simülatörü başlatıldı.')
            
    def spawn_drone(self, drone_id, pos):
        request = SpawnEntity.Request()
        request.name = f'drone_{drone_id}'
        
        # Model dosya yolunu doğru şekilde bul
        pkg_share_dir = os.path.join(os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0], 'share', 'skyshard_simulation')
        model_path = os.path.join(pkg_share_dir, 'models', 'drone_model', 'model.sdf')
        
        try:
            with open(model_path, 'r') as f:
                sdf_data = f.read()
            
            request.xml = sdf_data
            request.robot_namespace = f'/drone_{drone_id}'
            request.initial_pose.position = pos

            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f'Drone {drone_id} basariyla spawn edildi.')
            else:
                self.get_logger().error(f'Drone {drone_id} spawn edilemedi: {future.result().status_message}')
        
        except FileNotFoundError:
            # Derlemeden önce bu hatayı almamak için geçici olarak HOME dizinindeki varsayılan yolu kullanacağız
            # Ancak ROS2 ortamı kurulduktan sonra üstteki yol doğru çalışacaktır.
            self.get_logger().error(f'Drone model dosyasi bulunamadi. Varsayilan yolu kontrol edin.')
        except Exception as e:
            self.get_logger().error(f'Hata olustu: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    spawn_manager = SpawnManager()
    
    # Drone simülatör düğümlerini de aynı anda döndür
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(spawn_manager)
    for node in spawn_manager.drone_nodes:
        executor.add_node(node)
        
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        spawn_manager.destroy_node()
        for node in spawn_manager.drone_nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()