#!/usr/bin/env python3
# skyshard_simulation/scripts/marker_publisher.py

import rclpy
from rclpy.node import Node

# Router node'u zaten marker yayınladığı için bu node sadece yapı koruyucudur.
# İstenirse global durum görselleştirmeleri buraya eklenebilir.
class MarkerPublisherNode(Node):
    def __init__(self):
        super().__init__('marker_publisher_dummy')
        self.get_logger().info('Marker Publisher Node pasif olarak baslatildi (Gorsellestirme Router Node\'da).')

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
