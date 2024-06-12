import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import heapq


class PathFinder(Node):
    def __init__(self):
        super().__init__('path_finder')
        self.subscription = self.create_subscription(String, 'map_topic', self.find_shortest_path, 10)

    def find_shortest_path(self, msg):
        self.get_logger().info('Received map data: "%s"' % msg.data)
        
        # Parse the map data and apply Dijkstra's algorithm to find the shortest path
        # For example:
        map_data = msg.data.strip().split('\n')
        graph = {}
        for line in map_data:
            node1, node2, weight = line.split()
            weight = int(weight)
            if node1 not in graph:
                graph[node1] = {}
            if node2 not in graph:
                graph[node2] = {}
            graph[node1][node2] = weight
            graph[node2][node1] = weight
        
        start_node = 'A'
        end_node = 'D'
        shortest_path, cost = dijkstra(graph, start_node, end_node)
        
        self.get_logger().info('Shortest path from %s to %s: %s (cost: %d)' % (start_node, end_node, ' -> '.join(shortest_path), cost))


def dijkstra(graph, start, end):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    pq = [(0, start)]
    prev = {}

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_dist > distances[current_node]:
            continue

        if current_node == end:
            path = []
            node = end
            while node != start:
                path.append(node)
                node = prev[node]
            path.append(start)
            path.reverse()
            return path, current_dist

        for neighbor, weight in graph[current_node].items():
            new_dist = current_dist + weight
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                prev[neighbor] = current_node
                heapq.heappush(pq, (new_dist, neighbor))

    return [], float('inf')

def main(args=None):
    rclpy.init(args=args)
    path_finder = PathFinder()
    rclpy.spin(path_finder)
    path_finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()