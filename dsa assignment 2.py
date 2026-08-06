import math
import os

class KDNode:
    """Represents a node in the 2-D kD Tree."""
    def __init__(self, point, left=None, right=None):
        self.point = point  # (sensor_id, x, y)
        self.left = left
        self.right = right

class ForestFireMonitoringSystem:
    """Fully dynamic spatial monitoring system using a 2-D kD Tree."""
    def __init__(self):
        self.sensor_map = {}  # Fast O(1) lookup table: sensor_id -> (x, y)
        self.root = None
        self.last_nodes_visited = 0

    def calculate_height(self, node):
        """Computes the height of the kD Tree dynamically."""
        if not node:
            return 0
        return 1 + max(self.calculate_height(node.left), self.calculate_height(node.right))

    def _build_tree(self, points, depth=0):
        """Recursively builds a balanced kD tree by splitting axes alternately."""
        if not points:
            return None
        
        k = 2  # 2 dimensions (x, y)
        axis = depth % k
        
        # Sort points by the current splitting axis (index 1 for X, index 2 for Y)
        points.sort(key=lambda x: x[axis + 1])
        median_idx = len(points) // 2
        
        node = KDNode(points[median_idx])
        node.left = self._build_tree(points[:median_idx], depth + 1)
        node.right = self._build_tree(points[median_idx + 1:], depth + 1)
        return node

    def rebuild(self):
        """Rebuilds the entire tree from the sensor map to maintain balance."""
        points = [(sid, coord[0], coord[1]) for sid, coord in self.sensor_map.items()]
        self.root = self._build_tree(points, depth=0)

    def add_sensor(self, sid, x, y, outfile):
        """Dynamically adds a sensor and updates the spatial index."""
        self.sensor_map[sid] = (float(x), float(y))
        self.rebuild()
        outfile.write(f"Sensor Added:\n{sid} ({int(float(x))},{int(float(y))})\n")

    def remove_sensor(self, sid, outfile):
        """Dynamically removes a sensor and re-indexes the space."""
        if sid in self.sensor_map:
            del self.sensor_map[sid]
            self.rebuild()
            outfile.write(f"Sensor Removed:\n{sid}\n")

    def find_nearest(self, qx, qy, outfile):
        """Finds the nearest sensor dynamically using kD-Tree hypersphere pruning."""
        qx, qy = float(qx), float(qy)
        best = [None, float('inf')]
        nodes_visited = [0]

        def search(node, depth):
            if node is None:
                return
            nodes_visited[0] +=1
            
            
            sid, sx, sy = node.point
            dist = math.hypot(sx - qx, sy - qy) # Euclidean Norm
            
            if dist < best[1]:
                best[0] = node.point
                best[1] = dist
                
            axis = depth % 2
            q_val = qx if axis == 0 else qy
            n_val = sx if axis == 0 else sy
            
            next_b = node.left if q_val < n_val else node.right
            other_b = node.right if q_val < n_val else node.left
            
            search(next_b, depth + 1)
            
            if abs(q_val - n_val) < best[1]:
                search(other_b, depth + 1)

        search(self.root, 0)
        self.last_nodes_visited = nodes_visited[0]
        
        if best[0]:
            outfile.write(f"Nearest Sensor\n{best[0][0]} ({int(best[0][1])},{int(best[0][2])})\nDistance: {best[1]:.2f}\nNodes Visited: {nodes_visited[0]}\n")

    def find_in_rectangle(self, xmin, ymin, xmax, ymax, outfile):
        """Performs a dynamic rectangular range search using bounding-box pruning."""
        xmin, ymin, xmax, ymax = float(xmin), float(ymin), float(xmax), float(ymax)
        results = []
        nodes_visited = [0]

        def search(node, depth):
            if node is None:
                return
            nodes_visited[0] += 1
            
            sid, sx, sy = node.point
            if xmin <= sx <= xmax and ymin <= sy <= ymax:
                results.append(node.point)
                
            axis = depth % 2
            n_val = sx if axis == 0 else sy
            
            if axis == 0:
                if xmin <= n_val:
                    search(node.left, depth + 1)
                if xmax >= n_val:
                    search(node.right, depth + 1)
            else:
                if ymin <= n_val:
                    search(node.left, depth + 1)
                if ymax >= n_val:
                    search(node.right, depth + 1)

        search(self.root, 0)
        self.last_nodes_visited = nodes_visited[0]
        
        outfile.write(f"Sensors inside Region\n({int(xmin)},{int(ymin)}) to ({int(xmax)},{int(ymax)})\n")
        for r in results:
            outfile.write(f"{r[0]} ({int(r[1])},{int(r[2])})\n")
        outfile.write(f"Total Sensors: {len(results)}\n")

    def display_statistics(self, outfile):
        """Computes and writes live structural metrics of the kD Tree."""
        height = self.calculate_height(self.root)
        outfile.write(f"Tree Statistics\n---------------\nTotal Sensors : {len(self.sensor_map)}\nTree Height : {height}\nNodes Visited (Last Query): {self.last_nodes_visited}\n")

    def display_traversal(self, outfile):
        """Generates a live pre-order traversal sequence of the tree."""
        outfile.write("Preorder Traversal\n")
        def preorder(node):
            if node:
                outfile.write(f"{node.point[0]}\n")
                preorder(node.left)
                preorder(node.right)
        preorder(self.root)

    def process_file(self, input_filename, output_filename):
        """Processes commands dynamically from the input file."""
        with open(input_filename, 'r') as infile, open(output_filename, 'w') as outfile:
            for line in infile:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split("::")
                cmd = parts[0].strip()
                
                if cmd == "addSensor":
                    self.add_sensor(parts[1], parts[2], parts[3], outfile)
                elif cmd == "removeSensor":
                    self.remove_sensor(parts[1], outfile)
                elif cmd == "nearestSensor":
                    self.find_nearest(parts[1], parts[2], outfile)
                elif cmd == "rangeSearch":
                    self.find_in_rectangle(parts[1], parts[2], parts[3], parts[4], outfile)
                elif cmd == "treeStatistics":
                    self.display_statistics(outfile)
                elif cmd == "displayTree":
                    self.display_traversal(outfile)

if __name__ == "__main__":
    input_file = "comments.txt"
    output_file = "outputPS8.txt"
    
    # Generate default input file if missing
    if not os.path.exists(input_file):
        with open(input_file, 'w') as f:
            f.write("""addSensor::S101::12::18
addSensor::S102::25::30
addSensor::S103::40::12
addSensor::S104::55::45
addSensor::S105::28::22
nearestSensor::30::20
rangeSearch::10::10::35::35
removeSensor::S102
treeStatistics
displayTree
""")

    system = ForestFireMonitoringSystem()
    system.process_file(input_file, output_file)