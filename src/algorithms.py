# src/algorithms.py
import heapq  # Dùng cho hàng đợi ưu tiên (priority queue) trong A*, Greedy BFS, JPS, Bi-A*
import collections # Dùng cho hàng đợi (queue) trong BFS
import math # Cần cho sqrt trong heuristic_euclidean và chi phí đường chéo

# --- HEURISTICS ---
# Các hàm heuristic ước lượng chi phí từ một node đến node đích.

def heuristic_manhattan(node_a_rc, node_b_rc):
    """
    Tính khoảng cách Manhattan giữa hai node.
    Khoảng cách Manhattan là tổng của chênh lệch tuyệt đối của các tọa độ.
    Thích hợp cho lưới chỉ cho phép di chuyển ngang/dọc.

    Args:
        node_a_rc (tuple): Tọa độ (row, col) của node A.
        node_b_rc (tuple): Tọa độ (row, col) của node B.

    Returns:
        float: Khoảng cách Manhattan.
    """
    return abs(node_a_rc[0] - node_b_rc[0]) + abs(node_a_rc[1] - node_b_rc[1])

def heuristic_euclidean(node_a_rc, node_b_rc):
    """
    Tính khoảng cách Euclidean (đường chim bay) giữa hai node.
    Thích hợp cho lưới cho phép di chuyển theo đường chéo.

    Args:
        node_a_rc (tuple): Tọa độ (row, col) của node A.
        node_b_rc (tuple): Tọa độ (row, col) của node B.

    Returns:
        float: Khoảng cách Euclidean.
    """
    dr = node_a_rc[0] - node_b_rc[0]  # Chênh lệch hàng
    dc = node_a_rc[1] - node_b_rc[1]  # Chênh lệch cột
    return math.sqrt(dr*dr + dc*dc) # Công thức Pythagoras

def heuristic_zero(node_a_rc, node_b_rc):
    """
    Heuristic bằng không. Khi sử dụng heuristic này, thuật toán A* sẽ hoạt động
    tương tự như thuật toán Dijkstra (chỉ dựa vào chi phí thực tế g_cost).

    Args:
        node_a_rc (tuple): Tọa độ (row, col) của node A.
        node_b_rc (tuple): Tọa độ (row, col) của node B.

    Returns:
        int: 0.
    """
    return 0

# --- GRAPH REPRESENTATION ---
# Lớp Graph để biểu diễn lưới dưới dạng đồ thị, nơi các node là các ô
# và các cạnh là các đường di chuyển hợp lệ giữa các ô.

class Graph:
    """
    Lớp biểu diễn đồ thị đơn giản, dùng để lưu trữ các node và cạnh có trọng số.
    """
    def __init__(self):
        """Khởi tạo một đồ thị rỗng."""
        self.edges = {}  # Dictionary lưu các cạnh: from_node -> {to_node: weight}
        self.nodes = set() # Set lưu trữ tất cả các node trong đồ thị

    def add_node(self, node_coord_rc):
        """
        Thêm một node vào đồ thị.

        Args:
            node_coord_rc (tuple): Tọa độ (row, col) của node cần thêm.
        """
        self.nodes.add(node_coord_rc)

    def add_edge(self, from_node_rc, to_node_rc, weight):
        """
        Thêm một cạnh có hướng (directed edge) vào đồ thị.
        Nếu đồ thị là vô hướng, cần thêm cạnh ngược lại.

        Args:
            from_node_rc (tuple): Tọa độ (row, col) của node bắt đầu.
            to_node_rc (tuple): Tọa độ (row, col) của node kết thúc.
            weight (float): Trọng số (chi phí) của cạnh.
        """
        self.add_node(from_node_rc) # Đảm bảo node tồn tại trong set nodes
        self.add_node(to_node_rc)   # Đảm bảo node tồn tại trong set nodes
        if from_node_rc not in self.edges:
            self.edges[from_node_rc] = {} # Tạo dictionary cho node nếu chưa có
        self.edges[from_node_rc][to_node_rc] = weight # Gán trọng số cho cạnh

    def get_neighbors(self, node_coord_rc):
        """
        Lấy tất cả các node láng giềng và trọng số cạnh đến chúng từ một node cho trước.

        Args:
            node_coord_rc (tuple): Tọa độ (row, col) của node.

        Returns:
            iterator: Một iterator các tuple (neighbor_coord, weight).
                      Trả về iterator rỗng nếu node không có láng giềng hoặc không tồn tại.
        """
        return self.edges.get(node_coord_rc, {}).items()

    def get_edge_weight(self, from_node_rc, to_node_rc):
        """
        Lấy trọng số của cạnh giữa hai node.

        Args:
            from_node_rc (tuple): Tọa độ (row, col) của node bắt đầu.
            to_node_rc (tuple): Tọa độ (row, col) của node kết thúc.

        Returns:
            float: Trọng số của cạnh nếu tồn tại, ngược lại là float('inf').
        """
        return self.edges.get(from_node_rc, {}).get(to_node_rc, float("inf"))

# --- PATHFINDING ALGORITHMS ---
# Các thuật toán tìm đường đi.

def a_star_search(graph, start_node_rc, goal_node_rc, heuristic_func=heuristic_manhattan):
    """
    Thực hiện thuật toán A* để tìm đường đi ngắn nhất từ start_node đến goal_node.
    A* sử dụng hàm f_cost = g_cost + h_cost, trong đó:
    - g_cost: chi phí thực tế từ node bắt đầu đến node hiện tại.
    - h_cost: chi phí ước lượng (heuristic) từ node hiện tại đến node đích.

    Args:
        graph (Graph): Đối tượng đồ thị chứa các node và cạnh.
        start_node_rc (tuple): Tọa độ (row, col) của node bắt đầu.
        goal_node_rc (tuple): Tọa độ (row, col) của node đích.
        heuristic_func (function): Hàm heuristic để ước lượng chi phí.

    Returns:
        tuple: (path, cost, explored_nodes)
               - path (list of tuples or None): Danh sách các tọa độ (row, col) tạo thành đường đi,
                                                hoặc None nếu không tìm thấy đường.
               - cost (float): Chi phí của đường đi, hoặc float('inf') nếu không tìm thấy đường.
               - explored_nodes (list of tuples): Danh sách các node đã được khám phá (theo thứ tự).
    """
    # open_set là một hàng đợi ưu tiên (min-heap) lưu các node cần được xem xét.
    # Mỗi phần tử trong open_set là một tuple: (f_cost, g_cost, node, path_list)
    # - f_cost: Ưu tiên sắp xếp (g_cost + heuristic).
    # - g_cost: Chi phí thực tế từ start_node đến node này.
    # - node: Tọa độ (row, col) của node.
    # - path_list: Danh sách các node tạo thành đường đi từ start_node đến node này.
    open_set = []
    heapq.heappush(open_set, (heuristic_func(start_node_rc, goal_node_rc), 0, start_node_rc, [start_node_rc]))
    
    # closed_set_costs (thay vì closed_set dạng set) lưu trữ chi phí g_cost tốt nhất đã biết để đến một node.
    # Dùng để kiểm tra xem có đường đi mới tốt hơn đến một node đã từng được xem xét không.
    # {node: g_cost}
    closed_set_costs = {start_node_rc: 0}
    
    # all_explored_nodes_ordered lưu trữ thứ tự các node được pop ra từ open_set để phục vụ visualization.
    all_explored_nodes_ordered = []

    while open_set: # Khi open_set còn node để xem xét
        # Lấy node có f_cost nhỏ nhất từ open_set.
        _, g_cost_current, current_node, path = heapq.heappop(open_set)

        # Nếu đã tìm thấy một đường đi tốt hơn đến current_node này trước đó (thông qua một f_cost khác)
        # thì bỏ qua lần xử lý này. Điều này xảy ra khi một node được đẩy vào open_set nhiều lần
        # với các g_cost khác nhau.
        if g_cost_current > closed_set_costs.get(current_node, float('inf')):
            continue
        
        # Thêm node vào danh sách explored cho visualization.
        # Thực hiện sau khi các kiểm tra ở trên đã qua, và trước khi kiểm tra đích,
        # để đảm bảo thứ tự khám phá cho visualization là hợp lý (node được "mở" thực sự).
        if current_node not in all_explored_nodes_ordered: # Tránh thêm trùng lặp (mặc dù với closed_set_costs, điều này ít xảy ra)
             all_explored_nodes_ordered.append(current_node)

        # Nếu node hiện tại là node đích, đã tìm thấy đường đi.
        if current_node == goal_node_rc:
            return path, g_cost_current, all_explored_nodes_ordered # Trả về đường đi, chi phí và các node đã khám phá.

        # Duyệt qua các node láng giềng của node hiện tại.
        for neighbor, weight_to_neighbor in graph.get_neighbors(current_node):
            # Tính chi phí g_cost mới để đến láng giềng này.
            new_g_cost = g_cost_current + weight_to_neighbor
            
            # Nếu chi phí mới này tốt hơn (nhỏ hơn) chi phí đã biết để đến láng giềng
            # (hoặc nếu láng giềng chưa từng được xem xét - chi phí mặc định là vô cực).
            if new_g_cost < closed_set_costs.get(neighbor, float('inf')):
                closed_set_costs[neighbor] = new_g_cost # Cập nhật chi phí tốt nhất đến láng giềng.
                priority = new_g_cost + heuristic_func(neighbor, goal_node_rc) # Tính f_cost cho láng giềng.
                # Thêm láng giềng vào open_set với thông tin mới.
                heapq.heappush(open_set, (priority, new_g_cost, neighbor, path + [neighbor]))
                
    # Nếu open_set rỗng mà chưa tìm thấy node đích, nghĩa là không có đường đi.
    return None, float("inf"), all_explored_nodes_ordered

def dijkstra_search(graph, start_node_rc, goal_node_rc):
    """
    Thực hiện thuật toán Dijkstra.
    Về cơ bản là A* với hàm heuristic luôn trả về 0.

    Args:
        graph (Graph): Đồ thị.
        start_node_rc (tuple): Node bắt đầu.
        goal_node_rc (tuple): Node đích.

    Returns:
        tuple: (path, cost, explored_nodes) - tương tự A*.
    """
    return a_star_search(graph, start_node_rc, goal_node_rc, heuristic_func=heuristic_zero)

def bfs_search(graph, start_node_rc, goal_node_rc):
    """
    Thực hiện thuật toán Tìm kiếm theo chiều rộng (Breadth-First Search - BFS).
    BFS khám phá các node theo từng lớp, đảm bảo tìm thấy đường đi có ít bước nhất
    (nếu tất cả các cạnh có cùng trọng số 1). Nếu trọng số khác nhau, nó không đảm bảo
    đường đi có chi phí thấp nhất, nhưng vẫn tìm thấy đường đi.

    Args:
        graph (Graph): Đồ thị.
        start_node_rc (tuple): Node bắt đầu.
        goal_node_rc (tuple): Node đích.

    Returns:
        tuple: (path, cost, explored_nodes)
               - cost ở đây là tổng trọng số thực tế của các cạnh trên đường đi.
    """
    # queue là một hàng đợi (FIFO) lưu các node cần được xem xét.
    # Mỗi phần tử: (node, path_list, accumulated_actual_cost)
    queue = collections.deque([(start_node_rc, [start_node_rc], 0)])
    # visited_nodes (tương đương open set của BFS) lưu các node đã được thêm vào queue để tránh xử lý lặp.
    visited_nodes = {start_node_rc}
    all_explored_nodes_ordered = [] # Lưu thứ tự khám phá.

    while queue:
        current_node, path, current_path_actual_cost = queue.popleft() # Lấy node đầu tiên từ hàng đợi.
        all_explored_nodes_ordered.append(current_node) # Node được coi là explored khi nó được pop ra.

        if current_node == goal_node_rc:
            return path, current_path_actual_cost, all_explored_nodes_ordered # Tìm thấy đích.

        # Duyệt qua các láng giềng.
        for neighbor, weight_to_neighbor in graph.get_neighbors(current_node):
            if neighbor not in visited_nodes: # Nếu láng giềng chưa được thăm.
                visited_nodes.add(neighbor) # Đánh dấu đã thăm.
                new_path_cost = current_path_actual_cost + weight_to_neighbor # Tích lũy chi phí thực tế.
                queue.append((neighbor, path + [neighbor], new_path_cost)) # Thêm vào cuối hàng đợi.
                
    return None, float("inf"), all_explored_nodes_ordered # Không tìm thấy đường đi.

def greedy_bfs_search(graph, start_node_rc, goal_node_rc, heuristic_func=heuristic_manhattan):
    """
    Thực hiện thuật toán Tìm kiếm Tham lam theo lựa chọn Tốt nhất đầu tiên (Greedy Best-First Search).
    Thuật toán này luôn chọn node có vẻ gần đích nhất dựa trên hàm heuristic,
    mà không quan tâm đến chi phí thực tế đã đi (g_cost).
    Do đó, nó nhanh nhưng không đảm bảo tìm ra đường đi tối ưu.

    Args:
        graph (Graph): Đồ thị.
        start_node_rc (tuple): Node bắt đầu.
        goal_node_rc (tuple): Node đích.
        heuristic_func (function): Hàm heuristic.

    Returns:
        tuple: (path, cost, explored_nodes)
               - cost ở đây là tổng trọng số thực tế của các cạnh trên đường đi tìm được.
    """
    # open_set là hàng đợi ưu tiên, sắp xếp theo heuristic_to_goal.
    # (heuristic_to_goal, accumulated_g_cost, node, path_list)
    # accumulated_g_cost được mang theo để tính chi phí cuối cùng của đường đi, không dùng để sắp xếp.
    open_set = []
    heapq.heappush(open_set, (heuristic_func(start_node_rc, goal_node_rc), 0, start_node_rc, [start_node_rc]))
    
    # closed_set lưu các node đã được pop ra và xử lý hoàn toàn.
    closed_set = set()
    all_explored_nodes_ordered = []

    while open_set:
        h_cost_to_goal, g_cost_accumulated, current_node, path = heapq.heappop(open_set)

        if current_node in closed_set: # Nếu node đã được xử lý, bỏ qua.
            continue
        closed_set.add(current_node) # Đánh dấu đã xử lý.
        all_explored_nodes_ordered.append(current_node)

        if current_node == goal_node_rc: # Tìm thấy đích.
            return path, g_cost_accumulated, all_explored_nodes_ordered

        # Duyệt qua các láng giềng.
        for neighbor, weight_to_neighbor in graph.get_neighbors(current_node):
            if neighbor not in closed_set: # Nếu láng giềng chưa được xử lý.
                new_g_cost_for_neighbor = g_cost_accumulated + weight_to_neighbor # Tính g_cost mới.
                heuristic_for_neighbor = heuristic_func(neighbor, goal_node_rc) # Tính heuristic cho láng giềng.
                heapq.heappush(open_set, (heuristic_for_neighbor, new_g_cost_for_neighbor, neighbor, path + [neighbor]))

    return None, float("inf"), all_explored_nodes_ordered # Không tìm thấy đường đi.


# --- JUMP POINT SEARCH (JPS) ---
# JPS là một thuật toán tối ưu hóa A* cho lưới đều, bằng cách "nhảy" qua các node
# không có điểm quyết định (forced neighbors).
# Lưu ý: Phiên bản JPS dưới đây là cơ bản, chủ yếu xử lý di chuyển theo đường thẳng (cardinal).
# Việc xử lý đường chéo và forced neighbors phức tạp hơn trong JPS/JPS+.

# --- JPS HELPER FUNCTIONS ---
# --- JPS HELPER FUNCTIONS (Cải thiện) ---
def _is_walkable_jps(r, c, grid_data):
    rows, cols = len(grid_data), len(grid_data[0])
    if not (0 <= r < rows and 0 <= c < cols): return False
    return not grid_data[r][c].is_obstacle_type()

def _get_node_actual_cost_jps(r, c, grid_data):
    if not _is_walkable_jps(r,c,grid_data): return float('inf')
    return grid_data[r][c].cost

def _jps_jump(current_rc, dr, dc, grid_data, start_rc, goal_rc):
    """
    Hàm đệ quy thực hiện "bước nhảy" trong JPS.
    Tìm jump point tiếp theo theo hướng (dr, dc) từ current_rc.
    Cải thiện để xử lý cơ bản đường chéo và forced neighbors.
    """
    next_r, next_c = current_rc[0] + dr, current_rc[1] + dc

    if not _is_walkable_jps(next_r, next_c, grid_data): return None
    if (next_r, next_c) == goal_rc: return (next_r, next_c)

    # Kiểm tra forced neighbors
    if dr != 0 and dc == 0: # Di chuyển dọc (Cardinal)
        # Forced neighbor bên trái
        if (not _is_walkable_jps(next_r, next_c - 1, grid_data) and \
            _is_walkable_jps(next_r + dr, next_c - 1, grid_data)):
            return (next_r, next_c)
        # Forced neighbor bên phải
        if (not _is_walkable_jps(next_r, next_c + 1, grid_data) and \
            _is_walkable_jps(next_r + dr, next_c + 1, grid_data)):
            return (next_r, next_c)
    elif dc != 0 and dr == 0: # Di chuyển ngang (Cardinal)
        # Forced neighbor phía trên
        if (not _is_walkable_jps(next_r - 1, next_c, grid_data) and \
            _is_walkable_jps(next_r - 1, next_c + dc, grid_data)):
            return (next_r, next_c)
        # Forced neighbor phía dưới
        if (not _is_walkable_jps(next_r + 1, next_c, grid_data) and \
            _is_walkable_jps(next_r + 1, next_c + dc, grid_data)):
            return (next_r, next_c)
    elif dr != 0 and dc != 0: # Di chuyển chéo (Diagonal)
        # Kiểm tra xem có jump point nào nếu tiếp tục nhảy theo chiều ngang hoặc dọc từ (next_r, next_c)
        # Đây là một phần quan trọng của JPS+ để xác định forced neighbors trên đường chéo
        if _jps_jump((next_r, next_c), dr, 0, grid_data, start_rc, goal_rc) or \
           _jps_jump((next_r, next_c), 0, dc, grid_data, start_rc, goal_rc):
            return (next_r, next_c)
    
    # Nếu di chuyển chéo, và không có forced neighbor từ các thành phần ngang/dọc,
    # thì vẫn tiếp tục nhảy theo đường chéo đó.
    if dr != 0 and dc != 0:
        return _jps_jump((next_r, next_c), dr, dc, grid_data, start_rc, goal_rc)
    # Nếu di chuyển thẳng và không có forced neighbor, tiếp tục nhảy thẳng.
    elif dr != 0 or dc != 0 : # Chỉ tiếp tục nhảy nếu là di chuyển thẳng (đã check chéo ở trên)
         return _jps_jump((next_r, next_c), dr, dc, grid_data, start_rc, goal_rc)

    return None # Nên không bao giờ đến đây nếu dr hoặc dc khác 0

def jps_search(grid_data, start_rc, goal_rc, heuristic_func=heuristic_manhattan):
    open_set = []
    heapq.heappush(open_set, (heuristic_func(start_rc, goal_rc) + 0, 0, start_rc, None)) # f, g, node, parent
    came_from = {start_rc: None}
    g_costs = {start_rc: 0}
    explored_for_viz = []

    while open_set:
        _, g_current_jp, current_jp, _ = heapq.heappop(open_set)

        if current_jp in explored_for_viz: # Nếu jump point này đã được xử lý với cost tốt hơn hoặc bằng
            if g_current_jp >= g_costs.get(current_jp, float('inf')): # Thêm kiểm tra này
                 continue
        g_costs[current_jp] = g_current_jp # Cập nhật cost tốt nhất đến current_jp
        explored_for_viz.append(current_jp)


        if current_jp == goal_rc:
            # --- Tái tạo đường đi chi tiết ---
            path_of_jump_points = []
            temp = goal_rc
            while temp is not None:
                path_of_jump_points.append(temp)
                temp = came_from.get(temp)
            path_of_jump_points.reverse()

            if not path_of_jump_points: return None, float('inf'), explored_for_viz
            
            interpolated_path = [path_of_jump_points[0]]
            # Cost của ô start được tính một lần
            current_path_actual_cost = _get_node_actual_cost_jps(path_of_jump_points[0][0], path_of_jump_points[0][1], grid_data)
            if current_path_actual_cost == float('inf'): current_path_actual_cost = 0 # Start node cost should be normal

            for i in range(len(path_of_jump_points) - 1):
                p1 = path_of_jump_points[i]
                p2 = path_of_jump_points[i+1]
                r1, c1 = p1; r2, c2 = p2
                
                curr_r, curr_c = r1, c1
                while (curr_r, curr_c) != p2:
                    dr_step = (r2 > curr_r) - (r2 < curr_r)
                    dc_step = (c2 > curr_c) - (c2 < curr_c)
                    curr_r += dr_step
                    curr_c += dc_step
                    
                    if not _is_walkable_jps(curr_r, curr_c, grid_data):
                        return None, float('inf'), explored_for_viz # Lỗi nội suy

                    interpolated_path.append((curr_r, curr_c))
                    # Cộng cost của các ô trên đường đi (KHÔNG BAO GỒM Ô P1 LẦN NỮA)
                    current_path_actual_cost += _get_node_actual_cost_jps(curr_r, curr_c, grid_data)
            
            return interpolated_path, current_path_actual_cost, explored_for_viz

        # --- Xác định các jump point kế tiếp (successors) ---
        parent_jp = came_from.get(current_jp)
        successors_to_consider = []
        
        # Xác định hướng đi chuẩn hóa từ parent đến current_jp (nếu có)
        dr_norm, dc_norm = 0, 0
        if parent_jp:
            r_curr, c_curr = current_jp; r_par, c_par = parent_jp
            if r_curr != r_par: dr_norm = (r_curr - r_par) // abs(r_curr - r_par)
            if c_curr != c_par: dc_norm = (c_curr - c_par) // abs(c_curr - c_par)

        # Duyệt các hướng có thể đi (pruned set of neighbors for JPS)
        # Nếu là node bắt đầu (không có parent), hoặc di chuyển thẳng
        if parent_jp is None or (dr_norm == 0 or dc_norm == 0): # Đến từ di chuyển thẳng hoặc là start
            # Kiểm tra hướng thẳng (nếu có)
            if dr_norm != 0: # Đang đi dọc
                if _is_walkable_jps(current_jp[0] + dr_norm, current_jp[1], grid_data):
                    successors_to_consider.append((dr_norm, 0))
            if dc_norm != 0: # Đang đi ngang
                 if _is_walkable_jps(current_jp[0], current_jp[1] + dc_norm, grid_data):
                    successors_to_consider.append((0, dc_norm))
            
            # Kiểm tra forced neighbors cho di chuyển thẳng
            if dr_norm != 0: # Đang đi dọc
                if not _is_walkable_jps(current_jp[0], current_jp[1] - 1, grid_data) and \
                   _is_walkable_jps(current_jp[0] + dr_norm, current_jp[1] - 1, grid_data):
                    successors_to_consider.append((dr_norm, -1)) # Forced sang trái-chéo
                if not _is_walkable_jps(current_jp[0], current_jp[1] + 1, grid_data) and \
                   _is_walkable_jps(current_jp[0] + dr_norm, current_jp[1] + 1, grid_data):
                    successors_to_consider.append((dr_norm, 1)) # Forced sang phải-chéo
            elif dc_norm != 0: # Đang đi ngang
                if not _is_walkable_jps(current_jp[0] - 1, current_jp[1], grid_data) and \
                   _is_walkable_jps(current_jp[0] - 1, current_jp[1] + dc_norm, grid_data):
                    successors_to_consider.append((-1, dc_norm)) # Forced lên-chéo
                if not _is_walkable_jps(current_jp[0] + 1, current_jp[1], grid_data) and \
                   _is_walkable_jps(current_jp[0] + 1, current_jp[1] + dc_norm, grid_data):
                    successors_to_consider.append((1, dc_norm))   # Forced xuống-chéo
            
            # Nếu là node bắt đầu, thêm tất cả 8 hướng
            if parent_jp is None:
                all_directions = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
                for dr_add, dc_add in all_directions:
                    if (dr_add, dc_add) not in successors_to_consider and _is_walkable_jps(current_jp[0]+dr_add, current_jp[1]+dc_add, grid_data):
                        successors_to_consider.append((dr_add, dc_add))


        # Nếu di chuyển chéo đến current_jp (dr_norm != 0 and dc_norm != 0)
        elif dr_norm != 0 and dc_norm != 0:
            # 1. Nhảy theo hướng ngang (thành phần của đường chéo)
            if _is_walkable_jps(current_jp[0], current_jp[1] + dc_norm, grid_data):
                successors_to_consider.append((0, dc_norm))
            # 2. Nhảy theo hướng dọc (thành phần của đường chéo)
            if _is_walkable_jps(current_jp[0] + dr_norm, current_jp[1], grid_data):
                successors_to_consider.append((dr_norm, 0))
            # 3. Nhảy tiếp theo hướng chéo (nếu ô đó đi được)
            if _is_walkable_jps(current_jp[0] + dr_norm, current_jp[1] + dc_norm, grid_data):
                successors_to_consider.append((dr_norm, dc_norm))

        # Thực hiện nhảy cho các hướng đã xác định
        actual_successors_jp = []
        for dr_s, dc_s in set(successors_to_consider): # Dùng set để loại bỏ hướng trùng lặp
            jump_point = _jps_jump(current_jp, dr_s, dc_s, grid_data, start_rc, goal_rc)
            if jump_point:
                actual_successors_jp.append(jump_point)

        # Xử lý các jump point kế tiếp
        for successor_jp_node in set(actual_successors_jp):
            cost_segment = 0
            temp_r, temp_c = current_jp
            path_to_successor_segment = [] # Để tính cost chính xác

            while (temp_r, temp_c) != successor_jp_node:
                dr_step = (successor_jp_node[0] > temp_r) - (successor_jp_node[0] < temp_r)
                dc_step = (successor_jp_node[1] > temp_c) - (successor_jp_node[1] < temp_c)
                temp_r += dr_step
                temp_c += dc_step
                path_to_successor_segment.append((temp_r, temp_c)) # Lưu các ô trên đoạn đường
                
            # Tính cost của đoạn đường này
            for seg_r, seg_c in path_to_successor_segment:
                 current_cell_cost = _get_node_actual_cost_jps(seg_r, seg_c, grid_data)
                 if current_cell_cost == float('inf'):
                     cost_segment = float('inf'); break
                 cost_segment += current_cell_cost
            
            if cost_segment == float('inf'): continue

            new_g_to_successor_jp = g_current_jp + cost_segment

            if new_g_to_successor_jp < g_costs.get(successor_jp_node, float('inf')):
                g_costs[successor_jp_node] = new_g_to_successor_jp
                f_cost_val = new_g_to_successor_jp + heuristic_func(successor_jp_node, goal_rc)
                heapq.heappush(open_set, (f_cost_val, new_g_to_successor_jp, successor_jp_node, current_jp))
                came_from[successor_jp_node] = current_jp
                
    return None, float("inf"), explored_for_viz
# --- BIDIRECTIONAL A* SEARCH ---
# Tìm kiếm A* từ cả điểm bắt đầu và điểm kết thúc đồng thời.
# Hai quá trình tìm kiếm sẽ gặp nhau ở một điểm nào đó.

def bidirectional_a_star_search(graph, start_node_rc, goal_node_rc, heuristic_func=heuristic_manhattan):
    """
    Thực hiện thuật toán A* hai chiều.

    Args:
        graph (Graph): Đồ thị.
        start_node_rc (tuple): Node bắt đầu.
        goal_node_rc (tuple): Node đích.
        heuristic_func (function): Hàm heuristic.

    Returns:
        tuple: (path, cost, explored_nodes)
    """
    # --- Khởi tạo cho tìm kiếm xuôi (Forward Search: start -> goal) ---
    # open_fwd: (f_approx, g_cost, node)
    # f_approx là ước lượng f_cost, dùng heuristic từ node hiện tại đến goal.
    open_fwd = []; heapq.heappush(open_fwd, (heuristic_func(start_node_rc, goal_node_rc), 0, start_node_rc))
    g_fwd = {start_node_rc: 0} # g_cost từ start đến node trong tìm kiếm xuôi.
    parent_fwd = {start_node_rc: None} # Node cha trong tìm kiếm xuôi.
    
    # --- Khởi tạo cho tìm kiếm ngược (Backward Search: goal -> start) ---
    # open_bwd: (f_approx, g_cost, node)
    # f_approx dùng heuristic từ node hiện tại đến start.
    open_bwd = []; heapq.heappush(open_bwd, (heuristic_func(goal_node_rc, start_node_rc), 0, goal_node_rc))
    g_bwd = {goal_node_rc: 0} # g_cost từ goal đến node trong tìm kiếm ngược.
    parent_bwd = {goal_node_rc: None} # Node cha trong tìm kiếm ngược.
    
    # explored_..._viz: Set các node đã được pop ra từ open_set tương ứng (để visualize).
    explored_fwd_viz = set(); explored_bwd_viz = set()
    
    # mu: Chi phí của đường đi tốt nhất đã tìm thấy qua một điểm gặp nhau.
    # Khởi tạo là vô cực.
    mu = float('inf')
    meeting_node = None # Node nơi hai quá trình tìm kiếm gặp nhau và tạo ra đường đi tốt nhất.
    final_path = None

    # Vòng lặp chính: tiếp tục khi cả hai open_set còn node.
    while open_fwd and open_bwd:
        # --- Điều kiện dừng heuristic (nâng cao) ---
        # Nếu f_cost ước lượng nhỏ nhất của cả hai hướng đều lớn hơn hoặc bằng mu (chi phí tốt nhất đã tìm thấy),
        # thì có thể dừng lại vì không có khả năng tìm được đường đi tốt hơn.
        # Điều kiện đơn giản hơn: nếu tổng g_cost của các node đầu heap + một epsilon > mu.
        if open_fwd[0][0] >= mu and open_bwd[0][0] >= mu and meeting_node:
            break # Dừng nếu không có khả năng cải thiện.
        
        # --- Chọn hướng để mở rộng (Expand) ---
        # Có thể xen kẽ, hoặc chọn hướng có f_cost ước lượng nhỏ hơn.
        expand_fwd = True # Mặc định mở rộng tìm kiếm xuôi.
        if open_fwd and open_bwd:
            if open_bwd[0][0] < open_fwd[0][0]: # Nếu tìm kiếm ngược có f_approx tốt hơn.
                expand_fwd = False
        elif not open_fwd: # Nếu tìm kiếm xuôi hết node.
            expand_fwd = False


        if expand_fwd: # --- Mở rộng tìm kiếm xuôi ---
            _, g_curr_f, u_f = heapq.heappop(open_fwd) # Lấy node u_f từ open_fwd.
            explored_fwd_viz.add(u_f) # Thêm vào danh sách explored.

            # Kiểm tra xem u_f đã được xử lý bởi tìm kiếm ngược chưa.
            if u_f in g_bwd:
                path_cost_through_u_f = g_curr_f + g_bwd[u_f] # Tổng chi phí qua u_f.
                if path_cost_through_u_f < mu: # Nếu tốt hơn đường đi hiện tại.
                    mu = path_cost_through_u_f # Cập nhật mu.
                    meeting_node = u_f # Cập nhật điểm gặp nhau.
            
            # Tối ưu hóa: Nếu g_cost hiện tại của u_f đã lớn hơn hoặc bằng mu,
            # thì không cần mở rộng u_f nữa vì không thể tạo ra đường đi tốt hơn mu.
            if g_curr_f >= mu : continue
            
            # Duyệt qua các láng giềng v_f của u_f.
            for v_f, weight_uv_f in graph.get_neighbors(u_f):
                new_g_f = g_curr_f + weight_uv_f # g_cost mới đến v_f.
                if new_g_f < g_fwd.get(v_f, float('inf')): # Nếu tìm thấy đường tốt hơn đến v_f.
                    g_fwd[v_f] = new_g_f
                    parent_fwd[v_f] = u_f
                    f_v_fwd = new_g_f + heuristic_func(v_f, goal_node_rc) # f_cost ước lượng của v_f.
                    # Chỉ thêm vào open_fwd nếu có khả năng tạo ra đường đi tốt hơn mu.
                    if f_v_fwd < mu :
                        heapq.heappush(open_fwd, (f_v_fwd, new_g_f, v_f))
        else: # --- Mở rộng tìm kiếm ngược ---
            _, g_curr_b, u_b = heapq.heappop(open_bwd) # Lấy node u_b từ open_bwd.
            explored_bwd_viz.add(u_b)

            # Kiểm tra xem u_b đã được xử lý bởi tìm kiếm xuôi chưa.
            if u_b in g_fwd:
                path_cost_through_u_b = g_curr_b + g_fwd[u_b]
                if path_cost_through_u_b < mu:
                    mu = path_cost_through_u_b
                    meeting_node = u_b
            
            if g_curr_b >= mu : continue # Tối ưu hóa.

            # Duyệt qua các láng giềng v_b của u_b.
            # Giả sử đồ thị là vô hướng để có thể lấy láng giềng như tìm kiếm xuôi.
            for v_b, weight_uv_b in graph.get_neighbors(u_b):
                new_g_b = g_curr_b + weight_uv_b
                if new_g_b < g_bwd.get(v_b, float('inf')):
                    g_bwd[v_b] = new_g_b
                    parent_bwd[v_b] = u_b
                    # Heuristic cho tìm kiếm ngược là từ v_b đến start_node_rc.
                    f_v_bwd = new_g_b + heuristic_func(v_b, start_node_rc)
                    if f_v_bwd < mu :
                        heapq.heappush(open_bwd, (f_v_bwd, new_g_b, v_b))

    # --- Tái tạo đường đi ---
    if meeting_node: # Nếu đã tìm thấy một điểm gặp nhau tạo ra đường đi.
        path = []
        # Tái tạo phần đường đi từ start đến meeting_node (từ tìm kiếm xuôi).
        curr = meeting_node
        while curr is not None: path.append(curr); curr = parent_fwd.get(curr)
        path.reverse() # Đảo ngược để có thứ tự đúng.
        
        # Tái tạo phần đường đi từ goal đến meeting_node (từ tìm kiếm ngược),
        # và nối vào `path`. Bắt đầu từ node cha của meeting_node trong tìm kiếm ngược.
        curr = parent_bwd.get(meeting_node)
        while curr is not None: path.append(curr); curr = parent_bwd.get(curr)
        
        final_path = path
        # Kết hợp danh sách các node đã khám phá từ cả hai hướng cho visualization.
        # Thứ tự có thể không hoàn toàn theo thời gian thực của cả hai quá trình.
        combined_explored_viz = list(explored_fwd_viz.union(explored_bwd_viz))
        return final_path, mu, combined_explored_viz
        
    # Nếu không tìm thấy đường đi.
    return None, float("inf"), list(explored_fwd_viz.union(explored_bwd_viz))


# --- HÀM TIỆN ÍCH: TẠO ĐỒ THỊ TỪ LƯỚI ---
def create_graph_from_grid(grid_data):
    """
    Tạo một đối tượng Graph từ dữ liệu lưới game (grid_data).
    Các ô không phải chướng ngại vật sẽ trở thành node trong đồ thị.
    Các cạnh được tạo giữa các node láng giềng hợp lệ.

    Args:
        grid_data (list of list of GridNode): Dữ liệu lưới.

    Returns:
        Graph: Đối tượng đồ thị biểu diễn lưới.
    """
    graph = Graph()
    rows = len(grid_data)
    cols = len(grid_data[0]) if rows > 0 else 0
    allow_diagonal = True # Cho phép di chuyển chéo cho các thuật toán như A*, Dijkstra, Bi-A*.
                          # BFS và JPS có logic di chuyển riêng.

    for r_idx in range(rows):
        for c_idx in range(cols):
            current_grid_node_obj = grid_data[r_idx][c_idx]
            # Chỉ thêm node vào đồ thị nếu không phải là chướng ngại vật.
            if not current_grid_node_obj.is_obstacle_type():
                node_coord = (r_idx, c_idx)
                graph.add_node(node_coord) # Thêm node hiện tại vào đồ thị.
                
                # Xác định các hướng di chuyển có thể có.
                directions = [(0,1), (0,-1), (1,0), (-1,0)] # Hướng thẳng (Cardinal).
                if allow_diagonal:
                    directions.extend([(1,1), (1,-1), (-1,1), (-1,-1)]) # Hướng chéo (Diagonal).

                # Duyệt qua các hướng để tìm láng giềng.
                for dr, dc in directions:
                    nr, nc = r_idx + dr, c_idx + dc # Tọa độ láng giềng tiềm năng.
                    
                    # Kiểm tra xem láng giềng có nằm trong biên lưới không.
                    if 0 <= nr < rows and 0 <= nc < cols:
                        neighbor_grid_node_obj = grid_data[nr][nc]
                        # Nếu láng giềng không phải chướng ngại vật.
                        if not neighbor_grid_node_obj.is_obstacle_type():
                            neighbor_coord = (nr, nc)
                            cost_to_neighbor = neighbor_grid_node_obj.cost # Chi phí để đến láng giềng.
                            
                            # Xử lý chi phí cho di chuyển chéo và ngăn "cắt góc" (corner cutting).
                            if dr != 0 and dc != 0 and allow_diagonal: # Nếu là di chuyển chéo.
                                # Kiểm tra xem có đang cố gắng đi chéo qua góc của hai bức tường không.
                                # Nếu cả hai ô liền kề theo đường thẳng (tạo thành góc) đều là chướng ngại vật,
                                # thì không cho phép di chuyển chéo đó.
                                if grid_data[r_idx + dr][c_idx].is_obstacle_type() and \
                                   grid_data[r_idx][c_idx + dc].is_obstacle_type():
                                    continue # Bỏ qua, không thêm cạnh này.
                                
                                cost_to_neighbor *= math.sqrt(2) # Chi phí đường chéo là cost * căn 2.
                                
                            # Thêm cạnh từ node hiện tại đến láng giềng.
                            graph.add_edge(node_coord, neighbor_coord, cost_to_neighbor)
                            # Ghi chú: Nếu đồ thị là vô hướng, việc thêm cạnh ngược lại (neighbor -> current)
                            # sẽ tự động được xử lý khi vòng lặp duyệt đến ô neighbor_coord.
                            # graph.add_edge(neighbor_coord, node_coord, cost_to_neighbor)
    return graph