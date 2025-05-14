# main.py
import pygame
import pygame_gui
import sys
import time

# --- Import các cấu hình và module từ thư mục src ---
from config import (
    TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT, FPS,
    GRID_WIDTH, GRID_HEIGHT, UI_PANEL_WIDTH, GRID_ROWS, GRID_COLS, CELL_SIZE,
    WHITE, LIGHT_BLUE_BG, # DARK_GREY, GREY sẽ được xử lý bởi theme hoặc draw_grid_lines
    COLOR_ASTAR_PATH, COLOR_DIJKSTRA_PATH, COLOR_BFS_PATH, COLOR_GREEDY_PATH,
    COLOR_JPS_PATH, COLOR_BIDIR_PATH,
    # Các hằng số cho tốc độ animation từ config.py
    ANIM_VIZ_MIN_DELAY, ANIM_VIZ_MAX_DELAY
    # ANIM_SLIDER_MIN_VAL, ANIM_SLIDER_MAX_VAL, ANIM_SLIDER_DEFAULT_VAL # Nếu bạn dùng chúng để tính toán
)
from src.ui_panel import UIPanelManager
from src.sprite_manager import load_game_assets, get_background
from src.game_grid import create_grid, draw_grid_lines, get_clicked_grid_pos # GridNode không cần import trực tiếp
from src.algorithms import (
    create_graph_from_grid, a_star_search, dijkstra_search,
    bfs_search, greedy_bfs_search, heuristic_manhattan,
    jps_search,
    bidirectional_a_star_search
)
from src.maze_loader import MAZE_NAMES, apply_maze_to_grid
from src.agent import Agent
from pygame_gui.windows import UIMessageWindow # Để hiển thị hộp thoại thông báo

def main():
    """Hàm chính khởi chạy và quản lý vòng lặp của game Pathfinding Visualization."""
    # --- Khởi tạo Pygame và các module cơ bản ---
    pygame.init()       # Khởi tạo tất cả các module Pygame đã import
    pygame.font.init()  # Khởi tạo module font

    # --- Thiết lập màn hình và cửa sổ game ---
    screen = pygame.display.set_mode((TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT))
    pygame.display.set_caption("Pathfinding Visualization - Upgraded") # Đặt tiêu đề cho cửa sổ
    clock = pygame.time.Clock() # Tạo đối tượng Clock để kiểm soát FPS

    # --- Tải tài nguyên game (hình ảnh, âm thanh,...) ---
    load_game_assets() # Tải sprites và hình nền
    background_surface = get_background() # Lấy bề mặt hình nền đã được chuẩn bị

    # --- Khởi tạo pygame_gui UIManager ---
    # Thử tải theme từ file 'theme.json', nếu lỗi thì dùng theme mặc định.
    try:
        ui_manager = pygame_gui.UIManager((TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT), 'theme.json')
        print("Theme 'theme.json' loaded successfully.")
    except Exception as e:
        ui_manager = pygame_gui.UIManager((TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT)) # Fallback về theme mặc định
        print(f"Warning: Theme loading error ('{e}'). Using default UI theme.")

    # --- Định nghĩa các thuật toán tìm đường sẽ được sử dụng ---
    # Mỗi thuật toán là một dictionary chứa thông tin cần thiết để chạy và hiển thị.
    defined_algorithms = [
        {"name": "A*", "func": a_star_search, "is_graph_based": True, "heuristic": heuristic_manhattan, "path_color": COLOR_ASTAR_PATH, "line_thickness": 4},
        {"name": "Dijkstra", "func": dijkstra_search, "is_graph_based": True, "heuristic": None, "path_color": COLOR_DIJKSTRA_PATH, "line_thickness": 3},
        {"name": "BFS", "func": bfs_search, "is_graph_based": True, "heuristic": None, "path_color": COLOR_BFS_PATH, "line_thickness": 3},
        {"name": "Greedy BFS", "func": greedy_bfs_search, "is_graph_based": True, "heuristic": heuristic_manhattan, "path_color": COLOR_GREEDY_PATH, "line_thickness": 3},
        {"name": "JPS", "func": jps_search, "is_graph_based": False, "heuristic": heuristic_manhattan, "path_color": COLOR_JPS_PATH, "line_thickness": 4},
        {"name": "Bi-A*", "func": bidirectional_a_star_search, "is_graph_based": True, "heuristic": heuristic_manhattan, "path_color": COLOR_BIDIR_PATH, "line_thickness": 4}
    ]
    algorithm_names_for_ui = [algo["name"] for algo in defined_algorithms] # Lấy danh sách tên cho UI
    maze_names_for_ui = MAZE_NAMES # Lấy danh sách tên maze từ maze_loader

    # Khởi tạo UIPanelManager để quản lý các thành phần giao diện người dùng
    ui_panel_manager = UIPanelManager(ui_manager, algorithm_names_for_ui, maze_names_for_ui)

    # --- Khởi tạo các biến trạng thái của game ---
    game_grid = create_grid() # Tạo lưới ô vuông ban đầu
    start_node_pos = None     # Vị trí (row, col) của điểm bắt đầu, ban đầu là None
    end_node_pos = None       # Vị trí (row, col) của điểm kết thúc, ban đầu là None
    current_build_mode = "set_wall" # Chế độ xây dựng mặc định là "đặt tường"
    ui_panel_manager.update_build_mode_display(current_build_mode) # Đồng bộ hóa hiển thị chế độ xây dựng với UI

    path_results = {} # Dictionary để lưu trữ kết quả (đường đi, chi phí,...) của các thuật toán
    detailed_view_algo_name = "Overview / All Paths" # Thuật toán đang được xem chi tiết trên UI
    if ui_panel_manager.algo_dropdown: # Đảm bảo dropdown đã được tạo trước khi set giá trị
        ui_panel_manager.algo_dropdown.selected_option = detailed_view_algo_name # Đồng bộ dropdown với trạng thái

    active_agents = {} # Dictionary để lưu trữ các đối tượng Agent (xe)
    agent_speed = 2.5  # Tốc độ di chuyển của Agent (ô/giây)

    # --- Biến trạng thái cho Visualization Animation (hiển thị quá trình tìm đường) ---
    visualization_active = False  # True nếu animation đang chạy
    animation_paused = False      # True nếu animation đang tạm dừng
    nodes_to_visualize_explored = [] # Danh sách các ô đã khám phá để visualize
    nodes_to_visualize_path = []     # Danh sách các ô thuộc đường đi để visualize
    current_viz_explored_idx = 0     # Index hiện tại trong list nodes_to_visualize_explored
    current_viz_path_idx = 0         # Index hiện tại trong list nodes_to_visualize_path
    viz_delay_timer = 0.0            # Bộ đếm thời gian cho độ trễ giữa các bước animation
    
    # Tính toán độ trễ ban đầu cho mỗi node trong animation
    # Dựa trên giá trị mặc định của slider tốc độ (ví dụ: 5 trên thang 1-10)
    # và các hằng số ANIM_VIZ_MAX_DELAY, ANIM_VIZ_MIN_DELAY từ config.py.
    initial_slider_value = 5 # Giả sử slider có giá trị từ 1-10, 5 là trung bình
    # Công thức nội suy tuyến tính: delay = max_delay - (slider_val - min_slider_val) * (max_delay - min_delay) / (range_slider)
    # Giả sử slider_range = 10 - 1 = 9 (nếu slider từ 1 đến 10)
    current_viz_delay_per_node = ANIM_VIZ_MAX_DELAY - \
                                 (initial_slider_value - 1) * (ANIM_VIZ_MAX_DELAY - ANIM_VIZ_MIN_DELAY) / (10 - 1)

    # --- Vòng lặp chính của Game ---
    running = True
    while running:
        time_delta = clock.tick(FPS) / 1000.0 # Thời gian (giây) trôi qua kể từ frame trước
        mouse_pos = pygame.mouse.get_pos()    # Lấy vị trí chuột hiện tại

        # --- Cập nhật thông tin ô đang được trỏ chuột (hover) ---
        hovered_r_col = get_clicked_grid_pos(mouse_pos) # Lấy (row, col) của ô hover, hoặc (None,None)
        if hovered_r_col[0] is not None and mouse_pos[0] < GRID_WIDTH: # Nếu hover trong lưới
            hovered_node = game_grid[hovered_r_col[0]][hovered_r_col[1]] # Lấy đối tượng Node
            # Cập nhật thông tin hover lên UI panel
            ui_panel_manager.update_hover_info({
                "row": hovered_node.row, "col": hovered_node.col,
                "type": hovered_node.type, "cost": hovered_node.cost
            })
        else: # Nếu không hover trong lưới
            ui_panel_manager.update_hover_info(None) # Xóa thông tin hover

        # --- Xử lý Events (Input từ người dùng) ---
        for event in pygame.event.get(): # Duyệt qua hàng đợi sự kiện
            if event.type == pygame.QUIT: # Nếu người dùng nhấn nút đóng cửa sổ
                running = False # Kết thúc vòng lặp game

            # Chuyển sự kiện cho UIManager của pygame_gui xử lý (cho các thành phần UI)
            ui_manager.process_events(event)
            # Xử lý các hành động UI tùy chỉnh từ UIPanelManager
            ui_action = ui_panel_manager.process_ui_event(event)

            if ui_action: # Nếu có một hành động UI được trả về
                if isinstance(ui_action, str): # Nếu hành động là một chuỗi (thường là các nút bấm chính)
                    if ui_action == "run_algorithms":
                        # Kiểm tra xem điểm bắt đầu và kết thúc đã được đặt chưa
                        if not start_node_pos or not end_node_pos:
                            UIMessageWindow(rect=pygame.Rect((TOTAL_SCREEN_WIDTH // 2 - 175, TOTAL_SCREEN_HEIGHT // 2 - 75), (350, 150)),
                                            html_message="Please set <b>Start</b> and <b>End</b> points first.", manager=ui_manager, window_title="Input Required")
                        else:
                            # --- Chạy các thuật toán tìm đường ---
                            print("\n--- Running Pathfinding Algorithms ---")
                            # Reset trạng thái animation
                            visualization_active = False; animation_paused = False
                            if ui_panel_manager.pause_resume_button: ui_panel_manager.update_pause_button_text(animation_paused)
                            current_viz_explored_idx = 0; current_viz_path_idx = 0
                            nodes_to_visualize_explored.clear(); nodes_to_visualize_path.clear()
                            
                            # Tạo biểu diễn đồ thị từ lưới (nếu có thuật toán cần)
                            current_graph_repr = None
                            if any(algo.get("is_graph_based", True) for algo in defined_algorithms):
                                current_graph_repr = create_graph_from_grid(game_grid)
                            
                            # Xóa kết quả cũ và reset agent về điểm bắt đầu
                            path_results.clear()
                            for agent in active_agents.values():
                                if start_node_pos: agent.reset_to_start(start_node_pos)

                            # Reset trạng thái explored/path của các ô trên lưới
                            for r_nodes in game_grid:
                                for node in r_nodes:
                                    node.is_explored = False; node.is_path = False; node.path_color = None
                            
                            any_path_found_this_run = False # Cờ kiểm tra có thuật toán nào tìm được đường không
                            # Chạy lần lượt các thuật toán đã định nghĩa
                            for algo_config in defined_algorithms:
                                algo_name = algo_config["name"]; algo_func = algo_config["func"]
                                heuristic = algo_config.get("heuristic")
                                is_graph_based = algo_config.get("is_graph_based", True)
                                
                                start_time = time.perf_counter() # Bắt đầu đo thời gian
                                path, cost, explored_coords = (None, float('inf'), []) # Kết quả mặc định
                                try:
                                    if is_graph_based: # Thuật toán dựa trên đồ thị
                                        if not current_graph_repr: continue # Bỏ qua nếu không có đồ thị
                                        if heuristic: path, cost, explored_coords = algo_func(current_graph_repr, start_node_pos, end_node_pos, heuristic)
                                        else: path, cost, explored_coords = algo_func(current_graph_repr, start_node_pos, end_node_pos)
                                    else: # Thuật toán dựa trên lưới (ví dụ: JPS)
                                        if heuristic: path, cost, explored_coords = algo_func(game_grid, start_node_pos, end_node_pos, heuristic)
                                        else: path, cost, explored_coords = algo_func(game_grid, start_node_pos, end_node_pos)
                                except Exception as e: print(f"  Error running {algo_name}: {e}") # In lỗi nếu có
                                
                                time_taken_ms = (time.perf_counter() - start_time) * 1000 # Tính thời gian (ms)
                                print(f"  {algo_name}: Cost={cost if cost != float('inf') else 'N/A'}, Path={'Yes' if path else 'No'}, Explored={len(explored_coords)}, Time={time_taken_ms:.2f} ms")
                                
                                # Lưu kết quả của thuật toán
                                path_results[algo_name] = {
                                    "path": path, "cost": cost, "explored": explored_coords,
                                    "color": algo_config["path_color"], "time_ms": time_taken_ms,
                                    "line_thickness": algo_config.get("line_thickness", 3)
                                }
                                if path: any_path_found_this_run = True # Đánh dấu đã tìm thấy đường đi
                                
                                # Tạo hoặc cập nhật Agent nếu tìm thấy đường đi và có điểm bắt đầu
                                if path and start_node_pos:
                                    if algo_name not in active_agents: # Nếu chưa có Agent cho thuật toán này
                                        # Tạo key cho sprite dựa trên tên thuật toán
                                        sprite_key = f"car_{algo_name.lower().replace(' ', '_').replace('*','star')}"
                                        active_agents[algo_name] = Agent(start_node_pos, sprite_key, algo_name, speed=agent_speed)
                                    active_agents[algo_name].set_path(path) # Gán đường đi cho Agent
                                elif algo_name in active_agents: # Nếu không tìm thấy đường, xóa đường đi của Agent
                                    active_agents[algo_name].set_path(None)
                            
                            # --- Cập nhật UI và chuẩn bị cho animation ---
                            if detailed_view_algo_name != "Overview / All Paths": # Nếu đang xem chi tiết một thuật toán
                                if detailed_view_algo_name in path_results: # Nếu thuật toán đó có kết quả
                                    res = path_results[detailed_view_algo_name]
                                    ui_panel_manager.update_selected_algorithm_info(detailed_view_algo_name, res["cost"], len(res["explored"]), res["time_ms"])
                                    # Chuẩn bị dữ liệu cho animation
                                    if res["explored"]: nodes_to_visualize_explored = list(res["explored"])
                                    if res["path"]: nodes_to_visualize_path = list(res["path"])
                                    current_visualizing_algo_color = res["color"] # Màu cho visualization
                                    if nodes_to_visualize_explored or nodes_to_visualize_path: visualization_active = True # Kích hoạt animation
                                else: # Thuật toán đang xem không có kết quả (ví dụ: lỗi hoặc chưa chạy)
                                    ui_panel_manager.update_selected_algorithm_info(detailed_view_algo_name, "N/A", "N/A", "N/A")
                            else: # Nếu đang ở chế độ "Overview / All Paths"
                                ui_panel_manager.update_overview_summary(path_results, True) # Hiển thị bảng tóm tắt
                                ui_panel_manager.update_selected_algorithm_info(None, None, None) # Ẩn/reset phần chi tiết

                            # Hiển thị thông báo nếu không có thuật toán nào tìm được đường đi
                            if not any_path_found_this_run and not visualization_active:
                                UIMessageWindow(rect=pygame.Rect((TOTAL_SCREEN_WIDTH // 2 - 150, TOTAL_SCREEN_HEIGHT // 2 - 75), (300, 150)),
                                    html_message="No path found by any algorithm.", manager=ui_manager, window_title="Search Result")

                    elif ui_action == "reset_grid":
                        # Reset lưới, điểm bắt đầu/kết thúc, kết quả, agent
                        game_grid = create_grid(); start_node_pos = None; end_node_pos = None
                        path_results.clear(); active_agents.clear()
                        # Reset trạng thái animation và UI liên quan
                        visualization_active = False; animation_paused = False
                        if ui_panel_manager.pause_resume_button: ui_panel_manager.update_pause_button_text(animation_paused)
                        detailed_view_algo_name = "Overview / All Paths"
                        if ui_panel_manager.algo_dropdown: ui_panel_manager.algo_dropdown.selected_option = "Overview / All Paths"
                        if ui_panel_manager.maze_dropdown: ui_panel_manager.maze_dropdown.selected_option = "Custom"
                        ui_panel_manager.update_selected_algorithm_info(None, None, None)
                        ui_panel_manager.update_overview_summary(None, False) # Ẩn bảng tóm tắt
                        print("Grid Reset.")
                    
                    elif ui_action == "clear_paths":
                        # Xóa visualization (explored, path) trên lưới và reset agent
                        visualization_active = False; animation_paused = False
                        if ui_panel_manager.pause_resume_button: ui_panel_manager.update_pause_button_text(animation_paused)
                        for r_nodes in game_grid: # Reset cờ is_explored, is_path của các ô
                            for node in r_nodes:
                                node.is_explored = False; node.is_path = False; node.path_color = None
                        for agent in active_agents.values(): # Reset agent về điểm bắt đầu (nếu có)
                            if start_node_pos: agent.reset_to_start(start_node_pos)
                        print("Paths/Explored visualization cleared. Agents reset.")

                    elif ui_action == "toggle_pause_animation":
                        # Tạm dừng hoặc tiếp tục animation (nếu đang chạy)
                        if visualization_active:
                            animation_paused = not animation_paused
                            if ui_panel_manager.pause_resume_button: ui_panel_manager.update_pause_button_text(animation_paused)
                
                elif isinstance(ui_action, dict): # Nếu hành động là dictionary (thường từ dropdown, slider)
                    action_type = ui_action.get("type"); action_value = ui_action.get("value")

                    if action_type == "build_mode_changed":
                        # Thay đổi chế độ xây dựng (wall, trap, start, end)
                        current_build_mode = action_value
                        ui_panel_manager.update_build_mode_display(current_build_mode) # Cập nhật UI
                    
                    elif action_type == "algo_view_changed":
                        # Thay đổi thuật toán đang được xem chi tiết trên UI
                        detailed_view_algo_name = action_value
                        # Reset animation và trạng thái explored/path trên lưới
                        visualization_active = False; animation_paused = False
                        if ui_panel_manager.pause_resume_button: ui_panel_manager.update_pause_button_text(animation_paused)
                        current_viz_explored_idx = 0; current_viz_path_idx = 0
                        nodes_to_visualize_explored.clear(); nodes_to_visualize_path.clear()

                        for r_nodes in game_grid: # Xóa visualization cũ
                            for node in r_nodes:
                                node.is_explored = False; node.is_path = False; node.path_color = None
                        
                        if detailed_view_algo_name != "Overview / All Paths": # Nếu xem chi tiết một thuật toán
                            ui_panel_manager.update_overview_summary(None, False) # Ẩn bảng tóm tắt
                            if detailed_view_algo_name in path_results: # Nếu thuật toán này đã có kết quả
                                res = path_results[detailed_view_algo_name]
                                # Cập nhật thông tin và chuẩn bị animation cho thuật toán mới
                                ui_panel_manager.update_selected_algorithm_info(detailed_view_algo_name, res["cost"], len(res["explored"]), res["time_ms"])
                                if res["explored"]: nodes_to_visualize_explored = list(res["explored"])
                                if res["path"]: nodes_to_visualize_path = list(res["path"])
                                current_visualizing_algo_color = res["color"]
                                if nodes_to_visualize_explored or nodes_to_visualize_path: visualization_active = True
                            else: # Nếu thuật toán được chọn chưa có kết quả (ví dụ, trước lần chạy đầu tiên)
                                ui_panel_manager.update_selected_algorithm_info(detailed_view_algo_name, "N/A", "N/A", "N/A")
                        else: # Nếu chuyển về chế độ "Overview / All Paths"
                            ui_panel_manager.update_selected_algorithm_info(None, None, None) # Xóa thông tin chi tiết
                            ui_panel_manager.update_overview_summary(path_results, True) # Hiển thị bảng tóm tắt

                    elif action_type == "maze_selected":
                        # Chọn một mê cung mẫu từ dropdown
                        selected_maze_name = action_value
                        visualization_active = False; animation_paused = False # Reset animation
                        if ui_panel_manager.pause_resume_button: ui_panel_manager.update_pause_button_text(animation_paused)
                        if selected_maze_name != "Custom": # Nếu không phải là "Custom" (tự vẽ)
                            # Áp dụng mê cung mẫu vào lưới
                            new_start, new_end = apply_maze_to_grid(game_grid, selected_maze_name)
                            if new_start and new_end: # Nếu mê cung được tải thành công
                                start_node_pos = new_start; end_node_pos = new_end
                                path_results.clear(); active_agents.clear() # Xóa dữ liệu cũ
                                # Reset UI về chế độ overview
                                detailed_view_algo_name = "Overview / All Paths"
                                if ui_panel_manager.algo_dropdown: ui_panel_manager.algo_dropdown.selected_option = "Overview / All Paths"
                                ui_panel_manager.update_selected_algorithm_info(None, None, None)
                                ui_panel_manager.update_overview_summary(None, False)
                                # Reset trạng thái visualized của các ô
                                for r_nodes in game_grid:
                                    for node in r_nodes: node.is_explored = False; node.is_path = False
                            else: # Nếu tải mê cung lỗi, quay lại "Custom" trên dropdown
                                if ui_panel_manager.maze_dropdown: ui_panel_manager.maze_dropdown.selected_option = "Custom"
                        
                    elif action_type == "viz_speed_changed":
                        # Thay đổi tốc độ animation từ slider
                        slider_val = action_value # Giá trị từ 1 (chậm) đến 10 (nhanh)
                        # Tính toán lại độ trễ mỗi node dựa trên giá trị slider và các hằng số min/max delay
                        current_viz_delay_per_node = ANIM_VIZ_MAX_DELAY - \
                                                    (slider_val - 1) * (ANIM_VIZ_MAX_DELAY - ANIM_VIZ_MIN_DELAY) / (10 - 1) # Giả sử slider range 1-10

            # --- Xử lý input click chuột trên grid (để xây dựng) ---
            if event.type == pygame.MOUSEBUTTONDOWN: # Nếu có sự kiện nhấn chuột
                if mouse_pos[0] < GRID_WIDTH: # Chỉ xử lý click trong khu vực lưới
                    r_clicked, c_clicked = get_clicked_grid_pos(mouse_pos) # Lấy tọa độ ô được click
                    if r_clicked is not None and c_clicked is not None: # Nếu click trúng một ô hợp lệ
                        node = game_grid[r_clicked][c_clicked] # Lấy đối tượng Node tại ô đó
                        
                        if event.button == 1: # Left Click (Chuột trái)
                            if current_build_mode == "set_start":
                                if start_node_pos: game_grid[start_node_pos[0]][start_node_pos[1]].reset() # Xóa điểm bắt đầu cũ
                                node.make_start(); start_node_pos = (r_clicked, c_clicked)
                                # Reset tất cả agent về vị trí bắt đầu mới
                                for agent in active_agents.values(): 
                                    if start_node_pos: agent.reset_to_start(start_node_pos)
                            elif current_build_mode == "set_end":
                                if end_node_pos: game_grid[end_node_pos[0]][end_node_pos[1]].reset() # Xóa điểm kết thúc cũ
                                if not node.is_start_type(): node.make_end(); end_node_pos = (r_clicked, c_clicked)
                            elif current_build_mode == "set_wall":
                                if not node.is_start_type() and not node.is_end_type(): node.make_obstacle()
                            elif current_build_mode == "set_trap":
                                if not node.is_start_type() and not node.is_end_type(): node.make_trap()
                        elif event.button == 3: # Right Click (Chuột phải) - Xóa ô
                            if node.is_start_type(): start_node_pos = None
                            elif node.is_end_type(): end_node_pos = None
                            node.reset() # Reset ô về trạng thái mặc định (trống)

        # --- CẬP NHẬT TRẠNG THÁI GAME ---
        ui_manager.update(time_delta) # Cập nhật UIManager của pygame_gui
        
        # Cập nhật animation của từng ô trên lưới (ví dụ: hiệu ứng trap nhấp nháy)
        for r_nodes in game_grid:
            for node_obj in r_nodes:
                node_obj.update_animation(time_delta)
        
        # --- Logic cho Visualization Animation ---
        if visualization_active and not animation_paused: # Nếu animation đang chạy và không bị tạm dừng
            viz_delay_timer += time_delta # Tăng bộ đếm thời gian
            # Xử lý nhiều node nếu time_delta lớn hơn nhiều lần current_viz_delay_per_node
            # để animation không bị giật cục khi FPS thấp hoặc delay quá nhỏ.
            while viz_delay_timer >= current_viz_delay_per_node and visualization_active:
                viz_delay_timer -= current_viz_delay_per_node # Giữ lại phần dư cho lần sau
                
                # Visualize các ô đã duyệt (explored)
                if current_viz_explored_idx < len(nodes_to_visualize_explored):
                    r_ex, c_ex = nodes_to_visualize_explored[current_viz_explored_idx]
                    if 0 <= r_ex < GRID_ROWS and 0 <= c_ex < GRID_COLS: # Kiểm tra tọa độ hợp lệ
                        node_to_mark = game_grid[r_ex][c_ex]
                        # Chỉ đánh dấu là explored nếu không phải là start, end hoặc đã là path
                        if not (node_to_mark.is_start_type() or node_to_mark.is_end_type() or node_to_mark.is_path):
                            node_to_mark.is_explored = True
                    current_viz_explored_idx += 1
                # Visualize các ô thuộc đường đi (path)
                elif current_viz_path_idx < len(nodes_to_visualize_path):
                    r_p, c_p = nodes_to_visualize_path[current_viz_path_idx]
                    if 0 <= r_p < GRID_ROWS and 0 <= c_p < GRID_COLS: # Kiểm tra tọa độ hợp lệ
                        node_to_mark = game_grid[r_p][c_p]
                        # Chỉ đánh dấu là path nếu không phải là start hoặc end
                        if not (node_to_mark.is_start_type() or node_to_mark.is_end_type()):
                            node_to_mark.is_path = True; node_to_mark.is_explored = False # Ô path không còn là explored
                            node_to_mark.path_color = current_visualizing_algo_color # Gán màu cho đường đi
                    current_viz_path_idx += 1
                else: # Animation hoàn tất
                    visualization_active = False; animation_paused = False # Dừng animation
                    if ui_panel_manager.pause_resume_button: ui_panel_manager.update_pause_button_text(animation_paused)
                    break # Thoát khỏi vòng lặp while của viz_delay_timer

        # Cập nhật vị trí các Agent (nếu có và đang di chuyển)
        for agent_obj in active_agents.values():
            if not agent_obj.finished_path: # Nếu agent chưa đi hết đường
                 agent_obj.update(time_delta) # Cập nhật vị trí agent
        
        # --- VẼ LÊN MÀN HÌNH ---
        screen.fill(LIGHT_BLUE_BG) # Tô màu nền cho toàn bộ màn hình
        if background_surface: # Nếu có ảnh nền
            screen.blit(background_surface, (0, 0)) # Vẽ ảnh nền
        else: # Nếu không có ảnh nền, vẽ một hình chữ nhật màu trắng cho khu vực lưới
            pygame.draw.rect(screen, WHITE, (0, 0, GRID_WIDTH, GRID_HEIGHT))

        # Vẽ các ô trên lưới (bao gồm sprite, màu explored, màu path nếu có)
        for r_nodes in game_grid:
            for node_obj in r_nodes:
                node_obj.draw(screen)
        
        # Vẽ đường đi của TẤT CẢ các thuật toán khi ở chế độ "Overview" và không có animation nào đang chạy
        if detailed_view_algo_name == "Overview / All Paths" and not visualization_active:
            for algo_name, result_data in path_results.items():
                if result_data["path"]: # Nếu thuật toán này tìm được đường đi
                    path_nodes = result_data["path"]; color = result_data["color"]
                    thickness = result_data.get("line_thickness", 2) # Độ dày đường kẻ
                    # Vẽ các đoạn thẳng nối các ô trong đường đi
                    for i in range(len(path_nodes) - 1):
                        r1, c1 = path_nodes[i]; r2, c2 = path_nodes[i + 1]
                        # Tính tọa độ pixel trung tâm của ô
                        x1 = c1 * CELL_SIZE + CELL_SIZE // 2
                        y1 = r1 * CELL_SIZE + CELL_SIZE // 2
                        x2 = c2 * CELL_SIZE + CELL_SIZE // 2
                        y2 = r2 * CELL_SIZE + CELL_SIZE // 2
                        pygame.draw.line(screen, color, (x1, y1), (x2, y2), thickness)
        
        draw_grid_lines(screen) # Vẽ các đường kẻ của lưới lên trên các ô
        
        # Vẽ các Agent lên trên lưới và đường đi
        for agent_obj in active_agents.values():
            agent_obj.draw(screen)
            
        ui_manager.draw_ui(screen) # Vẽ các thành phần UI (panel, nút, dropdown,...) lên trên cùng
        
        pygame.display.flip() # Cập nhật toàn bộ nội dung màn hình để hiển thị

    # --- Kết thúc Pygame khi vòng lặp chính dừng ---
    pygame.quit()
    sys.exit() # Thoát chương trình

if __name__ == '__main__':
    main()