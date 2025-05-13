# main.py
import pygame
import pygame_gui
import sys
import time

# Import các cấu hình và module từ thư mục src
from config import (
    TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT, FPS,
    GRID_WIDTH, GRID_HEIGHT, UI_PANEL_WIDTH, GRID_ROWS, GRID_COLS, CELL_SIZE,
    WHITE, LIGHT_BLUE_BG, DARK_GREY, GREY,
    COLOR_ASTAR_PATH, COLOR_DIJKSTRA_PATH, COLOR_BFS_PATH, COLOR_GREEDY_PATH,
    COLOR_JPS_PATH, COLOR_BIDIR_PATH, # Đảm bảo các màu này đã có trong config.py
    # Các hằng số cho tốc độ animation nếu bạn đưa ra config
    ANIM_VIZ_MIN_DELAY, ANIM_VIZ_MAX_DELAY
)
from src.ui_panel import UIPanelManager
from src.sprite_manager import load_game_assets, get_background
from src.game_grid import create_grid, draw_grid_lines, get_clicked_grid_pos
from src.algorithms import (
    create_graph_from_grid, a_star_search, dijkstra_search,
    bfs_search, greedy_bfs_search, heuristic_manhattan,
    jps_search,
    bidirectional_a_star_search
)
from src.maze_loader import MAZE_NAMES, apply_maze_to_grid # Đảm bảo MAZE_NAMES được export đúng
from src.agent import Agent
from pygame_gui.windows import UIMessageWindow # Để hiển thị các hộp thoại thông báo


def main():
    """Hàm chính khởi chạy và quản lý vòng lặp game."""
    pygame.init() # Khởi tạo tất cả các module của Pygame
    pygame.font.init() # Khởi tạo module font của Pygame

    # Thiết lập màn hình chính
    screen = pygame.display.set_mode((TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT))
    pygame.display.set_caption("Pathfinding Visualization - Upgraded") # Đặt tiêu đề cửa sổ
    clock = pygame.time.Clock() # Tạo đối tượng Clock để quản lý FPS

    # Tải tài nguyên game (hình ảnh, âm thanh nếu có)
    load_game_assets()
    background_surface = get_background() # Lấy bề mặt nền đã được chuẩn bị

    # Khởi tạo pygame_gui UIManager với theme (nếu có)
    try:
        ui_manager = pygame_gui.UIManager((TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT), 'theme.json')
        print("Theme 'theme.json' loaded successfully.")
    except Exception as e: # Bắt Exception chung hơn để xử lý lỗi tải theme
        ui_manager = pygame_gui.UIManager((TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT)) # Dùng theme mặc định nếu có lỗi
        print(f"Warning: Theme loading error ('{e}'). Using default UI theme.")

    # --- Định nghĩa các thuật toán tìm đường ---
    # Mỗi thuật toán được định nghĩa là một dictionary chứa tên, hàm thực thi,
    # loại (graph-based hay grid-based), heuristic (nếu có), màu đường đi và độ dày đường kẻ.
    defined_algorithms = [
        {"name": "A*", "func": a_star_search, "is_graph_based": True, "heuristic": heuristic_manhattan, "path_color": COLOR_ASTAR_PATH, "line_thickness": 4},
        {"name": "Dijkstra", "func": dijkstra_search, "is_graph_based": True, "heuristic": None, "path_color": COLOR_DIJKSTRA_PATH, "line_thickness": 3},
        {"name": "BFS", "func": bfs_search, "is_graph_based": True, "heuristic": None, "path_color": COLOR_BFS_PATH, "line_thickness": 3},
        {"name": "Greedy BFS", "func": greedy_bfs_search, "is_graph_based": True, "heuristic": heuristic_manhattan, "path_color": COLOR_GREEDY_PATH, "line_thickness": 3},
        {"name": "JPS", "func": jps_search, "is_graph_based": False, "heuristic": heuristic_manhattan, "path_color": COLOR_JPS_PATH, "line_thickness": 4},
        {"name": "Bi-A*", "func": bidirectional_a_star_search, "is_graph_based": True, "heuristic": heuristic_manhattan, "path_color": COLOR_BIDIR_PATH, "line_thickness": 4}
    ]
    algorithm_names_for_ui = [algo["name"] for algo in defined_algorithms] # Lấy danh sách tên thuật toán cho UI Panel
    maze_names_for_ui = MAZE_NAMES # Lấy danh sách tên mê cung từ maze_loader

    # Khởi tạo UIPanelManager để quản lý các thành phần UI
    ui_panel_manager = UIPanelManager(ui_manager, algorithm_names_for_ui, maze_names_for_ui)

    # --- Biến trạng thái của Game ---
    game_grid = create_grid() # Tạo lưới ô vuông ban đầu
    start_node_pos = None # Vị trí nút bắt đầu (row, col)
    end_node_pos = None # Vị trí nút kết thúc (row, col)
    current_build_mode = "set_wall" # Chế độ xây dựng mặc định là đặt tường
    ui_panel_manager.update_build_mode_display(current_build_mode) # Cập nhật hiển thị chế độ xây dựng trên UI

    path_results = {} # Dictionary lưu trữ kết quả tìm đường của các thuật toán
    detailed_view_algo_name = "Overview / All Paths" # Thuật toán đang được xem chi tiết (mặc định là tổng quan)
    ui_panel_manager.algo_dropdown.selected_option = detailed_view_algo_name # Đảm bảo dropdown trên UI khớp

    active_agents = {} # Dictionary lưu trữ các đối tượng Agent (nếu có)
    agent_speed = 2.5 # Tốc độ di chuyển của Agent

    # --- Biến trạng thái của Visualization Animation ---
    visualization_active = False # Cờ cho biết animation có đang chạy không
    animation_paused = False # Cờ cho biết animation có đang tạm dừng không
    nodes_to_visualize_explored = [] # Danh sách các nút đã duyệt để visualize
    nodes_to_visualize_path = [] # Danh sách các nút thuộc đường đi để visualize
    current_viz_explored_idx = 0 # Index hiện tại trong danh sách nút đã duyệt
    current_viz_path_idx = 0 # Index hiện tại trong danh sách nút đường đi
    viz_delay_timer = 0.0 # Bộ đếm thời gian cho độ trễ giữa các bước animation
    
    # Tính toán độ trễ ban đầu cho mỗi node dựa trên giá trị mặc định hoặc config.
    # Giá trị này sẽ được cập nhật khi người dùng thay đổi slider tốc độ.
    # Giả sử slider có 10 mức, giá trị 5 là trung bình.
    # ANIM_VIZ_MAX_DELAY là chậm nhất (slider ở 1), ANIM_VIZ_MIN_DELAY là nhanh nhất (slider ở 10).
    initial_slider_value = 5 # Giá trị slider ban đầu
    # Công thức nội suy tuyến tính để tính độ trễ từ giá trị slider
    current_viz_delay_per_node = ANIM_VIZ_MAX_DELAY - (initial_slider_value - 1) * \
                                 (ANIM_VIZ_MAX_DELAY - ANIM_VIZ_MIN_DELAY) / (10 - 1)
    # Hoặc gán trực tiếp nếu không dùng công thức trên:
    # current_viz_delay_per_node = 0.01 # Giá trị mặc định (tương ứng slider = 5)


    running = True # Biến điều khiển vòng lặp chính của game
    while running:
        time_delta = clock.tick(FPS) / 1000.0 # Thời gian trôi qua kể từ frame trước (tính bằng giây)
        mouse_pos = pygame.mouse.get_pos() # Lấy vị trí chuột hiện tại

        # --- Lấy thông tin ô đang được trỏ chuột (hover) trên lưới ---
        hovered_r_col = get_clicked_grid_pos(mouse_pos) # Trả về (row, col) hoặc (None, None)
        if hovered_r_col[0] is not None and mouse_pos[0] < GRID_WIDTH: # Nếu chuột đang trỏ vào một ô trong lưới
            hovered_node_data = game_grid[hovered_r_col[0]][hovered_r_col[1]]
            # Cập nhật thông tin ô hover lên UI panel
            ui_panel_manager.update_hover_info({
                "row": hovered_node_data.row, "col": hovered_node_data.col,
                "type": hovered_node_data.type, "cost": hovered_node_data.cost
            })
        else: # Nếu chuột không trỏ vào ô nào trong lưới
            ui_panel_manager.update_hover_info(None) # Xóa thông tin hover trên UI

        # --- XỬ LÝ EVENTS ---
        for event in pygame.event.get(): # Duyệt qua hàng đợi sự kiện
            if event.type == pygame.QUIT: # Nếu người dùng đóng cửa sổ
                running = False # Kết thúc vòng lặp game

            # Chuyển sự kiện cho UIManager của pygame_gui xử lý
            ui_manager.process_events(event)
            # Xử lý sự kiện UI từ UIPanelManager (nút bấm, dropdown, slider)
            ui_action = ui_panel_manager.process_ui_event(event)

            if ui_action: # Nếu có hành động từ UI được trả về
                if isinstance(ui_action, str): # Nếu hành động là một chuỗi (thường là các nút bấm chính)
                    if ui_action == "run_algorithms":
                        # Kiểm tra xem điểm bắt đầu và kết thúc đã được đặt chưa
                        if not start_node_pos or not end_node_pos:
                            UIMessageWindow( rect=pygame.Rect((TOTAL_SCREEN_WIDTH // 2 - 175, TOTAL_SCREEN_HEIGHT // 2 - 75), (350, 150)),
                                html_message="Please set <b>Start</b> and <b>End</b> points first.", manager=ui_manager, window_title="Input Required")
                        else:
                            # Chạy các thuật toán tìm đường
                            print("\n--- Running Pathfinding Algorithms ---")
                            # Reset trạng thái animation
                            visualization_active = False; animation_paused = False
                            ui_panel_manager.update_pause_button_text(animation_paused) # Cập nhật text nút Pause
                            current_viz_explored_idx = 0; current_viz_path_idx = 0
                            nodes_to_visualize_explored.clear(); nodes_to_visualize_path.clear()
                            
                            # Tạo biểu diễn đồ thị từ lưới (nếu có thuật toán dùng đồ thị)
                            current_graph_repr = None 
                            if any(algo.get("is_graph_based", True) for algo in defined_algorithms):
                                current_graph_repr = create_graph_from_grid(game_grid)
                            
                            # Xóa kết quả cũ và reset agent về vị trí bắt đầu
                            path_results.clear()
                            for agent in active_agents.values():
                                if start_node_pos: agent.reset_to_start(start_node_pos)

                            # Reset trạng thái is_explored, is_path của các ô trên lưới
                            for r_nodes in game_grid:
                                for node in r_nodes:
                                    node.is_explored = False; node.is_path = False; node.path_color = None
                            
                            any_path_found_this_run = False # Cờ kiểm tra có thuật toán nào tìm được đường đi không
                            # Chạy từng thuật toán đã định nghĩa
                            for algo_config in defined_algorithms:
                                algo_name = algo_config["name"]; algo_func = algo_config["func"]
                                heuristic = algo_config.get("heuristic")
                                is_graph_based = algo_config.get("is_graph_based", True)
                                
                                start_time = time.perf_counter() # Bắt đầu đo thời gian
                                path, cost, explored_coords = (None, float('inf'), []) # Kết quả mặc định
                                try:
                                    if is_graph_based: # Nếu thuật toán dựa trên đồ thị
                                        if not current_graph_repr: continue # Bỏ qua nếu không có đồ thị
                                        if heuristic: path, cost, explored_coords = algo_func(current_graph_repr, start_node_pos, end_node_pos, heuristic)
                                        else: path, cost, explored_coords = algo_func(current_graph_repr, start_node_pos, end_node_pos)
                                    else: # Nếu thuật toán dựa trên lưới (ví dụ: JPS)
                                        if heuristic: path, cost, explored_coords = algo_func(game_grid, start_node_pos, end_node_pos, heuristic)
                                        else: path, cost, explored_coords = algo_func(game_grid, start_node_pos, end_node_pos)
                                except Exception as e: print(f"  Error running {algo_name}: {e}") # In lỗi nếu có
                                
                                time_taken_ms = (time.perf_counter() - start_time) * 1000 # Tính thời gian thực thi (ms)
                                print(f"  {algo_name}: Cost={cost if cost != float('inf') else 'N/A'}, Path={'Yes' if path else 'No'}, Explored={len(explored_coords)}, Time={time_taken_ms:.2f} ms")
                                
                                # Lưu kết quả của thuật toán
                                path_results[algo_name] = {
                                    "path": path, "cost": cost, "explored": explored_coords,
                                    "color": algo_config["path_color"], "time_ms": time_taken_ms,
                                    "line_thickness": algo_config.get("line_thickness", 3)
                                }
                                if path: any_path_found_this_run = True # Đánh dấu đã tìm thấy đường đi
                                
                                # Cập nhật hoặc tạo agent nếu tìm thấy đường đi
                                if path: 
                                    if algo_name not in active_agents: # Nếu chưa có agent cho thuật toán này
                                        # Tạo key cho sprite dựa trên tên thuật toán
                                        sprite_key = f"car_{algo_name.lower().replace(' ', '_').replace('*','star')}"
                                        active_agents[algo_name] = Agent(start_node_pos, sprite_key, algo_name, speed=agent_speed)
                                    active_agents[algo_name].set_path(path) # Gán đường đi cho agent
                                elif algo_name in active_agents: # Nếu không tìm thấy đường đi, xóa đường đi của agent (nếu có)
                                    active_agents[algo_name].set_path(None)
                            
                            # Cập nhật UI và chuẩn bị cho animation
                            if detailed_view_algo_name != "Overview / All Paths":
                                # Nếu đang xem chi tiết một thuật toán cụ thể
                                if detailed_view_algo_name in path_results:
                                    res = path_results[detailed_view_algo_name]
                                    # Cập nhật thông tin thuật toán lên UI
                                    ui_panel_manager.update_selected_algorithm_info(detailed_view_algo_name, res["cost"], len(res["explored"]), res["time_ms"])
                                    # Chuẩn bị dữ liệu cho animation
                                    if res["explored"]: nodes_to_visualize_explored = list(res["explored"])
                                    if res["path"]: nodes_to_visualize_path = list(res["path"])
                                    current_visualizing_algo_color = res["color"] # Màu cho visualization
                                    if nodes_to_visualize_explored or nodes_to_visualize_path: visualization_active = True # Kích hoạt animation
                                else: 
                                    # Thuật toán đang xem chi tiết không có kết quả (ví dụ, lỗi hoặc chưa chạy)
                                    ui_panel_manager.update_selected_algorithm_info(detailed_view_algo_name, "N/A", "N/A", "N/A")
                            else: 
                                # Nếu đang ở chế độ "Overview / All Paths"
                                ui_panel_manager.update_overview_summary(path_results, True) # Hiển thị bảng tóm tắt
                                ui_panel_manager.update_selected_algorithm_info(None, None, None) # Ẩn/reset phần chi tiết

                            # Kiểm tra xem có đường đi nào được tìm thấy không để hiển thị thông báo
                            any_path_found_overall = any(res.get("path") for res in path_results.values())
                            if not any_path_found_overall and not visualization_active: # Nếu không có đường đi và không có animation
                                UIMessageWindow(rect=pygame.Rect((TOTAL_SCREEN_WIDTH // 2 - 150, TOTAL_SCREEN_HEIGHT // 2 - 75), (300, 150)),
                                    html_message="No path found by any algorithm.", manager=ui_manager, window_title="Search Result")


                    elif ui_action == "reset_grid":
                        # Reset lưới, điểm bắt đầu/kết thúc, kết quả, agent
                        game_grid = create_grid(); start_node_pos = None; end_node_pos = None
                        path_results.clear(); active_agents.clear()
                        # Reset trạng thái animation
                        visualization_active = False; animation_paused = False
                        ui_panel_manager.update_pause_button_text(animation_paused)
                        # Reset UI về trạng thái mặc định
                        detailed_view_algo_name = "Overview / All Paths"
                        ui_panel_manager.algo_dropdown.selected_option = "Overview / All Paths"
                        ui_panel_manager.maze_dropdown.selected_option = "Custom" # Reset dropdown maze
                        ui_panel_manager.update_selected_algorithm_info(None, None, None)
                        ui_panel_manager.update_overview_summary(None, False) # Ẩn bảng tóm tắt
                        print("Grid Reset.")
                    
                    elif ui_action == "clear_paths":
                        # Xóa visualization (explored, path) trên lưới
                        visualization_active = False; animation_paused = False
                        ui_panel_manager.update_pause_button_text(animation_paused)
                        for r_nodes in game_grid:
                            for node in r_nodes:
                                node.is_explored = False; node.is_path = False; node.path_color = None
                        # Reset agent về vị trí bắt đầu (nếu có)
                        for agent in active_agents.values():
                            if start_node_pos: agent.reset_to_start(start_node_pos)
                        print("Paths/Explored visualization cleared. Agents reset.")

                    elif ui_action == "toggle_pause_animation":
                        # Tạm dừng hoặc tiếp tục animation
                        if visualization_active: # Chỉ có tác dụng nếu animation đang chạy
                            animation_paused = not animation_paused
                            ui_panel_manager.update_pause_button_text(animation_paused) # Cập nhật text nút
                
                elif isinstance(ui_action, dict): # Nếu hành động là dictionary (thường là từ dropdown, slider)
                    action_type = ui_action.get("type") # Loại hành động
                    action_value = ui_action.get("value") # Giá trị của hành động

                    if action_type == "build_mode_changed":
                        # Thay đổi chế độ xây dựng (wall, trap, start, end)
                        current_build_mode = action_value
                        ui_panel_manager.update_build_mode_display(current_build_mode) # Cập nhật UI
                    
                    elif action_type == "algo_view_changed":
                        # Thay đổi thuật toán đang được xem chi tiết
                        detailed_view_algo_name = action_value
                        # Reset animation và trạng thái explored/path trên lưới
                        visualization_active = False; animation_paused = False
                        ui_panel_manager.update_pause_button_text(animation_paused)
                        current_viz_explored_idx = 0; current_viz_path_idx = 0
                        nodes_to_visualize_explored.clear(); nodes_to_visualize_path.clear()

                        for r_nodes in game_grid:
                            for node in r_nodes:
                                node.is_explored = False; node.is_path = False; node.path_color = None
                        
                        if detailed_view_algo_name != "Overview / All Paths": # Nếu xem chi tiết một thuật toán
                            ui_panel_manager.update_overview_summary(None, False) # Ẩn bảng tóm tắt
                            if detailed_view_algo_name in path_results: # Nếu thuật toán này đã có kết quả
                                res = path_results[detailed_view_algo_name]
                                # Cập nhật thông tin và chuẩn bị animation
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
                        # Reset animation
                        visualization_active = False; animation_paused = False
                        ui_panel_manager.update_pause_button_text(animation_paused)
                        if selected_maze_name != "Custom": # Nếu không phải là "Custom" (tự vẽ)
                            # Áp dụng mê cung mẫu vào lưới
                            new_start, new_end = apply_maze_to_grid(game_grid, selected_maze_name)
                            if new_start and new_end: # Nếu mê cung được tải thành công
                                start_node_pos = new_start; end_node_pos = new_end
                                path_results.clear(); active_agents.clear() # Xóa dữ liệu cũ
                                # Reset UI về chế độ overview
                                detailed_view_algo_name = "Overview / All Paths"
                                ui_panel_manager.algo_dropdown.selected_option = "Overview / All Paths"
                                ui_panel_manager.update_selected_algorithm_info(None, None, None)
                                ui_panel_manager.update_overview_summary(None, False)
                                # Reset trạng thái visualized của các ô (apply_maze_to_grid chỉ reset type)
                                for r_nodes in game_grid:
                                    for node in r_nodes: node.is_explored = False; node.is_path = False
                            else: # Nếu tải mê cung lỗi, quay lại "Custom"
                                ui_panel_manager.maze_dropdown.selected_option = "Custom"
                        # else: Chế độ "Custom", người dùng tự xây dựng, không cần hành động cụ thể ở đây

                    elif action_type == "viz_speed_changed":
                        # Thay đổi tốc độ animation từ slider
                        slider_val = action_value # Giá trị từ 1 (chậm) đến 10 (nhanh)
                        # Tính toán lại độ trễ mỗi node dựa trên giá trị slider
                        # Công thức nội suy tuyến tính:
                        # delay = max_delay - (slider_val - min_slider_val) * (max_delay - min_delay) / (max_slider_val - min_slider_val)
                        current_viz_delay_per_node = ANIM_VIZ_MAX_DELAY - \
                                                    (slider_val - 1) * (ANIM_VIZ_MAX_DELAY - ANIM_VIZ_MIN_DELAY) / (10 - 1)


            # --- Xử lý input click chuột trên grid (để xây dựng) ---
            if event.type == pygame.MOUSEBUTTONDOWN: # Nếu có sự kiện nhấn chuột
                if mouse_pos[0] < GRID_WIDTH: # Chỉ xử lý click trong khu vực lưới (không phải trên UI panel)
                    r_clicked, c_clicked = get_clicked_grid_pos(mouse_pos) # Lấy tọa độ ô được click
                    if r_clicked is not None and c_clicked is not None: # Nếu click trúng một ô hợp lệ
                        node = game_grid[r_clicked][c_clicked] # Lấy đối tượng Node tại ô đó
                        
                        if event.button == 1: # Left Click (Chuột trái)
                            if current_build_mode == "set_start":
                                # Đặt điểm bắt đầu
                                if start_node_pos: game_grid[start_node_pos[0]][start_node_pos[1]].reset() # Xóa điểm bắt đầu cũ (nếu có)
                                node.make_start(); start_node_pos = (r_clicked, c_clicked)
                                # Reset tất cả agent về vị trí bắt đầu mới
                                for agent in active_agents.values(): agent.reset_to_start(start_node_pos)
                            elif current_build_mode == "set_end":
                                # Đặt điểm kết thúc
                                if end_node_pos: game_grid[end_node_pos[0]][end_node_pos[1]].reset() # Xóa điểm kết thúc cũ (nếu có)
                                if not node.is_start_type(): # Không cho đặt điểm kết thúc trùng điểm bắt đầu
                                    node.make_end(); end_node_pos = (r_clicked, c_clicked)
                            elif current_build_mode == "set_wall":
                                # Đặt tường (chướng ngại vật)
                                if not node.is_start_type() and not node.is_end_type(): # Không cho đặt tường lên điểm bắt đầu/kết thúc
                                    node.make_obstacle()
                            elif current_build_mode == "set_trap":
                                # Đặt bẫy (ô có chi phí cao)
                                if not node.is_start_type() and not node.is_end_type(): # Không cho đặt bẫy lên điểm bắt đầu/kết thúc
                                    node.make_trap()
                        elif event.button == 3: # Right Click (Chuột phải) - Xóa ô
                            if node.is_start_type(): start_node_pos = None # Nếu xóa điểm bắt đầu, cập nhật biến
                            elif node.is_end_type(): end_node_pos = None # Nếu xóa điểm kết thúc, cập nhật biến
                            node.reset() # Reset ô về trạng thái mặc định (trống)

        # --- CẬP NHẬT TRẠNG THÁI GAME ---
        ui_manager.update(time_delta) # Cập nhật UIManager của pygame_gui
        
        # Cập nhật animation của từng ô trên lưới (nếu có, ví dụ: hiệu ứng trap nhấp nháy)
        for r_nodes in game_grid:
            for node_obj in r_nodes:
                node_obj.update_animation(time_delta)
        
        # Logic cho visualization animation (hiệu ứng duyệt ô, vẽ đường đi)
        if visualization_active and not animation_paused: # Nếu animation đang chạy và không bị tạm dừng
            viz_delay_timer += time_delta # Tăng bộ đếm thời gian
            if viz_delay_timer >= current_viz_delay_per_node: # Nếu đủ thời gian trễ
                viz_delay_timer -= current_viz_delay_per_node # Reset timer (giữ lại phần dư để chính xác hơn)
                
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
                else: # Animation hoàn tất (đã visualize hết explored và path)
                    visualization_active = False; animation_paused = False # Dừng animation
                    ui_panel_manager.update_pause_button_text(animation_paused) # Cập nhật text nút Pause

        # Cập nhật vị trí các agent (nếu có và đang di chuyển)
        for agent_obj in active_agents.values():
            if not agent_obj.finished_path: # Nếu agent chưa đi hết đường
                 agent_obj.update(time_delta) # Cập nhật vị trí agent
        
        # --- VẼ LÊN MÀN HÌNH ---
        screen.fill(LIGHT_BLUE_BG) # Tô màu nền cho toàn bộ màn hình
        if background_surface: # Nếu có ảnh nền
            screen.blit(background_surface, (0, 0)) # Vẽ ảnh nền
        else: # Nếu không có ảnh nền, vẽ một hình chữ nhật màu trắng cho khu vực lưới
            pygame.draw.rect(screen, WHITE, (0, 0, GRID_WIDTH, GRID_HEIGHT))

        # Vẽ các ô trên lưới
        for r_nodes in game_grid:
            for node_obj in r_nodes:
                node_obj.draw(screen)
        
        # Vẽ đường đi của tất cả các thuật toán khi ở chế độ "Overview" và không có animation nào đang chạy
        if detailed_view_algo_name == "Overview / All Paths" and not visualization_active:
            for algo_name, result_data in path_results.items():
                if result_data["path"]: # Nếu thuật toán này tìm được đường đi
                    path_nodes = result_data["path"]; color = result_data["color"]
                    thickness = result_data.get("line_thickness", 2) # Độ dày đường kẻ
                    # Vẽ các đoạn thẳng nối các ô trong đường đi
                    for i in range(len(path_nodes) - 1):
                        r1, c1 = path_nodes[i]; r2, c2 = path_nodes[i + 1]
                        # Tính tọa độ pixel trung tâm của ô
                        x1, y1 = c1 * CELL_SIZE + CELL_SIZE // 2, r1 * CELL_SIZE + CELL_SIZE // 2
                        x2, y2 = c2 * CELL_SIZE + CELL_SIZE // 2, r2 * CELL_SIZE + CELL_SIZE // 2
                        pygame.draw.line(screen, color, (x1, y1), (x2, y2), thickness)
        
        draw_grid_lines(screen) # Vẽ các đường kẻ của lưới
        
        # Vẽ các agent
        for agent_obj in active_agents.values(): 
            agent_obj.draw(screen)
            
        ui_manager.draw_ui(screen) # Vẽ các thành phần UI (panel, nút, dropdown,...)
        
        pygame.display.flip() # Cập nhật toàn bộ nội dung màn hình

    # Kết thúc Pygame khi vòng lặp chính dừng
    pygame.quit()
    sys.exit() # Thoát chương trình

if __name__ == '__main__':
    main()