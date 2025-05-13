# src/ui_panel.py
import pygame
import pygame_gui
# Import các hằng số từ config nếu cần (ví dụ: UI_PANEL_WIDTH, TOTAL_SCREEN_HEIGHT, GRID_WIDTH)
from config import UI_PANEL_WIDTH, TOTAL_SCREEN_HEIGHT, GRID_WIDTH # Đảm bảo import đúng

class UIPanelManager:
    """
    Quản lý việc khởi tạo, cập nhật và xử lý sự kiện cho bảng điều khiển UI.
    Bảng điều khiển này chứa các nút, dropdown, và nhãn thông tin.
    """
    def __init__(self, ui_manager, algorithm_names, maze_names):
        """
        Khởi tạo UIPanelManager.

        :param ui_manager: Trình quản lý pygame_gui.
        :param algorithm_names: Danh sách tên các thuật toán.
        :param maze_names: Danh sách tên các mê cung mẫu.
        """
        self.manager = ui_manager
        self.algorithm_names = algorithm_names
        self.maze_names = maze_names

        self.control_panel = None # Panel chính chứa tất cả các UI element khác

        # UI Elements - Khai báo trước các thành phần UI sẽ được tạo
        self.instructions_box = None
        self.run_button = None
        self.reset_button = None
        self.clear_paths_button = None

        self.build_mode_label_title = None
        self.wall_mode_button = None
        self.trap_mode_button = None
        self.start_mode_button = None
        self.end_mode_button = None
        self.current_build_mode_display_label = None

        self.algo_select_label_title = None
        self.algo_dropdown = None
        self.maze_select_label_title = None
        self.maze_dropdown = None

        self.selected_algo_info_title_label = None
        self.selected_algo_name_label = None
        self.selected_algo_cost_label = None
        self.selected_algo_explored_label = None
        self.selected_algo_time_label = None

        self.animation_controls_title_label = None
        self.pause_resume_button = None
        self.viz_speed_label = None
        self.viz_speed_slider = None

        self.hover_info_title_label = None
        self.hover_coord_label = None
        self.hover_type_label = None
        self.hover_cost_label = None
        
        self.overview_summary_box = None

        # Biến theo dõi vị trí Y hiện tại để sắp xếp các UI element theo chiều dọc
        self.current_y_offset = 10 # Bắt đầu với padding 10px từ trên xuống

        # Gọi các hàm để khởi tạo từng phần của UI
        self._setup_panel()
        self._setup_instructions_box()
        self._setup_control_buttons()
        self._setup_build_mode_controls()
        self._setup_dropdowns()
        self._setup_selected_algorithm_info_labels()
        self._setup_animation_controls()
        self._setup_hover_info_labels()
        self._setup_overview_summary_box() # Gọi cuối cùng để tính remaining_height một cách chính xác
        
        # Thiết lập trạng thái ban đầu cho một số UI
        self.update_build_mode_display("set_wall")
        self.update_pause_button_text(False) # Ban đầu animation không tạm dừng
        self.update_overview_summary(None, False) # Ẩn hộp tóm tắt tổng quan ban đầu

    def _add_spacing(self, amount=10):
        """Thêm khoảng cách dọc và cập nhật self.current_y_offset."""
        self.current_y_offset += amount

    def _setup_panel(self):
        """Khởi tạo UIPanel chính làm container cho các UI element khác."""
        panel_rect = pygame.Rect(GRID_WIDTH, 0, UI_PANEL_WIDTH, TOTAL_SCREEN_HEIGHT)
        self.control_panel = pygame_gui.elements.UIPanel(
            relative_rect=panel_rect,
            manager=self.manager,
            object_id="#control_panel" # ID để styling qua theme
        )

    def _setup_instructions_box(self):
        """Khởi tạo hộp văn bản hiển thị hướng dẫn điều khiển."""
        instructions_html = (
            "<font face=verdana size=3>" # size=3 là hợp lệ cho HTML cơ bản
            "<b>Controls:</b><br>"
            "  <b>LMB:</b> Place Item<br>" #   để tạo khoảng trắng
            "  <b>RMB:</b> Erase Cell<br>"
            "Use UI buttons for build mode."
            "</font>"
        )
        box_height = 75 # Chiều cao của hộp hướng dẫn
        self.instructions_box = pygame_gui.elements.UITextBox(
            html_text=instructions_html,
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, box_height),
            manager=self.manager, container=self.control_panel, object_id="#instructions_box"
        )
        self.current_y_offset += box_height
        self._add_spacing() # Thêm khoảng cách sau hộp hướng dẫn

    def _setup_control_buttons(self):
        """Khởi tạo các nút điều khiển chính: Run, Reset, Clear Visuals."""
        button_width_full = UI_PANEL_WIDTH - 20 # Chiều rộng nút chiếm gần hết panel
        button_height_main = 40 # Chiều cao cho nút chính (Run)
        
        # Nút Run Algorithms
        self.run_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, self.current_y_offset, button_width_full, button_height_main),
            text='Run Algorithms', manager=self.manager, container=self.control_panel, object_id="#run_button"
        )
        self.current_y_offset += button_height_main
        self._add_spacing(5) # Khoảng cách nhỏ sau nút Run

        # Nút Reset Grid và Clear Visuals (nằm trên cùng một hàng)
        button_width_half = (UI_PANEL_WIDTH - 30) // 2 # -20 for panel padding, -10 for space between buttons
        button_height_secondary = 35 # Chiều cao cho các nút phụ
        
        self.reset_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, self.current_y_offset, button_width_half, button_height_secondary),
            text='Reset Grid', manager=self.manager, container=self.control_panel, object_id="#reset_button"
        )
        self.clear_paths_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10 + button_width_half + 10, self.current_y_offset, button_width_half, button_height_secondary),
            text='Clear Visuals', manager=self.manager, container=self.control_panel, object_id="#clear_paths_button"
        )
        self.current_y_offset += button_height_secondary
        self._add_spacing() # Khoảng cách sau hàng nút Reset/Clear

    def _setup_build_mode_controls(self):
        """Khởi tạo các nút điều khiển chế độ xây dựng (Wall, Trap, Start, End) và nhãn hiển thị."""
        title_height = 20 # Chiều cao cho nhãn tiêu đề section
        
        # Tiêu đề section "Build Mode"
        self.build_mode_label_title = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
            text="Build Mode:", manager=self.manager, container=self.control_panel, object_id="#section_title"
        )
        self.current_y_offset += title_height
        self._add_spacing(5)

        # Các nút chọn chế độ xây dựng (Wall, Trap, Start, End)
        button_width_quarter = (UI_PANEL_WIDTH - 20 - 3 * 5) // 4 # Chiều rộng cho mỗi nút (4 nút, 3 khoảng cách 5px)
        button_height_small = 30 # Chiều cao cho các nút này
        current_x = 10 # Vị trí X bắt đầu cho nút đầu tiên
        
        modes = [("Wall", "#wall_button", "set_wall"), ("Trap", "#trap_button", "set_trap"),
                 ("Start", "#start_button", "set_start"), ("End", "#end_button", "set_end")]
        
        button_elements = [] # List để lưu các button vừa tạo
        for text, obj_id, mode_val in modes:
            btn = pygame_gui.elements.UIButton(
                relative_rect=pygame.Rect(current_x, self.current_y_offset, button_width_quarter, button_height_small),
                text=text, manager=self.manager, container=self.control_panel, object_id=obj_id
            )
            button_elements.append(btn)
            current_x += button_width_quarter + 5 # Cập nhật vị trí X cho nút tiếp theo
        
        # Gán các nút đã tạo vào các thuộc tính tương ứng của class
        self.wall_mode_button, self.trap_mode_button, self.start_mode_button, self.end_mode_button = button_elements
        self.current_y_offset += button_height_small
        self._add_spacing(5)

        # Nhãn hiển thị chế độ xây dựng hiện tại
        self.current_build_mode_display_label = pygame_gui.elements.UILabel(
             relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
             text="Current Mode: Wall", manager=self.manager, container=self.control_panel, object_id="#build_mode_display"
        )
        self.current_y_offset += title_height
        self._add_spacing()

    def _setup_dropdowns(self):
        """Khởi tạo các menu thả xuống (dropdown) để chọn thuật toán và mê cung mẫu."""
        title_height = 20
        dropdown_height = 35

        # Tiêu đề và dropdown chọn thuật toán để xem chi tiết
        self.algo_select_label_title = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
            text="View Algorithm Details:", manager=self.manager, container=self.control_panel, object_id="#section_title"
        )
        self.current_y_offset += title_height
        self._add_spacing(5)
        
        dropdown_options_algo = ["Overview / All Paths"] + self.algorithm_names # Thêm lựa chọn "Overview"
        self.algo_dropdown = pygame_gui.elements.UIDropDownMenu(
            options_list=dropdown_options_algo, starting_option=dropdown_options_algo[0],
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, dropdown_height),
            manager=self.manager, container=self.control_panel
        )
        self.current_y_offset += dropdown_height
        self._add_spacing()

        # Tiêu đề và dropdown chọn mê cung mẫu
        self.maze_select_label_title = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
            text="Load Preset Maze:", manager=self.manager, container=self.control_panel, object_id="#section_title"
        )
        self.current_y_offset += title_height
        self._add_spacing(5)
        
        dropdown_options_maze = ["Custom"] + self.maze_names # Thêm lựa chọn "Custom" (người dùng tự vẽ)
        self.maze_dropdown = pygame_gui.elements.UIDropDownMenu(
            options_list=dropdown_options_maze, starting_option=dropdown_options_maze[0],
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, dropdown_height),
            manager=self.manager, container=self.control_panel
        )
        self.current_y_offset += dropdown_height
        self._add_spacing()

    def _setup_selected_algorithm_info_labels(self):
        """Khởi tạo các nhãn hiển thị thông tin chi tiết của thuật toán được chọn."""
        title_height = 20
        info_label_height = 18 # Chiều cao cho mỗi nhãn thông tin
        info_label_spacing = 2 # Khoảng cách giữa các nhãn thông tin

        # Tiêu đề section "Selected Algorithm Info"
        self.selected_algo_info_title_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
            text="Selected Algorithm Info:", manager=self.manager, container=self.control_panel, object_id="#section_title"
        )
        self.current_y_offset += title_height
        self._add_spacing(5)

        # Các nhãn hiển thị thông tin (Tên, Cost, Nodes Explored, Time)
        # Các comment sau giải thích việc chọn label_content_width và label_x_offset
        # Cần đảm bảo UI_PANEL_WIDTH - 30 đủ rộng cho cả text và thẻ HTML
        # Nếu UI_PANEL_WIDTH = 280, thì UI_PANEL_WIDTH - 30 = 250.
        # Chuỗi "Algorithm: <font color='#A0D0FF'>Greedy BFS</font>" có thể dài hơn 250px tùy font.
        # Thử tăng chiều rộng một chút, hoặc giảm padding (từ 15 thành 10 mỗi bên)
        label_content_width = UI_PANEL_WIDTH - 20 # Giảm padding, tăng không gian cho text
        label_x_offset = 10 # Padding trái

        labels_data = [
            ("selected_algo_name_label", "Algorithm: -"),
            ("selected_algo_cost_label", "Cost: -"),
            ("selected_algo_explored_label", "Nodes Explored: -"),
            ("selected_algo_time_label", "Time (ms): -")
        ]
        for attr_name, default_text in labels_data:
            label = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(label_x_offset, self.current_y_offset, label_content_width, info_label_height),
                text=default_text, manager=self.manager, container=self.control_panel, object_id="#info_label"
            )
            setattr(self, attr_name, label) # Gán nhãn vào thuộc tính của class
            self.current_y_offset += info_label_height + info_label_spacing
        
        self.current_y_offset -= info_label_spacing # Loại bỏ khoảng cách thừa sau nhãn cuối cùng
        self._add_spacing() # Thêm khoảng cách chuẩn sau group này


    def _setup_animation_controls(self):
        """Khởi tạo các điều khiển cho animation (Pause/Resume, Speed Slider)."""
        title_height = 20
        button_height_small = 30
        # slider_area_height = 35 # Chứa cả label và slider - không dùng trực tiếp nhưng để tham khảo
        slider_height = 20 # Chiều cao của thanh slider

        # Tiêu đề section "Animation Controls"
        self.animation_controls_title_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
            text="Animation Controls:", manager=self.manager, container=self.control_panel, object_id="#section_title"
        )
        self.current_y_offset += title_height
        self._add_spacing(5)

        # Nút Pause/Resume và Slider tốc độ (nằm trên cùng một hàng)
        button_pause_width = (UI_PANEL_WIDTH - 30) // 2 # Nút pause chiếm nửa chiều rộng có sẵn
        self.pause_resume_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, self.current_y_offset, button_pause_width, button_height_small),
            text="Pause Anim", manager=self.manager, container=self.control_panel, object_id="#pause_button"
        )
        
        # Khu vực cho nhãn và slider tốc độ
        slider_x_start = 10 + button_pause_width + 10 # Vị trí X bắt đầu của slider group
        slider_width_available = UI_PANEL_WIDTH - 20 - button_pause_width - 10 # Phần còn lại cho slider group

        self.viz_speed_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(slider_x_start, self.current_y_offset, slider_width_available, 15), # Nhãn nằm trên slider
            text="Speed (1-S, 10-F)", manager=self.manager, container=self.control_panel, object_id="#small_label"
        )
        self.viz_speed_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pygame.Rect(slider_x_start, self.current_y_offset + 15, slider_width_available, slider_height), # Slider nằm dưới nhãn
            start_value=5, value_range=(1, 10), click_increment=1,
            manager=self.manager, container=self.control_panel
        )
        # Chiều cao của hàng này được quyết định bởi chiều cao của nút Pause/Resume
        self.current_y_offset += button_height_small 
        self._add_spacing()

    def _setup_hover_info_labels(self):
        """Khởi tạo các nhãn hiển thị thông tin của ô đang được trỏ chuột vào."""
        title_height = 20
        info_label_height = 18
        info_label_spacing = 0 # Không cần spacing giữa các info label này, chúng nối tiếp nhau

        # Tiêu đề section "Hovered Cell Info"
        self.hover_info_title_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
            text="Hovered Cell Info:", manager=self.manager, container=self.control_panel, object_id="#section_title"
        )
        self.current_y_offset += title_height
        self._add_spacing(5)

        # Các nhãn hiển thị thông tin (Coord, Type, Cost)
        labels_data_hover = [
            ("hover_coord_label", "Coord: -"),
            ("hover_type_label", "Type: -"),
            ("hover_cost_label", "Cost: -")
        ]
        for attr_name, default_text in labels_data_hover:
            label = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(15, self.current_y_offset, UI_PANEL_WIDTH - 30, info_label_height), # Padding trái 15px
                text=default_text, manager=self.manager, container=self.control_panel, object_id="#info_label"
            )
            setattr(self, attr_name, label)
            self.current_y_offset += info_label_height + info_label_spacing # info_label_spacing = 0
        
        self._add_spacing() # Thêm spacing sau group này
        
    def _setup_overview_summary_box(self):
        """Khởi tạo hộp văn bản hiển thị tóm tắt kết quả của tất cả các thuật toán."""
        title_height = 20

        # Tiêu đề "Algorithms Overview" - không cần self vì chỉ dùng cục bộ
        overview_title_label = pygame_gui.elements.UILabel( 
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, title_height),
            text="Algorithms Overview:", manager=self.manager, container=self.control_panel, object_id="#section_title"
        )
        self.current_y_offset += title_height
        self._add_spacing(5)

        # Tính toán chiều cao còn lại cho overview box để nó lấp đầy phần còn lại của panel
        remaining_height = TOTAL_SCREEN_HEIGHT - self.current_y_offset - 10 # 10 là padding dưới cùng mong muốn
        box_height = max(50, remaining_height) # Đảm bảo chiều cao tối thiểu là 50px

        self.overview_summary_box = pygame_gui.elements.UITextBox(
            html_text="<font face=verdana size=3>(Run algorithms for summary)</font>", # size=3 cho font chữ
            relative_rect=pygame.Rect(10, self.current_y_offset, UI_PANEL_WIDTH - 20, box_height),
            manager=self.manager, container=self.control_panel, object_id="#overview_box"
        )
        self.overview_summary_box.visible = False # Ban đầu ẩn đi
        # self.current_y_offset không cần cập nhật thêm nếu đây là element cuối cùng theo chiều dọc

    # --- Update Methods ---
    def update_build_mode_display(self, mode_name):
        """
        Cập nhật nhãn hiển thị chế độ xây dựng hiện tại (Wall, Trap, Start, End).
        :param mode_name: Chuỗi định danh chế độ (ví dụ: "set_wall").
        """
        if self.current_build_mode_display_label:
            display_text = mode_name.replace("set_", "").capitalize() # Chuyển "set_wall" thành "Wall"
            self.current_build_mode_display_label.set_text(f"Current Mode: {display_text}")

    def update_selected_algorithm_info(self, algo_name, cost, explored_count, time_ms=None):
        """
        Cập nhật các nhãn thông tin chi tiết cho thuật toán được chọn từ dropdown.
        :param algo_name: Tên thuật toán.
        :param cost: Chi phí đường đi.
        :param explored_count: Số nút đã duyệt.
        :param time_ms: Thời gian thực thi (ms).
        """
        if not (self.selected_algo_name_label and self.selected_algo_cost_label and 
                self.selected_algo_explored_label and self.selected_algo_time_label):
            return # Tránh lỗi nếu elements chưa được tạo

        if algo_name and algo_name != "Overview / All Paths": # Nếu có thuật toán cụ thể được chọn
            import html
            escaped_algo_name = html.escape(algo_name) # Escape tên thuật toán để tránh lỗi HTML injection
            self.selected_algo_name_label.set_text(f"Algorithm: {escaped_algo_name}")
            
            # Định dạng chi phí
            cost_str = f"{cost:.1f}" if isinstance(cost, (int, float)) and cost != float('inf') else str(cost)
            if cost == float('inf'): cost_str = "<font color='#FFB0B0'>N/A</font>" # Màu đỏ nhạt cho giá trị không hợp lệ/vô cực
            self.selected_algo_cost_label.set_text(f"Cost: {cost_str}")
            
            # Định dạng số nút đã duyệt
            explored_str = str(explored_count) if isinstance(explored_count, int) else str(explored_count) # Xử lý trường hợp "N/A"
            self.selected_algo_explored_label.set_text(f"Explored Nodes: {explored_str}")
            
            # Định dạng thời gian thực thi
            if time_ms is not None:
                if isinstance(time_ms, (int, float)):
                    self.selected_algo_time_label.set_text(f"Time: {time_ms:.1f} ms")
                else: # Xử lý trường hợp "N/A" hoặc "Error"
                    self.selected_algo_time_label.set_text(f"Time: {time_ms}")
            else: # Nếu không có thông tin thời gian
                 self.selected_algo_time_label.set_text("Time: -")
        else: # Nếu chọn "Overview" hoặc không có thuật toán nào, reset các nhãn
            self.selected_algo_name_label.set_text("Algorithm: -")
            self.selected_algo_cost_label.set_text("Cost: -")
            self.selected_algo_explored_label.set_text("Nodes Explored: -")
            self.selected_algo_time_label.set_text("Time (ms): -")

    def update_hover_info(self, node_data):
        """
        Cập nhật thông tin hiển thị khi trỏ chuột qua một ô trên lưới.
        :param node_data: Dictionary chứa thông tin của ô (row, col, type, cost).
        """
        if not (self.hover_coord_label and self.hover_type_label and self.hover_cost_label):
            return # Tránh lỗi nếu elements chưa được tạo

        if node_data: # Nếu có dữ liệu ô
            self.hover_coord_label.set_text(f"Coord: ({node_data['row']}, {node_data['col']})")
            self.hover_type_label.set_text(f"Type: {node_data['type'].capitalize()}")
            cost_text = f"{node_data['cost']}" if node_data['cost'] != float('inf') else "Infinite"
            self.hover_cost_label.set_text(f"Cost: {cost_text}")
        else: # Nếu không trỏ vào ô nào, reset thông tin
            self.hover_coord_label.set_text("Coord: -")
            self.hover_type_label.set_text("Type: -")
            self.hover_cost_label.set_text("Cost: -")

    def update_pause_button_text(self, is_paused):
        """
        Cập nhật văn bản trên nút Pause/Resume animation.
        :param is_paused: True nếu animation đang tạm dừng, False nếu đang chạy.
        """
        if self.pause_resume_button:
            self.pause_resume_button.set_text("Resume Anim" if is_paused else "Pause Anim")

    def update_overview_summary(self, path_results_dict, is_overview_mode):
        """
        Cập nhật hộp văn bản tóm tắt kết quả của tất cả các thuật toán.
        Hiển thị hoặc ẩn hộp này tùy theo chế độ xem.
        :param path_results_dict: Dictionary chứa kết quả của các thuật toán (cost, explored, time_ms).
        :param is_overview_mode: True nếu đang ở chế độ xem "Overview".
        """
        if not self.overview_summary_box:
            return # Tránh lỗi nếu element chưa được tạo

        if is_overview_mode and path_results_dict and len(path_results_dict) > 0:
            html_content = "<font face=verdana size=3><b>Algorithms Overview:</b><br>" # size=3 cho font chữ
            html_content += "<table width='100%' border=0 cellpadding=1 cellspacing=0>" # cellspacing=0 để không có khoảng cách giữa các cell
            # Header của bảng
            html_content += ("<tr><td width='35%'><b>Algorithm</b></td>"
                             "<td width='20%' align=right><b>Cost</b></td>"
                             "<td width='20%' align=right><b>Expl.</b></td>"
                             "<td width='25%' align=right><b>Time(ms)</b></td></tr>")
            
            # Sắp xếp kết quả theo chi phí (tăng dần), sau đó theo thời gian (tăng dần)
            sorted_results = sorted(
                path_results_dict.items(), 
                key=lambda item: (
                    (item[1]['cost'] if isinstance(item[1]['cost'], (int, float)) and item[1]['cost'] != float('inf') else float('inf')), # Ưu tiên sắp xếp theo cost
                    item[1]['time_ms'] if isinstance(item[1]['time_ms'], (int, float)) else float('inf') # Xử lý time_ms có thể là str "N/A"
                )
            )

            # Thêm từng hàng dữ liệu vào bảng
            for algo_name, data in sorted_results:
                cost_val = data.get('cost', float('inf'))
                cost_str = f"{cost_val:.1f}" if isinstance(cost_val, (int, float)) and cost_val != float('inf') else "N/A"
                
                explored_val = data.get('explored', []) # Mặc định là list rỗng nếu không có
                explored_str = str(len(explored_val)) if isinstance(explored_val, list) else "N/A"
                
                time_val = data.get('time_ms', float('inf'))
                time_str = f"{time_val:.1f}" if isinstance(time_val, (int, float)) else "N/A"
                
                html_content += f"<tr><td>{algo_name}</td><td align=right>{cost_str}</td><td align=right>{explored_str}</td><td align=right>{time_str}</td></tr>"
            
            html_content += "</table></font>"
            self.overview_summary_box.set_text(html_content)
            self.overview_summary_box.visible = True # Hiển thị hộp tóm tắt
        else:
            # Nếu không ở chế độ overview hoặc không có kết quả, hiển thị thông báo mặc định và ẩn hộp
            self.overview_summary_box.set_text("<font face=verdana size=3>(Run algorithms for summary)</font>") # size=3
            self.overview_summary_box.visible = False


    # --- Event Processing ---
    def process_ui_event(self, event):
        """
        Xử lý các sự kiện từ pygame_gui liên quan đến các element trong panel này.
        Trả về một chuỗi hoặc dictionary mô tả hành động cần thực hiện, hoặc None nếu không có hành động nào.
        :param event: Sự kiện pygame.
        :return: Mô tả hành động hoặc None.
        """
        # Xử lý sự kiện nhấn nút
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == self.run_button: return "run_algorithms"
            if event.ui_element == self.reset_button: return "reset_grid"
            if event.ui_element == self.clear_paths_button: return "clear_paths"
            # Các nút thay đổi chế độ xây dựng trả về dictionary để phân biệt
            if event.ui_element == self.wall_mode_button: return {"type": "build_mode_changed", "value": "set_wall"}
            if event.ui_element == self.trap_mode_button: return {"type": "build_mode_changed", "value": "set_trap"}
            if event.ui_element == self.start_mode_button: return {"type": "build_mode_changed", "value": "set_start"}
            if event.ui_element == self.end_mode_button: return {"type": "build_mode_changed", "value": "set_end"}
            if event.ui_element == self.pause_resume_button: return "toggle_pause_animation"
        
        # Xử lý sự kiện thay đổi lựa chọn trong dropdown menu
        if event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
            if event.ui_element == self.algo_dropdown:
                return {"type": "algo_view_changed", "value": event.text} # event.text chứa lựa chọn mới
            if event.ui_element == self.maze_dropdown:
                return {"type": "maze_selected", "value": event.text}
        
        # Xử lý sự kiện di chuyển thanh trượt (slider)
        if event.type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
            if event.ui_element == self.viz_speed_slider:
                return {"type": "viz_speed_changed", "value": event.value} # event.value chứa giá trị mới của slider
        
        return None # Không có sự kiện UI nào được xử lý bởi panel này