# src/game_grid.py
import pygame
from config import (CELL_SIZE, GRID_ROWS, GRID_COLS, COST_NORMAL_CELL, COST_TRAP_CELL,
                    RED, GREEN, BLUE, BROWN, WHITE, ORANGE, GREY, COLOR_EXPLORED_NODE)
from src.sprite_manager import get_sprite # Import hàm lấy sprite từ sprite_manager

class GridNode:
    """
    Đại diện cho một ô (node) đơn lẻ trong lưới game.
    Mỗi node lưu trữ thông tin về vị trí, loại, chi phí, và trạng thái hiển thị.
    """
    def __init__(self, row, col):
        """
        Khởi tạo một GridNode.

        Args:
            row (int): Chỉ số hàng của node trong lưới.
            col (int): Chỉ số cột của node trong lưới.
        """
        self.row = row  # Chỉ số hàng (grid coordinate)
        self.col = col  # Chỉ số cột (grid coordinate)
        self.x_pixel = col * CELL_SIZE # Tọa độ x pixel trên màn hình (góc trên bên trái)
        self.y_pixel = row * CELL_SIZE # Tọa độ y pixel trên màn hình (góc trên bên trái)
        self.type = "normal"  # Loại node: "normal", "obstacle", "trap", "start", "end"
        self.cost = COST_NORMAL_CELL # Chi phí để đi qua node này (mặc định là chi phí ô thường)
        
        # Thuộc tính này có thể được sử dụng để đánh dấu đường đi do người chơi vẽ (chưa dùng tới)
        self.is_player_path_node = False

        # Thuộc tính cho hiệu ứng nhấp nháy (pulsating effect) của một số loại node (start, end, trap)
        self.pulsate_alpha = 255  # Giá trị alpha hiện tại (độ trong suốt)
        self.pulsate_direction = -1 # Hướng thay đổi alpha (-1: giảm, 1: tăng)
        self.PULSATE_SPEED = 200  # Tốc độ thay đổi alpha (đơn vị alpha mỗi giây)
        self.MIN_ALPHA = 120      # Giá trị alpha tối thiểu
        self.MAX_ALPHA = 255      # Giá trị alpha tối đa
        
        # Cờ và màu sắc cho visualization của thuật toán AI
        self.is_path = False # True nếu node này là một phần của đường đi tìm được bởi AI
        self.path_color = None # Màu sắc của đường đi (nếu is_path = True)
        self.is_explored = False # True nếu node này đã được thuật toán AI khám phá
        self.explored_color = COLOR_EXPLORED_NODE # Màu mặc định cho node đã khám phá


    def get_map_element_sprite(self):
        """
        Lấy sprite tương ứng với loại node hiện tại.

        Returns:
            pygame.Surface or None: Sprite của node nếu có, ngược lại là None.
        """
        if self.type == "obstacle": return get_sprite("wall")
        if self.type == "trap": return get_sprite("trap")
        if self.type == "start": return get_sprite("start_flag")
        if self.type == "end": return get_sprite("end_flag")
        return None # Node "normal" không có sprite cụ thể (chỉ là nền)

    def update_animation(self, dt):
        """
        Cập nhật trạng thái animation của node (hiệu ứng nhấp nháy).

        Args:
            dt (float): Thời gian delta (thời gian trôi qua từ frame trước, tính bằng giây).
        """
        # Chỉ áp dụng hiệu ứng nhấp nháy cho các loại node "start", "end", và "trap"
        if self.type in ["start", "end", "trap"]:
            # Thay đổi giá trị alpha dựa trên tốc độ và hướng
            self.pulsate_alpha += self.pulsate_direction * self.PULSATE_SPEED * dt
            # Giữ giá trị alpha trong khoảng MIN_ALPHA và MAX_ALPHA
            if self.pulsate_alpha < self.MIN_ALPHA:
                self.pulsate_alpha = self.MIN_ALPHA
                self.pulsate_direction = 1 # Đảo hướng khi đạt min
            elif self.pulsate_alpha > self.MAX_ALPHA:
                self.pulsate_alpha = self.MAX_ALPHA
                self.pulsate_direction = -1 # Đảo hướng khi đạt max
        else:
            # Reset alpha về giá trị tối đa cho các loại node không có animation
            self.pulsate_alpha = self.MAX_ALPHA

    def draw(self, screen):
        """
        Vẽ node lên màn hình.

        Args:
            screen (pygame.Surface): Bề mặt màn hình để vẽ lên.
        """
        rect_to_draw = pygame.Rect(self.x_pixel, self.y_pixel, CELL_SIZE, CELL_SIZE)
        
        # --- Lớp 1: Vẽ trạng thái "explored" (nếu có) ---
        # Chỉ vẽ màu "explored" nếu node là "normal" và đã được khám phá.
        # Điều này tránh vẽ đè lên các sprite quan trọng như start/end/obstacle/trap.
        if self.is_explored and self.type == "normal":
            # Tạo một bề mặt tạm thời với kênh alpha để vẽ màu explored trong suốt
            s = pygame.Surface((CELL_SIZE, CELL_SIZE), pygame.SRCALPHA)
            s.fill(self.explored_color) # Tô màu explored (đã định nghĩa với alpha)
            screen.blit(s, rect_to_draw.topleft)

        # --- Lớp 2: Vẽ sprite của thành phần bản đồ hoặc màu fallback ---
        element_sprite = self.get_map_element_sprite() # Lấy sprite tương ứng
        if element_sprite:
            # Tạo bản sao của sprite để có thể thay đổi alpha mà không ảnh hưởng sprite gốc
            temp_sprite = element_sprite.copy()
            # Áp dụng hiệu ứng nhấp nháy (thay đổi alpha) cho start, end, trap
            if self.type in ["start", "end", "trap"]:
                temp_sprite.set_alpha(int(self.pulsate_alpha))
            screen.blit(temp_sprite, (self.x_pixel, self.y_pixel))
        elif self.type != "normal": # Nếu không có sprite VÀ node không phải là "normal"
            # Sử dụng màu fallback nếu sprite bị thiếu
            color_map = {"obstacle": RED, "trap": BROWN, "start": GREEN, "end": BLUE}
            fallback_color = color_map.get(self.type, WHITE) # Mặc định là WHITE nếu type lạ
            
            if self.type in ["start", "end", "trap"]: # Vẽ màu fallback với hiệu ứng alpha
                s_fallback = pygame.Surface((CELL_SIZE, CELL_SIZE), pygame.SRCALPHA)
                r, g, b = fallback_color
                s_fallback.fill((r, g, b, int(self.pulsate_alpha))) # Áp dụng alpha vào màu
                screen.blit(s_fallback, rect_to_draw.topleft)
            else: # Vẽ màu fallback không có alpha (ví dụ: obstacle không nhấp nháy)
                pygame.draw.rect(screen, fallback_color, rect_to_draw)
        
        # --- Lớp 3: Vẽ đường đi của AI (nếu có) ---
        # Vẽ đè lên trên sprite/màu nền và explored.
        if self.is_path and self.path_color:
            s_path = pygame.Surface((CELL_SIZE, CELL_SIZE), pygame.SRCALPHA)
            # Tô màu đường đi với độ trong suốt (alpha = 150) để có thể nhìn thấy lớp dưới.
            s_path.fill((*self.path_color, 150))
            screen.blit(s_path, rect_to_draw.topleft)

        # --- Lớp 4: Vẽ đường đi của người chơi (nếu có) --- (Hiện chưa được sử dụng tích cực)
        # Vẽ đè lên trên các lớp khác.
        if self.is_player_path_node:
            s_player_path = pygame.Surface((CELL_SIZE, CELL_SIZE), pygame.SRCALPHA)
            # Sử dụng màu ORANGE với độ trong suốt thấp (alpha = 100)
            s_player_path.fill((ORANGE[0], ORANGE[1], ORANGE[2], 100))
            screen.blit(s_player_path, (self.x_pixel, self.y_pixel))

    # --- Các phương thức thay đổi trạng thái của Node ---
    def make_obstacle(self):
        """Chuyển node thành chướng ngại vật (wall)."""
        self.type = "obstacle"
        self.cost = float("inf") # Chi phí vô cực, không thể đi qua
        self.pulsate_alpha = self.MAX_ALPHA # Obstacle không nhấp nháy
        self._reset_path_flags() # Xóa trạng thái path/explored cũ

    def make_start(self):
        """Chuyển node thành điểm bắt đầu."""
        self.type = "start"
        self.cost = COST_NORMAL_CELL # Điểm bắt đầu có chi phí như ô thường
        self.pulsate_alpha = self.MAX_ALPHA # Reset alpha để bắt đầu hiệu ứng
        self._reset_path_flags()

    def make_end(self):
        """Chuyển node thành điểm kết thúc."""
        self.type = "end"
        self.cost = COST_NORMAL_CELL # Điểm kết thúc có chi phí như ô thường
        self.pulsate_alpha = self.MAX_ALPHA # Reset alpha
        self._reset_path_flags()

    def make_trap(self):
        """Chuyển node thành bẫy."""
        self.type = "trap"
        self.cost = COST_TRAP_CELL # Bẫy có chi phí cao hơn
        self.pulsate_alpha = self.MAX_ALPHA # Reset alpha
        self._reset_path_flags()

    def reset(self):
        """Reset node về trạng thái bình thường (ô trống)."""
        self.type = "normal"
        self.cost = COST_NORMAL_CELL
        self.is_player_path_node = False
        self.pulsate_alpha = self.MAX_ALPHA # Ô thường không nhấp nháy
        self._reset_path_flags() # Xóa trạng thái path/explored
        
    def _reset_path_flags(self):
        """Hàm nội bộ để reset các cờ liên quan đến path và explored."""
        self.is_path = False
        self.path_color = None
        self.is_explored = False

    # --- Các phương thức kiểm tra loại Node ---
    def is_obstacle_type(self):
        """Kiểm tra xem node có phải là chướng ngại vật không."""
        return self.type == "obstacle"

    def is_start_type(self):
        """Kiểm tra xem node có phải là điểm bắt đầu không."""
        return self.type == "start"

    def is_end_type(self):
        """Kiểm tra xem node có phải là điểm kết thúc không."""
        return self.type == "end"

# --- Các hàm tiện ích liên quan đến Grid ---
def create_grid():
    """
    Tạo một lưới game 2D mới, bao gồm các đối tượng GridNode.

    Returns:
        list of list of GridNode: Lưới game dưới dạng một list 2 chiều.
    """
    grid = []
    for r in range(GRID_ROWS): # Duyệt qua từng hàng
        row_nodes = []
        for c in range(GRID_COLS): # Duyệt qua từng cột trong hàng đó
            row_nodes.append(GridNode(r, c)) # Tạo một GridNode và thêm vào hàng hiện tại
        grid.append(row_nodes) # Thêm hàng đã hoàn thành vào lưới
    return grid

def draw_grid_lines(screen):
    """
    Vẽ các đường kẻ cho lưới game lên màn hình.

    Args:
        screen (pygame.Surface): Bề mặt màn hình để vẽ lên.
    """
    # Vẽ các đường kẻ ngang
    for r in range(GRID_ROWS + 1): # Cần GRID_ROWS + 1 đường kẻ ngang
        pygame.draw.line(screen, GREY, (0, r * CELL_SIZE), (GRID_COLS * CELL_SIZE, r * CELL_SIZE))
    # Vẽ các đường kẻ dọc
    for c in range(GRID_COLS + 1): # Cần GRID_COLS + 1 đường kẻ dọc
        pygame.draw.line(screen, GREY, (c * CELL_SIZE, 0), (c * CELL_SIZE, GRID_ROWS * CELL_SIZE))

def get_clicked_grid_pos(mouse_pos_tuple):
    """
    Chuyển đổi tọa độ pixel của chuột trên màn hình sang tọa độ (hàng, cột) của ô trong lưới.

    Args:
        mouse_pos_tuple (tuple): Tuple (x, y) chứa tọa độ pixel của chuột.

    Returns:
        tuple or (None, None): Tuple (row, col) nếu click nằm trong lưới hợp lệ,
                                ngược lại trả về (None, None).
    """
    mouse_x, mouse_y = mouse_pos_tuple
    # Kiểm tra xem click có nằm trong khu vực lưới game không
    if 0 <= mouse_x < GRID_COLS * CELL_SIZE and 0 <= mouse_y < GRID_ROWS * CELL_SIZE:
        # Tính toán chỉ số hàng và cột dựa trên CELL_SIZE
        row = mouse_y // CELL_SIZE
        col = mouse_x // CELL_SIZE
        # Đảm bảo hàng và cột tính được vẫn nằm trong giới hạn của lưới
        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            return row, col
    return None, None # Trả về None nếu click ra ngoài lưới hoặc không hợp lệ