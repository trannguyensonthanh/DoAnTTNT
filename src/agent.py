# src/agent.py
import pygame
import math  # Dùng cho các phép tính lượng giác (góc, khoảng cách)
import random # Dùng cho hiệu ứng hạt bụi ngẫu nhiên và màu fallback
from config import CELL_SIZE # Kích thước ô để tính toán tọa độ
from src.sprite_manager import get_sprite # Hàm để lấy sprite đã được tải

class Agent:
    """
    Đại diện cho một thực thể (ví dụ: xe) có thể di chuyển trên lưới theo một đường đi.
    Agent có sprite riêng, có thể xoay theo hướng di chuyển và tạo hiệu ứng bụi.
    """
    def __init__(self, start_node_rc, sprite_key, name="Agent", speed=2.5):
        """
        Khởi tạo một Agent.

        Args:
            start_node_rc (tuple): Tọa độ (row, col) ban đầu của agent.
            sprite_key (str): Key để lấy sprite của agent từ sprite_manager.
            name (str, optional): Tên của agent (để hiển thị). Mặc định là "Agent".
            speed (float, optional): Tốc độ di chuyển của agent, tính bằng số ô mỗi giây.
                                     Mặc định là 2.5.
        """
        self.row, self.col = start_node_rc # Vị trí (hàng, cột) hiện tại trên lưới
        # Tính tọa độ pixel trung tâm của agent dựa trên vị trí lưới
        self.x_center = self.col * CELL_SIZE + CELL_SIZE // 2
        self.y_center = self.row * CELL_SIZE + CELL_SIZE // 2
        
        self.name = name # Tên của agent
        self.path_nodes = [] # Danh sách các tuple (row, col) tạo thành đường đi
        self.current_path_index = 0 # Chỉ số của node tiếp theo trên đường đi cần đến
        self.speed = speed # Tốc độ di chuyển (ô/giây)
        self.finished_path = True # Cờ cho biết agent đã hoàn thành đường đi chưa (ban đầu là True)
        self.angle = 0 # Góc xoay của agent (độ), 0 độ là hướng sang phải

        # Tải sprite gốc cho agent
        self.original_image = get_sprite(sprite_key)
        if not self.original_image: # Nếu không tải được sprite (ví dụ: file không tồn tại)
            print(f"Warning: Sprite '{sprite_key}' for agent '{name}' not found. Using fallback color.")
            self.image_to_draw = None # Sẽ vẽ màu fallback thay vì sprite
            # Tạo một màu ngẫu nhiên để phân biệt các agent không có sprite
            self.fallback_color = (random.randint(50,150), random.randint(50,150), random.randint(50,150))
        else:
            # self.image_to_draw là sprite sẽ được vẽ lên màn hình (đã xoay)
            self.image_to_draw = self.original_image.copy() # Tạo bản sao để không ảnh hưởng sprite gốc

        # --- Thuộc tính cho hiệu ứng hạt bụi (Dust Particle Effect) ---
        # Hiệu ứng này là tùy chọn, có thể đơn giản hóa hoặc loại bỏ nếu cần.
        self.dust_particles = [] # Danh sách các hạt bụi hiện có
        self.dust_emit_timer = 0.0 # Bộ đếm thời gian để tạo hạt bụi mới
        self.dust_emit_interval = 0.08  # Khoảng thời gian (giây) giữa mỗi lần tạo bụi
        self.max_dust_particles = 25 # Số lượng hạt bụi tối đa
        self.is_moving_for_dust = False # Cờ cho biết agent có đang di chuyển để tạo bụi không

    def set_path(self, new_path_nodes):
        """
        Thiết lập một đường đi mới cho agent.

        Args:
            new_path_nodes (list of tuples or None): Danh sách các tọa độ (row, col) của đường đi mới.
                                                     Nếu là None hoặc list rỗng, agent sẽ không có đường đi.
        """
        self.path_nodes = new_path_nodes if new_path_nodes else [] # Gán đường đi mới
        self.current_path_index = 0 # Reset chỉ số về node đầu tiên
        self.finished_path = not bool(self.path_nodes) # True nếu path rỗng, False nếu có path
        self.dust_particles.clear() # Xóa các hạt bụi cũ

        if self.path_nodes: # Nếu có đường đi mới
            # Đặt agent về vị trí node đầu tiên của đường đi
            self.row, self.col = self.path_nodes[0]
            self.x_center = self.col * CELL_SIZE + CELL_SIZE // 2
            self.y_center = self.row * CELL_SIZE + CELL_SIZE // 2
            self.angle = 0 # Reset góc xoay
            if len(self.path_nodes) > 1: # Nếu có ít nhất hai node để xác định hướng
                self._update_angle_to_next_node() # Cập nhật góc ban đầu
            if self.original_image: # Cập nhật sprite đã xoay ban đầu
                 self.image_to_draw = pygame.transform.rotate(self.original_image, self.angle)
        else: # Nếu không có đường đi
            self.finished_path = True # Đánh dấu là đã hoàn thành
            self.is_moving_for_dust = False # Không di chuyển, không tạo bụi


    def _update_angle_to_next_node(self):
        """
        Hàm nội bộ để tính toán và cập nhật góc xoay của agent
        sao cho nó hướng về node tiếp theo trên đường đi.
        """
        # Kiểm tra xem có node tiếp theo không
        if self.current_path_index + 1 < len(self.path_nodes):
            next_r, next_c = self.path_nodes[self.current_path_index + 1] # Lấy tọa độ node tiếp theo
            # Tính tọa độ pixel trung tâm của node tiếp theo
            target_x = next_c * CELL_SIZE + CELL_SIZE // 2
            target_y = next_r * CELL_SIZE + CELL_SIZE // 2
            
            # Tính vector hướng từ agent đến node tiếp theo
            dx = target_x - self.x_center
            dy = target_y - self.y_center
            
            if dx != 0 or dy != 0: # Tránh lỗi atan2(0,0) nếu agent đã ở vị trí đích
                # Tính góc bằng hàm atan2.
                # Pygame có trục y hướng xuống, nên dy thường âm khi đi lên.
                # math.atan2(y, x) trả về góc (radian) so với trục x dương.
                # Để 0 độ hướng sang phải, và góc tăng ngược chiều kim đồng hồ: atan2(-dy, dx)
                self.angle = math.degrees(math.atan2(-dy, dx))

    def _emit_dust_particle(self):
        """
        Hàm nội bộ để tạo ra một hạt bụi mới phía sau agent.
        """
        # Chỉ tạo hạt bụi nếu chưa đạt số lượng tối đa
        if len(self.dust_particles) < self.max_dust_particles:
            # Tạo hạt bụi phía sau xe, hơi lệch so với tâm
            # Góc tạo bụi là góc của xe + 180 độ (phía sau) + một chút ngẫu nhiên để tạo độ tản ra
            rad_angle = math.radians(self.angle + 180 + random.uniform(-20, 20))
            offset_dist = CELL_SIZE * 0.3 # Khoảng cách từ tâm xe đến vị trí tạo bụi
            
            # Tính toán vị trí tạo bụi dựa trên hướng hiện tại của xe
            offset_x_from_car_center = math.cos(rad_angle) * offset_dist
            offset_y_from_car_center = -math.sin(rad_angle) * offset_dist # Trục y của Pygame ngược

            particle_x = self.x_center + offset_x_from_car_center
            particle_y = self.y_center + offset_y_from_car_center
            
            # Thuộc tính ngẫu nhiên cho hạt bụi
            size = random.randint(1, 4) # Kích thước hạt bụi
            lifetime = random.uniform(0.15, 0.4) # Thời gian tồn tại (giây)
            color_val = random.randint(150, 190) # Giá trị màu xám
            alpha_start = random.randint(80, 150) # Độ trong suốt ban đầu
            color_rgb = (color_val, color_val, color_val) # Màu RGB
            vel_x = random.uniform(-15, 15) # Vận tốc ngang ngẫu nhiên (pixel/giây)
            vel_y = random.uniform(-15, 15) # Vận tốc dọc ngẫu nhiên (pixel/giây)

            # Thêm hạt bụi mới vào danh sách
            self.dust_particles.append({
                "x": particle_x, "y": particle_y, "size": size,
                "lifetime": lifetime, "initial_lifetime": lifetime, # Lưu lifetime ban đầu để tính %
                "alpha_start": alpha_start, "color_rgb": color_rgb,
                "vel_x": vel_x, "vel_y": vel_y
            })

    def _update_dust_particles(self, dt):
        """
        Hàm nội bộ để cập nhật trạng thái của tất cả các hạt bụi (vị trí, lifetime, alpha, size).

        Args:
            dt (float): Thời gian delta (giây).
        """
        new_particles = [] # Danh sách hạt bụi còn tồn tại sau khi cập nhật
        for p in self.dust_particles:
            p["lifetime"] -= dt # Giảm thời gian tồn tại
            if p["lifetime"] > 0: # Nếu hạt bụi vẫn còn sống
                # Cập nhật vị trí dựa trên vận tốc
                p["x"] += p["vel_x"] * dt
                p["y"] += p["vel_y"] * dt
                # Làm mờ dần (fade out) và thu nhỏ hạt bụi theo thời gian
                p["current_alpha"] = int((p["lifetime"] / p["initial_lifetime"]) * p["alpha_start"])
                p["current_size"] = max(1, int(p["size"] * (p["lifetime"] / p["initial_lifetime"])))
                new_particles.append(p)
        self.dust_particles = new_particles # Cập nhật lại danh sách hạt bụi

    def update(self, dt):
        """
        Cập nhật trạng thái của agent mỗi frame (di chuyển, xoay, hiệu ứng bụi).

        Args:
            dt (float): Thời gian delta (giây).
        """
        self.is_moving_for_dust = False # Reset cờ di chuyển cho hiệu ứng bụi
        # Kiểm tra xem agent có đang trên đường đi và chưa đến node cuối cùng không
        if not self.finished_path and self.path_nodes and self.current_path_index < len(self.path_nodes) - 1:
            self._update_angle_to_next_node() # Cập nhật góc xoay trước khi di chuyển
            if self.original_image: # Xoay sprite nếu có
                self.image_to_draw = pygame.transform.rotate(self.original_image, self.angle)

            # Lấy tọa độ (hàng, cột) của node đích tiếp theo
            target_r, target_c = self.path_nodes[self.current_path_index + 1]
            # Tính tọa độ pixel trung tâm của node đích
            target_x_center = target_c * CELL_SIZE + CELL_SIZE // 2
            target_y_center = target_r * CELL_SIZE + CELL_SIZE // 2

            # Tính vector hướng và khoảng cách đến node đích
            dx = target_x_center - self.x_center
            dy = target_y_center - self.y_center
            distance_to_target = math.sqrt(dx**2 + dy**2)

            # Tính quãng đường agent di chuyển được trong frame này (pixel)
            # self.speed (ô/giây) * CELL_SIZE (pixel/ô) * dt (giây/frame) = pixel/frame
            effective_speed_pixels = self.speed * CELL_SIZE * dt

            if distance_to_target <= effective_speed_pixels:
                # Nếu agent có thể đến được node đích trong frame này (hoặc đã vượt qua)
                # -> "Snap" agent đến chính xác vị trí node đích
                self.x_center, self.y_center = target_x_center, target_y_center
                self.row, self.col = target_r, target_c # Cập nhật vị trí lưới của agent
                self.current_path_index += 1 # Chuyển sang node tiếp theo trên đường đi
                # Kiểm tra xem đã đến node cuối cùng của đường đi chưa
                if self.current_path_index >= len(self.path_nodes) - 1:
                    self.finished_path = True # Đánh dấu hoàn thành đường đi
                self.is_moving_for_dust = True # Vẫn coi là di chuyển khi snap tới điểm để tạo bụi
            else:
                # Nếu chưa đến được node đích, di chuyển agent một đoạn về phía đó
                move_x = (dx / distance_to_target) * effective_speed_pixels # Thành phần di chuyển x
                move_y = (dy / distance_to_target) * effective_speed_pixels # Thành phần di chuyển y
                self.x_center += move_x
                self.y_center += move_y
                self.is_moving_for_dust = True # Đang di chuyển
        
        # --- Cập nhật hiệu ứng bụi ---
        self.dust_emit_timer += dt # Tăng bộ đếm thời gian tạo bụi
        # Nếu agent đang di chuyển và đã đến lúc tạo bụi mới
        if self.is_moving_for_dust and self.dust_emit_timer >= self.dust_emit_interval:
            self.dust_emit_timer = 0 # Reset bộ đếm
            self._emit_dust_particle() # Tạo hạt bụi mới
        self._update_dust_particles(dt) # Cập nhật tất cả các hạt bụi


    def draw(self, screen):
        """
        Vẽ agent (xe và hiệu ứng bụi) lên màn hình.

        Args:
            screen (pygame.Surface): Bề mặt màn hình để vẽ lên.
        """
        # --- Vẽ các hạt bụi trước (để chúng xuất hiện phía sau xe) ---
        for p in self.dust_particles:
            # Chỉ vẽ hạt bụi nếu nó còn nhìn thấy được (alpha > 0 và size > 0)
            if p["current_alpha"] > 0 and p["current_size"] > 0:
                # Tạo một surface tạm thời cho mỗi hạt bụi để có thể đặt alpha riêng
                # Kích thước surface gấp đôi kích thước hạt bụi để vẽ hình tròn ở tâm
                particle_surf = pygame.Surface((p["current_size"] * 2, p["current_size"] * 2), pygame.SRCALPHA)
                pygame.draw.circle(
                    particle_surf,
                    (*p["color_rgb"], p["current_alpha"]), # Màu RGB + alpha
                    (p["current_size"], p["current_size"]), # Tâm của particle_surf
                    p["current_size"] # Bán kính (bằng kích thước hiện tại)
                )
                # Vẽ surface hạt bụi lên màn hình chính
                # Vị trí vẽ là (x - bán kính, y - bán kính) để tâm hạt bụi trùng với (p["x"], p["y"])
                screen.blit(particle_surf, (int(p["x"] - p["current_size"]), int(p["y"] - p["current_size"])))

        # --- Vẽ agent (xe) ---
        if self.image_to_draw: # Nếu có sprite để vẽ
            # Lấy hình chữ nhật bao quanh sprite đã xoay, với tâm tại vị trí của agent
            rect = self.image_to_draw.get_rect(center=(int(self.x_center), int(self.y_center)))
            # Vẽ sprite lên màn hình tại vị trí rect.topleft
            screen.blit(self.image_to_draw, rect.topleft)
        elif self.original_image is None: # Nếu không có sprite gốc (lỗi tải)
             # Vẽ một hình tròn làm fallback
             pygame.draw.circle(screen, self.fallback_color, (int(self.x_center), int(self.y_center)), CELL_SIZE // 3)

        # --- Vẽ tên của agent phía trên nó (tùy chọn) ---
        if True: # Có thể thêm một cờ self.show_name để bật/tắt hiển thị tên
            # Nên tải font một lần trong __init__ hoặc truyền từ ngoài vào thay vì mỗi lần draw
            font = pygame.font.SysFont("Arial", 11, bold=True)
            # Tạo surface chứa text tên agent, với màu chữ và màu nền (có alpha)
            text_surf = font.render(self.name, True, (10,10,10), (230,230,230,180))
            # Lấy hình chữ nhật của text_surf, đặt tâm của nó phía trên agent một chút
            text_rect = text_surf.get_rect(center=(int(self.x_center), int(self.y_center - CELL_SIZE * 0.6)))
            screen.blit(text_surf, text_rect) # Vẽ text lên màn hình
            
    def reset_to_start(self, start_rc):
        """
        Đặt lại agent về vị trí bắt đầu được chỉ định và xóa đường đi hiện tại.
        Thường được gọi khi người dùng đặt lại điểm bắt đầu trên lưới hoặc reset lưới.

        Args:
            start_rc (tuple): Tọa độ (row, col) mới của điểm bắt đầu.
        """
        self.row, self.col = start_rc # Cập nhật vị trí lưới
        # Cập nhật vị trí pixel
        self.x_center = self.col * CELL_SIZE + CELL_SIZE // 2
        self.y_center = self.row * CELL_SIZE + CELL_SIZE // 2
        self.path_nodes = [] # Xóa đường đi
        self.current_path_index = 0 # Reset chỉ số
        self.finished_path = True # Đánh dấu đã hoàn thành (vì không có path)
        self.angle = 0 # Reset góc
        self.dust_particles.clear() # Xóa bụi
        if self.original_image: # Reset sprite về trạng thái ban đầu (không xoay)
            self.image_to_draw = pygame.transform.rotate(self.original_image, self.angle)