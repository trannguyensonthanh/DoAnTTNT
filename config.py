import pygame

# --- KÍCH THƯỚC ---
# Các hằng số định nghĩa kích thước của các thành phần trong game.

# Kích thước của một ô vuông đơn vị trong lưới game (tính bằng pixel).
# Đây là đơn vị cơ bản để tính toán vị trí và kích thước của các đối tượng trên lưới.
CELL_SIZE = 32

# Số lượng hàng trong lưới game.
# Ví dụ: GRID_HEIGHT // CELL_SIZE (600 // 32 ~ 18) được điều chỉnh để đảm bảo lưới vừa khít.
GRID_ROWS = 25
# Số lượng cột trong lưới game.
# Ví dụ: GRID_WIDTH // CELL_SIZE (800 // 32 = 25).
GRID_COLS = 25

# Chiều rộng của khu vực lưới game (tính bằng pixel).
# Được tính bằng số cột nhân với kích thước mỗi ô.
GRID_WIDTH = GRID_COLS * CELL_SIZE    # 25 * 32 = 800
# Chiều cao của khu vực lưới game (tính bằng pixel).
# Được tính bằng số hàng nhân với kích thước mỗi ô.
# (Điều chỉnh từ 600 xuống 576 để đảm bảo số ô chẵn và vừa khít).
GRID_HEIGHT = GRID_ROWS * CELL_SIZE   # 18 * 32 = 576

# Chiều rộng của bảng điều khiển UI (User Interface) nằm ở bên phải màn hình (tính bằng pixel).
UI_PANEL_WIDTH = 280

# Tổng chiều rộng của cửa sổ game (tính bằng pixel).
# Bằng chiều rộng lưới cộng với chiều rộng của UI panel.
TOTAL_SCREEN_WIDTH = GRID_WIDTH + UI_PANEL_WIDTH
# Tổng chiều cao của cửa sổ game (tính bằng pixel).
# Hiện tại bằng chiều cao của lưới game.
TOTAL_SCREEN_HEIGHT = GRID_HEIGHT

# --- MÀU SẮC ---
# Các hằng số định nghĩa các giá trị màu RGB (Red, Green, Blue).

WHITE = (255, 255, 255) # Màu trắng.
BLACK = (0, 0, 0)       # Màu đen.
GREY = (200, 200, 200)  # Màu xám, thường dùng cho các đường kẻ của lưới (grid lines).
DARK_GREY = (50, 50, 50)  # Màu xám đậm, dùng làm màu nền dự phòng cho UI panel nếu theme không tải được.
LIGHT_BLUE_BG = (230, 240, 255) # Màu xanh dương nhạt, dùng làm màu nền chính cho khu vực lưới game.

# Màu sắc dự phòng cho các loại ô đặc biệt trên lưới (sử dụng nếu sprite/hình ảnh bị lỗi).
RED = (255, 0, 0)       # Màu đỏ, dự phòng cho ô chướng ngại vật (Obstacle).
GREEN = (0, 255, 0)     # Màu xanh lá, dự phòng cho ô bắt đầu (Start).
BLUE = (0, 0, 255)      # Màu xanh dương, dự phòng cho ô kết thúc (End).
BROWN = (139, 69, 19)   # Màu nâu, dự phòng cho ô bẫy (Trap).
ORANGE = (255, 165, 0)  # Màu cam, dùng để làm nổi bật đường đi của người chơi/agent.

# Màu sắc cho quá trình visualization (hiển thị các bước tìm đường).
# Màu xám nhạt với độ trong suốt (alpha channel = 100) cho các ô đã được thuật toán khám phá.
COLOR_EXPLORED_NODE = (220, 220, 220, 100)

# Màu sắc đặc trưng cho đường đi của từng thuật toán tìm đường.
COLOR_ASTAR_PATH = (255, 195, 80)    # Vàng cam cho A*.
COLOR_DIJKSTRA_PATH = (80, 180, 255) # Xanh dương nhạt cho Dijkstra.
COLOR_BFS_PATH = (180, 80, 220)      # Tím cho BFS.
COLOR_GREEDY_PATH = (100, 220, 100)  # Xanh lá cây nhạt cho Greedy BFS.
COLOR_JPS_PATH = (0, 200, 200)       # Xanh cyan cho JPS (Jump Point Search).
COLOR_BIDIR_PATH = (120, 80, 220)    # Tím đậm hơn / Indigo cho Bi-directional A*.

# --- CHI PHÍ Ô ---
# Các hằng số định nghĩa chi phí để di chuyển qua các loại ô khác nhau.

COST_NORMAL_CELL = 1    # Chi phí di chuyển qua một ô bình thường.
COST_TRAP_CELL = 10     # Chi phí di chuyển qua một ô bẫy (cao hơn bình thường).

# --- ĐƯỜNG DẪN TÀI NGUYÊN ---
# Các hằng số định nghĩa đường dẫn đến các thư mục chứa tài nguyên của game.

ASSETS_DIR = "assets" # Thư mục gốc chứa tất cả tài nguyên.
IMAGES_DIR = f"{ASSETS_DIR}/images" # Thư mục con chứa hình ảnh (sprites, background).
FONTS_DIR = f"{ASSETS_DIR}/fonts"   # Thư mục con chứa các file font tùy chỉnh (nếu có).

# --- GAME SETTINGS ---
# Các cài đặt chung cho game.

FPS = 60 # Số khung hình mỗi giây (Frames Per Second) mà game cố gắng duy trì.

# --- ANIMATION VISUALIZATION ---
# Các hằng số liên quan đến cài đặt tốc độ của animation hiển thị quá trình tìm đường.
# Những giá trị này có thể được điều chỉnh bởi người dùng thông qua slider trên UI.

# Giá trị tối thiểu của slider điều khiển tốc độ animation.
ANIM_SLIDER_MIN_VAL = 1
# Giá trị tối đa của slider điều khiển tốc độ animation.
ANIM_SLIDER_MAX_VAL = 10
# Giá trị mặc định khởi tạo cho slider tốc độ animation.
ANIM_SLIDER_DEFAULT_VAL = 5

# Độ trễ (tính bằng giây) giữa việc hiển thị mỗi node trong quá trình animation.
# Độ trễ tối đa (chậm nhất), tương ứng khi slider ở giá trị MIN_VAL.
ANIM_VIZ_MAX_DELAY = 0.05
# Độ trễ tối thiểu (nhanh nhất), tương ứng khi slider ở giá trị MAX_VAL.
ANIM_VIZ_MIN_DELAY = 0.001
# ANIM_VIZ_DEFAULT_DELAY (độ trễ mặc định) thường được tính toán trong `main.py`
# dựa trên `ANIM_SLIDER_DEFAULT_VAL` và các giá trị min/max delay.
# Hoặc có thể đặt một giá trị cố định nếu không muốn tính toán phức tạp:
# ANIM_VIZ_DEFAULT_DELAY = 0.01