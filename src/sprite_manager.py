# src/sprite_manager.py
import pygame
import os
from config import CELL_SIZE, GRID_WIDTH, GRID_HEIGHT, IMAGES_DIR

# Dictionary toàn cục để lưu trữ tất cả các sprite đã được tải.
# Key là tên định danh của sprite (ví dụ: "wall", "car_astar"),
# value là đối tượng pygame.Surface của sprite đã được scale.
SPRITES = {}

# Biến toàn cục để lưu trữ hình ảnh nền của khu vực lưới game.
BACKGROUND_IMAGE = None

def load_game_assets():
    """
    Tải tất cả các tài sản hình ảnh cần thiết cho game, bao gồm sprite cho các
    thành phần bản đồ, sprite cho agent (xe), và hình ảnh nền.
    Các sprite được scale về kích thước phù hợp và lưu vào dictionary `SPRITES`.
    Hình nền được scale để vừa với kích thước lưới game và lưu vào `BACKGROUND_IMAGE`.
    """
    global SPRITES, BACKGROUND_IMAGE # Khai báo sử dụng các biến toàn cục
    print("--- Starting to load game assets ---")

    # Kiểm tra xem thư mục chứa hình ảnh có tồn tại không.
    if not os.path.exists(IMAGES_DIR):
        print(f"ERROR: Image directory not found at {IMAGES_DIR}")
        # Ghi chú: Việc tạo thư mục ở đây có thể là một lựa chọn,
        # nhưng thường thì người dùng nên đảm bảo cấu trúc thư mục đúng trước khi chạy.
        # os.makedirs(IMAGES_DIR, exist_ok=True)
        return # Thoát hàm nếu không tìm thấy thư mục hình ảnh.

    # --- Tải Hình Nền ---
    ground_path = os.path.join(IMAGES_DIR, "ground.png") # Đường dẫn đến file hình nền
    try:
        # Tải hình ảnh từ file.
        raw_background_image = pygame.image.load(ground_path)
        # Tối ưu hóa hình ảnh bằng cách chuyển đổi định dạng pixel (convert).
        # Điều này giúp tăng tốc độ vẽ hình ảnh lên màn hình.
        BACKGROUND_IMAGE = raw_background_image.convert()
        # Scale hình nền để vừa với kích thước của khu vực lưới game.
        BACKGROUND_IMAGE = pygame.transform.scale(BACKGROUND_IMAGE, (GRID_WIDTH, GRID_HEIGHT))
        print(f"Successfully loaded background: {ground_path}")
    except pygame.error as e:
        # Xử lý lỗi nếu không tải được hình nền.
        print(f"ERROR loading background {ground_path}: {e}")
        BACKGROUND_IMAGE = None # Đặt là None để có thể fallback sang màu nền đơn sắc.

    # --- Tải Sprite cho các Thành Phần Bản Đồ (Map Elements) ---
    # Dictionary định nghĩa các sprite cho map và tên file tương ứng.
    map_element_files = {
        "wall": "wall.png",         # Sprite cho tường
        "trap": "trap.png",         # Sprite cho bẫy
        "start_flag": "start_flag.png", # Sprite cho cờ điểm bắt đầu
        "end_flag": "end_flag.png",   # Sprite cho cờ điểm kết thúc
    }
    for key_name, filename in map_element_files.items():
        path = os.path.join(IMAGES_DIR, filename) # Đường dẫn đầy đủ đến file sprite
        try:
            # Tải hình ảnh từ file.
            image = pygame.image.load(path)
            # Chuyển đổi định dạng pixel và giữ lại kênh alpha (độ trong suốt) bằng convert_alpha().
            # Quan trọng cho các sprite có phần trong suốt.
            image_alpha = image.convert_alpha()
            # Scale sprite về kích thước của một ô (CELL_SIZE x CELL_SIZE).
            SPRITES[key_name] = pygame.transform.scale(image_alpha, (CELL_SIZE, CELL_SIZE))
            print(f"Loaded MAP sprite: '{key_name}' from {path}")
        except pygame.error as e:
            # Xử lý lỗi nếu không tải được sprite.
            print(f"ERROR loading MAP sprite '{key_name}' from {path}: {e}")
            SPRITES[key_name] = None # Đặt là None, GridNode sẽ dùng màu fallback.

    # --- Tải Sprite cho Xe (Agent) ---
    # Dictionary định nghĩa các sprite cho xe và tên file tương ứng.
    # Các sprite này sẽ được sử dụng bởi class Agent.
    car_sprite_files = {
        "car_astar": "car_astar.png",       # Xe cho thuật toán A*
        "car_dijkstra": "car_dijkstra.png", # Xe cho thuật toán Dijkstra
        "car_bfs": "car_bfs.png",           # Xe cho thuật toán BFS
        "car_greedy": "car_greedy.png",     # Xe cho thuật toán Greedy BFS
        "default_car": "default_car.png",   # Xe mặc định (nếu có)
        # Thêm các sprite xe khác ở đây nếu cần, ví dụ: "car_jps", "car_bidir"
    }
    for key_name, filename in car_sprite_files.items():
        path = os.path.join(IMAGES_DIR, filename) # Đường dẫn đầy đủ đến file sprite xe
        try:
            # Tải hình ảnh xe và giữ kênh alpha.
            image = pygame.image.load(path).convert_alpha()
            # Scale sprite xe nhỏ hơn một chút so với kích thước ô (85% CELL_SIZE)
            # để xe trông vừa vặn hơn khi di chuyển trong ô.
            car_width = int(CELL_SIZE * 0.85)
            car_height = int(CELL_SIZE * 0.85)
            SPRITES[key_name] = pygame.transform.scale(image, (car_width, car_height))
            print(f"Loaded CAR sprite: '{key_name}' from {path}")
        except pygame.error as e:
            # Xử lý lỗi nếu không tải được sprite xe.
            print(f"ERROR loading CAR sprite '{key_name}' from {path}: {e}")
            SPRITES[key_name] = None # Sprite xe sẽ không được hiển thị nếu lỗi.

    print("--- Finished loading game assets ---")
    # In cảnh báo nếu các sprite quan trọng không tải được.
    if SPRITES.get("wall") is None:
        print("Warning: wall.png failed. Walls will use fallback color.")
    if SPRITES.get("trap") is None:
        print("Warning: trap.png failed. Traps will use fallback color.")

def get_sprite(key):
    """
    Truy xuất một sprite đã được tải từ dictionary `SPRITES` bằng key của nó.

    Args:
        key (str): Tên định danh của sprite cần lấy (ví dụ: "wall", "car_astar").

    Returns:
        pygame.Surface or None: Đối tượng pygame.Surface của sprite nếu tìm thấy,
                                 hoặc None nếu sprite không tồn tại.
    """
    return SPRITES.get(key)

def get_background():
    """
    Truy xuất hình ảnh nền đã được tải.

    Returns:
        pygame.Surface or None: Đối tượng pygame.Surface của hình nền,
                                 hoặc None nếu hình nền không được tải.
    """
    return BACKGROUND_IMAGE