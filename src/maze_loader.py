# src/maze_loader.py
from config import GRID_ROWS, GRID_COLS # Import kích thước lưới để định nghĩa maze

# --- ĐỊNH NGHĨA CÁC MẪU MÊ CUNG (MAZE PATTERNS) ---
# MAZE_PATTERNS là một dictionary, trong đó mỗi key là tên của một mẫu maze,
# và value là một dictionary khác chứa thông tin chi tiết của maze đó.
# Mỗi mẫu maze bao gồm:
#   - 'start': Tuple (row, col) tọa độ của điểm bắt đầu.
#   - 'end': Tuple (row, col) tọa độ của điểm kết thúc.
#   - 'obstacles': List các tuple (row, col) tọa độ của các ô chướng ngại vật (tường).
#   - 'traps': List các tuple (row, col) tọa độ của các ô bẫy (chi phí cao).
MAZE_PATTERNS = {
    "Empty Field": { # Một maze trống, hữu ích để kiểm tra cơ bản hoặc cho người dùng tự xây.
        "start": (1, 1), # Điểm bắt đầu ở gần góc trên bên trái.
        "end": (GRID_ROWS - 2, GRID_COLS - 2), # Điểm kết thúc ở gần góc dưới bên phải.
        "obstacles": [], # Không có chướng ngại vật.
        "traps": []      # Không có bẫy.
    },
    "Simple Wall": { # Maze với một bức tường dọc ở giữa, có một lỗ nhỏ để đi qua.
        "start": (GRID_ROWS // 2, 1), # Bắt đầu ở giữa hàng, cột thứ hai.
        "end": (GRID_ROWS // 2, GRID_COLS - 2), # Kết thúc ở giữa hàng, cột gần cuối.
        # Tạo một bức tường dọc ở giữa lưới (GRID_COLS // 2).
        # Chừa một ô trống ở hàng giữa (r != GRID_ROWS // 2) để làm lối đi.
        "obstacles": [(r, GRID_COLS // 2) for r in range(GRID_ROWS) if r != GRID_ROWS // 2],
        "traps": []
    },
    "Trap Bridge": { # Maze có các bức tường và một "cây cầu" làm bằng các ô bẫy.
        "start": (1, 1),
        "end": (GRID_ROWS - 2, GRID_COLS - 2),
        "obstacles": [ # Định nghĩa các đoạn tường
                        # Tường bên trái, phần trên
                        (r, GRID_COLS // 3) for r in range(0, GRID_ROWS // 2 - 1)] + \
                     [ # Tường bên trái, phần dưới
                        (r, GRID_COLS // 3) for r in range(GRID_ROWS // 2 + 2, GRID_ROWS)] + \
                     [ # Tường bên phải, phần trên
                        (r, 2 * GRID_COLS // 3) for r in range(0, GRID_ROWS // 2 - 1)] + \
                     [ # Tường bên phải, phần dưới
                        (r, 2 * GRID_COLS // 3) for r in range(GRID_ROWS // 2 + 2, GRID_ROWS)],
        # Tạo một cây cầu bằng bẫy ở giữa, nối hai phần của maze.
        "traps": [(GRID_ROWS // 2, c) for c in range(GRID_COLS // 3 + 1, 2 * GRID_COLS // 3)]
    },
    "Spiral Trap": { # Maze có cấu trúc xoắn ốc với các bẫy bên trong.
        # Lưu ý: Các tọa độ này cần được kiểm tra kỹ để đảm bảo chúng nằm trong giới hạn
        # của GRID_ROWS và GRID_COLS hiện tại.
        "start": (1,1),
        "end": (GRID_ROWS//2, GRID_COLS//2), # Kết thúc ở trung tâm của vòng xoáy (gần đúng).
        "obstacles": [ # Các bức tường tạo thành vòng xoáy bên ngoài.
                        (r,2) for r in range(1,GRID_ROWS-2)] + \
                     [(GRID_ROWS-3,c) for c in range(2,GRID_COLS-2)] + \
                     [(r,GRID_COLS-3) for r in range(2,GRID_ROWS-2)] + \
                     [(2,c) for c in range(4,GRID_COLS-3)],
        "traps": [ # Các bẫy tạo thành vòng xoáy bên trong.
                   (r,4) for r in range(4,GRID_ROWS-4)] + \
                   [(GRID_ROWS-5,c) for c in range(4,GRID_COLS-4)] + \
                   [(r,GRID_COLS-5) for r in range(4,GRID_ROWS-6)] + \
                   [(4,c) for c in range(6,GRID_COLS-5)] + \
                   # Đường dẫn bẫy cuối cùng dẫn đến điểm kết thúc.
                   # Cần đảm bảo `c` trong `range(6, GRID_COLS//2)` không vượt quá giới hạn.
                   [(GRID_ROWS//2, c) for c in range(6, GRID_COLS//2)]
    }
    # Người dùng có thể thêm các định nghĩa maze phức tạp hơn tại đây.
    # Ví dụ: maze_patterns["My Awesome Maze"] = { ... }
}

# MAZE_NAMES là một list chứa tên của tất cả các mẫu maze đã được định nghĩa.
# Được sử dụng để hiển thị trong dropdown menu trên UI.
MAZE_NAMES = list(MAZE_PATTERNS.keys())

def apply_maze_to_grid(game_grid_ref, maze_name):
    """
    Áp dụng một mẫu maze đã chọn lên đối tượng lưới game (`game_grid_ref`).
    Hàm này sẽ:
    1. Reset toàn bộ lưới về trạng thái mặc định (ô trống).
    2. Đặt các ô chướng ngại vật (obstacles) theo mẫu maze.
    3. Đặt các ô bẫy (traps) theo mẫu maze.
    4. Đặt ô bắt đầu (start node) và ô kết thúc (end node) theo mẫu maze.
    Hàm sẽ kiểm tra tính hợp lệ của tọa độ và tránh đặt các thành phần chồng chéo không mong muốn.

    Args:
        game_grid_ref (list of list of Node): Tham chiếu đến đối tượng lưới game,
                                              là một_list 2 chiều chứa các đối tượng Node.
        maze_name (str): Tên của mẫu maze cần áp dụng (phải là một key trong `MAZE_PATTERNS`).

    Returns:
        tuple (tuple or None, tuple or None): Một tuple chứa vị trí (row, col) mới của điểm bắt đầu
                                               và điểm kết thúc. Trả về (None, None) nếu có lỗi
                                               hoặc nếu maze không hợp lệ.
    """
    # Kiểm tra xem tên maze có tồn tại trong danh sách các mẫu không.
    if maze_name not in MAZE_PATTERNS:
        print(f"Error: Maze pattern '{maze_name}' not found.")
        return None, None # Trả về None nếu không tìm thấy mẫu.

    pattern = MAZE_PATTERNS[maze_name] # Lấy thông tin chi tiết của mẫu maze đã chọn.
    new_start_pos = None # Biến lưu vị trí điểm bắt đầu mới.
    new_end_pos = None   # Biến lưu vị trí điểm kết thúc mới.

    # --- Bước 1: Reset toàn bộ lưới ---
    # Duyệt qua tất cả các ô trong lưới và gọi phương thức `reset()`
    # để đưa chúng về trạng thái "normal" (ô trống, không có thuộc tính đặc biệt).
    for r_idx in range(GRID_ROWS):
        for c_idx in range(GRID_COLS):
            game_grid_ref[r_idx][c_idx].reset()

    # --- Bước 2: Đặt các ô chướng ngại vật (Obstacles) ---
    # Lấy danh sách tọa độ chướng ngại vật từ mẫu maze (mặc định là list rỗng nếu không có).
    for r, c in pattern.get("obstacles", []):
        # Kiểm tra xem tọa độ có nằm trong giới hạn của lưới không.
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            game_grid_ref[r][c].make_obstacle() # Đặt ô làm chướng ngại vật.
        else:
            # In cảnh báo nếu tọa độ nằm ngoài giới hạn.
            print(f"Warning: Obstacle ({r},{c}) in '{maze_name}' is out of bounds.")


    # --- Bước 3: Đặt các ô bẫy (Traps) ---
    # Lấy danh sách tọa độ bẫy từ mẫu maze.
    for r, c in pattern.get("traps", []):
        # Kiểm tra tọa độ hợp lệ.
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            # Đảm bảo không đặt bẫy lên một ô đã là chướng ngại vật.
            if not game_grid_ref[r][c].is_obstacle_type():
                 game_grid_ref[r][c].make_trap() # Đặt ô làm bẫy.
            else:
                # In cảnh báo nếu cố gắng đặt bẫy lên chướng ngại vật.
                print(f"Warning: Trap ({r},{c}) in '{maze_name}' on an obstacle. Skipped.")
        else:
            # In cảnh báo nếu tọa độ bẫy nằm ngoài giới hạn.
            print(f"Warning: Trap ({r},{c}) in '{maze_name}' is out of bounds.")


    # --- Bước 4: Đặt ô bắt đầu (Start Node) ---
    # Lấy tọa độ điểm bắt đầu từ mẫu maze (mặc định là (None, None) nếu không có).
    sr, sc = pattern.get("start", (None,None))
    # Kiểm tra xem tọa độ có hợp lệ và nằm trong lưới không.
    if sr is not None and 0 <= sr < GRID_ROWS and 0 <= sc < GRID_COLS:
        # Chỉ đặt điểm bắt đầu nếu ô đó là ô "normal" (trống).
        # Điều này tránh đặt điểm bắt đầu lên chướng ngại vật hoặc bẫy đã được đặt trước đó.
        if game_grid_ref[sr][sc].type == "normal":
            game_grid_ref[sr][sc].make_start() # Đặt ô làm điểm bắt đầu.
            new_start_pos = (sr, sc) # Lưu vị trí điểm bắt đầu mới.
        else:
            # In cảnh báo nếu không thể đặt điểm bắt đầu.
            print(f"Warning: Start pos ({sr},{sc}) for '{maze_name}' is on obstacle/trap. Not set.")
    else:
        # In cảnh báo nếu tọa độ điểm bắt đầu không hợp lệ hoặc ngoài giới hạn.
        print(f"Warning: Start pos for '{maze_name}' invalid or out of bounds.")


    # --- Bước 5: Đặt ô kết thúc (End Node) ---
    # Lấy tọa độ điểm kết thúc từ mẫu maze.
    er, ec = pattern.get("end", (None,None))
    # Kiểm tra tọa độ hợp lệ.
    if er is not None and 0 <= er < GRID_ROWS and 0 <= ec < GRID_COLS:
        # Đảm bảo điểm kết thúc là một ô "normal" và không trùng với điểm bắt đầu đã đặt.
        if game_grid_ref[er][ec].type == "normal" and (er,ec) != new_start_pos :
            game_grid_ref[er][ec].make_end() # Đặt ô làm điểm kết thúc.
            new_end_pos = (er, ec) # Lưu vị trí điểm kết thúc mới.
        else:
            # In cảnh báo nếu không thể đặt điểm kết thúc.
            print(f"Warning: End pos ({er},{ec}) for '{maze_name}' invalid (on obstacle/trap/start). Not set.")
    else:
        # In cảnh báo nếu tọa độ điểm kết thúc không hợp lệ hoặc ngoài giới hạn.
        print(f"Warning: End pos for '{maze_name}' invalid or out of bounds.")

    print(f"Applied maze: {maze_name}") # Thông báo đã áp dụng maze thành công.
    return new_start_pos, new_end_pos # Trả về vị trí điểm bắt đầu và kết thúc mới.