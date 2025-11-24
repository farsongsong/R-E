import pygame
import heapq
import math
import time
import smbus
from collections import deque

# ----------------------
# A* Pathfinding (4-direction) - (unchanged)
# ----------------------
def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def neighbors(pos, grid):
    r, c = pos
    rows = len(grid)
    cols = len(grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)] 

    for dr, dc in directions:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
            cost = 1
            yield (nr, nc), cost

def a_star(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    g = {start: 0}
    closed = set()
    
    while open_set:
        _, current_g, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        if current in closed:
            continue
        closed.add(current)
        for n, move_cost in neighbors(current, grid):
            new_g = current_g + move_cost
            if new_g < g.get(n, float('inf')):
                g[n] = new_g
                came_from[n] = current
                f = new_g + heuristic(n, goal)
                heapq.heappush(open_set, (f, new_g, n))
    return None

# ----------------------
# Pygame Visualization (Settings) - (unchanged)
# ----------------------
WIDTH, HEIGHT = 600, 600
ROWS, COLS = 30, 30
CELL = WIDTH // COLS
pygame.init()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Map + A* (Single Wheel Pivot Turn)")

grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]
start = None
goal = None
path = None
mouse_down_left = False
mouse_down_right = False

# ----------------------
# I2C Control for the Car - (unchanged)
# ----------------------
class YB_Pcb_Car(object):
    def get_i2c_device(self, address, i2c_bus):
        self._addr = address
        try: return smbus.SMBus(i2c_bus if i2c_bus is not None else 1)
        except Exception: return None

    def __init__(self):
        try: 
            self._device = self.get_i2c_device(0x16, 1)
            if not self._device: raise Exception("I2C device not found.")
        except Exception:
            self._device = None

    def Ctrl_Car(self, l_dir, l_speed, r_dir, r_speed):
        try:
            reg = 0x01
            data = [l_dir, l_speed, r_dir, r_speed]
            self.write_array(reg, data)
        except Exception:
            pass
            
    def write_u8(self, reg, data):
        try:
            if self._device: self._device.write_byte_data(self._addr, reg, data)
        except Exception as e:
            pass

    def write_array(self, reg, data):
        try:
            if self._device: self._device.write_i2c_block_data(self._addr, reg, data)
        except Exception as e:
            pass

    def Car_Run(self, speed1, speed2):
        try: self.Ctrl_Car(1, speed1, 1, speed2)
        except: pass

    def Car_Back(self, speed1, speed2):
        try:
            self.Ctrl_Car(0, speed1, 0, speed2)
        except:
            print ('Car_Back I2C error')

    def Car_Stop(self):
        try: 
            reg = 0x02
            self.write_u8(reg, 0x00)
        except: pass

# -------------------------------------------------------------
# CONFIGURATION CONSTANTS (요청에 따라 변경됨)
# -------------------------------------------------------------

# 1. 속도 상수 (분리)
SPEED_FORWARD = 30  # 직진 시 속도
SPEED_TURN = 100     # 회전 시 속도

# 2. 이동 시간 상수 (실제 물리 거리 반영)
CELL_SIZE_CM = 7.5
MOVE_TIME_PER_CM = 0.25 

# 3. 정밀 회전 시간 상수 (요청에 따라 0.85로 고정)
TURN_TIME_90_DEG = 0.85 

# 계산된 최종 이동 시간
FORWARD_TIME_PER_CELL = CELL_SIZE_CM * MOVE_TIME_PER_CM


# ----------------------
# Action Sequence Generation and Execution
# ----------------------

def get_direction_vector(curr, next_pos):
    return (next_pos[0] - curr[0], next_pos[1] - curr[1])

def get_vector_angle(dr, dc):
    if dr == 0 and dc == 1: return 0    # 동
    if dr == -1 and dc == 0: return 90   # 북
    if dr == 0 and dc == -1: return 180  # 서
    if dr == 1 and dc == 0: return 270  # 남
    return None

def generate_action_sequence(path):
    """A* 경로를 오른쪽 바퀴 선회 기반 행동 배열로 변환합니다."""
    
    if len(path) < 2:
        return []

    current_heading = 90 
    action_sequence = []
    
    # 1. 초기 정렬 로직 (unchanged)
    dr_init, dc_init = get_direction_vector(path[0], path[1])
    target_angle_init = get_vector_angle(dr_init, dc_init)
    
    if target_angle_init is not None:
        angle_diff_init = target_angle_init - current_heading
        if angle_diff_init > 180: angle_diff_init -= 360
        if angle_diff_init < -180: angle_diff_init += 360
        
        if abs(angle_diff_init) > 10:
            if angle_diff_init == 90: action_sequence.append('TURN_LEFT_90')
            elif angle_diff_init == -90: action_sequence.append('TURN_RIGHT_90')
            elif abs(angle_diff_init) == 180: action_sequence.append('TURN_180')
            
        current_heading = target_angle_init 
    
    # 2. 경로의 나머지 부분에 대한 행동 생성 (unchanged)
    for i in range(len(path) - 1):
        curr = path[i]
        next_pos = path[i+1]
        dr, dc = get_direction_vector(curr, next_pos)

        target_angle = get_vector_angle(dr, dc)
        
        angle_diff = target_angle - current_heading
        if angle_diff > 180: angle_diff -= 360
        if angle_diff < -180: angle_diff += 360
        
        if abs(angle_diff) > 10: 
            if angle_diff == 90: action_sequence.append('TURN_LEFT_90')
            elif angle_diff == -90: action_sequence.append('TURN_RIGHT_90')
            elif abs(angle_diff) == 180: action_sequence.append('TURN_180')
        
        action_sequence.append('FORWARD')
        
        current_heading = target_angle
        
    return action_sequence

def execute_action_sequence(action_sequence, car):
    """생성된 행동 배열을 오른쪽 바퀴 선회 방식으로 순차적으로 실행합니다. (분리된 속도 적용)"""
    
    s_fwd = SPEED_FORWARD
    s_turn = SPEED_TURN
    t_fwd = FORWARD_TIME_PER_CELL
    t_turn = TURN_TIME_90_DEG
    
    print(action_sequence)
    
    for action in action_sequence:
        print(f"Executing action: {action}")
        
        if action == 'FORWARD':
            # 1. 전진: 직진 속도 적용 (Car_Run은 내부적으로 Ctrl_Car(1, s, 1, s)를 호출함)
            car.Car_Run(s_fwd, s_fwd) 
            time.sleep(t_fwd) 
        
        elif action == 'TURN_LEFT_90':
            # 2. 좌회전 선회: 오른쪽 바퀴만 전진 (회전 속도 적용)
            # l_speed=0, r_dir=1(전진), r_speed=s_turn
            car.Car_Run(0, s_turn) 
            time.sleep(t_turn) #? 
        
        elif action == 'TURN_RIGHT_90':
            # 3. 우회전 선회: 오른쪽 바퀴만 후진 (회전 속도 적용)
            # l_speed=0, r_dir=0(후진), r_speed=s_turn
            #Beacause of Car_Back, need Run
            car. Car_Run(s_fwd, s_fwd)
            time.sleep(t_fwd)
            car.Car_Back(0, s_turn) 
            time.sleep(t_turn) #?

        elif action == 'TURN_180':
            # 4. 180도 선회: 오른쪽 바퀴만 후진 (회전 속도 적용)
            car.Ctrl_Car(0, 0, 0, s_turn) 
            time.sleep(t_turn * 2) 
            
        #car.Car_Stop()
    
    car.Car_Stop()
    print("Action sequence finished.")


# ----------------------
# Main Loop - (unchanged)
# ----------------------

car = YB_Pcb_Car()
driving = False 

running = True
while running:
    
    # 드로잉 로직
    win.fill((255, 255, 255))
    for r in range(ROWS):
        for c in range(COLS):
            rect = pygame.Rect(c * CELL, r * CELL, CELL, CELL)
            if grid[r][c] == 1: pygame.draw.rect(win, (0, 0, 0), rect) 
            pygame.draw.rect(win, (200, 200, 200), rect, 1) 

    if start: pygame.draw.rect(win, (0, 255, 0), (start[1]*CELL, start[0]*CELL, CELL, CELL)) 
    if goal: pygame.draw.rect(win, (255, 0, 0), (goal[1]*CELL, goal[0]*CELL, CELL, CELL)) 
    if path:
        for (r, c) in path:
            pygame.draw.rect(win, (0, 0, 255), (c*CELL + CELL//4, r*CELL + CELL//4, CELL//2, CELL//2)) 
    pygame.display.update()

    # 이벤트 처리
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False; break
            
        elif event.type == pygame.MOUSEBUTTONDOWN and not driving:
            if event.button == 1: mouse_down_left = True
            elif event.button == 3: mouse_down_right = True
        
        elif event.type == pygame.MOUSEBUTTONUP and not driving:
            if event.button == 1: mouse_down_left = False
            elif event.button == 3: mouse_down_right = False
                
        elif event.type == pygame.MOUSEMOTION and (mouse_down_left or mouse_down_right) and not driving:
            x, y = event.pos
            r, c = y // CELL, x // CELL
            if 0 <= r < ROWS and 0 <= c < COLS:
                if mouse_down_left: grid[r][c] = 1 
                elif mouse_down_right: grid[r][c] = 0 

        elif event.type == pygame.KEYDOWN and not driving:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            pos = (mouse_y // CELL, mouse_x // CELL) 
            
            if event.key == pygame.K_s: start = pos
            elif event.key == pygame.K_g: goal = pos
            elif event.key == pygame.K_RETURN and start and goal:
                path = a_star(grid, start, goal)
                if path:
                    driving = True
                    print("\n--- Path found. Generating action sequence ---")
                    action_list = generate_action_sequence(path)
                    execute_action_sequence(action_list, car)
                    driving = False 

pygame.quit()
