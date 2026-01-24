# First name Last name

"""
Description:
- FEATURE: STALEMATE STRATEGY & SOLO CLUTCH.
- Logic:
  1. If our flag is stolen, 'Hunters' (non-carriers) prioritize killing the enemy carrier.
  2. The 'Carrier' (our agent with flag) retreats to spawn and camps (defends) to wait for the flag return.
  3. EXCEPTION: If the Carrier is the LAST SURVIVOR (Solo), they abandon defense and hunt the enemy carrier themselves.
- Tech: Uses shared 'heartbeats' to detect active teammates and visual scanning to track the enemy carrier.
"""

from config import *
import random
from collections import deque
import heapq 
import time

class Agent:
    
    def __init__(self, color, index):
        self.color = color
        self.index = index
        self.last_pos = None
        self.stuck_counter = 0
        
        self.current_path_queue = [] 
        self.path_history = [] 
        self.my_spawn = None
        
        if self.color == "blue":
            self.my_flag_tile = ASCII_TILES["blue_flag"]
            self.enemy_flag_tile = ASCII_TILES["red_flag"]
            self.my_agents = [ASCII_TILES["blue_agent"], ASCII_TILES["blue_agent_f"]]
            self.enemy_agents = [ASCII_TILES["red_agent"], ASCII_TILES["red_agent_f"]]
            # Tko ima moju zastavu? (Crveni s 'f'lagom)
            self.enemy_with_my_flag = ASCII_TILES["red_agent_f"] 
            self.attack_vector = (1, 0)
        else: # red
            self.my_flag_tile = ASCII_TILES["red_flag"]
            self.enemy_flag_tile = ASCII_TILES["blue_flag"]
            self.my_agents = [ASCII_TILES["red_agent"], ASCII_TILES["red_agent_f"]]
            self.enemy_agents = [ASCII_TILES["blue_agent"], ASCII_TILES["blue_agent_f"]]
            # Tko ima moju zastavu? (Plavi s 'f'lagom)
            self.enemy_with_my_flag = ASCII_TILES["blue_agent_f"]
            self.attack_vector = (-1, 0)

    def update_map(self, visible_world, position, shared_knowledge):
        if 'game_map' not in shared_knowledge:
            shared_knowledge['game_map'] = {}
        if 'visited' not in shared_knowledge:
            shared_knowledge['visited'] = set()
        
        # --- HEARTBEAT SYSTEM (Tko je ziv?) ---
        if 'heartbeats' not in shared_knowledge:
            shared_knowledge['heartbeats'] = {}
        # Zapisi trenutno vrijeme kao dokaz da sam ziv
        shared_knowledge['heartbeats'][self.index] = time.time()

        center_index = 4
        my_x, my_y = position
        shared_knowledge['visited'].add(position)
        
        if self.my_spawn is None:
            self.my_spawn = position
            shared_knowledge[f'{self.color}_spawn'] = position

        for local_y in range(len(visible_world)):
            for local_x in range(len(visible_world[0])):
                char = visible_world[local_y][local_x]
                abs_x = my_x + (local_x - center_index)
                abs_y = my_y + (local_y - center_index)
                
                if char in ['#', ' ', self.enemy_flag_tile, self.my_flag_tile]:
                    shared_knowledge['game_map'][(abs_x, abs_y)] = char
                
                if char == self.enemy_flag_tile:
                    shared_knowledge['enemy_flag_pos'] = (abs_x, abs_y)
                
                if char == self.my_flag_tile:
                    shared_knowledge['my_flag_pos'] = (abs_x, abs_y)
                
                # --- DETEKCIJA KRADLJIVCA ZASTAVE ---
                # Ako vidimo neprijatelja koji nosi NASU zastavu
                if char == self.enemy_with_my_flag:
                    shared_knowledge['enemy_carrier_pos'] = (abs_x, abs_y)
                    shared_knowledge['our_flag_stolen'] = True
                
                # Ako vidimo nasu zastavu na podu/bazi, znaci da nije ukradena (reset)
                if char == self.my_flag_tile:
                    shared_knowledge['our_flag_stolen'] = False
                    if 'enemy_carrier_pos' in shared_knowledge:
                        del shared_knowledge['enemy_carrier_pos']

    def get_active_teammates_count(self, shared_knowledge):
        """Vraca broj zivih suigraca (ukljucujuci mene)."""
        if 'heartbeats' not in shared_knowledge:
            return 1
        
        count = 0
        now = time.time()
        # Smatramo agenta zivim ako se javio unutar zadnjih 0.5 sekundi
        for idx, last_seen in shared_knowledge['heartbeats'].items():
            if now - last_seen < 0.5:
                count += 1
        return count

    def get_full_path(self, start_pos, target_pos, shared_knowledge, holding_flag=False):
        game_map = shared_knowledge.get('game_map', {})
        visited_globally = shared_knowledge.get('visited', set())
        
        final_target = target_pos
        
        # --- DESTINATION LOGIC (RETURN HOME) ---
        if holding_flag and target_pos:
            sx, sy = target_pos
            neighbors = [(sx+1, sy), (sx-1, sy), (sx, sy+1), (sx, sy-1)]
            best_neighbor = None
            min_dist = float('inf')
            
            for nx, ny in neighbors:
                tile = game_map.get((nx, ny))
                is_walkable = (tile == ' ' or tile == self.enemy_flag_tile or (nx, ny) == start_pos or tile is None)
                if tile == '#': is_walkable = False
                
                if is_walkable:
                    dist = abs(start_pos[0] - nx) + abs(start_pos[1] - ny)
                    if dist < min_dist:
                        min_dist = dist
                        best_neighbor = (nx, ny)
            
            if best_neighbor:
                final_target = best_neighbor
                if start_pos == final_target:
                    return [] 

        # --- A* ALGORITHM ---
        pq = []
        heapq.heappush(pq, (0, 0, start_pos))
        came_from = {start_pos: None}
        cost_so_far = {start_pos: 0}
        
        found_target = None
        steps = 0
        MAX_STEPS = 500
        
        while pq and steps < MAX_STEPS:
            steps += 1
            _, current_cost, current = heapq.heappop(pq)
            
            if final_target and current == final_target:
                found_target = current
                break
            
            if not final_target and not holding_flag:
                x, y = current
                is_frontier = False
                for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                    if (x+dx, y+dy) not in game_map:
                        is_frontier = True
                        break
                if is_frontier:
                    ax, ay = self.attack_vector
                    dist_start = (start_pos[0]*ax + start_pos[1]*ay)
                    dist_curr = (current[0]*ax + current[1]*ay)
                    if dist_curr > dist_start:
                        found_target = current
                        break

            x, y = current
            neighbors = [((x, y-1), "up"), ((x, y+1), "down"), ((x-1, y), "left"), ((x+1, y), "right")]
            
            for next_pos, direction in neighbors:
                tile = game_map.get(next_pos)
                
                is_walkable = (tile == ' ' or tile == self.enemy_flag_tile)
                if tile == self.my_flag_tile: is_walkable = False
                if tile is None and final_target: is_walkable = False

                if is_walkable:
                    new_cost = current_cost + 1
                    if not holding_flag and next_pos in visited_globally:
                        new_cost += 5 
                    
                    if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                        cost_so_far[next_pos] = new_cost
                        priority = new_cost
                        if final_target:
                            priority += abs(final_target[0] - next_pos[0]) + abs(final_target[1] - next_pos[1])
                        else:
                            ax, ay = self.attack_vector
                            priority -= (next_pos[0]*ax + next_pos[1]*ay) * 2

                        heapq.heappush(pq, (priority, new_cost, next_pos))
                        came_from[next_pos] = current

        if found_target:
            path = []
            step = found_target
            while step != start_pos:
                path.append(step)
                step = came_from.get(step)
                if step is None: return []
            path.reverse()
            return path
        return []

    def check_for_enemy(self, visible_world):
        center = 4
        directions = [(0, -1, "up"), (0, 1, "down"), (-1, 0, "left"), (1, 0, "right")]
        
        for dx, dy, dirname in directions:
            for dist in range(1, 5):
                nx, ny = center + dx*dist, center + dy*dist
                if not (0 <= nx < 9 and 0 <= ny < 9):
                    break
                tile = visible_world[ny][nx]
                if tile == '#' or tile in self.my_agents:
                    break 
                if tile in self.enemy_agents:
                    return dirname
        return None

    def update(self, visible_world, position, can_shoot, holding_flag, shared_knowledge, hp, ammo):
        self.update_map(visible_world, position, shared_knowledge)
        
        # --- 0. STOP & SHOOT ---
        shoot_dir = self.check_for_enemy(visible_world)
        if shoot_dir:
            if can_shoot:
                return "shoot", shoot_dir
            else:
                return None, None
        
        # --- PREPARATION ---
        if self.last_pos != position:
             self.path_history.append(self.last_pos) 
             if len(self.path_history) > 300: self.path_history.pop(0)

        if self.last_pos == position:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        self.last_pos = position

        action = "move"
        direction = None
        
        # --- 1. STRATEGY SELECTION ---
        target = None
        our_flag_stolen = shared_knowledge.get('our_flag_stolen', False)
        enemy_carrier_pos = shared_knowledge.get('enemy_carrier_pos', None)
        active_teammates = self.get_active_teammates_count(shared_knowledge)
        
        # A. AKO IMAM ZASTAVU
        if holding_flag:
            
            # SCENARIJ 1: Nasa zastava je UKRADENA (Stalemate)
            if our_flag_stolen:
                
                # a) SOLO MODE: Ako sam zadnji prezivjeli, ja moram biti heroj
                if active_teammates == 1:
                    if enemy_carrier_pos:
                        target = enemy_carrier_pos
                    elif 'enemy_spawn' in shared_knowledge:
                         target = shared_knowledge['enemy_spawn']
                
                # b) TEAM MODE: Ima jos zivih
                else:
                    target = self.my_spawn
                    if target:
                        tx, ty = target
                        if abs(position[0]-tx) + abs(position[1]-ty) <= 2:
                            return None, None # CAMPING
            
            # SCENARIJ 2: Nasa zastava je SIGURNA
            else:
                if 'my_flag_pos' in shared_knowledge:
                    target = shared_knowledge['my_flag_pos']
                else:
                    target = self.my_spawn
                    
                if target:
                    tx, ty = target
                    if abs(position[0]-tx) + abs(position[1]-ty) <= 1:
                        if tx > position[0]: direction = "right"
                        elif tx < position[0]: direction = "left"
                        elif ty > position[1]: direction = "down"
                        else: direction = "up"
                        return "move", direction

        # B. AKO NEMAM ZASTAVU (Hunter / Attacker)
        else:
            
            # --- NOVO: RESUPPLY PRIORITY ---
            # Provjeri jesmo li vec na spawnu (unutar radijusa 2)
            is_at_home = False
            if self.my_spawn:
                sx, sy = self.my_spawn
                if abs(position[0]-sx) + abs(position[1]-sy) <= 2:
                    is_at_home = True

            # Uvjet za povratak: Prazan ammo ILI Kriticni HP
            # Uvjet za ostanak: Ako sam vec doma, ostani dok se ne napunim skroz (HP<3 ili Ammo<10)
            needs_resupply = (ammo == 0 or hp <= 1)
            still_recharging = (is_at_home and (ammo < 10 or hp < 3))

            if needs_resupply or still_recharging:
                target = self.my_spawn
                # Ako smo stigli blizu spawna (radijus 2), stani i puni se
                if is_at_home:
                    return None, None
            
            # --- STANDARDNA LOGIKA (samo ako ne trebam resupply) ---
            else:
                if our_flag_stolen and enemy_carrier_pos:
                    target = enemy_carrier_pos
                elif 'enemy_flag_pos' in shared_knowledge:
                    target = shared_knowledge['enemy_flag_pos']

        # --- 2. PATH EXECUTION & PLANNING ---
        if our_flag_stolen and not holding_flag and enemy_carrier_pos:
             self.current_path_queue = [] 

        if self.stuck_counter > 2:
            self.current_path_queue = [] 
        
        if self.current_path_queue:
            next_step = self.current_path_queue[0]
            if abs(next_step[0] - position[0]) + abs(next_step[1] - position[1]) != 1:
                self.current_path_queue = [] 
            else:
                self.current_path_queue.pop(0)
                nx, ny = next_step
                px, py = position
                if nx > px: direction = "right"
                elif nx < px: direction = "left"
                elif ny > py: direction = "down"
                else: direction = "up"
                return action, direction
        
        if not self.current_path_queue:
            if not target:
                if 'enemy_flag_pos' in shared_knowledge:
                    target = shared_knowledge['enemy_flag_pos']
            
            new_path = self.get_full_path(position, target, shared_knowledge, holding_flag)
            
            if new_path:
                self.current_path_queue = new_path
                next_step = self.current_path_queue.pop(0)
                nx, ny = next_step
                px, py = position
                if nx > px: direction = "right"
                elif nx < px: direction = "left"
                elif ny > py: direction = "down"
                else: direction = "up"
            else:
                direction = random.choice(["up", "down", "left", "right"])
            
        return action, direction

    def terminate(self, reason):
        if reason == "died":
            print(f"{self.color} agent {self.index} died.")
