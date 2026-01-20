# First name Last name

from config import *
import random
from collections import deque
import heapq 

class Agent:
    
    def __init__(self, color, index):
        self.color = color
        self.index = index
        self.last_pos = None
        self.stuck_counter = 0
        
        # PATH MEMORY
        self.current_path_queue = [] 
        
        # HISTORY
        self.path_history = [] 
        self.my_spawn = None
        
        if self.color == "blue":
            self.my_flag_tile = ASCII_TILES["blue_flag"]
            self.enemy_flag_tile = ASCII_TILES["red_flag"]
            self.attack_direction = "right"
            self.return_direction = "left"
            self.attack_vector = (1, 0)
        else: # red
            self.my_flag_tile = ASCII_TILES["red_flag"]
            self.enemy_flag_tile = ASCII_TILES["blue_flag"]
            self.attack_direction = "left"
            self.return_direction = "right"
            self.attack_vector = (-1, 0)

    def update_map(self, visible_world, position, shared_knowledge):
        if 'game_map' not in shared_knowledge:
            shared_knowledge['game_map'] = {}
        if 'visited' not in shared_knowledge:
            shared_knowledge['visited'] = set()

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
                
                # Store static elements
                if char in ['#', ' ', self.enemy_flag_tile, self.my_flag_tile]:
                    shared_knowledge['game_map'][(abs_x, abs_y)] = char
                
                # Detect flag positions
                if char == self.enemy_flag_tile:
                    shared_knowledge['enemy_flag_pos'] = (abs_x, abs_y)
                
                if char == self.my_flag_tile:
                    shared_knowledge['my_flag_pos'] = (abs_x, abs_y)

    def get_full_path(self, start_pos, target_pos, shared_knowledge, holding_flag=False):
        game_map = shared_knowledge.get('game_map', {})
        visited_globally = shared_knowledge.get('visited', set())
        
        final_target = target_pos
        
        # Destination logic - return home
        if holding_flag and target_pos:
            sx, sy = target_pos
            neighbors = [(sx+1, sy), (sx-1, sy), (sx, sy+1), (sx, sy-1)]
            best_neighbor = None
            min_dist = float('inf')
            
            for nx, ny in neighbors:
                tile = game_map.get((nx, ny))
                # Walkable check
                is_walkable = (tile == ' ' or tile == self.enemy_flag_tile or (nx, ny) == start_pos or tile is None)
                if tile == '#': is_walkable = False # Walls are never walkable
                
                if is_walkable:
                    dist = abs(start_pos[0] - nx) + abs(start_pos[1] - ny)
                    if dist < min_dist:
                        min_dist = dist
                        best_neighbor = (nx, ny)
            
            if best_neighbor:
                final_target = best_neighbor
                if start_pos == final_target:
                    return [] # Arrived at neighbor

        # A* ALGORITHM
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
            
            # Exploration (Attack Mode)
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

            # Neighbor Expansion
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

        # Path Reconstruction
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

    def update(self, visible_world, position, can_shoot, holding_flag, shared_knowledge, hp, ammo):
        self.update_map(visible_world, position, shared_knowledge)
        
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
        
        # 1. Path Following
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
        
        # 2. Path Planning
        if not self.current_path_queue:
            target = None
            
            # Return Home
            if holding_flag:
                if 'my_flag_pos' in shared_knowledge:
                    target = shared_knowledge['my_flag_pos']
                else:
                    target = self.my_spawn
                
                # Force Touchdown
                # If distance to flag is 1, force a move into it to trigger win
                if target:
                    tx, ty = target
                    if abs(position[0]-tx) + abs(position[1]-ty) <= 1:
                        # Calculate direction towards the flag
                        if tx > position[0]: direction = "right"
                        elif tx < position[0]: direction = "left"
                        elif ty > position[1]: direction = "down"
                        else: direction = "up"
                        return "move", direction

            # Attack Mode
            elif 'enemy_flag_pos' in shared_knowledge:
                target = shared_knowledge['enemy_flag_pos']
            
            # Calculate Path
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
                # Fallbacks
                if holding_flag:
                     if self.path_history:
                         while self.path_history and self.path_history[-1] == position:
                             self.path_history.pop()
                         if self.path_history:
                             prev = self.path_history.pop()
                             if prev[0] > position[0]: direction = "right"
                             elif prev[0] < position[0]: direction = "left"
                             elif prev[1] > position[1]: direction = "down"
                             else: direction = "up"
                else:
                    direction = random.choice(["up", "down", "left", "right"])

        if can_shoot and random.random() > 0.8: pass
            
        return action, direction

    def terminate(self, reason):
        if reason == "died":
            print(f"{self.color} agent {self.index} died.")

