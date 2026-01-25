# Karlo Vlašić
# Petar Car

from config import *
import random
from collections import deque
import heapq 
import time

class Agent:
    
    # Inicijalizacija agenta
    def __init__(self, color, index):
        self.color = color
        self.index = index
        self.last_pos = None
        self.stuck_counter = 0
        
        self.current_path_queue = [] 
        self.path_history = [] 
        self.my_spawn = None
        
        # Plavi tim
        if self.color == "blue":
            self.my_flag_tile = ASCII_TILES["blue_flag"]
            self.enemy_flag_tile = ASCII_TILES["red_flag"]
            self.my_agents = [ASCII_TILES["blue_agent"], ASCII_TILES["blue_agent_f"]]
            self.enemy_agents = [ASCII_TILES["red_agent"], ASCII_TILES["red_agent_f"]]
            self.enemy_with_my_flag = ASCII_TILES["red_agent_f"] 
            self.attack_vector = (1, 0)
        # Crveni tim
        else: 
            self.my_flag_tile = ASCII_TILES["red_flag"]
            self.enemy_flag_tile = ASCII_TILES["blue_flag"]
            self.my_agents = [ASCII_TILES["red_agent"], ASCII_TILES["red_agent_f"]]
            self.enemy_agents = [ASCII_TILES["blue_agent"], ASCII_TILES["blue_agent_f"]]
            self.enemy_with_my_flag = ASCII_TILES["blue_agent_f"]
            self.attack_vector = (-1, 0)

    # Azurira zajednicko znanje tima na temelju onoga sto agent trenutno vidi
    # Otkriva zidove, biljezi pozicije zastava i salje signal da je agent ziv (heartbeat)
    def update_map(self, visible_world, position, shared_knowledge):
        if 'game_map' not in shared_knowledge:
            shared_knowledge['game_map'] = {}
        if 'visited' not in shared_knowledge:
            shared_knowledge['visited'] = set()
        
        # Heartbeat - svaki agent zapise trenutno vrijeme u rjecnik
        # Sluzi da ostali agenti znaju koliko nas je jos zivih
        if 'heartbeats' not in shared_knowledge:
            shared_knowledge['heartbeats'] = {}
        shared_knowledge['heartbeats'][self.index] = time.time()

        center_index = 4
        my_x, my_y = position
        shared_knowledge['visited'].add(position)
        
        # Spremanje spawn lokacije ako jos nije poznata
        if self.my_spawn is None:
            self.my_spawn = position
            shared_knowledge[f'{self.color}_spawn'] = position

        # Skeniranje vidnog polja (9x9 mreza oko agenta)
        for local_y in range(len(visible_world)):
            for local_x in range(len(visible_world[0])):
                char = visible_world[local_y][local_x]
                abs_x = my_x + (local_x - center_index)
                abs_y = my_y + (local_y - center_index)
                
                # Spremanje statickih objekata (zidovi, zastave) u globalnu mapu
                if char in ['#', ' ', self.enemy_flag_tile, self.my_flag_tile]:
                    shared_knowledge['game_map'][(abs_x, abs_y)] = char
                
                # Detekcija pozicije neprijateljske zastave
                if char == self.enemy_flag_tile:
                    shared_knowledge['enemy_flag_pos'] = (abs_x, abs_y)
                
                # Detekcija pozicije vlastite zastave
                if char == self.my_flag_tile:
                    shared_knowledge['my_flag_pos'] = (abs_x, abs_y)
                
                # Detekcija kradje: Ako vidimo neprijatelja s nasom zastavom
                # Sprema njegovu poziciju i oznacava da je zastava ukradena
                if char == self.enemy_with_my_flag:
                    shared_knowledge['enemy_carrier_pos'] = (abs_x, abs_y)
                    shared_knowledge['our_flag_stolen'] = True
                
                # Ako vidimo nasu zastavu na njenom mjestu, znaci da nije ukradena
                # Brise info o kradji i poziciji neprijateljskog nositelja
                if char == self.my_flag_tile:
                    shared_knowledge['our_flag_stolen'] = False
                    if 'enemy_carrier_pos' in shared_knowledge:
                        del shared_knowledge['enemy_carrier_pos']

    # Izracunava koliko je suigraca trenutno zivo provjerom heartbeat vremena
    # Ako je zadnji signal bio prije manje od 0.5 sekundi, agent se smatra zivim
    def get_active_teammates_count(self, shared_knowledge):
        if 'heartbeats' not in shared_knowledge:
            return 1
        
        count = 0
        now = time.time()
        for idx, last_seen in shared_knowledge['heartbeats'].items():
            if now - last_seen < 0.5:
                count += 1
        return count

    # Glavna funkcija za pronalazenje puta (Pathfinding) koristeci A* algoritam
    # Vraca listu koraka od start_pos do target_pos
    def get_full_path(self, start_pos, target_pos, shared_knowledge, holding_flag=False):
        game_map = shared_knowledge.get('game_map', {})
        visited_globally = shared_knowledge.get('visited', set())
        
        final_target = target_pos
        
        # Logika za povratak sa zastavom
        if holding_flag and target_pos:
            sx, sy = target_pos
            neighbors = [(sx+1, sy), (sx-1, sy), (sx, sy+1), (sx, sy-1)]
            best_neighbor = None
            min_dist = float('inf')
            
            # Provjera jesu li susjedna polja prohodna za kretanje, osiguravajuci da se agent ne pokuša kretati kroz zidove
            for nx, ny in neighbors:
                tile = game_map.get((nx, ny))
                is_walkable = (tile == ' ' or tile == self.enemy_flag_tile or (nx, ny) == start_pos or tile is None)
                if tile == '#': is_walkable = False
                
                if is_walkable:
                    # Manhattan udaljenost do startne pozicije
                    dist = abs(start_pos[0] - nx) + abs(start_pos[1] - ny) 
                    if dist < min_dist: # Nadi najblizi susjedni cvor
                        min_dist = dist
                        best_neighbor = (nx, ny)

            # postavi najblizi susjedni cvor (od zastave) kao cilj, ako agent stoji na tom polju,
            # odmah prekida potragu za putem jer je cilj postignut
            if best_neighbor:
                final_target = best_neighbor
                if start_pos == final_target:
                    return [] 

        # A* Algoritam inicijalizacija
        pq = []
        heapq.heappush(pq, (0, 0, start_pos))
        came_from = {start_pos: None}
        cost_so_far = {start_pos: 0}
        
        found_target = None
        steps = 0
        MAX_STEPS = 500
        
        # Glavna petlja A* algoritma
        while pq and steps < MAX_STEPS:
            steps += 1
            _, current_cost, current = heapq.heappop(pq)
            
            if final_target and current == final_target:
                found_target = current
                break
            
            # Ako nema specificnog cilja, trazimo frontier polja (rubove mape)
            if not final_target and not holding_flag:
                x, y = current
                is_frontier = False
                for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]: # Provjera susjednih polja
                    if (x+dx, y+dy) not in game_map: # Ako je susjedno polje neotkriveno
                        is_frontier = True # Postavi kao frontier
                        break
                if is_frontier: # Ako smo na frontieru
                    # Preferiraj rubove u smjeru napada
                    ax, ay = self.attack_vector
                    dist_start = (start_pos[0]*ax + start_pos[1]*ay)
                    dist_curr = (current[0]*ax + current[1]*ay)
                    if dist_curr > dist_start:
                        found_target = current 
                        break
            
            # Istraživanje susjednih polja
            x, y = current
            neighbors = [((x, y-1), "up"), ((x, y+1), "down"), ((x-1, y), "left"), ((x+1, y), "right")]
            
            # Evaluacija susjednih polja
            for next_pos, direction in neighbors:
                tile = game_map.get(next_pos)

                # Ako je polje prohodno
                is_walkable = (tile == ' ' or tile == self.enemy_flag_tile)
                if tile == self.my_flag_tile: is_walkable = False  # Ne prolazi kroz vlastitu zastavu
                if tile is None and final_target: is_walkable = False # Ne prolazi kroz neotkrivena polja ako ima cilj

                if is_walkable: # Ako je polje prohodno
                    new_cost = current_cost + 1 # svaki korak kosta za 1 vise
                    # Ako agent ne nosi zastavu, poskupljujemo kretanje kroz vec posjecena polja za 5
                    # kako bismo potaknuli istrazivanje novih podrucja
                    
                    if not holding_flag and next_pos in visited_globally:
                        new_cost += 5 
                    
                    # Ako je ovo najbolji put do next_pos, spremi ga
                    if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                        cost_so_far[next_pos] = new_cost # Azuriraj najbolji trosak do next_pos
                        priority = new_cost
                        # Manhattan udaljenost do cilja ili smjer napada
                        if final_target:
                            priority += abs(final_target[0] - next_pos[0]) + abs(final_target[1] - next_pos[1])
                        else: # Nema cilja, preferiraj kretanje u smjeru napada
                            ax, ay = self.attack_vector
                            # preferiraj napredak u smjeru napada; heap uzima najmanju vrijednost kao prioritet
                            priority -= (next_pos[0]*ax + next_pos[1]*ay) * 2 

                        heapq.heappush(pq, (priority, new_cost, next_pos))
                        came_from[next_pos] = current

        # Rekonstrukcija puta unatrag
        if found_target:
            path = []
            step = found_target
            while step != start_pos:
                path.append(step)
                step = came_from.get(step) # Korak unatrag
                if step is None: return []
            path.reverse()
            return path
        return []

    # Provjerava vidno polje za neprijateljima u liniji paljbe (gore, dolje, lijevo, desno)
    # Vraca smjer u kojem treba pucati ako je neprijatelj vidljiv i nema prepreka
    def check_for_enemy(self, visible_world):
        center = 4 # Centar 9x9 vidnog polja
        directions = [(0, -1, "up"), (0, 1, "down"), (-1, 0, "left"), (1, 0, "right")]
        
        for dx, dy, dirname in directions: # Provjera u 4 smjera
            for dist in range(1, 5):
                nx, ny = center + dx*dist, center + dy*dist # Koordinate koje provjeravamo
                # Prekid ako izlazimo iz vidnog polja
                if not (0 <= nx < 9 and 0 <= ny < 9):
                    break
                tile = visible_world[ny][nx] # Tile na tim koordinatama
                # Prekid ako je zid ili nas igrac na putu
                if tile == '#' or tile in self.my_agents:
                    break 
                # Nasli neprijatelja
                if tile in self.enemy_agents:
                    return dirname
        return None

    # Glavna metoda koja se poziva svaki tik igre
    # Odreduje se akcija agenta
    def update(self, visible_world, position, can_shoot, holding_flag, shared_knowledge, hp, ammo):
        self.update_map(visible_world, position, shared_knowledge)
        
        # Stop & Shoot logika: Ako vidimo neprijatelja, a ne mozemo pucati (cooldown),
        # stani na mjestu da se cooldown resetira.
        shoot_dir = self.check_for_enemy(visible_world) # Provjera za neprijatelja u liniji paljbe
        if shoot_dir:
            if can_shoot: # Ako mozemo pucati
                return "shoot", shoot_dir # Pucaj u smjeru neprijatelja
            else:
                return None, None # Stani na mjestu da se cooldown resetira
        
        # Odrzavanje povijesti kretanja i detekcija zaglavljivanja
        if self.last_pos != position:
             self.path_history.append(self.last_pos) # Dodaj staru poziciju u povijest
             if len(self.path_history) > 300: self.path_history.pop(0) # Ogranicenje povijesti na 300 zapisa

        if self.last_pos == position: # Ako se nismo pomaknuli
            self.stuck_counter += 1 # Povecaj brojac zaglavljivanja
        else:
            self.stuck_counter = 0 # Resetiraj brojac ako smo se pomaknuli
        self.last_pos = position # Azuriraj zadnju poziciju

        action = "move"
        direction = None
        
        # Odabir strategije i cilja
        target = None
        our_flag_stolen = shared_knowledge.get('our_flag_stolen', False) # Provjera da li je nasa zastava ukradena
        enemy_carrier_pos = shared_knowledge.get('enemy_carrier_pos', None) # Pozicija neprijateljskog nositelja nase zastave
        active_teammates = self.get_active_teammates_count(shared_knowledge) # Broj zivih suigraca
        
        # A. Ako agent nosi zastavu
        if holding_flag:
            
            # Scenarij 1: Zastava je ukradena (Stalemate)
            if our_flag_stolen:
                # Ako je samo jedan agent ziv, lovi neprijatelja sa zastavom
                if active_teammates == 1:
                    if enemy_carrier_pos: # Ako znamo poziciju neprijateljskog nositelja
                        target = enemy_carrier_pos
                    elif 'enemy_spawn' in shared_knowledge: # Ako ne znamo, idi na spawn neprijatelja
                         target = shared_knowledge['enemy_spawn']
                
                # Ako je barem jos jedan agent ziv, kampiraj na spawn lokaciji
                else:
                    target = self.my_spawn
                    if target:
                        tx, ty = target
                        if abs(position[0]-tx) + abs(position[1]-ty) <= 2:
                            return None, None # CAMPING
            
            # Scenarij 2: Zastava NIJE ukradena
            else:
                if 'my_flag_pos' in shared_knowledge:
                    target = shared_knowledge['my_flag_pos']
                else:
                    target = self.my_spawn
                    
                # Force Touchdown: Prisilno kretanje u zastavu za pobjedu
                if target:
                    tx, ty = target
                    if abs(position[0]-tx) + abs(position[1]-ty) <= 1:
                        if tx > position[0]: direction = "right"
                        elif tx < position[0]: direction = "left"
                        elif ty > position[1]: direction = "down"
                        else: direction = "up"
                        return "move", direction

        # B. Ako agent NE nosi zastavu (Lovac/Napadac)
        else:
            
            # Provjera za punjenje municije (Resupply)
            is_at_home = False
            if self.my_spawn:
                sx, sy = self.my_spawn
                # Ako smo 2 koraka blizu spawna, smatramo da smo kod kuce za resupply
                if abs(position[0]-sx) + abs(position[1]-sy) <= 2:
                    is_at_home = True
            
            # uvjeti za resupply
            needs_resupply = (ammo == 0 or hp <= 1)
            still_recharging = (is_at_home and (ammo < 10 or hp < 3))

            # Ako treba punjenje, idi na spawn i ostani tamo
            if needs_resupply or still_recharging:
                target = self.my_spawn
                if is_at_home: # Ako smo kod kuce, ostani tamo
                    return None, None
            
            # Ako ne treba punjenje, nastavi s napadom
            else:
                # Prioritet: Povrat nase ukradene zastave
                if our_flag_stolen and enemy_carrier_pos:
                    target = enemy_carrier_pos
                # Inace: Napad na neprijateljsku zastavu
                elif 'enemy_flag_pos' in shared_knowledge:
                    target = shared_knowledge['enemy_flag_pos']

        # Izvrsavanje puta (Path Execution)
        # Ako se cilj stalno mice (neprijatelj), resetiraj put
        if our_flag_stolen and not holding_flag and enemy_carrier_pos:
             self.current_path_queue = [] 

        # Ako smo zaglavljeni, resetiraj put
        if self.stuck_counter > 2:
            self.current_path_queue = [] 
        
        # Ako imamo put, slijedi ga
        if self.current_path_queue:
            next_step = self.current_path_queue[0]
            # Provjera da li je sljedeci korak validan (susjedan cvor)
            if abs(next_step[0] - position[0]) + abs(next_step[1] - position[1]) != 1:
                self.current_path_queue = [] # Resetiraj put ako nije validan
            else:
                self.current_path_queue.pop(0) # Ukloni korak iz queuea
                nx, ny = next_step # Sljedeci korak
                px, py = position # Trenutna pozicija

                ## Odredi smjer kretanja
                if nx > px: direction = "right"
                elif nx < px: direction = "left"
                elif ny > py: direction = "down"
                else: direction = "up"
                return action, direction # Vrati akciju i smjer
        
        # Ako nemamo put, izracunaj novi pomocu A*
        if not self.current_path_queue:
            if not target: # Ako nemamo cilj, zastava neprijatelja je prioritet
                if 'enemy_flag_pos' in shared_knowledge:
                    target = shared_knowledge['enemy_flag_pos']
            
            # Pozovi A* algoritam za novi put
            new_path = self.get_full_path(position, target, shared_knowledge, holding_flag)
            
            if new_path: # Ako je pronaden put
                self.current_path_queue = new_path # Spremi put u queue
                next_step = self.current_path_queue.pop(0) # Uzmi sljedeci korak
                nx, ny = next_step
                px, py = position

                # Odredi smjer kretanja
                if nx > px: direction = "right"
                elif nx < px: direction = "left"
                elif ny > py: direction = "down"
                else: direction = "up"
            else: # Nema pronadenog puta, random kretanje
                direction = random.choice(["up", "down", "left", "right"])
            
        return action, direction # Vrati akciju i smjer

    def terminate(self, reason):
        if reason == "died":
            print(f"{self.color} agent {self.index} died.")
