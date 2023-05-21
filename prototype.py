"""
    3D Game
"""
################ 3D Game ######################################################
#region
import tkinter as tk
import math
import random
#endregion
################ Type Aliases   ###############################################
#region
vec2 = tuple[float, float]
ivec2 = tuple[int, int]
line_segment = tuple[vec2, vec2]
#endregion
################ Constants     ################################################
#region
#0: debug, 1: development, 2:play
MODE = 0
SCREEN_WIDTH = 450
SCREEN_HEIGHT = 300
CENTER = (SCREEN_WIDTH//2,SCREEN_HEIGHT//2)
NEAR_PLANE = ((-1, -0.01), (1, -0.01))
SPAWN_RATE = 1.0

DRAKE_BODY = 0
DRAKE_FACE = 1
DRAKE_MISC = 2

DRAKE_MODEL = {
    DRAKE_BODY: (
        ((-1.5, 0.0,  9.0), ( 0.0, 0.0, 10.0)),
        (( 0.0, 0.0, 10.0), (-0.5, 0.0,  9.0)),
        ((-0.5, 0.0,  9.0), ( 0.0, 0.0,  5.0)),
        (( 0.0, 0.0,  5.0), ( 2.0, 0.0,  5.0)),
        (( 2.0, 0.0,  5.0), ( 1.0, 0.0,  3.0)),
        (( 1.0, 0.0,  3.0), (-1.5, 0.0,  5.0)),
        ((-1.5, 0.0,  5.0), (-1.5, 0.0,  9.0)),
    ),

    DRAKE_FACE: (
        ((-1.25, 0.0, 8.0), (-0.75, 0.0, 8.1)),
        ((-0.75, 0.0, 8.1), (-0.75, 0.0, 7.0)),
        ((-0.75, 0.0, 7.0), (-1.25, 0.0, 6.9)),
    ),

    DRAKE_MISC: (
        (( -1.1, 0.0, 7.8), (-1.05, 0.0, 7.8)), #left eye
        ((-0.95, 0.0, 7.9), ( -0.9, 0.0, 7.9)), #right eye
        ((-1.15, 0.0, 7.2), (-0.85, 0.0, 7.2)), #mouth
        (( -0.5, 0.0, 4.2), ( -0.5, 0.0, 0.0)), #left leg
        ((  0.5, 0.0, 3.4), (  0.5, 0.0, 0.0)), #right leg
    )
}

DRAKE_COLORS = {
    DRAKE_BODY: "yellow",
    DRAKE_FACE: "brown",
    DRAKE_MISC: "white",
}
#endregion
################ Helper Functions #############################################
#region
def round(point: vec2) -> ivec2:
    return (int(point[0]),int(point[1]))

def translate(point: vec2, translation: vec2) -> vec2:
    (x,y) = point
    (dx,dy) = translation
    return (x + dx,y + dy)

def rotate_point(point: vec2, angle: float) -> vec2:
    (x,y) = point
    theta = math.radians(angle)
    rotated_x = x*math.cos(theta) + y*math.sin(theta)
    rotated_y = -x*math.sin(theta) + y*math.cos(theta)
    return (rotated_x,rotated_y)

def clip_line(line_a: line_segment,line_b: line_segment) -> line_segment:
    ((x1,y1),(x2,y2)) = line_a
    ((x3,y3),(x4,y4)) = line_b

    num_a = (x1*y2 - y1*x2)
    num_b = (x3*y4 - y3*x4)
    den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
    x = (num_a*(x3 - x4) - (x1 - x2)*num_b)/den
    y = (num_a*(y3 - y4) - (y1 - y2)*num_b)/den
    return (x,y)

def scale(point: vec2, factor_x: float, factor_y: float = None) -> vec2:

    if factor_y==None:
        factor_y = factor_x
    (x,y) = point
    return (x*factor_x,y*factor_y)

def dot_product(u: vec2, v: vec2) -> float:
    (u1,u2) = u
    (v1,v2) = v
    return u1*v1 + u2*v2

def quick_distance(pos_a: vec2, pos_b: vec2) -> float:
    dx = abs(pos_a[0] - pos_b[0])
    dy = abs(pos_a[1] - pos_b[1])
    return dx + dy

def near(a: float, b: float) -> bool:

    return abs(a - b) < 0.01

def world_to_view_transform(
    point: vec2,
    camera_position: vec2, 
    camera_direction: float) -> vec2:
    """ apply world to view coordinate transformation """

    #subtract camera position
    cam = (-camera_position[0], -camera_position[1])
    pos_view = translate(point, cam)

    #rotate 90 degrees counter clockwise, 
    # then opposite camera motion
    opposite_cam = 90 - camera_direction
    return rotate_point(pos_view,opposite_cam)

def view_to_screen_transform(
    pos_a: vec2, pos_b: vec2, 
    z_bottom: float, z_top: float, z_camera: float) -> list[ivec2] | None:
    
    #fetch the top-down coordinates
    (x_a,depth_a) = pos_a
    (x_b,depth_b) = pos_b

    if depth_a >= 0 and depth_b >= 0:
        #Both endpoints behind player
        return None

    if depth_a >= 0:
        #Endpoint A behind player, clip it.
        (x_a,depth_a) = clip_line((pos_a, pos_b), NEAR_PLANE)

    if depth_b >= 0:
        #Endpoint B behind player, clip it.
        (x_b,depth_b) = clip_line((pos_a, pos_b), NEAR_PLANE)

    #Depth correction, work out height for top and bottom,
    #then divide by depth
    depth_a *= -1
    depth_a = max(depth_a,0.01)
    x_a = x_a / depth_a
    top_a = -(z_top - z_camera)/depth_a
    bottom_a = -(z_bottom - z_camera)/depth_a

    depth_b *= -1
    depth_b = max(depth_b,0.01)
    x_b = x_b / depth_b
    top_b = -(z_top - z_camera) / depth_b
    bottom_b = -(z_bottom - z_camera) / depth_b

    points = [
                (x_a,top_a),
                (x_b,top_b),
                (x_b,bottom_b),
                (x_a,bottom_a)
            ]
    
    for i in range(len(points)):
        points[i] = scale(points[i],SCREEN_WIDTH//2,SCREEN_HEIGHT//2)
        points[i] = round(translate(points[i],CENTER))

    return points

def view_to_screen_transform_simple(
    pos_a: vec2, pos_b: vec2, 
    z_a: float, z_b: float, z_camera: float) -> list[ivec2] | None:
    
    #fetch the top-down coordinates
    (x_a,depth_a) = pos_a
    (x_b,depth_b) = pos_b

    if depth_a >= 0 and depth_b >= 0:
        #Both endpoints behind player
        return None

    if depth_a >= 0:
        #Endpoint A behind player, clip it.
        (x_a,depth_a) = clip_line((pos_a, pos_b), NEAR_PLANE)

    if depth_b >= 0:
        #Endpoint B behind player, clip it.
        (x_b,depth_b) = clip_line((pos_a, pos_b), NEAR_PLANE)

    #Depth correction, work out height for top and bottom,
    #then divide by depth
    depth_a *= -1
    depth_a = max(depth_a,0.01)
    x_a = x_a / depth_a
    y_a = -(z_a - z_camera)/depth_a

    depth_b *= -1
    depth_b = max(depth_b,0.01)
    x_b = x_b / depth_b
    y_b = -(z_b - z_camera) / depth_b

    points = [
                (x_a,y_a),
                (x_b,y_b)
            ]
    
    for i in range(len(points)):
        points[i] = scale(points[i],SCREEN_WIDTH//2,SCREEN_HEIGHT//2)
        points[i] = round(translate(points[i],CENTER))

    return points
#endregion
################ Model   ######################################################
#region

class Entity:


    def __init__(self, x: float, y: float, z: float, height: float, size: float):

        self._position = (x, y)
        self._z = z
        self._height = height
        self._size = size

    def get_position(self) -> vec2:

        return self._position
    
    def set_position(self, new_position: vec2) -> None:

        self._position = new_position
    
    def get_top(self) -> float:

        return self._z + self._height
    
    def get_bottom(self) -> float:

        return self._z
    
    def get_size(self) -> float:

        return self._size

class Player(Entity):

    def __init__(self,x,y,direction):

        super().__init__(x=x,y=y,z=0,height=30, size=12)
        self.direction = direction
        self.room: Room = None
        self.speed = 2
        self.energy = 0
        self.sector: Sector = None

    def setRoom(self,newRoom):
        self.room = newRoom
        self.recalculateSector()

    def recalculateSector(self):
        for s in self.room.getSectors():
            if s.inSector(self._position):
                self.sector = s
                break

    def move(self,dx,dy):
        #check movement in x and y direction separately
        temp = [0,0]

        check = (dx,0)
        could_move_to = translate(self._position,check)
        if not self.sector.hitWall(could_move_to,self._size,check):
            temp[0] = dx

        check = (0,dy)
        could_move_to = translate(self._position,check)
        if not self.sector.hitWall(could_move_to,self._size,check):
            temp[1] = dy

        self._position = translate(self._position,temp)
        self.sector = self.sector.newSector(self._position)
        if self.sector is None:
            #may have crossed a door!
            #select nearest door
            for d in self.room.doors:
                if quick_distance(self._position,d.mid)<=32:
                    self.setRoom(d.getRoom(self._position))
                    break

class Wall:


    def __init__(self, pos_a: vec2, pos_b: vec2, backface_visible:bool = False):
        
        self.pos_a = pos_a
        self.pos_b = pos_b
        self.backface_visible = backface_visible
        self.z = 0
        self.height = 80
        self.tag = "wall"

        #calculate normal
        dx = self.pos_b[0]-self.pos_a[0]
        dy = self.pos_b[1]-self.pos_a[1]
        self.normal = (0,0)
        if dx==0:
            #vertical wall
            self.normal = (dy/abs(dy),0)
        else:
            #horizontal wall
            self.normal = (0,-dx/abs(dx))

    def getLine(self) -> line_segment:
        return (self.pos_a,self.pos_b)

class Door(Wall):

    def __init__(self, 
        pos_a: vec2, pos_b: vec2, 
        room_lu: "Room", room_rd: "Room"):

        super().__init__(pos_a,pos_b, True)
        self.room_lu = room_lu
        self.room_lu.addDoor(self)
        self.room_rd = room_rd
        self.room_rd.addDoor(self)
        self.is_open = False
        self.mid = scale(translate(self.pos_a,self.pos_b),0.5)

    def getRoom(self, pos: vec2) -> "Room":

        if self.normal[0]==0:
            #horizontal door
            if pos[1] < self.pos_a[1]:
                return self.room_lu
            else:
                return self.room_rd
        else:
            #vertical door
            if pos[0] < self.pos_a[0]:
                return self.room_lu
            else:
                return self.room_rd

    def open(self) -> None:
        self.room_lu.activate()
        self.room_rd.activate()
        self.is_open = True

    def close(self, player_position: vec2) -> None:

        self.is_open = False
        if self.normal[0]==0:
            #horizontal door
            if player_position[1] < self.pos_a[1]:
                self.room_rd.deactivate()
            else:
                self.room_lu.deactivate()
        else:
            #vertical door
            if player_position[0] < self.pos_a[0]:
                self.room_rd.deactivate()
            else:
                self.room_lu.deactivate()

    def update(self, player_position: vec2) -> None:

        if quick_distance(self.mid, player_position) <= 32:
            if not self.is_open:
                self.open()
        elif self.is_open:
            self.close(player_position)

class Sector:

    def __init__(
        self, pos: vec2, 
        size: vec2, sides: list[bool]):

        self.position = pos
        self.size = size
        self.sides = sides
        self.tag = ""

        self.pos_a = self.position
        self.pos_b = (self.position[0],self.position[1]+self.size[1])
        self.pos_c = (self.position[0]+self.size[0],self.position[1]+self.size[1])
        self.pos_d = (self.position[0]+self.size[0],self.position[1])

        #meta-data
        self.walls: list[Wall] = []
        self.drake_nanas: list[Entity] = []
        self.connects_ab = None
        self.connects_bc = None
        self.connects_cd = None
        self.connects_da = None
        #construct walls
        if sides[0]:
            #north
            self.walls.append(Wall(self.pos_d,self.pos_a))
        if sides[1]:
            #east
            self.walls.append(Wall(self.pos_c,self.pos_d))
        if sides[2]:
            #south
            self.walls.append(Wall(self.pos_b,self.pos_c))
        if sides[3]:
            #west
            self.walls.append(Wall(self.pos_a,self.pos_b))
        
        self.spawn_drake_nanas()
    
    def spawn_drake_nanas(self) -> None:

        if random.uniform(a=0.0, b=1.0) < SPAWN_RATE:
            x = self.pos_a[0] + random.uniform(a=0.0, b=self.size[0])
            y = self.pos_a[1] + random.uniform(a=0.0, b=self.size[1])
            self.drake_nanas.append(
                Entity(x=x, y=y, z=0, height = 40, size=12))

    def getCorners(self) -> tuple[vec2]:

        return (
                    self.pos_a,
                    self.pos_b,
                    self.pos_c,
                    self.pos_d
                )

    def inSector(self, pos: vec2) -> bool:

        if pos[0] < self.pos_a[0]:
            return False
        if pos[0] > self.pos_c[0]:
            return False
        if pos[1] < self.pos_a[1]:
            return False
        if pos[1] > self.pos_c[1]:
            return False
        return True
    
    def newSector(self, pos: vec2) -> "Sector":

        #west
        if pos[0] < self.pos_a[0]:
            return self.connects_ab
        #east
        if pos[0] > self.pos_c[0]:
            return self.connects_cd
        #north
        if pos[1] < self.pos_a[1]:
            return self.connects_da
        #south
        if pos[1] > self.pos_c[1]:
            return self.connects_bc
        return self
    
    def hitWall(self, pos: vec2, size: vec2, velocity: vec2) -> bool:

        (vx,vy) = velocity
        if vx<0:
            #west
            west = pos[0] - size
            if west < self.pos_a[0] and self.sides[3]:
                return True
        elif vx>0:
            #east
            east = pos[0] + size
            if east > self.pos_c[0] and self.sides[1]:
                return True
        
        if vy<0:
            #north
            north = pos[1] - size
            if north < self.pos_a[1] and self.sides[0]:
                return True
        elif vy>0:
            #south
            south = pos[1] + size
            if south > self.pos_c[1] and self.sides[2]:
                return True
        return False

    def is_connected(self) -> bool:

        return not(
            (self.connects_ab is None and not self.sides[0]) \
            or (self.connects_bc is None and not self.sides[1]) \
            or (self.connects_cd is None and not self.sides[2]) \
            or (self.connects_da is None and not self.sides[3])
        )

class Room:


    def __init__(self):

        self.sectors: list[Sector] = []
        self.tag = ""
        self.doors: list[Door] = []
        self.active = False

    def addSector(self, sector: Sector) -> None:

        if sector not in self.sectors:
            self.sectors.append(sector)

    def addDoor(self, door: Door) -> None:

        if door not in self.doors:
            self.doors.append(door)

    def activate(self) -> None:
        
        self.active = True

    def deactivate(self) -> None:
        
        self.active = False

    def getSectors(self) -> list[Sector]:

        return self.sectors

    def update(self, player_position: vec2) -> None:

        for door in self.doors:
            door.update(player_position)

class Scene:

    def __init__(self, filename: str):

        self.rooms: list[Room] = []
        self.active_rooms: list[Room] = []
        self.sectors: list[Sector] = []
        self.unconnected_sectors: list[Sector] = []
        
        self.import_data(filename)

        self.unconnected_sectors = []
    
    def import_data(self, filename):

        with open(filename,'r') as f:
            line:str = f.readline()
            while line:
                tag,_,rest = line.partition("(")
                parameters,_,_ = rest.partition(")")
                parameters = parameters.split(",")

                match tag[0]:
                    case "r":
                        self.add_room(tag)
                    case 's':
                        self.add_sector(tag, parameters)
                    case 'd':
                        self.add_door(tag, parameters)
                    case 'p':
                        self.add_player(tag, parameters)
                
                line = f.readline()
    
    def add_room(self, tag: str):

        #room
        # r()
        r = Room()
        self.rooms.append(r)
        r.tag = tag
    
    def add_sector(self, tag: str, parameters: list[str]):
        
        #sector
        # s(x,y,width,height,n,e,s,w,room)
        x      = 32*float(parameters[0])
        y      = 32*(50-float(parameters[1]))
        width  = 32*float(parameters[2])
        height = 32*float(parameters[3])
        n      = int(parameters[4])
        e      = int(parameters[5])
        s      = int(parameters[6])
        w      = int(parameters[7])
        room   = parameters[8]
        pos    = (x, y)
        size   = (width, height)
        sides  = (n,e,s,w)

        sector = Sector(pos,size,sides)
        self.find_room(room).addSector(sector)
        sector.tag = tag
        self.sectors.append(sector)
        self.unconnected_sectors.append(sector)
        self.connect_sector(sector)
    
    def add_door(self, tag: str, parameters: list[str]):

        #door
        # s(x_a,y_a,x_b,y_b,room_lu,room_rd)
        x_a     = 32*float(parameters[0])
        y_a     = 32*(50-float(parameters[1]))
        x_b     = 32*float(parameters[2])
        y_b     = 32*(50-float(parameters[3]))
        room_lu = self.find_room(parameters[4])
        room_rd = self.find_room(parameters[5])
        pos_a = (x_a, y_a)
        pos_b = (x_b, y_b)
        
        d = Door(pos_a,pos_b,room_lu,room_rd)
        d.tag = tag
    
    def add_player(self, tag: str, parameters: list[str]):

        #player
        # p(x,y,direction,room)
        x         = 32*float(parameters[0])
        y         = 32*(50-float(parameters[1]))
        direction = float(parameters[2])
        room      = self.find_room(parameters[3])

        self.player = Player(x, y, direction)
        self.player.room = room
        self.player.room.activate()
        self.player.recalculateSector()
    
    def connect_sector(self, sector: Sector):

        #attempt to connect with unconnected sectors
        A = sector.pos_a
        B = sector.pos_b
        C = sector.pos_c
        D = sector.pos_d

        for s2 in self.unconnected_sectors:

            if s2 is sector:
                continue

            hasA = False
            hasB = False
            hasC = False
            hasD = False
            corners = s2.getCorners()
            #do any corners match?
            for corner in corners:
                if near(A[0], corner[0]) and near(A[1], corner[1]):
                    hasA = True
                elif near(B[0], corner[0]) and near(B[1], corner[1]):
                    hasB = True
                elif near(C[0], corner[0]) and near(C[1], corner[1]):
                    hasC = True
                elif near(D[0], corner[0]) and near(D[1], corner[1]):
                    hasD = True
            if hasA and hasB:
                sector.connects_ab = s2
                s2.connects_cd = sector
            elif hasB and hasC:
                sector.connects_bc = s2
                s2.connects_da = sector
            elif hasC and hasD:
                sector.connects_cd = s2
                s2.connects_ab = sector
            elif hasD and hasA:
                sector.connects_da = s2
                s2.connects_bc = sector
        
        for sector in self.unconnected_sectors:
            if sector.is_connected():
                self.unconnected_sectors.remove(sector)

    def find_room(self, tag) -> Room | None:
        for r in self.rooms:
            if r.tag == tag:
                return r
        return None

    def spin_player(self, amount) -> None:

        self.player.direction += amount

        if self.player.direction > 360:
            self.player.direction -= 360

        if self.player.direction < 0:
            self.player.direction += 360

    def move_player(self, amount) -> None:

        dx =  amount * math.cos(math.radians(self.player.direction))
        dy = -amount * math.sin(math.radians(self.player.direction))

        self.player.move(dx,dy)
    
    def update(self) -> None:

        for room in self.active_rooms:
            room.update(self.player.get_position())

        self.active_rooms = []
        for room in self.rooms:
            if room.active:
                self.active_rooms.append(room)
#endregion
################ View    ######################################################
#region
class StatusBar(tk.Frame):

    def __init__(self, parent: tk.Tk, **kwargs):

        super().__init__(master = parent, bg="black", **kwargs)

        self.position_label = tk.Label(self, text = "Position:")
        self.position_label.pack(side = tk.LEFT)

        self.sector_label = tk.Label(self, text = "Sector:")
        self.sector_label.pack(side = tk.LEFT)

        self.room_label = tk.Label(self, text = "Room:")
        self.room_label.pack(side = tk.LEFT)

        self.active_rooms_label = tk.Label(self, text = "Active Rooms:")
    
    def redraw(self, scene: Scene) -> None:

        player = scene.player
        x,y = round(player.get_position())

        self.position_label.config(text = f"Position: ({x}, {y})")
        self.sector_label.config(text = f"Sector: {player.sector.tag}")
        self.room_label.config(text = f"Room: {player.room.tag}")
        self.active_rooms_label.config(text = f"Active Rooms: {len(scene.active_rooms)}")

class MapView(tk.Canvas):

    def __init__(self, parent: tk.Tk, **kwargs):

        super().__init__(master = parent, bg="black", **kwargs)
    
    def redraw(self, scene: Scene):

        self.delete("all")

        for room in scene.active_rooms:
            
            self.draw_walls(room, scene.player)
            
            self.draw_doors(room, scene.player)

            for sector in room.getSectors():

                for drake in sector.drake_nanas:
                    self.draw_entity(drake, "yellow", scene.player)
        
        self.create_oval(CENTER[0] - 6, CENTER[1] - 6, CENTER[0] + 6, CENTER[1] + 6, fill = "red")
    
    def draw_walls(self, 
        room: Room, camera: Player) -> None:

        camera_position = camera.get_position()
        camera_direction = camera.direction
        camera_sector = camera.sector

        for sector in room.sectors:
                color = "green"
                if sector is camera_sector:
                    color = "red"
                for wall in sector.walls:
                    pos_a = translate(
                        world_to_view_transform(
                            wall.pos_a, camera_position, camera_direction),
                        CENTER
                    )
                    pos_b = translate(
                        world_to_view_transform(
                            wall.pos_b, camera_position, camera_direction), 
                        CENTER
                    )
                    self.create_line(pos_a[0], pos_a[1], pos_b[0], pos_b[1], fill = color)
    
    def draw_doors(self,
        room: Room, camera: Player) -> None:

        camera_position = camera.get_position()
        camera_direction = camera.direction

        for door in room.doors:

            color = "yellow"
            if door.is_open:
                color = "cyan"
            
            pos_a = translate(
                world_to_view_transform(
                    door.pos_a, camera_position, camera_direction),
                    CENTER
            )
            
            pos_b = translate(
                world_to_view_transform(
                    door.pos_b, camera_position, camera_direction), 
                CENTER
            )
            
            self.create_line(pos_a[0], pos_a[1], pos_b[0], pos_b[1], fill = color)

    def draw_entity(self, entity: Entity, color: str, camera: Player) -> None:

        camera_position = camera.get_position()
        camera_direction = camera.direction

        pos = translate(
            world_to_view_transform(
                entity.get_position(), camera_position, camera_direction
            ),CENTER)
        
        radius = int(entity.get_size() / 2)
        self.create_oval(
            pos[0] - radius, pos[1] - radius, 
            pos[0] + radius, pos[1] + radius, 
            fill = color)

class GameView(tk.Canvas):

    def __init__(self, parent: tk.Tk, **kwargs):

        super().__init__(master = parent, bg="black", **kwargs)

        self.crosshair_lines = (
            ((CENTER[0] - 8,     CENTER[1]), (CENTER[0] + 8,     CENTER[1])),
            ((    CENTER[0], CENTER[1] - 8), (    CENTER[0], CENTER[1] + 8))
        )

    def redraw(self, scene: Scene):

        self.delete("all")

        for room in scene.active_rooms:
            
            self.draw_walls(room, scene.player)
            
            self.draw_doors(room, scene.player)

            for sector in room.getSectors():
                for drake in sector.drake_nanas:
                    self.draw_entity(drake, scene.player)
        
        #crosshair
        for line in self.crosshair_lines:
            pos_a, pos_b = line
            self.create_line(
                pos_a[0], pos_a[1], 
                pos_b[0], pos_b[1], fill="white")
    
    def draw_walls(self, 
        room: Room, camera: Player) -> None:

        for sector in room.sectors:
                color = "green"
                for wall in sector.walls:
                    self.draw_wall(wall, color, camera)
    
    def draw_doors(self,
        room: Room, camera: Player) -> None:

        for door in room.doors:

            color = "yellow"
            if door.is_open:
                color = "cyan"
            
            self.draw_wall(door, color, camera)
    
    def draw_wall(self, wall: Wall, color: str, camera: Player) -> None:

        camera_position = camera.get_position()
        camera_direction = camera.direction
        camera_z = camera.get_top()

        #backface test
        wall_pos = (-wall.pos_a[0], -wall.pos_a[1])
        wall_to_viewer = translate(camera_position, wall_pos)
        if (dot_product(wall_to_viewer, wall.normal) < 0)\
            and not wall.backface_visible:
            return

        pos_a = world_to_view_transform(
            wall.pos_a, camera_position, camera_direction)
                    
        pos_b = world_to_view_transform(
            wall.pos_b, camera_position, camera_direction)

        edge_table = view_to_screen_transform(
            pos_a, pos_b, 
            wall.z, wall.z + wall.height, camera_z
        )

        if edge_table is None:
            return

        self.create_polygon(edge_table, color)
    
    def draw_entity(self, entity: Entity, camera: Player) -> None:

        camera_position = camera.get_position()
        camera_direction = camera.direction
        camera_z = camera.get_top()

        pos = world_to_view_transform(
                entity.get_position(), camera_position, camera_direction)
        
        edge_table = []
        size = entity.get_size()
        scale = (size/2, size/2, size/2)

        color = DRAKE_COLORS[DRAKE_BODY]
        for line_segment in DRAKE_MODEL[DRAKE_BODY]:
            pos_a, pos_b = line_segment

            new_points = view_to_screen_transform_simple(
                pos_a = (scale[0]*(pos_a[0]) + pos[0], scale[0]*(pos_a[1]) + pos[1]), 
                pos_b = (scale[1]*(pos_b[0]) + pos[0], scale[1]*(pos_b[1]) + pos[1]), 
                z_a = scale[2]*(pos_a[2]), z_b = scale[2]*(pos_b[2]), z_camera=camera_z
            )

            if new_points is None:
                continue

            for point in new_points:
                edge_table.append(point)

        if len(edge_table) == 0:
            return

        self.create_polygon(edge_table, color)

        edge_table = []
        color = DRAKE_COLORS[DRAKE_FACE]
        for line_segment in DRAKE_MODEL[DRAKE_FACE]:
            pos_a, pos_b = line_segment

            new_points = view_to_screen_transform_simple(
                pos_a = (scale[0]*(pos_a[0]) + pos[0], scale[0]*(pos_a[1]) + pos[1]), 
                pos_b = (scale[1]*(pos_b[0]) + pos[0], scale[1]*(pos_b[1]) + pos[1]), 
                z_a = scale[2]*(pos_a[2]), z_b = scale[2]*(pos_b[2]), z_camera=camera_z
            )

            if new_points is None:
                continue

            for point in new_points:
                edge_table.append(point)

        if len(edge_table) == 0:
            return

        self.create_polygon(edge_table, color)

        color = DRAKE_COLORS[DRAKE_MISC]
        for line_segment in DRAKE_MODEL[DRAKE_MISC]:
            pos_a, pos_b = line_segment

            edge_table = view_to_screen_transform_simple(
                pos_a = (scale[0]*(pos_a[0]) + pos[0], scale[0]*(pos_a[1]) + pos[1]), 
                pos_b = (scale[1]*(pos_b[0]) + pos[0], scale[1]*(pos_b[1]) + pos[1]), 
                z_a = scale[2]*(pos_a[2]), z_b = scale[2]*(pos_b[2]), z_camera=camera_z
            )

            if not edge_table is None:
                self.create_polygon(edge_table, color)
    
    def create_polygon(self, edge_table: list[ivec2], color: str) -> None:

        line_count = len(edge_table)
        for i in range(line_count):
            pos_a = edge_table[i]
            pos_b = edge_table[(i + 1) % line_count]

            self.create_line(pos_a[0], pos_a[1], pos_b[0], pos_b[1], fill = color)
#endregion
################ Control   ####################################################
#region
class App:

    def __init__(self, root: tk.Tk):

        self.root = root

        self.game_frame = tk.Frame(self.root)
        if MODE < 2:
            self.map_view = MapView(self.game_frame, width = SCREEN_WIDTH, height = SCREEN_HEIGHT)
            self.map_view.pack(side=tk.LEFT)
        self.projected_view = GameView(self.game_frame, width=SCREEN_WIDTH, height = SCREEN_HEIGHT)
        self.projected_view.pack(side=tk.LEFT)
        self.game_frame.pack(side=tk.TOP)

        if MODE == 0:
            self.status_bar = StatusBar(self.root)
            self.status_bar.pack(side=tk.TOP)
        
        self.scene = Scene("level.txt")

        self.keys_down = {}

        self.bind_events()
    
    def bind_events(self):

        self.root.bind('<Key>', self.handle_key_press)
        self.root.bind('<KeyRelease>', self.handle_key_release)
    
    def handle_key_press(self, event) -> None:

        self.keys_down[event.keysym] = True
    
    def handle_key_release(self, event) -> None:

        self.keys_down[event.keysym] = False

    def handle_key_state(self) -> None:
        
        if "Left" in self.keys_down and self.keys_down["Left"]:
            self.scene.spin_player(1)
        if "Right" in self.keys_down and self.keys_down["Right"]:
            self.scene.spin_player(-1)
        if "Up" in self.keys_down and self.keys_down["Up"]:
            self.scene.move_player(1)
        if "Down" in self.keys_down and self.keys_down["Down"]:
            self.scene.move_player(-1)
    
    def update(self) -> None:

        self.handle_key_state()

        self.scene.update()

        if MODE==0:
            self.status_bar.redraw(self.scene)
        
        if MODE < 2:
            self.map_view.redraw(self.scene)
        
        self.projected_view.redraw(self.scene)
        
        self.root.after(16, self.update)
#endregion
###############################################################################
root = tk.Tk()
app = App(root)
app.update()
root.mainloop()