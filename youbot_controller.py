"""youbot_controller controller."""
#TODO: detect critical zombies bug
#TODO: better scoring function?

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor
import numpy as np

from youbot_zombie import *   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs


class BerryMetadata():
    def __init__(self, x1, x2, y, color):
        self.x1 = x1
        self.x2 = x2
        self.y = y
        self.color = color
        
MAX_SPEED = 14.81        
SPEED = 14.81/2
RSPEED = 13.5/2
ANG_VEL = 0.835
PI = 3.14159265
THIRTY_DEG_STEPS = 4
LF_berries = False
# time to rotate pi/6 = pi/6/0.835
# timer steps = time/.128
COLOR_DIST_THRESH = 300

STR_TO_RGB = {
    'orange': [212,140,95],
    'red': [224,68,48],
    'yellow': [224,214,32],
    'pink': [209,137,181]
}

BERRIES_CHARS = ['o', 'r', 'y', 'p']

BERRIES_PIXELS = {
    (212,140,95): ('orange', 0),
    (195,125,86): ('orange', 1),
    (151, 91, 63): ('orange', 2),

    (224,68,48): ('red', 0),
    (207,62,47): ('red', 1),
    (165,44,34): ('red', 2),
    (70,19,18): ('red', 3),

    (224,214,32): ('yellow', 0),
    (219,208,34): ('yellow', 1),
    (192,180,32): ('yellow', 2),
    (160,149,22): ('yellow', 3),
    (71,70,14): ('yellow', 4),

    (209,137,181): ('pink', 0),
    (198,127,171): ('pink', 1),
    (151,91,131): ('pink', 2),
    (64,40,71): ('pink', 3),
}

FLOOR_PIXELS = [
    (216,183,171),
    (207,156,146),
    (220, 201, 197),
    (212, 170, 152),
    (221, 193, 180),
    (214, 178, 167),
    (221, 186, 170),
    (177,131,117),
    (209,166,148),
    (217, 194, 189),
    (184, 135, 117),
    (190, 139, 120),
    (202, 141, 132),
    (201, 151, 142),
    (202, 157, 147),
    (203, 151, 143),
    (202, 144, 132)
]
ZOMBIES_PIXELS = {
    'b': [
        (36,149,235),
        (36, 91, 170),
        (29, 76, 149),
        (14, 43, 102),
        (23, 65, 137),
        (30, 81, 159),
        (34, 87, 164),
        (28, 74, 146),
        (32, 97, 181),
        (28, 86, 167),
        (30, 101, 188),
        (14, 40, 94),
        (8, 33, 74),
        (10, 31, 66),
        (28, 93, 177),
        (13, 40, 102),
        (18, 58, 127),
        (30, 81, 159),
        (26, 87, 170),
        (15, 39, 89),
        (10, 39, 94),	
        (30, 141, 228),
        (24, 113, 202),
        (17, 75, 150),
        (10, 41, 99),
        (33, 161, 242),
        (23, 108, 195),
        (9, 35, 81),
        (30, 137, 225),	
        (25, 113, 203),	
        (9, 31, 65),
        (20, 43, 97),
        (13, 50, 119),
        (12, 32, 67),
        (12, 38, 87),
    ],
    'g': [
        (13,80,15),
        (16,96,18),
        (17,105,19),
        (18,113,20),
        (10,55,14),
        (28,181,31),
        (36,195,39),
        (36,198,39),
        (28,142,31),
        (31,176,34),
        (12,53,16),
        (12,70,14),
        (16,95,18),
        (19,114,20),
        (25,153,27),
        (10,47,12),
        (37,205,40),
        (34,170,37),
        (30,160,33),
        (28,142,31),
        (33,172,36),
    ],
    'l': [	
        (157,68,225),
        (141,58,221),
        (120, 49, 189),
        (103,42,170),
        (96,39,161),
        (93,38,158),
        (81,33,142),
        (64,27,122),
        (44,20,98),
        (188,84,244),
        (179,78,240),
        (174,76,237),
        (97,42,167),
        (135,57,206),
        (137,57,209),
        (44,21,103),
        (43,20,93),
        (36,16,69),
        (101,41,164),
        (140,61,209),
    ],
    'a': [
        (35,225,200),
        (32,203,176),
        (22,116,105),
        (10,73,75),
        (11,72,73),
        (26,202,174),
        (20,154,131),
        (27,203,175),
        (26,196,168),
        (33,186,158),
        (36,205,177),
        (38,223,196),
        (35,233,210),
        (21,162,137),
        (27,193,164),
        (38,210,182),
        (21,157,133),
        (19,148,125),
        (15,114,98),
        (21,157,132),
        (14,105,92),
        (11,83,77),
        (9,71,69),
        (11,87,80),
        (17,129,111),
        (15,117,101),
        (13,97,87),
        (10,72,70),
        (11,69,68),
        (36,229,204),
        (9, 52, 45),
        
    ]
}

def rgb_to_char(pixel):
    for color in BERRIES_PIXELS:
        if dist(pixel, color) <= COLOR_DIST_THRESH:
            return BERRIES_PIXELS[color][0][0]
            

    for color in FLOOR_PIXELS:
        if dist(pixel, color) <= COLOR_DIST_THRESH:
            return 'f'
            
    for c in ZOMBIES_PIXELS:
        for color in ZOMBIES_PIXELS[c]:
            if dist(pixel, color) <= COLOR_DIST_THRESH:
                return c
                
    return '0'
                
def base_forwards(wheels):
    # print("Called")
    for wheel in wheels:
        wheel.setPosition(float('inf'))
        wheel.setVelocity(SPEED)

def base_reset(wheels):
    print("resetting")
    for wheel in wheels:
        wheel.setVelocity(0.0)

def turn_right(wheels):
    speeds = [SPEED, 0.25*SPEED, SPEED, 0.25*SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])    

def rotate_right(wheels):
    speeds = [-RSPEED, RSPEED, -RSPEED, RSPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
def rotate_left(wheels):
    speeds = [RSPEED, -RSPEED, RSPEED, -RSPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
def turn_left(wheels):
    speeds = [.25*SPEED, SPEED, .25*SPEED, SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
        
def reverse_right(wheels):
    speeds = [-.25*SPEED, -SPEED, -.25*SPEED, -SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
        
def reverse_left(wheels):
    speeds = [-SPEED, -.25*SPEED, -SPEED, -.25*SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])

def reverse_back(wheels):
    speeds = [-SPEED, -SPEED, -SPEED, -SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])

def strafe_left(wheels):
    speeds = [SPEED, -SPEED, -SPEED, SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
def strafe_right(wheels):
    speeds = [-SPEED, SPEED, SPEED, -SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])

def dist(a, b):
    return sum([(a[i]-b[i])**2 for i in range(len(a))])


'''
def get_zombie_locs(world_pixel_info, zombies_pixels):
    # (row, [(average column, color)])
    locs = []
    for i in range(len(world_pixel_info) - 1, len(world_pixel_info) // 2, -1):
        row_zombies = []
        xrange = []
        color = ''
        prev = False
        for j in range(len(world_pixel_info[i])):
            if world_pixel_info[i][j] in zombies_pixels:
                if prev:
                    continue
                print(world_pixel_info[i][j])
                seen = False
                if i < len(world_pixel_info) - 1:
                    for k in range(j-3, j+4):
                        if k < 0 or k >= len(world_pixel_info[i]):
                            continue
                        if world_pixel_info[i+1][k] in zombies_pixels:
                            seen = True
                            break
                if seen:
                    prev = True
                    continue
                    
                if len(xrange) == 0:
                    xrange.append(j)
                    color = world_pixel_info[i][j]
            else:
                prev = False
                if len(xrange) == 1:
                    xrange.append(j)
                    row_zombies.append((sum(xrange)/2, color))
                    xrange = []
                    color = ''
        if len(xrange) == 1:
            xrange.append(len(world_pixel_info[i]))
            row_zombies.append((sum(xrange)/2, color))
        if len(row_zombies):
            locs.append((i, row_zombies))
    print(locs)
    return locs
'''    
def compute_world_pixel_info(dict_image_tups, width, height):
    for pixel_info, image in dict_image_tups:
        for y in range(height):
            row = []
            for x in range(width):
                r = image[x][y][0]
                g = image[x][y][1]
                b = image[x][y][2]
                pixel_info[y][x] = rgb_to_char([r,g,b])
                
def dfs(pixels, visited, row, col, color, d):
    st = []
    st.append((row, col))
    visited[row][col] = True
    dr = [-1, -1, -1, 0, 0, +1, +1, +1]
    dc = [-1, 0, +1, -1, +1, -1, 0, +1]
    
    while len(st):
        nxt = st.pop()
        if nxt[0] > d[0]:
            d[0] = nxt[0]
        for k in range(len(dr)):
            nr = dr[k] + nxt[0]
            nc = dc[k] + nxt[1]
            try:
                if pixels[nr][nc] == color and not visited[nr][nc]:
                    st.append((nr, nc))
                    visited[nr][nc] = True
            except Exception as e:
                continue
            
def get_world_stats(pixels):
    zombies_info = []
    berries_info = []
    floor_pixel_count = 0
    visited = np.zeros(pixels.shape, dtype=bool)
    
    for i in range(len(pixels)):
        for j in range(len(pixels[i])):
            if pixels[i][j] == 'f':
                floor_pixel_count += 1
            if pixels[i][j] in ZOMBIES_PIXELS and not visited[i][j]:
                d = [0]
                dfs(pixels, visited, i, j, pixels[i][j], d)
                zombies_info.append((d[0], pixels[i][j]))
            elif pixels[i][j] in BERRIES_CHARS and not visited[i][j]:
                d = [0]
                dfs(pixels, visited, i, j, pixels[i][j], d)
                berries_info.append((d[0], pixels[i][j]))
    return zombies_info, berries_info, floor_pixel_count
    
def has_critical_zombies_or_wall(image):
    zombie = False
    wall = False
    for i in range(len(image)-1, 33, -1):
        cnt = 0
        for j in range(len(image[i])):
            char = rgb_to_char(image[i][j])
            if char in ZOMBIES_PIXELS:
                zombies = True
            elif char == '0':
                cnt += 1
        if i >= 45 and cnt >= 0.7*len(image[i]):
            wall = True
    return (zombie, wall)
    
def approaching_wall(image):
    for i in range(len(image)-1, 53, -1):
        cnt = 0
        for j in range(len(image[i])):
            if rgb_to_char(image[i][j]) == '0':
                cnt += 1
        if cnt >= 0.7*len(image[i]):
            return True
        '''
        cnt = 0
        for j in range(len(world_pixel_info[i])):
            cnt += (world_pixel_info[i][j] == '0') # '0' means not a floor, zombie, or berry pixel
        if cnt >= 0.7 * len(world_pixel_info[i]): # 70% of the row is non floor/zombie/berry
            if i >= 0.6 * len(world_pixel_info): # this is past 60% of the rows
                return True
        '''
    return False
    
def compute_scores(pixels, i):
    zombies_info, berries_info, floor_pixel_count = get_world_stats(pixels)
    score = -2*len(zombies_info)
    #print('zombies info = {}, i = {}'.format(zombies_info, i))
    #print('zombie before score', score)
    for z in zombies_info:
        dist = z[0] 
        if dist >= 36:
            score -= 15
        elif dist >= 34:
            score -= 5
        elif dist >= 33:
            score -= 3
    #print('zombie score', score)
    #print('berries_info', berries_info)
    berries_mult = LF_berries + 1
    score += berries_mult*len(berries_info)
    for b in berries_info:
        dist = b[0]
        if dist >= 35:
            score += LF_berries*1.5
    #print('floor pixels', floor_pixel_count, score)
    score += floor_pixel_count / (128*64/3)
    #print('score\n', score)
    return score
    
def compute_high_level_goal(world_pixels_tup):
    #print(compute_scores(world_pixels_tup[0]))
    
    pixel_scores = []
    i = 0
    for world_pixels in world_pixels_tup:
        #print(i)
        width = world_pixels.shape[1]
        pixel_scores.append((compute_scores(world_pixels[:, 0:width//3],i), i))
        pixel_scores.append((compute_scores(world_pixels[:, width//3: 2*width//3],i+1), i+1))
        pixel_scores.append((compute_scores(world_pixels[:, 2*width//3:],i+2), i+2))
        i += 3
    mx_tup = max(pixel_scores)
    #print(pixel_scores)
    #print(mx_tup)
    if mx_tup[1] == 0:
        return i
    else:
        return mx_tup[1]
        
def get_berry_metadata(camera, image):
    width = camera.getWidth()
    height = camera.getHeight()
    
    all_berry_metadata = []
    first = False
    second = False
    
    x1 = -1
    x2 = -2
    #print(len(world_pixel_info), len(world_pixel_info[0]), height, width)
    for y in range(height):
        for x in range(width):
            pix = rgb_to_char(image[y][x])
            if pix in BERRIES_CHARS:
                if not first:
                    x1 = x
                    first = True
                else:
                    x2 = x
                    second = True
            elif first and second:
                berry_metadata = BerryMetadata(x1, x2, y, pix)
                all_berry_metadata.append(berry_metadata)
                first = False
                second = False
                x1 = -1
                x2 = -1
                
    return all_berry_metadata
    
def get_closest_berry(all_berry_metadata):
    max_size = 0
    closest_berry_metadata = None
    
    for metadata in all_berry_metadata:
        size = metadata.y
        if size > max_size:
            max_size = size
            closest_berry_metadata = metadata
            
    return closest_berry_metadata
    
    
def drive_to_berry(fr, fl, br, bl, camera, image):
    image_mid =  camera.getWidth() // 2
    all_berry_metadata = get_berry_metadata(camera, image)
    closest = get_closest_berry(all_berry_metadata)
    #print('berries', len(all_berry_metadata))
    berry_center_position = -1
    if closest:
        berry_center_position = (closest.x2 + closest.x1) // 2
    
    error = abs(image_mid - berry_center_position)
    gain = error / image_mid
    
    #print("image mid", image_mid)
    #print("berry center ", berry_center_position)
    
    fr.setVelocity(.5 * MAX_SPEED)
    fl.setVelocity(.5 * MAX_SPEED)
    br.setVelocity(.5 * MAX_SPEED)
    bl.setVelocity(.5 * MAX_SPEED)
    
    THRESHOLD = 2
    
    if berry_center_position == -1:
        print("berry not found")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif image_mid - THRESHOLD < berry_center_position < image_mid + THRESHOLD:
        print("berry aligned go straight")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif berry_center_position < image_mid:
        print("berry on the left")
        fr.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    else:
        print("berry on the right")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        
    
#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    accelerometer = robot.getDevice("accelerometer")
    accelerometer.enable(timestep)
    
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    camera2 = robot.getDevice("ForwardHighResSmallFov")
    camera2.enable(timestep)
    
    camera3 = robot.getDevice("ForwardHighRes")
    camera3.enable(timestep)
    
    camera4 = robot.getDevice("ForwardHighResSmall")
    camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    
    #inertialUnit = robot.getDevice("inertial unit")
    #inertialUnit.enable(timestep)
    
    lightSensor = robot.getDevice("light sensor")
    lightSensor.enable(timestep)
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    
    rangeFinder = robot.getDevice("range-finder")
    rangeFinder.enable(timestep)
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))

    fr.setVelocity(0)
    fl.setVelocity(0)
    br.setVelocity(0)
    bl.setVelocity(0)   
    
    i=0
           
    tl_count = 0
    tr_count = 0
    
    rr = 0
    lr = 0
    sr = 0
    sl = 0
    
    zero_count = 0
    goback = 0
    
    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
        wheels = [fr, fl, br, bl]
       # rotate_left(wheels)
        
        #print(compass.getValues())
        #continue
        
        if timer < 5:
            
            continue
        '''
        accel_vector = accelerometer.getValues()
        print(zero_count)
        if abs(accel_vector[0]) < 0.1 and abs(accel_vector[1]) < 0.1:
            zero_count += 1
            if zero_count == 20:
                goback = 15
                
        else:
            zero_count = 0
        ''' 
        if goback:
            reverse_back(wheels)
            goback -= 1
            continue
        if rr:
            rotate_right(wheels)
            rr -= 1
            continue
        if sl:
            strafe_left(wheels)
            sl -= 1
            continue
        if sr:
            strafe_right(wheels)
            sr -= 1 
            continue
        if lr:
            rotate_left(wheels)
            lr -= 1
            continue
        if tl_count:
            
            reverse_left(wheels)
            tl_count -= 1
            continue
        elif tr_count:
            
            reverse_right(wheels)
            tr_count -= 1
            continue



        # print(receiver.getQueueLength(), receiver.getData())
        '''
        if receiver.getQueueLength():
            print(receiver.getQueueLength())
            print(receiver.getData())
            receiver.nextPacket()
        '''
        
        image1 = camera1.getImageArray()
        imageB = camera5.getImageArray()
        imageR = camera6.getImageArray()
        imageL = camera7.getImageArray()
        if image1:
            with open('im1.txt', 'w') as f:
                f.write(str(image1))
        '''
        if image2:
            with open("im2.txt", "w") as f:
                f.write(str(image2))
        '''
        
        width = camera1.getWidth()  
        height = camera1.getHeight()

        LF_berries = robot_info[0] < 80 or robot_info[1] < 70 # maybe change based on health, energy metric
        
        
        nxt_dir = None
        if i % 2 == 0:
            image1_T = np.transpose(np.array(image1), (1,0,2)).tolist()
            imageR_T = np.transpose(np.array(imageR), (1,0,2)).tolist()
            imageL_T = np.transpose(np.array(imageL), (1,0,2)).tolist()
            
            critical_zombies, wall = has_critical_zombies_or_wall(image1_T) 
            Rwall = approaching_wall(imageR_T)
            Lwall = approaching_wall(imageL_T)
            if Rwall:
                sl = 4
                continue
            if Lwall:
                sr = 4
                continue
            with open('imF.txt', 'w') as f:
                f.write(str(image1))
            with open('imR.txt', 'w') as f:
                f.write(str(imageR))
            with open('imB.txt', 'w') as f:
                f.write(str(imageB))
            with open('imL.txt', 'w') as f:
                f.write(str(imageL))       
            if (LF_berries and i%200 == 0) or (not LF_berries and i%40 == 0) or critical_zombies or wall:
                print('critical_zombie: {}, wall: {}'.format(critical_zombies, wall))
                world_pixels_F, world_pixels_B, world_pixels_R, world_pixels_L = [np.full((height,width), '0') for _ in range(4)]
                compute_world_pixel_info([(world_pixels_F, image1), (world_pixels_R, imageR), (world_pixels_B, imageB), (world_pixels_L, imageL)], width, height)        

                with open('world_info_F.txt', 'w') as f:
                    f.write(str(world_pixels_F.tolist()))
                with open('world_info_R.txt', 'w') as f:
                    f.write(str(world_pixels_R.tolist()))
                with open('world_info_B.txt', 'w') as f:
                    f.write(str(world_pixels_B.tolist()))
                with open('world_info_L.txt', 'w') as f:
                    f.write(str(world_pixels_L.tolist()))
                nxt_dir = compute_high_level_goal([np.array(world_pixels_F), np.array(world_pixels_R), np.array(world_pixels_B), np.array(world_pixels_L)])
            if nxt_dir:
                nxt_dir -= 1
                print(nxt_dir)
                if nxt_dir <= 6:
                    rr = THIRTY_DEG_STEPS*nxt_dir
                else:
                    lr = THIRTY_DEG_STEPS*(12-nxt_dir)
                
                #turn 
            elif LF_berries:
                drive_to_berry(fr, fl, br, bl, camera1, image1_T)
                # drive to closest berry without zombie on/near it
            else:
                base_forwards(wheels)
        else:
            base_forwards(wheels)
        
        i+=1
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()