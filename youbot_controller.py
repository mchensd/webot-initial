"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs
import random
SPEED = 4.0
def base_forwards(wheels):
    # print("Called")
    for wheel in wheels:
        wheel.setPosition(float('inf'))
        wheel.setVelocity(SPEED)

def base_reset(wheels):
    print("resetting")
    for wheel in wheels:
        wheel.setVelocity(0.0)

def base_turn_right(wheels):
    speeds = [-.25*SPEED, -SPEED, -.25*SPEED, -SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
        
def base_turn_left(wheels):
    speeds = [-SPEED, -.25*SPEED, -SPEED, -.25*SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
        
def dist(a, b):
    return sum([(a[i]-b[i])**2 for i in range(len(a))])

def approaching_wall(world_pixel_info):
    for i in range(len(world_pixel_info)):
        cnt = 0
        for j in range(len(world_pixel_info[i])):
            cnt += (world_pixel_info[i][j] == '0') # '0' means not a floor, zombie, or berry pixel
        print('cnt: ', cnt)
        if cnt >= 0.7 * len(world_pixel_info[i]): # 70% of the row is non floor/zombie/berry
            if i >= 0.6 * len(world_pixel_info): # this is past 60% of the rows
                return True
    return False
                    
                
    
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
        if tl_count:
            base_turn_left(wheels)
            tl_count -= 1
            continue
        elif tr_count:
            base_turn_right(wheels)
            tr_count -= 1
            continue
        berries_pixels = {
            (212,140,95): ('orange', 0),
            (195,125,86): ('orange', 1),
            (151, 91, 63): ('orange', 2),
            (63,39,32): ('orange', 3),

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

        floor_pixels = [
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
        zombies_pixels = {
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
            ],
            'g': [

            ],
            'p': [	
            ],
            'a': [
            ]
        }
        str_to_rgb = {
            'orange': [212,140,95],
            'red': [224,68,48],
            'yellow': [224,214,32],
            'pink': [209,137,181]
        }
        COLOR_DIST_THRESH = 300
        # print(receiver.getQueueLength(), receiver.getData())
        '''
        if receiver.getQueueLength():
            print(receiver.getQueueLength())
            print(receiver.getData())
            receiver.nextPacket()
        '''
        
        image1 = camera1.getImageArray()
        image2 = camera2.getImageArray()
        
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
        
        world_pixel_info = []
        if i%15 == 0:
            world_pixel_info = [['0']*width for _ in range(height)]
        
        berries_pixels_out = [] 
        
        if i % 15 == 0: ## Only run every 15 timesteps for now because this is computationally heavy
            for y in range(height):
                row = []
                for x in range(width):
                    r = image1[x][y][0]
                    g = image1[x][y][1]
                    b = image1[x][y][2]
                    nxt_pixel = [0,0,0]
                    for color in berries_pixels:
                        if dist([r,g,b], color) <= COLOR_DIST_THRESH:
                            nxt_pixel = str_to_rgb[berries_pixels[color][0]]
                            world_pixel_info[y][x] = berries_pixels[color][0][0]
                            break
                    #row.append(nxt_pixel)
                #berries_pixels_out.append(row)
        '''        
            with open('berries_processed.txt', 'w') as f:
                f.write(str(berries_pixels_out))
        '''
        
        floor_pixels_out = []
        if i % 15 == 0: # Only run every 15 timesteps for now because this is computationally heavy
            for y in range(height):
                row = []
                for x in range(width):
                    r = image1[x][y][0]
                    g = image1[x][y][1]
                    b = image1[x][y][2]
                    nxt_pixel = [0,0,0]
                    for color in floor_pixels:
                        if dist([r,g,b], color) <= COLOR_DIST_THRESH:
                            nxt_pixel = [255,255,255]
                            world_pixel_info[y][x] = 'f'
                            break
                    #row.append(nxt_pixel)
                #floor_pixels_out.append(row)
                
            '''
            with open('floor_processed.txt', 'w') as f:
                f.write(str(floor_pixels_out))
            '''
        
        
        
        
        if i%15 == 0: # Only run every 15 timesteps for now because this is computationally heavy
            zombies_pixels_out = []
            for y in range(height):
                row = []
                for x in range(width):
                    r = image1[x][y][0]
                    g = image1[x][y][1]
                    b = image1[x][y][2]
                    nxt_pixel = [0,0,0]
                    for c in zombies_pixels:
                        for color in zombies_pixels[c]:
                            if dist([r,g,b], color) <= COLOR_DIST_THRESH:
                                world_pixel_info[y][x] = c
                                nxt_pixel = zombies_pixels[c][0]
                                break
                    #row.append(nxt_pixel)
                #zombies_pixels_out.append(row)
        ''' 
        with open('zombies_processed.txt', 'w') as f:
            f.write(str(zombies_pixels_out))
        '''
        if len(world_pixel_info):
            with open('world_info.txt', 'w') as f:
                f.write(str(world_pixel_info))
        
        if approaching_wall(world_pixel_info) and not (tl_count or tr_count):
            print("turning")
            if random.random() < 0.5:
                tl_count = 75
            else:
                tr_count = 75
            
        
        else:
            base_forwards(wheels)
        if i==300:
            i = 0
        
        i+=1
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
