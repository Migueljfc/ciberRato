import sys
import xml.etree.ElementTree as ET

from croblink import *
from math import *
from astar import *
from itertools import permutations


CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    positions = []              #lista em que 1 é ter parede e 0 é nao ter em cada posicao goal com a seguinte estrutura [frente, tras, direita, esquerda]
    visited_pos = set()         #set com as posiçoes visitadas
    known_pos = set()           #set com as posiçoes conhecidas
    walls = set()               #set com as paredes conhecidas
    position_goal = (0,0)       #estado goal   
    initial_state = (0,0)       #estado inicial
    current_state = (0,0)       #estado em que o robo se encontra
    not_visited_pos = []        #posicoes conhecidas para as quais o robo ainda nao foi 
    arr = [[1 for i in range(55)] for j in range(27)]   #array que vai desenhar o mapa
    firstrun = True             #variavel que indica se é o primeiro ciclo para colocar no array I em vez de X
    isLooping = False           
    beacons = {}                #guarda o numero do beacon e a posicao
    pathBeacons = []            #posicoes que fazem o caminho mais curto entre os targets
    beaconId = []
    path = [] 

    def __init__(self,rob_name, rob_id, angles, host,filename):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host, filename)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                self.initial_state = (self.measures.x,self.measures.y)
                self.position_goal = self.initial_state
                self.current_state = (self.measures.x,self.measures.y)
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.wander()
            
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()      
            
    def wander(self):
        self.mapping()
        self.design()
        self.isLoop()
        while(self.isLooping):
            self.path = self.pathfind(self.known_pos)
            if self.path == None:
                self.pathToBeacon()
                self.pathWrite() 
            else:
                self.follow_path()
        self.positions.clear()

    def rotate(self, angle):
        if(angle == 90):
            #print("rotating to 90 ")
            if self.measures.compass >= 90:    
                while self.measures.compass > 90:
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
            else:
                while self.measures.compass < 90:
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
        elif(angle == -90):
            #print("rotating to -90")
            if self.measures.compass >= 45 or self.measures.compass < -90: 
                while self.measures.compass >= 45 or self.measures.compass < -90:    
                    self.driveMotors(-0.1,+0.1)
                    self.readSensors()
            else:
                while self.measures.compass  < 45 and self.measures.compass > -90:    
                    self.driveMotors(+0.1,-0.1)
                    self.readSensors()
        elif(angle == 0):
            #print("rotating to 0")
            if self.measures.compass <= 0:
                while self.measures.compass < -1:
                    self.driveMotors(-0.1,+0.1)
                    self.readSensors()
            else: 
                while self.measures.compass > 1:
                    self.driveMotors(+0.1,-0.1)
                    self.readSensors()
        elif(angle == 180):
            #print("rotating to 180")   
            if self.measures.compass <= 0 :
                while self.measures.compass < 177 and self.measures.compass <=0:
                    self.driveMotors(+0.1,-0.1)
                    self.readSensors()
            else:
                while self.measures.compass > -177 and self.measures.compass >0:
                    self.driveMotors(-0.1,+0.1)
                    self.readSensors()
        
        self.driveMotors(0.00,-0.00)  


    # método para descobrir paredes
    def searchWall(self):
        center_id = 0 
        back_id = 3
        right_id = 2
        left_id = 1
        x = -int(self.initial_state[0] - self.current_state[0])
        y = -int(self.initial_state[1] - self.current_state[1])
        if self.firstrun : 
            self.arr[13][27] = 'I'
            self.firstrun = False
        else: 
            self.arr[13-y][27+x] = 'X'
        #print("SEARCH WALLS...")
        if self.measures.irSensor[center_id] < 0.9:
                #print("Não há parede em frente")
            self.positions.append(0)
            if self.roundCompass() == 90:
                self.arr[12-y][27+x] = 'X'
               
            elif self.roundCompass() == -90:
                self.arr[14-y][27+x] = 'X'
            
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[13-y][26+x] = 'X'
             
            elif self.roundCompass() == 0:
                self.arr[13-y][28+x] = 'X'
            
        else:
            #print("Há parede em frente.")
            self.positions.append(1)       
            if self.roundCompass() == 90:
                self.arr[12-y][27+x] = '-'
                self.walls.add((x,y+1))
                
            elif self.roundCompass() == -90:
                self.arr[14-y][27+x] = '-'
                self.walls.add((x,y-1))
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[13-y][26+x] = '|'
                self.walls.add((x-1,y))
            elif self.roundCompass() == 0:
                self.arr [13-y][28+x]= '|'
                self.walls.add((x+1,y))
        if self.measures.irSensor[back_id] < 1.3:
            #print("Não há parede atrás")
            self.positions.append(0)
            if self.roundCompass() == 90:
                self.arr[14-y][27+x] = 'X'
             
            elif self.roundCompass() == -90:
                self.arr[12-y][27+x] = 'X'
            
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[13-y][28+x] = 'X'
            
            elif self.roundCompass() == 0:
                self.arr[13-y][26+x] = 'X'
            
        else:
            #print("Há parede atrás")
            self.positions.append(1)
            if self.roundCompass() == 90:
                self.arr[14-y][27+x] = '-'
                self.walls.add((x,y-1))
            elif self.roundCompass() == -90:
                self.arr[12-y][27+x] = '-'
                self.walls.add((x,y+1))
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[13-y][28+x] = '|'
                self.walls.add((x+1,y))
            elif self.roundCompass() == 0:
                self.arr[13-y][26+x] = '|'
                self.walls.add((x-1,y))
        if self.measures.irSensor[right_id] < 1.3:
            #print("Não há parede à direita")
            self.positions.append(0)
            if self.roundCompass() == 90:
                self.arr[13-y][28+x] = 'X'
               
            elif self.roundCompass() == -90:
                self.arr[13-y][26+x] = 'X'
              
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[12-y][27+x] = 'X'
               
            elif self.roundCompass() == 0:
                self.arr[14-y][27+x] = 'X'
             
        else:
            #print("Há parede à direita.")
            self.positions.append(1)
            if self.roundCompass() == 90:
                self.arr[13-y][28+x] = '|'
                self.walls.add((x+1,y))
            elif self.roundCompass() == -90:
                self.arr[13-y][26+x] = '|'
                self.walls.add((x-1,y))
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[12-y][27+x] = '-'
                self.walls.add((x,y+1))
            elif self.roundCompass() == 0:
                self.arr[14-y][27+x] = '-'
                self.walls.add((x,y-1))
        if self.measures.irSensor[left_id] < 1.3:
            #print("Não há parede à esquerda")
            self.positions.append(0)
            if self.roundCompass() == 90:
                self.arr[13-y][26+x] = 'X'
            elif self.roundCompass() == -90:
                self.arr[14-y][27+x] = 'X'
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[14-y][27+x] = 'X'
            elif self.roundCompass() == 0:
                self.arr[12-y][27+x] = 'X'
        else:
            #print("Há parede à esquerda.")
            self.positions.append(1)
            if self.roundCompass() == 90:
                self.arr[13-y][26+x] = '|'
                self.walls.add((x-1,y))
            elif self.roundCompass() == -90:
                self.arr[13-y][28+x] = '|'
                self.walls.add((x+1,y))
            elif self.roundCompass() == 180 or self.roundCompass() == -180:
                self.arr[14-y][27+x] = '-'
                self.walls.add((x,y-1))
            elif self.roundCompass() == 0:
                self.arr[12-y][27+x] = '-'
                self.walls.add((x,y+1))
        
        if(self.measures.ground > -1):
            #print("BEACON" + str(self.measures.ground))
            self.arr[13-y][27+x] = str(self.measures.ground)
            self.beacons[(x,y)] = int(self.measures.ground)

        self.visited_pos.add((x,y))
        self.known_pos.add((x,y))
          
    def errorCalc(self, angle, pos):
        if(angle == 0):
            posError =  pos[1] - self.measures.y                
        elif(angle == 180):
            posError =  self.measures.y - pos[1]
        elif(angle == -90):
            posError =  pos[0] - self.measures.x               
        else:
            posError =  self.measures.x - pos[0]
            
        angError = angle - self.measures.compass                
        
        if(self.measures.compass < 0 and angle == 180):
            angError =  - (angle - abs(self.measures.compass))  

        return angError*0.01 + posError*0.1

    def mapping(self):
        x = -int(self.initial_state[0] - self.current_state[0])
        y = -int(self.initial_state[1] - self.current_state[1])

        print("x: ", x, "y: ", y)

        self.readSensors()
        self.searchWall()
        if self.positions[0] == 0:
            if self.positions[2] == 1 and self.positions[3] == 1:
                if self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.position_goal = self.calc_next(180,self.current_state)
                elif self.roundCompass() == 0:
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == -90:
                    self.position_goal = self.calc_next(-90,self.current_state)
                elif self.roundCompass() == 90:
                    self.position_goal = self.calc_next(90,self.current_state)
            elif self.positions[2] == 0 and self.positions[3] == 1:
                if self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.not_visited_pos.append((x-2,y))    
                    self.known_pos.add((x-2,y))
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state)
                elif self.roundCompass() == 0:
                    self.not_visited_pos.append((x+2,y))    
                    self.known_pos.add((x+2,y))
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)
                elif self.roundCompass() == -90:
                    self.not_visited_pos.append((x,y-2))    
                    self.known_pos.add((x,y-2))
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
                elif self.roundCompass() == 90:
                    self.not_visited_pos.append((x,y+2))    
                    self.known_pos.add((x,y+2))
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
            elif self.positions[2] == 1 and self.positions[3] == 0:
                if self.roundCompass() == 180 or self.roundCompass() == -180: 
                    self.not_visited_pos.append((x-2,y))    
                    self.known_pos.add((x-2,y))
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)
                elif self.roundCompass() == 0:
                    self.not_visited_pos.append((x+2,y))    
                    self.known_pos.add((x+2,y))
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state)
                elif self.roundCompass() == -90:
                    self.not_visited_pos.append((x,y-2))    
                    self.known_pos.add((x,y-2))
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == 90:
                    self.not_visited_pos.append((x,y+2))    
                    self.known_pos.add((x,y+2))
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
        elif self.positions[0] == 1:
            if self.positions[2] == 1 and self.positions[3] == 1:
                if self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == 0:
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
                elif self.roundCompass() == 90:
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)
                elif self.roundCompass() == -90:
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state)
            elif self.positions[2] == 1 and self.positions[3] == 0 :
                if self.roundCompass() == -90:
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == 0:
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state)
                elif self.roundCompass() == 90:
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
                elif self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)    
            elif self.positions[2] == 0 and self.positions[3] == 1:
                if self.roundCompass() == 90:
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == 0:
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)
                elif self.roundCompass() == -90:
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
                elif self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state)
            elif self.positions[2] == 0 and self.positions[3] == 0:                            
                if self.roundCompass() == 0:
                    self.not_visited_pos.append((x,y-2))    
                    self.known_pos.add((x,y-2))  
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state)         
                elif self.roundCompass() == 90:
                    self.not_visited_pos.append((x+2,y))
                    self.known_pos.add((x+2,y))
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
                elif self.roundCompass() == -90:
                    self.not_visited_pos.append((x-2,y))
                    self.known_pos.add((x-2,y))
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.not_visited_pos.append((x,y+2))
                    self.known_pos.add((x,y+2))
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)        
        self.current_state = self.position_goal

    def calc_next(self, angle, position):
        goalPos = [0,0]
        if angle == 0:
            #print("if1")
            goalPos = [position[0] + 2, position[1]]   
        elif angle == 180 or angle == -180:
            #print("if2")
            goalPos = [position[0] - 2, position[1]]
        elif angle == 90:
            #print("if3")
            goalPos = [position[0], position[1] + 2]
        elif angle == -90:
            #print("if4")
            goalPos = [position[0], position[1] - 2]

        return (self.move(goalPos,angle))
    
    def move(self,goalPos,angle):
        move = True
        while move:
            currentPos = (self.measures.x, self.measures.y)
            err = self.errorCalc(angle,currentPos)
            self.driveMotors(0.15 - err, 0.15 +err)
            self.readSensors()
            if(angle == 0):
                if(abs(currentPos[0] - goalPos[0]) < 0.4):
                    self.driveMotors(0.0,0.0)
                    #print("ANGULO 0")
                    move  = False
            elif(angle == 180 or angle == -180):
                if(abs(currentPos[0] - goalPos[0]) < 0.4):
                    self.driveMotors(0.0,0.0)
                    #print("ANGULO 180")
                    move  = False
            elif(angle == 90):
                if(abs(currentPos[1] - goalPos[1]) < 0.4):
                    self.driveMotors(0.0,0.0)
                    #print("ANGULO 90")
                    move  = False
            elif(angle == -90):
                if(abs(currentPos[1] - goalPos[1]) < 0.4):
                    #print("ANGULO -90")
                    self.driveMotors(0.0,0.0)
                    move  = False          

        self.driveMotors(0.00,0.00)
        return goalPos

    def follow_path(self):        
        while len(self.path) > 0:
            x2 = int(self.current_state[0] - self.initial_state[0])
            y2 = int(self.current_state[1] - self.initial_state[1])

            pos = self.path.pop(0)
            objetivo = [self.initial_state[0] + pos[0] , self.initial_state[1] + pos[1]]
            
            dif_x = x2 - pos[0] 
            dif_y = y2 - pos[1]                                                  
            
            if dif_x == 2 and dif_y == 0:                                            
                self.rotate(180)
                #print("180")
                (x2,y2) = self.move(objetivo,180)
            elif dif_y == 2 and dif_x == 0: 
                self.rotate(-90)
                #print("-90")
                (x2,y2) = self.move(objetivo,-90)
            elif dif_y == -2 and dif_x == 0:
                self.rotate(90)
                #print("90")
                (x2,y2) = self.move(objetivo,90)
            elif dif_x == -2 and dif_y == 0:
                self.rotate(0)
                #print("0")
                (x2,y2) = self.move(objetivo,0)
            
            self.current_state = objetivo 
           
        self.isLooping = False
        self.path.clear()

    def roundCompass(self):
        if -10 < self.measures.compass < 10:
            return 0
        elif 80 < self.measures.compass < 100:
            return 90
        elif -100 < self.measures.compass < -80:
            return -90
        elif self.measures.compass <= -170 or self.measures.compass >= 170:
            return 180 * self.measures.compass / abs(self.measures.compass)

    def pathfind(self,maze):
        visited = True

        if len(self.not_visited_pos) == 0 or self.isDone():
            self.finish()
            visited = False
            return None
        
        while(visited):
            goal = self.not_visited_pos.pop(-1)
            if goal not in self.visited_pos:
                visited = False
            
        x = -int(self.initial_state[0] - self.current_state[0])
        y = -int(self.initial_state[1] - self.current_state[1])

        start = (x,y)
        
        self.path = astar(start,goal,maze,self.walls)
        self.path = list(reversed(self.path))

        return self.path

    def pathToBeacon(self):
        #self.beacons.popitem()
        listaPos = list(self.beacons.keys())
        #print("LEN: ", len(listaPos))
        #print("LISTA ANTES DE FOR: ", listaPos)
        for i in range(1,len(self.beacons.values())):
            self.beaconId.append(i)
        
        #print("BEACON: ", self.beaconId)
        perm = list(permutations(self.beaconId))
        
        #print("PERM: ", perm

        for i in perm:
            aux1 = astar((0,0),listaPos[i[0]],self.visited_pos,self.walls)
            oldBeacon = listaPos[i[0]]
            for k in range(len(i)-1):
                aux1 = astar(oldBeacon, listaPos[i[k]], self.visited_pos, self.walls) + aux1
                oldBeacon = listaPos[i[k]]
            aux1 = astar(oldBeacon, (0,0), self.visited_pos, self.walls) + aux1  
            if len(self.pathBeacons) == 0:
                self.pathBeacons = aux1.copy()
            elif len(self.pathBeacons) == len(aux1):
                self.pathBeacons = aux1.copy()

    def isDone(self):
        count = 0
        for i in self.not_visited_pos:
            if i in self.visited_pos:
                count += 1
        
        if count == len(self.not_visited_pos):
            return True

        return False

    def pathWrite(self):
        f = open(self.filename + ".path", "w")
        for i in self.pathBeacons:
            f.write(str(i[0]) + " " + str(i[1]))
            if i in self.beacons.keys():
                num = self.beacons.get(i)
                f.write(" #" + str(num))
            f.write("\n")

    def design(self):
        f= open(self.filename + ".map", "w")
        for row in self.arr:
            for elem in row:
                if(elem == 1):
                    f.write(' ')
                else:
                    f.write(elem)
                    
            f.write('\n')

    def isLoop(self):
        x = -int(self.initial_state[0] - self.current_state[0])
        y = -int(self.initial_state[1] - self.current_state[1])

        if self.roundCompass() == 0:
            if (x+2,y) in self.visited_pos and (x-2,y) in self.visited_pos and (x,y+2) in self.visited_pos and (x,y-2) in self.visited_pos:
                self.isLooping = True
                
        elif self.roundCompass() == 180 or self.roundCompass() == -180:
            if (x+2,y) in self.visited_pos and (x-2,y) in self.visited_pos and (x,y+2) in self.visited_pos and (x,y-2) in self.visited_pos:
                self.isLooping = True
                
        elif self.roundCompass() == 90:
            if (x+2,y) in self.visited_pos and (x-2,y) in self.visited_pos and (x,y+2) in self.visited_pos and (x,y-2) in self.visited_pos:
                self.isLooping = True
                
        elif self.roundCompass() == -90:
            if (x+2,y) in self.visited_pos and (x-2,y) in self.visited_pos and (x,y+2) in self.visited_pos and (x,y-2) in self.visited_pos:
                self.isLooping = True
        else:
            self.isLooping = False 


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:     # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:                # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1

rob_name = "pClient"
host = "localhost"
pos = 1
mapc = None
filename = "mapping"
for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        filename = str((sys.argv[i + 1]))
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host,filename)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
