import sys
from typing import Counter
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from astar import *


CELLROWS=7
CELLCOLS=14

#last_valid_state = (0,0)  
class MyRob(CRobLinkAngs):
    positions = []           #lista em que 1 é ter parede e 0 é nao ter em cada posicao goal com a seguinte estrutura [frente, tras, direita, esquerda]
    visited_pos = []         #lista com as posiçoes visitadas
    known_pos = set()        #set com as posiçoes conhecidas
    walls = set()            #set com as paredes conhecidas
    position_goal = (0,0)    #estado goal   
    initial_state = (0,0)    #estado inicial
    current_state = (0,0)    #estado em que o robo se encontra
    not_visited_pos = []     #posicoes conhecidas para as quais o robo ainda nao foi 
    arr = [[1 for i in range(55)] for j in range(27)]   #array que vai desenhar o mapa
    firstrun = True          #variavel que indica se é o primeiro ciclo para colocar no array I em vez de X
    isLooping = False 
    last_pos = (0,0)
    lastOut = (0,0)          #ultimo parametro enviado para o driveMotors  
    current_cell = (0,0)     #celula 
    def __init__(self,rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

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
                self.position_goal = (0.0,0.0)
                self.current_state = (0.0,0.0)
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

        path = []  
        self.mapping()
        #print(self.walls)
        self.design()
        self.isLoop()
        if(self.isLooping):
            path = self.pathfind(self.known_pos)
            self.follow_path(path)
        self.positions.clear()
       

        
    def rotate(self, angle):
        print("ANGLE OF ROTATION " + str(angle))
        print("COMPASS"+ str(self.measures.compass))
        err = self.compassErrCalc(angle)
        if(angle == 90):
            #print("rotating to 90 ")
            if self.measures.compass >= 90:    
                while self.measures.compass > 90:
                    
                    self.driveMotors(+0.03,-0.03 )
                    self.readSensors()
            else:
                while self.measures.compass < 90:
    
                    self.driveMotors(+0.03,-0.03 )
                    self.readSensors()
        
        elif(angle == -90):
            #print("rotating to -90")
            if self.measures.compass >= 45 or self.measures.compass < -90: 
                while self.measures.compass >= 45 or self.measures.compass < -90:  
                    self.driveMotors(+0.03 ,-0.03 )
                    self.readSensors()
            else:
                while self.measures.compass  < 45 and self.measures.compass > -90:  
                    self.driveMotors(+0.03 ,-0.03 )
                    self.readSensors()
        elif(angle == 0):
            #print("rotating to 0")
            if self.measures.compass <= 0:
                while self.measures.compass < -1:
    
                    self.driveMotors(+0.03 ,-0.03 )
                    self.readSensors()
            else: 
                while self.measures.compass > 1:
                
                    self.driveMotors(+0.03 ,-0.03 )
                    self.readSensors()
        elif(angle == 180):
            #print("rotating to 180")   
            if self.measures.compass <= 0 :
                while self.measures.compass < 177 and self.measures.compass <=0:
                
                    self.driveMotors(+0.03 ,-0.03)
                    self.readSensors()
            else:
                while self.measures.compass > -177 and self.measures.compass >0:
                    self.driveMotors(+0.03 ,-0.03 )
                    self.readSensors()
            
            
        
        self.driveMotors(0.00 - err,-0.00 + err)  
        

    # método para descobrir paredes
    def searchWall(self):
        center_id = 0 
        back_id = 3
        right_id = 2
        left_id = 1
        x = int(self.current_state[0])
        y = int(self.current_state[1])
        print("X =" + str(x) + "Y =" + str(y))
        if self.firstrun : 
            self.arr[13][27] = 'I'
            self.firstrun = False
        else: 
            self.arr[13-y][27+x] = 'X'
        print("SEARCH WALLS...")
        print(self.measures.beaconReady)
        print(self.measures.beacon)
        if self.measures.irSensor[center_id] < 1.3:
            print("Não há parede em frente")
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
            print("Há parede em frente.")
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
            print("Não há parede atrás")
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
            print("Há parede atrás")
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
            print("Não há parede à direita")
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
            print("Há parede à direita.")
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
            print("Não há parede à esquerda")
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
            print("Há parede à esquerda.")
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

        self.visited_pos.append((x,y))
        self.known_pos.add((x,y))

    def compassErrCalc(self,angle):
        return (angle - self.measures.compass)*0.01

    def errorCalc(self, angle, pos):
        right_id = 2
        left_id = 1
        errorDir = angle - self.measures.compass  
        if(self.measures.compass < 0 and angle == 180):
            errorDir =  - (angle - abs(self.measures.compass))  
        errorGeo = 0
        if self.measures.irSensor[left_id] > 2.1:
            errorGeo =  2.1 - self.measures.irSensor[left_id]
        elif self.measures.irSensor[right_id] > 2.1:
            errorGeo = self.measures.irSensor[right_id] - 2.1
        return errorDir*0.01 + errorGeo*0.15

    def mapping(self):
        x = self.current_state[0]
        y = self.current_state[1]
        self.readSensors()
        self.searchWall()
        print("centrado? " + str(self.is_centered_cell()) + str(self.current_cell) )
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
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state)
                elif self.roundCompass() == 0:
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)
                elif self.roundCompass() == -90:
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
                elif self.roundCompass() == 90:
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
            elif self.positions[2] == 1 and self.positions[3] == 0:
                if self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.rotate(-90)
                    self.position_goal = self.calc_next(-90,self.current_state)
                    """ elif self.roundCompass() == 0:
                    self.not_visited_pos.append((x,y+2))    
                    self.known_pos.add((x,y+2))
                    self.rotate(90)
                    self.position_goal = self.calc_next(90,self.current_state) """
                elif self.roundCompass() == -90:
                    self.rotate(0)
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == 90:
                    self.rotate(180)
                    self.position_goal = self.calc_next(180,self.current_state)
                else:
                    self.position_goal = self.calc_next(0,self.current_state)
        elif self.positions[0] == 1:
            if self.positions[1] == 0 and self.positions[2] == 1 and self.positions[3] == 1:
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
            elif self.positions[1] == 0 and self.positions[2] == 1 and self.positions[3] == 0 :
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
                    self.not_visited_pos.append((x,y+2))    
                    self.known_pos.add((x,y+2))  
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
        #print("MOVE FRONT")
        goalPos = [0,0]
        if angle == 0:
            #print("if1")
            goalPos = [position[0] + 2, position[1]]   
            self.current_cell = (self.current_cell[0] + 1,self.current_cell[1])
        elif angle == 180 or angle == -180:
            #print("if2")
            goalPos = [position[0] - 2, position[1]]
            self.current_cell = (self.current_cell[0] - 1,self.current_cell[1])
        elif angle == 90:
            #print("if3")
            goalPos = [position[0], position[1] + 2]
            self.current_cell = (self.current_cell[0],self.current_cell[1] + 1)
        elif angle == -90:
            #print("if4")
            goalPos = [position[0], position[1] - 2]
            self.current_cell = (self.current_cell[0],self.current_cell[1] - 1)

        return(self.move(self.current_state,goalPos,angle))
    
    def move(self,currentPos,goalPos,angle):
        print("MOVE...")
        move = True
        while move:
            
            err = self.compassErrCalc(angle)
            self.driveMotors(0.1 - err , 0.1 + err)
            currentPos = self.next_pos_calc()
            self.lastOut = (0.1 - err,0.1 + err )
            self.readSensors()
            print("Current State = "+ str(currentPos))
            print("Goal State = "+ str(goalPos))
            print("GPS X = " + str(-(self.initial_state[0] - self.measures.x)) + "GPS Y = " + str(-(self.initial_state[1] - self.measures.y)))
            dif = [currentPos[0] - (-(self.initial_state[0] - self.measures.x)), currentPos[1] - (-(self.initial_state[1] - self.measures.y))]
            print("Diference = " + str(dif))
            print("_________________________________________________________")

            if(angle == 0):
                err = self.compassErrCalc(angle)
                if(abs(currentPos[0] - goalPos[0]) < 0.3):
                    self.driveMotors(0.0 - err,0.0 + err)
                    #print("ANGULO 0")
                    move  = False
            elif(angle == 180 or angle == -180):
                err = self.compassErrCalc(angle)
                if(abs(currentPos[0] - goalPos[0]) < 0.3):
                    self.driveMotors(0.0 - err,0.0 + err)
                    #print("ANGULO 180")
                    move  = False
            elif(angle == 90):
                err = self.compassErrCalc(angle)
                if(abs(currentPos[1] - goalPos[1]) < 0.3):
                    self.driveMotors(0.0 - err,0.0 + err)
                    #print("ANGULO 90")
                    move  = False
            elif(angle == -90):
                err = self.compassErrCalc(angle)
                if(abs(currentPos[1] - goalPos[1]) < 0.3):
                    #print("ANGULO -90")
                    self.driveMotors(0.0 - err,0.0 + err)
                    move  = False          

        return goalPos

    def next_pos_calc(self):
        angle = self.roundCompass()
        print("angle = " + str(angle))
        print(CELLCOLS)
        if self.last_pos == (0,0):
            lin = (self.lastOut[0]+self.lastOut[1])/2 /2
        else:
            lin = (self.lastOut[0] + self.lastOut[1]) / 2 

        xt = self.last_pos[0] + lin * cos(radians(angle))
        yt = self.last_pos[1] + lin * sin(radians(angle))
        self.last_pos = (xt,yt)
        return (xt,yt)
        #out = (ini + self.lastOut) /2

    #def next_pos_rotate(self):
    
    def is_centered_cell(self):
        if (self.current_cell == (self.current_state[0]-1, self.current_state[1])) or (self.current_cell == (self.current_state[0],self.current_state[1] - 1)):
            return True
        if (self.current_cell == (self.current_state[0] + 1, self.current_state[1])) or (self.current_cell == (self.current_state[0],self.current_state[1] + 1)):
            return True
        return False


    def follow_path(self,path):
        x = self.current_state[0]
        y = self.current_state[1]
        i = 0
        

        while len(path) > 0:     
            for pos in path:   
                objetivo = [self.current_state[0] - pos[0], self.current_state[1] - pos[1]]                                                    
                if (objetivo[0] - x == 0 and objetivo[1] - y == 0):
                    continue
                elif(x +objetivo[0]) > 0:                                            
                    self.rotate(0)
                    (x,y) = self.move(objetivo,0)
                elif(x +objetivo[0]) < 0:
                    self.rotate(180)
                    (x,y) = self.move(objetivo,180)
                elif(y + objetivo[1]) > 0: 
                    self.rotate(90)
                    (x,y) = self.move(objetivo,90)
                elif(y + objetivo[1]) < 0:
                    self.rotate(-90)
                    (x,y) =self.move(objetivo,-90)
                del path[i]
                i+=1
        self.isLooping = False 
            

    def roundCompass(self):
        if -10 < self.measures.compass < 10:
            return 0
        elif 80 < self.measures.compass < 100:
            return 90
        elif -100 < self.measures.compass < -80:
            return -90
        elif self.measures.compass <= -170 or self.measures.compass >= 170:
            return 180 * self.measures.compass / abs(self.measures.compass)
        return self.measures.compass
        
    def pathfind(self,maze):
        visited = True
        while(visited):
            goal = self.not_visited_pos.pop()
            if goal not in self.visited_pos:
                visited = False
            
        x =  self.current_state[0]
        y = self.current_state[1]
        
        start = (x,y)
        
        test =  (6,0)
        path = astar(start,goal,maze,self.walls)
        path = list(reversed(path))
        
        return path

    def design(self):
        f= open("mapping.out", "w")
        for row in self.arr:
            for elem in row:
                if(elem == 1):
                    f.write(' ')
                else:
                    f.write(elem)
                    
            f.write('\n')

    def isLoop(self):
        possibleLoop = 0
        c = Counter(self.visited_pos)
        for i in c:
            if c[i] >= 2:
                possibleLoop+=1
            else:
                possibleLoop = 0
            if possibleLoop == 5:
                self.isLooping = True
        
        
          
        

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
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

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
