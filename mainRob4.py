from pickle import FALSE
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
    position_goal = (0.0,0.0)    #estado goal   
    initial_state = (0.0,0.0)    #estado inicial
    current_state = (0.0,0.0)    #estado em que o robo se encontra
    not_visited_pos = []     #posicoes conhecidas para as quais o robo ainda nao foi 
    arr = [[1 for i in range(55)] for j in range(27)]   #array que vai desenhar o mapa
    firstrun = True          #variavel que indica se é o primeiro ciclo para colocar no array I em vez de X
    isLooping = False 
    last_pos = (0.0,0.0)
    lastOut = (0.0,0.0)          #ultimo parametro enviado para o driveMotors  
    current_cell = (0,0)     #celula 
    beacons = {}
    pathBeacons = []            #posicoes que fazem o caminho mais curto entre os targets
    beaconId = []
    path = [] 

    def __init__(self,rob_name, rob_id, angles, host, filename):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host,filename)
        
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
                print("Nº BEACONS " + str(self.nBeacons))
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
                #self.driveMotors(0.0,0.0)
            
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()      
            
    def wander(self):
        
        path = []  
        self.mapping()
        print("ORIENTATION"+ str(self.measures.compass))
        print("CURRENT_STATE = " + str(self.current_state))
        print("GPS X = " + str(-(self.initial_state[0] - self.measures.x)) + "GPS Y = " + str(-(self.initial_state[1] - self.measures.y)))
        print("centrado? " + str(self.is_centered_cell()) + str(self.current_cell))
        self.positions.clear()
        if not self.is_centered_cell() and not self.firstrun:
            self.correct_pos()    
            print("CORRECT STATE = " + str(self.current_state)) 
        dif = [self.current_state[0] - (-(self.initial_state[0] - self.measures.x)), self.current_state[1] - (-(self.initial_state[1] - self.measures.y))]
        print("Diference = " + str(dif))
        
        print("__________________________________________")
        #print("CURRENT CELL = " + str(self.current_cell))
        #print(self.walls)
        self.design()
        self.isLoop()
        if(self.isLooping):
            path = self.pathfind(self.known_pos)
            self.follow_path(path)

        if (int)(self.simTime)-(self.measures.time) < 400:
            path = self.path_to_zero(self.visited_pos)
            print("PATH",path)
            self.follow_path(path)
            self.finish()
        self.positions.clear()
       


    def rotate(self, angle):
        print("ANGLE OF ROTATION " + str(angle))
        #print("COMPASS"+ str(self.measures.compass))
        #err = self.compassErrCalc(angle)
        if(angle == 90):
            #print("rotating to 90 ")
            if self.measures.compass >= 90:    
                while self.measures.compass > 90:
                    
                    self.driveMotors(+0.03,-0.03 )
                    self.readSensors()
            else:
                while self.measures.compass < 90:
    
                    self.driveMotors(-0.03,+0.03 )
                    self.readSensors()
        
        elif(angle == -90):
            #print("rotating to -90")
            if self.measures.compass >= 45 or self.measures.compass < -90: 
                while self.measures.compass >= 45 or self.measures.compass < -90:  
                    self.driveMotors(-0.03 ,+0.03 )
                    self.readSensors()
            else:
                while self.measures.compass  < 45 and self.measures.compass > -90:  
                    self.driveMotors(+0.03 ,-0.03 )
                    self.readSensors()
        elif(angle == 0):
            #print("rotating to 0")
            if self.measures.compass <= 0:
                while self.measures.compass < -1:
    
                    self.driveMotors(-0.03 ,+0.03)
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
                    self.driveMotors(-0.03 ,+0.03 )
                    self.readSensors()
            
        
        #self.driveMotors(0.00 - err,-0.00 + err)  
    def errorCorrectionSensors(self, direction):  
        right_id = 1
        left_id = 2  
        errorDir = direction - self.measures.compass  # directional error
        if(self.measures.compass < 0 and direction == 180):
            errorDir =  - (direction - abs(self.measures.compass))  # directional error
        errorGeo = 0
        if self.measures.irSensor[left_id] > 2.1:
            errorGeo =  2.1 - self.measures.irSensor[left_id]
            # print("CORRECT CLOSE TO WALL LEFT")
        elif self.measures.irSensor[right_id] > 2.1:
            errorGeo = self.measures.irSensor[right_id] - 2.1
            # print("CORRECT CLOSE TO WALL RIGHT")
        return errorDir*0.01 + errorGeo*0.1
    # método para descobrir paredes
    def searchWall(self):
        center_id = 0 
        back_id = 3
        right_id = 2
        left_id = 1
        x = round(self.current_state[0])
        y = round(self.current_state[1])
        #print("X =" + str(x) + "Y =" + str(y))
        
        #print("SEARCH WALLS...")
        if (self.measures.irSensor[center_id] + self.measures.irSensor[back_id]) /2 < 1.3:
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
        if self.measures.irSensor[right_id] < 1.1:
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
        if self.measures.irSensor[left_id] < 1.1:
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
            self.beacons[int(self.measures.ground)] = (x,y)
        else:
            self.arr[13-y][27+x] = 'X'
        self.visited_pos.append((x,y))
        self.known_pos.add((x,y))

    # def compassErrCalc(self,angle):
    #     if(self.measures.compass < 0 and angle == 180):
    #         return  (- (angle - abs(self.measures.compass))) * 0.01
    #     return (angle - self.measures.compass)*0.01

    # def GpsErrorCalc(self):
    #     right_id = 2
    #     left_id = 1
    #     center_id = 0
    #     errorGPS = 0
    #     if self.measures.irSensor[left_id] > 2.1:
    #         errorGPS =  2.1 - self.measures.irSensor[left_id]
    #     elif self.measures.irSensor[right_id] > 2.1:
    #         errorGPS = self.measures.irSensor[right_id] - 2.1
    #     elif self.measures.irSensor[center_id] > 2.1:
    #         errorGPS = self.measures.irSensor[center_id] - 2.1 

    #     return errorGPS*0.15

    def mapping(self):
        x = self.current_state[0]
        y = self.current_state[1]   
        if(self.firstrun):
            self.rotate(self.opposite_angle())
            self.searchWall()
            self.rotate(self.opposite_angle())
            self.firstrun = False

        self.readSensors() 
        self.searchWall()
        
        if self.positions[0] == 0:
            if self.positions[1] == 1 and self.positions[2] == 1:
                if self.roundCompass() == 180 or self.roundCompass() == -180:
                    self.position_goal = self.calc_next(180,self.current_state)
                    #print("GOAL: ", int(self.position_goal[0] - self.current_state[0]), ",", int(self.position_goal[1] - self.current_state[1]))
                elif self.roundCompass() == 0:
                    self.position_goal = self.calc_next(0,self.current_state)
                elif self.roundCompass() == -90:
                    self.position_goal = self.calc_next(-90,self.current_state)
                elif self.roundCompass() == 90:
                    self.position_goal = self.calc_next(90,self.current_state)
            elif self.positions[1] == 0 and self.positions[2] == 1:
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
            elif self.positions[1] == 1 and self.positions[2] == 0:
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
            if self.positions[1] == 1 and self.positions[2] == 1:
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
            elif self.positions[1] == 1 and self.positions[2] == 0 :
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
            elif self.positions[1] == 0 and self.positions[2] == 1:
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
            elif self.positions[1] == 0 and self.positions[2] == 0:                            
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

    def opposite_angle(self):
        angle = (self.roundCompass() + 180) % 360
        if angle == 270 :
            angle = -90
        return angle


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

        return(self.move(goalPos,angle))
    
    def move(self,goalPos,angle):
        move = True
        while move:
            #if self.roundCompass() == 0 or self.roundCompass == 180:
            currentPos = self.align(0.07,self.measures.compass,0.01,self.roundCompass())
            #self.align(0.10,self.measures.compass,0.01,self.roundCompass())
            #currentPos = self.next_pos_calc()

            self.readSensors()
            # print("Current State = "+ str(currentPos))
            # print("Goal State = "+ str(goalPos))
            # print("GPS X = " + str(-(self.initial_state[0] - self.measures.x)) + "GPS Y = " + str(-(self.initial_state[1] - self.measures.y)))
            # dif = [currentPos[0] - (-(self.initial_state[0] - self.measures.x)), currentPos[1] - (-(self.initial_state[1] - self.measures.y))]
            # print("Diference = " + str(dif))
            # print("_________________________________________________________")
            #print("Diference = ",(abs(currentPos[0] - goalPos[0]),abs(currentPos[1] - goalPos[1])))
            #print("goalPos = ", [round(goalPos[0]),round(goalPos[1])])
            if(angle == 0):
                #err = self.compassErrCalc(angle)
                if(abs(currentPos[0] - round(goalPos[0])) < 0.1):
                    #self.driveMotors(0.0 - err,0.0 + err)
                    #print("ANGULO 0")
                    move  = False
            elif(angle == 180 or angle == -180):
                #self.align(0.10,self.measures.compass,0.01,self.roundCompass())
                #err = self.compassErrCalc(angle)
                if(abs(currentPos[0] - round(goalPos[0])) < 0.1):
                    #self.driveMotors(0.0 - err,0.0 + err)
                    #print("ANGULO 180")
                    move  = False
            elif(angle == 90):
                #err = self.compassErrCalc(angle)
                if(abs(currentPos[1] - round(goalPos[1])) < 0.1):
                    #self.driveMotors(0.0 - err,0.0 + err)
                    #print("ANGULO 90")
                    move  = False
            elif(angle == -90):
                #err = self.compassErrCalc(angle)
                if(abs(currentPos[1] - round(goalPos[1])) < 0.1):
                    #print("ANGULO -90")
                    #self.driveMotors(0.0 - err,0.0 + err)
                    move  = False          

        return goalPos

    def next_pos_calc(self):
        angle = self.roundCompass()
        #templastOut = self.lastOut
        #self.lastOut = ((self.current_motors[0] + templastOut[0]) / 2, self.current_motors[1] + templastOut[1] /2)
        
        if self.last_pos == (0,0):
            lin = (self.lastOut[0]+self.lastOut[1])/2 /2
        else:
            lin = (self.lastOut[0] + self.lastOut[1]) / 2 

        xt = self.last_pos[0] + lin * cos(radians(angle))
        yt = self.last_pos[1] + lin * sin(radians(angle))
        self.last_pos = (xt,yt)
        return (xt,yt)
  
    def align(self,lin,m,k,angle):
        rot = k*(m-angle)
        rw = lin-(rot/2)
        lw = lin+(rot/2)
        self.driveMotors(lw,rw)
        (x,y) = self.next_pos_calc()
        #self.current_motors = (lw,rw)
        self.lastOut = (lw,rw)
        return (x,y)
    
    def is_centered_cell(self):
        if (self.current_cell == (self.current_state[0]-1, self.current_state[1])) or (self.current_cell == (self.current_state[0],self.current_state[1] - 1)):
            return True
        if (self.current_cell == (self.current_state[0] + 1, self.current_state[1])) or (self.current_cell == (self.current_state[0],self.current_state[1] + 1)):
            return True
        return False

    def dist_to_wall(self, id):
        return 1/self.measures.irSensor[id]

    def correct_pos(self):
        center_id = 0 
        back_id = 3
        right_id = 2
        left_id = 1
        (x,y) = (self.current_state[0], self.current_state[1])
        print("BEFORE CORRECTION = " + str((x,y)))
        self.searchWall()
        if self.roundCompass()==0:
            if self.positions[0]==1:
                print("IF 0 - Parede a frente")
                wall = (x+1, y)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[0] = round(wall[0])-((self.dist_to_wall(center_id) + self.dist_to_wall(back_id))/2 + 0.5)
            if self.positions[1] == 1:
                print("IF 0 - Parede a direita")
                wall = (x, y-1)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                print(self.dist_to_wall(right_id))
                self.current_state[1] = round(wall[1]) - (self.dist_to_wall(right_id) + 0.5)
            if self.positions[2]==1:
                print("IF 0 - Parede a esquerda")
                wall = (x, y+1)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[1] = round(wall[1]) - (self.dist_to_wall(left_id) + 0.5)

        if self.roundCompass()==-180 or self.roundCompass() == 180:
            if self.positions[0]==1:
                wall = (x-1, y)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[0] = round(wall[0])+((self.dist_to_wall(center_id) + self.dist_to_wall(back_id))/2 + 0.5)

            if self.positions[1] == 1:
                wall = (x, y+1)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[1] = round(wall[1]) - (self.dist_to_wall(right_id) + 0.5)
            if self.positions[2]==1:
                wall = (x, y-1)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[1] = round(wall[1]) + (self.dist_to_wall(left_id) + 0.5)

        if self.roundCompass()==90:
            if self.positions[0]==1:
                print("IF 90 - Parede a frente")
                wall = (x, y+1)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[1] = round(wall[1]) - ((self.dist_to_wall(center_id) + self.dist_to_wall(back_id))/2 + 0.5) 

            if self.positions[1] == 1:
                print("IF 90 - Parede a direita")
                wall = (x-1, y)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[0] = round(wall[0]) - (self.dist_to_wall(right_id) + 0.5)
            if self.positions[2]==1:
                print("IF 90 - Parede a esquerda")
                wall = (x+1, y)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[0] = round(wall[0]) + (self.dist_to_wall(left_id) + 0.5)

        if self.roundCompass()==-90:
            if self.positions[0]==1:
                wall = (x, y-1)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[1] = round(wall[1]) + ((self.dist_to_wall(center_id) + self.dist_to_wall(back_id))/2+0.5) 

            if self.positions[1] == 1:
                wall = (x+ 1, y)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[0] = round(wall[0]) + (self.dist_to_wall(right_id) + 0.5)
            if self.positions[2]==1:
                wall = (x-1, y)
                print("WALL POSITION", [round(wall[0]),round(wall[1])])
                self.current_state[0] = round(wall[0]) - (self.dist_to_wall(left_id) + 0.5)

        #self.current_state = (round(self.current_state[0]),round(self.current_state[1]))
        #return self.current_state
    def follow_path(self,path):
        print("LOOP IS TRUE")

        while len(path) > 0:
            x2 = int(self.current_state[0] - self.initial_state[0])
            y2 = int(self.current_state[1] - self.initial_state[1])

            print("x2: " , x2 , "y2: " , y2)    # 12, -8

            pos = path.pop(0)
            objetivo = [self.initial_state[0] + pos[0] , self.initial_state[1] + pos[1]]
            
            print("obj_x: " , int(objetivo[0] - self.initial_state[0]), "obj_y: ", int(objetivo[1] - self.initial_state[1])) 

            dif_x = x2 - pos[0] 
            dif_y = y2 - pos[1]                                                  
            
            if dif_x == 2 and dif_y == 0:                                            
                self.rotate(180)
                print("180")
                (x2,y2) = self.move(objetivo,180)
            elif dif_y == 2 and dif_x == 0: 
                self.rotate(-90)
                print("-90")
                (x2,y2) = self.move(objetivo,-90)
            elif dif_y == -2 and dif_x == 0:
                self.rotate(90)
                print("90")
                (x2,y2) = self.move(objetivo,90)
            elif dif_x == -2 and dif_y == 0:
                self.rotate(0)
                print("0")
                (x2,y2) = self.move(objetivo,0)
            
            self.current_state = objetivo 
           

        print("LOOOP IS FALSE")
        self.isLooping = False
        path.clear()
            

    def roundCompass(self):
        if -15 < self.measures.compass < 15:
            return 0
        elif 75 < self.measures.compass< 105:
            return 90
        elif -105 < self.measures.compass <-75:
            return -90
        elif self.measures.compass <= -170 or self.measures.compass >= 170:
            return 180 * self.measures.compass / abs(self.measures.compass)

    def pathfind(self,maze):
        visited = True
        while(visited):
            goal = self.not_visited_pos.pop()
            if goal not in self.visited_pos:
                visited = False
            
        x = int(self.current_state[0])
        y = int(self.current_state[1])
        
        start = (x,y)
        
        path = astar(start,goal,maze,self.walls)
        path = list(reversed(path))
        
        print("path: " , path)

        return path
    
    def path_to_zero(self,maze):
       
        goal = (0,0)
            
        x = int(self.current_state[0])
        y = int(self.current_state[1])
        
        start = (x,y)
        
        path = astar(start,goal,maze,self.walls)
        path = list(reversed(path))
        
        print("path: " , path)

        return path

    def design(self):
        f= open(self.filename + ".map", "w")
        for row in self.arr:
            for elem in row:
                if(elem == 1):
                    f.write(' ')
                else:
                    f.write(elem)
                    
            f.write('\n')

    # def isLoop2(self):
    #     possibleLoop = 0
    #     c = Counter(self.visited_pos)
    #     for i in c:
    #         if c[i] >= 2:
    #             possibleLoop+=1
    #         else:
    #             possibleLoop = 0
    #         if possibleLoop == 5:
    #             self.isLooping = True

    def isLoop(self):
        x = int(self.current_state[0])
        y = int(self.current_state[1])

        #print("VISITED POS: " , self.visited_pos)

        """ if (x+2,y) in self.visited_pos and (x-2,y) in self.visited_pos and (x,y+2) in self.visited_pos and (x,y-2) in self.visited_pos:
            print("SELFISLOOOOPOOOING")
            self.isLooping = True
        else:
            self.isLooping = False """

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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,0.0],host,filename)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

