import sys
from croblink import *
from math import *
import numpy 
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

#last_valid_state = (0,0)  
class MyRob(CRobLinkAngs):
    Matrix =  numpy.zeros((28,56),dtype='U1')  
    walls = {}
    goal_state = (0,0)
    initial_state = (0,0)
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
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)

                self.initial_state = (self.measures.x,self.measures.y)
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
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        current_state = (self.measures.x-self.initial_state[0],self.measures.y-self.initial_state[1])
        #print(current_state)
        #if ((current_state[0]*10)%2) == 0.0 or ((current_state[1]*10)%2) == 0.0:
            #self.moveHor()
        list = self.searchWall(current_state)

        if list[0] == 1 and list[2] == 1 and list[3] == 0:
            if self.measures.compass > 85 and self.measures.compass < 95:
                self.driveMotors(0.0, 0.0)
                self.moveVer()
            else:
                self.driveMotors(-0.01, +0.01)
        elif list[0] == 1 and list[2] == 0 and list[3] == 1:
            if self.measures.compass > -85 and self.measures.compass < -95:
                self.driveMotors(0.0, 0.0)
                self.moveVer()
            else:
                self.driveMotors(-0.01, +0.01)               
        else:
            self.moveHor()



        """ self.Matrix[27][1] = 'x'
        print(self.Matrix)
        #print(list[0])
        if list[0] == 1 :
            #print("0000000000000000000000000000000000000000000000000000")
            self.goal_state = (self.goal_state[0] + 2, self.goal_state[1])
            self.moveVer()
            
            
            
            

        elif list[2] == 0 :
            print("222222222222222222222222222222222222222222222222222222222")
            self.goal_state = (self.goal_state[0], self.goal_state[1]- 2)  
            self.rotateUp()
            self.moveHor()
            
            
        elif list[1] == 0 :
            print("1111111111111111111111111111111111111111111111111")
            self.goal_state = (self.goal_state[0] - 2, self.goal_state[1])
            self.moveHor()
        elif list[3] == 0:
            print("3333333333333333333333333333333333333333333333333333333333")
            if self.rotateUp():
                self.moveVer()
                for i in self.walls:
                    print(i)  
        

        else:
            #print("EEEEEEEEEEEEEEEEEEEEEEEELLLLLLLLLLLLLLLLLLLLLLLLLLSSSSSSSSSSSSSSSEEEEEEEEEEEEE")
            self.moveHor()

        #print("SAI")
        sys.exit() """
                    
        
        
            
       


    # métodos para orientar o robot
    def rotateUp(self): 
        for i in range(1000): 
            
            #print(self.measures.compass)
            if self.measures.compass == 90:
                self.driveMotors(0.0,0.0)
                print("TA AQUI ")
                return True
                
            else:
                self.driveMotors(-0.01,+0.01)
            
    def rotateDown(self):
        if self.measures.compass < -95 or self.measures.compass > -85:
            self.driveMotors(-0.1,+0.1)
            return True
        else:
            return False
        
    def rotateLeft(self):
        if self.measures.compass < -175 or self.measures.compass > 175:
            self.driveMotors(-0.1,+0.1)
            return True
        else:
            return False

    def rotateRight(self):
        if self.measures.compass < -5 or self.measures.compass > 5:
            self.driveMotors(-0.1,+0.1)
            return True
        else:
            return False

    # métodos para descobrir paredes

    def searchWall(self,state):
        list = []                   # frente, tras, direita, esquerda
        center_id = 0 
        back_id = 3
        right_id = 2
        left_id = 1
        self.driveMotors(0.0,0.0)
        if -5 < self.measures.compass < 5 :
            if self.measures.irSensor[center_id] < 2.17:
                ##print("Não há parede em frente")
                list.append(0)
            else:
                ##print("Há parede em frente.")
                list.append(1)
            if self.measures.irSensor[back_id] < 2.17:
                ##print("Não há parede atrás")
                list.append(0)
            else:
                ##print("Há parede atrás")
                list.append(1)
            if self.measures.irSensor[right_id] < 2.17:
                ##print("Não há parede à direita")
                list.append(0)
            else:
                ##print("Há parede à direita.")
                list.append(1)
            if self.measures.irSensor[left_id] < 2.17:
                #print("Não há parede à esquerda")
                list.append(0)
            else:
                #print("Há parede à esquerda.")
                list.append(1)
        elif 85 < self.measures.compass < 95:
            if self.measures.irSensor[right_id] < 2.17:
                #print("Não há parede em frente")
                list.append(0)
            else:
                #print("Há parede em frente.")
                list.append(1)

            if self.measures.irSensor[left_id] < 2.17:
                #print("Não há parede atrás")
                list.append(0)
            else:
                #print("Há parede atrás")
                list.append(1)

            if self.measures.irSensor[back_id] < 2.17:
                #print("Não há parede à direita")
                list.append(0)
            else:
                #print("Há parede à direita.")
                list.append(1)

            if self.measures.irSensor[center_id] < 2.17:
                #print("Não há parede à esquerda")
                list.append(0)
            else:
                #print("Há parede à esquerda.")
                list.append(1)
        elif -175 < self.measures.compass < 175:
            if self.measures.irSensor[back_id] < 2.17:
                #print("Não há parede em frente")
                list.append(0)
            else:
                #print("Há parede em frente.")
                list.append(1)
            if self.measures.irSensor[center_id] < 2.17:
                #print("Não há parede atrás")
                list.append(0)
            else:
                #print("Há parede atrás")
                list.append(1)

            if self.measures.irSensor[left_id] < 2.17:
                #print("Não há parede à direita")
                list.append(0)
            else:
                #print("Há parede à direita.")
                list.append(1)
            if self.measures.irSensor(right_id) < 2.17:
                #print("Não há parede à esquerda")
                list.append(0)
            else:
                #print("Há parede à esquerda.")
                list.append(1)
        elif -85 < self.measures.compass < -95:
            if self.measures.irSensor[left_id] < 2.17:
                #print("Não há parede em frente")
                list.append(0)
            else:
                #print("Há parede em frente.")
                list.append(1)

            if self.measures.irSensor[right_id] < 2.17:
                #print("Não há parede atrás")
                list.append(0)
            else:
                #print("Há parede atrás")
                list.append(1)

            if self.measures.irSensor[center_id] < 2.17:
                #print("Não há parede à direita")
                list.append(0)
            else:
                #print("Há parede à direita.")
                list.append(1)

            if self.measures.irSensor[back_id] < 2.17:
                #print("Não há parede à esquerda")
                list.append(0)
            else:
                #print("Há parede à esquerda.")
                list.append(1)

        self.walls[state] = list
        return list
    
    # mover o robot
        
    def moveHor(self):
        self.driveMotors(0.1,0.1)        
        #while self.measures.x - 0.00001 < self.goal_state[0] < self.measures.x + 0.00001:
        if isclose(self.measures.x,self.goal_state[0]) and isclose(self.measures.y,self.goal_state[1]):
            print("CHEGOU AO SEU DESTINO")
            self.driveMotors(0.0,0.0)
            return True
    
    def moveVer(self):
        self.driveMotors(0.01,0.01)
        
    def mapping(self):
        self.Matrix[13,27] == 'X'
        for i in self.walls:
            if self.walls == [1]:                 
                self.Matrix[i[0]+13][i[1]+27] = 'X'
                self.Matrix[i[0]+14][i[1]+27] = '|'

                
                
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