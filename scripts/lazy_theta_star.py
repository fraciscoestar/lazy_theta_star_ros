#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import sqrt


class Planner:

    def __init__(self):

        # Load map
        self.map = np.genfromtxt('worlds/map.csv', delimiter=',')
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.resolution = 1

        # Print map
        image = np.flipud(self.map)
        plt.figure()
        plt.imshow(image, cmap=plt.get_cmap('binary'))
        plt.show()


    def bresenham(self, start_cell, goal_cell):
        #ALGORITMO DE BRESENHAM
        x1 = start_cell[0]
        y1 = start_cell[1]
        x2 = goal_cell[0]
        y2 = goal_cell[1]

        # print('Bresenham from ' + str(start_cell) + ' to ' + str(goal_cell))

        cells = [] # vector con las celdas por las que pasa la linea que une los puntos

        steep = abs(y2 - y1) > abs(x2 - x1) #si la pendiente de la recta es mayor a 1
        if steep:
            x1, y1 = y1, x1 # invertimos la recta
            x2, y2 = y2, x2

        if (x1 > x2):
            x1, x2 = x2, x1 #cambiamos los puntos para ir de izq a der
            y1, y2 = y2, y1

        deltaY = abs(y2 - y1)
        deltaX = x2 - x1
        error = 0

        y = y1
        ystep = 1
        if y1 > y2:
            ystep = -1

        for x in range(x1,x2+1):

            if steep:
                cells.append([y, x])
            else:
                cells.append([x, y])

            error = error + deltaY
            if (2*error >= deltaX): #hay un cambio en y
                y = y + ystep
                error = error - deltaX


        return cells

    def line_of_sight(self, cells):
        for c in cells:
            if self.map[c[1], c[0]] == 1:
                return False
        return True

    def setVertex(self, cell, close):
        celda_elegida = []

        padre_celda = [cell[2], cell[3]]
        celda = [cell[0], cell[1]]

        celdasCamino = self.bresenham(padre_celda, celda)
        line_of_sight = self.line_of_sight(celdasCamino)

        if not line_of_sight:
            vecinos = self.calcNearCells(celda)
            vecinos_closed = []
            celda_padre_anterior = [close[-1][2], close[-1][3]]
            for cell in vecinos:
                for i in range(len(close)):
                    if cell[0] == close[i][0] and cell[1] == close[i][1]: # esta en lista cerrada
                        vecinos_closed.append(cell)
                        vecinos_closed[-1].append(close[i][-1] + self.calcHn(celda_padre_anterior, cell)) # g(n) + 1
            min_g = 1000
            pos = 0
            for i in range(len(vecinos_closed)):
            # for i, cell in vecinos_closed:
                if vecinos_closed[i][-1] < min_g:
                    min_g = vecinos_closed[i][-1]
                    pos = i
            celda_elegida.append(vecinos_closed[pos][0]) # actualizo x padre
            celda_elegida.append(vecinos_closed[pos][1]) # actualizo y padre
            celda_elegida.append(min_g) # actualizo g(n) padre

        return celda_elegida

    def compute_path(self, start_cell, goal_cell):
        path = []

        open = []
        close = []

        path_found = False
        openclose = 0
        listPos = 0

        open.append(start_cell)
        open[0].append(start_cell[0]) #padre de la inicial es ella misma
        open[0].append(start_cell[1]) #padre de la inicial es ella misma
        open[0].append(self.calcHn(start_cell, goal_cell)) # h(n) de celda origen
        open[0].append(0) # g(n) de celda origen

        while not path_found:

            close.append(open[0])  # anado la primera posicion de la lista abierta a la cerrada
            open.pop(0)  # elimino el primer elemento de la abierta

            celda_elegida = self.setVertex(close[-1], close)

            if len(celda_elegida) is not 0:
                close[-1][2] = celda_elegida[0] # actualizo x padre
                close[-1][3] = celda_elegida[1] # actualizo y padre
                close[-1][-1] = celda_elegida[2] # actualizo g(n) padre

            if close[-1][0] == goal_cell[0] and close[-1][1] == goal_cell[1]:  # si la celda es la final, acabo
                path_found = True
                print('Camino encontrado!')
                continue
            else:
                # calculo las celdas adyacentes
                celdasAdyacentes = self.calcNearCells(close[-1])

                for cell in celdasAdyacentes:
                    openclose = 0
                    for i in range(len(open)):
                        if cell[0] == open[i][0] and cell[1] == open[i][1]: # esta en lista abierta
                            openclose = 1
                            listPos = i
                    for i in range(len(close)):
                        if cell[0] == close[i][0] and cell[1] == close[i][1]: # esta en lista cerrada
                            openclose = 2
                            listPos = i
                    if openclose != 1 and openclose != 2:
                        openclose = 0 # no esta en ninguna lista


                    celda_padre_anterior = [close[-1][2], close[-1][3]]
                    if openclose == 0:
                        open.append(cell)
                        open[-1].append(celda_padre_anterior[0]) # anado x celda padre
                        open[-1].append(celda_padre_anterior[1]) # anado y celda padre
                        open[-1].append(self.calcHn(cell, goal_cell)) # calculo h(n)
                        open[-1].append(close[-1][-1] + self.calcHn(celda_padre_anterior, cell)) # close[-1][-1] es el g(n) de la celda padre


                    elif openclose == 1: # esta en la lista abierta
                        Gn_nueva = close[-1][-1] + 1 # gn del nuevo posible padre
                        if Gn_nueva < open[listPos][-1]:
                            open[listPos][-1] = Gn_nueva # actualizo gn
                            open[listPos][2] = celda_padre_anterior[0] # actualizo celda padre
                            open[listPos][3] = celda_padre_anterior[1]

                    else:  # esta en la lista cerrada
                        Gn_nueva = close[-1][-1] + 1 # gn del nuevo posible padre
                        if Gn_nueva < close[listPos][-1]:
                            close[listPos][-1] = Gn_nueva # actualizo gn
                            close[listPos][2] = celda_padre_anterior[0] # actualizo celda padre
                            close[listPos][3] = celda_padre_anterior[1]

                            aux = close[listPos] # movemos el elemento al final de la lista
                            close.pop(listPos)
                            close.append(aux)

                    # REORDENAMOS LA LISTA ABIERTA
                    if len(open) is not 0:
                        for i in range(len(open)-1):
                            for j in range(len(open)-1):
                                fn = open[j][-1] + open[j][-2]
                                fn_1 = open[j+1][-1] + open[j+1][-2]
                                if fn > fn_1:
                                    aux = open[j]
                                    open[j] = open[j+1]
                                    open[j+1] = aux
                    else:
                        path_found = True # para acabar el bucle
                        print('No ha sido posible encontrar ningun camino...')

        inStartCell = False
        celda = [close[-1][0], close[-1][1]]

        while not inStartCell:
            for i in range(len(close)): # saber en que posicion de la lista cerrada esta la siguiente celda padre
                if celda[0] == close[i][0] and celda[1] == close[i][1]:
                    listPos = i
            if close[listPos][0] == start_cell[0] and close[listPos][1] == start_cell[1]:
                inStartCell = True
            else:
                path.append([close[listPos][0], close[listPos][1]])
                celda = [close[listPos][2], close[listPos][3]] # miramos su celda padre

        path.append(start_cell)
        path.reverse()


        # Print path
        x = []
        y = []

        for point in path:
            x.append(point[0])
            y.append(point[1])

        image = np.flipud(self.map)
        plt.figure()
        plt.imshow(image, cmap=plt.get_cmap('binary'))
        plt.plot(x, y, 'ro-', linewidth=2, markersize=5)
        plt.show()

        return path

    def calcHn(self, cell, goal_cell):
        return sqrt((goal_cell[0] - cell[0])**2 + (goal_cell[1] - cell[1])**2)

    def calcNearCells(self, cell):
        cellX = cell[0]
        cellY = cell[1]
        celdasAdyacentes = []

        if cellY > 0:
            if self.map[cellY - 1, cellX] == 0:  # arriba
                celdasAdyacentes.append([cellX, cellY - 1])
        if cellY < self.height:
            if self.map[cellY + 1, cellX] == 0:  # abajo
                celdasAdyacentes.append([cellX, cellY + 1])
        if cellX > 0:
            if self.map[cellY, cellX - 1] == 0:  # izq
                celdasAdyacentes.append([cellX - 1, cellY])
        if cellX < self.width:
            if self.map[cellY, cellX + 1] == 0:  # der
                celdasAdyacentes.append([cellX + 1, cellY])

        return celdasAdyacentes

    def goto(self):

        initial_pose = Pose()
        goal_pose = Pose()
        # Get the input from the user.
        initial_pose.x = input("> x inicial: ")
        initial_pose.y = input("> y inicial: ")

        goal_pose.x = input("> x final: ")
        goal_pose.y = input("> y final: ")

        # Compute current and goal cell
        current_cell = [initial_pose.x, initial_pose.y]
        goal_cell = [goal_pose.x, goal_pose.y]

        x1 = [current_cell[0], goal_cell[0]]
        y1 = [current_cell[1], goal_cell[1]]

        # image = np.flipud(self.map)
        # plt.figure()
        # ax = plt.gca()
        # plt.imshow(image, cmap=plt.get_cmap('binary'))
        # plt.plot(x1, y1, 'ro-', linewidth=2, markersize=5)
        #
        # plt.show()

        path = self.compute_path(current_cell, goal_cell)



if __name__ == '__main__':
    try:
        rospy.init_node('robot_planner', anonymous=True)

        x = Planner()
        x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
