import numpy as np

color = {
    "r" : 0,
    "g" : 0,
    "b" : 0
}

class LedMatrix:
    matrix: np.array
    ledTowers: np.array

    def __init__(self, shape = (16,16,16), ledTowersShape = (4,4)):
        self.matrix = self.__initMatrix(shape[0], shape[1], shape[2])

    def __initMatrix(self, height: int, width: int, depth: int) -> None: 
        matrix = []
        for _ in range(height):
            face = []
            for _ in range(width):
                row = []
                for _ in range(depth):
                    row.append(color)
                face.append(row)
            matrix.append(face)
        return np.array(matrix)
    
    def __updateTowers(self):
        shape = self.matrix.shape
        ledTowersShape = self.ledTowers.shape

        towerWidth = int(shape[1] / ledTowersShape[0])
        towerDepth = int(shape[2] / ledTowersShape[1])
        ledTowers = []
        for i in range(ledTowersShape[1]):
            ledTowerRow = []
            for j in range(ledTowersShape[0]):
                ledTower = self.matrix[
                    towerDepth * i : towerDepth * (i + 1), 
                    0 : shape[0], 
                    towerWidth * j : towerWidth * (j + 1)
                ]
                ledTowerRow.append(ledTower)
            ledTowers.append(ledTowerRow)
        self.ledTowers = np.array(ledTowers)
    
    def __unravelTower(this, matrix: np.array):
        height = matrix.shape[1]
        width = matrix.shape[2]
        depth = matrix.shape[0]

        h = height - 1
        w = d = 0
        verDir = -1
        horDir = 1

        array = []
        arrLen = height * width * depth
        index = 0
        while index < arrLen:
            array.append(matrix[d, h, w])
            index += 1
            w += horDir
            if index % width == 0:
                h += verDir
                w -= horDir
                horDir *= -1
                if index % (height * width) == 0:
                    d += 1
                    h -= verDir
                    verDir *= -1
        
        return array



