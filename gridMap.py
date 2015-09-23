import DrawingWindowStandalone as dw
reload(dw)
import math

defaultWindowWidth = 400

class GridMap:
    def __init__(self, xN, yN, windowWidth = defaultWindowWidth):
        self.xN = xN
        self.yN = yN
        self.gridSquareSize = int(float(windowWidth)/max(self.xN, self.yN))
        self.makeWindow(windowWidth)

    def makeWindow(self, windowWidth = defaultWindowWidth, title = 'World'):
        """
        Create a window of the right dimensions representing the grid map.
        Store in C{self.window}.
        """
        dx = self.gridSquareSize * self.xN
        dy = self.gridSquareSize * self.yN
        self.window = dw.DrawingWindow(windowWidth, windowWidth,
                                       0, max(dx, dy), 0, max(dx, dy), 
                                       title)

    def drawWorld(self, objects):
        self.window.clear()
        for xIndex in range(self.xN):
            for yIndex in range(self.yN):
                self.drawSquare((xIndex, yIndex), color='gray')
        for (name, loc, color) in objects:
            #XXXif not loc: loc = heldObjectLoc(self.robotLoc, self.grasp)
            self.drawSquare(loc, color=color, name=name)
        self.window.canvas.update()

    def drawSquare(self, indices, color = 'white', name = ''):
        (i, j) = indices
        (x, y) = i * self.gridSquareSize, j * self.gridSquareSize
        size = self.gridSquareSize*0.75
        self.window.drawRect((x, y), (x+size, y+size), color)
        if name:
            self.window.drawText(x+size/2, y+size/2, name)

    def drawPath(self, path):
        """
        Draws list of cells;  first one is purple, last is yellow,
        rest are blue
        @param path: list of pairs of C{(ix, iy)} grid indices
        """
        self.drawSquare(path[0], 'purple')
        for p in path[1:-1]:
            self.drawSquare(p, color = 'blue')
        self.drawSquare(path[-1], 'gold')
