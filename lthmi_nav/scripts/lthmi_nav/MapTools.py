#!/usr/bin/env python

from PIL import Image
import re
from random import randint

map_notation="""
                          CELL x-ccordinate
              0       1       2       3       4       5
        4 +-------+-------+-------+-------+-------+-------+
          |       |       |       |       |       |       |
 V  y     |       |       |       |       |       |       |  3
 E  -     |       |       |       |       |       |       |
 R  c   3 +-------+-------+-------+-------+-------+-------+     C  y
 T  o     |       |       |       |       |       |       |     E  -
 E  o     |       |       |       |       |       |       |  2  L  c
 X  r     |       |       |       |       |       |       |     L  o
    d   2 +-------+-------+-------+-------+-------+-------+        o
    i     |       |       |       |       |       |       |        r
    n     |       |       |       |       |       |       |  1     d
    a     |       |       |       |       |       |       |        i
    t   1 +-------+-------+-------+-------+-------+-------+        n
    e     |       |       |       |       |       |       |        a
          |       |       |       |       |       |       |  0     t
          |       |       |       |       |       |       |
        0 +-------+-------+-------+-------+-------+-------+
          0       1       2       3       4       5       6 
                        VERTEX x-coordinate

Cell can be free or occupied
Vertex can be blocked (surrounded by 4-occupied cells), or unblocked (otherwise)

"""

class GridMap:
    OCCUPIED = True
    FREE = False
    
    def __init__(self, width, height, data):
        self.width  = width
        self.height = height
        self.data   = data
    
    @classmethod
    def fromText(cls, mapFile):
        """
        """
        height = None
        width  = None
        data = None
        for k in range(0,4):
            line = mapFile.readline()
            m = re.match(r"\s*height\s+(\d+)\s*", line)
            if m:
                height = int(m.group(1))
            m = re.match(r"\s*width\s+(\d+)\s*", line)
            if m:
                width = int(m.group(1))
        if width is None or height is None:
            raise Exception("Failed to parse map file: couldn't read width or height")
        else:
            data = [cls.FREE]*(width*height)

            for y in range(height-1,-1,-1):
                line = mapFile.readline().rstrip()
                #rospy.loginfo("experimentator: map line (y=%d, length=%d) read= %s", y, len(line), y*width, (y+1)*width-1, str([(-1 if c=="." else -2) for c in line]))
                data[y*width:(y+1)*width] = [(cls.FREE if c=="." else cls.OCCUPIED) for c in line]
        return cls(width, height, data)


    @classmethod
    def fromImage(cls, imgFile, freeIntensity=100):
        """
            imgFile: file name or file-lie object
        """
        img = Image.open(imgFile)
        px = img.load()
        width  = img.size[0]
        height = img.size[1]
        data = [cls.FREE]*(width*height)
        for y in range(height):
            for x in range(width):
                data[y*width+x] = cls.FREE if px[x,y]>=freeIntensity else cls.OCCUPIED
        return cls(width, height, data)

    def get(self, x, y):
        return self.data[x+self.width*y]
    
    def put(self, x, y, val):
        self.data[x+self.width*y] = val

    def isFree(self, x,y=None):
        if y is None:
            return self.data[x]==self.FREE
        else:
            return self.get(x,y)==self.FREE

    def isVertexUnblocked(self, x,y):
        return self.isFree(x,y  ) or self.isFree(x-1,y) or self.isFree(x,y-1) or self.isFree(x-1,y-1)

    def addBorder(self):
        newdata = [self.OCCUPIED]*(self.width+2)*(self.height+2)
        for x in range(0,self.width):
            for y in range(0,self.height):
                newdata[(y+1)*(self.width+2)+x+1] = self.get(x,y)
        self.data = newdata
        self.width  += 2
        self.height += 2
        
    def genRandUnblockedVertex(self):
        while True:
            x = randint(0,self.width-1)  #randint(a, b)   a <= N <= b
            y = randint(0,self.height-1)
            #print "try (%d,%d)" % (x,y)
            if self.isVertexUnblocked(x,y):
                return [x,y]

    def genRandPath(self, npoints=2):
        path = [self.genRandUnblockedVertex()]
        for k in range(npoints-1):
            while True:
                v = self.genRandUnblockedVertex()
                if v[0]!=path[-1][0] or v[1]!=path[-1][1]:
                    break
            path.append(v)
        return path
        

    def findDiagObst(self, stopOnFirst=True):
        diags = []
        for x in range(self.width-1):
            for y in range(self.height-1):
                if self.isFree(x,y) and self.isFree(x+1,y+1) and (not self.isFree(x+1,y)) and (not self.isFree(x,y+1)):
                    diags.append(  ((x+1,y),(x,y+1))  )
                    if stopOnFirst:
                        return diags
                elif (not self.isFree(x,y)) and (not self.isFree(x+1,y+1)) and self.isFree(x+1,y) and self.isFree(x,y+1):
                    diags.append(  ((x,y),(x+1,y+1))  )
                    if stopOnFirst:
                        return diags
        return diags

    def calcStats(self):
        stats = {'width': self.width, 'height': self.height, 
                 'cells': {'free': 0, 'total': self.width * self.height}, 
                 'verts': {'unblocked': 0, 'total': (self.width+1) * (self.height+1)}}
        for x in range(self.width):
            for y in range(self.height): 
                #stats['cells']['total'] += 1
                if self.isFree(x,y):
                    stats['cells']['free'] += 1
                if x<self.width-1 and y<self.height-1 and self.isVertexUnblocked(x+1,y+1):
                    stats['verts']['unblocked'] += 1
        stats['cells']['occup']   = stats['cells']['total'] - stats['cells']['free']
        stats['verts']['blocked'] = stats['verts']['total'] - stats['verts']['unblocked']
        stats['cells']['ratio']   = float(stats['cells']['occup'])/stats['cells']['total']
        stats['verts']['ratio']   = float(stats['verts']['blocked'])/stats['verts']['total']
        return stats

    def printAsText(self, outputFile):
        outputFile.write("type octile\nheight %d\nwidth %d\nmap\n" % (self.height, self.width))
        for y in range(self.height-1,-1,-1):
            line = ''.join(('.' if self.isFree(x,y) else '@') for x in range(self.width))
            outputFile.write(line+"\n")

    def genScene(self, outputFile, n_blocks, block_size=1, display_map_path="anonymous.map"):
        outputFile.write("version 1\n")
        for block in range(n_blocks):
            for k in range(block_size):
                pair = self.genRandPath()
                outputFile.write("%d\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t1.0000\n" % (block, display_map_path, self.width, self.height, pair[0][0], pair[0][1], pair[1][0], pair[1][1]));




import sys
import StringIO
doc="""
USAGE:
    Calculate map stsistics (number of free cells, number of unblocked vertices, etc):
        ./MapTools.py stats <input.bmp

    Convert image file into map file:
        ./MapTools.py img2map [{free_pixel_min_intensity}] <input.bmp >output.map
        
    Add 1 occupied cell border:
        ./MapTools.py add_border <input.map >output.map
        
    Find diagonal obstacles:
        ./MapTools.py find_diags <input.map
        
    Print a path with random way-points to STDOUT:
        ./MapTools.py gen_path {number_of_point} <input.map
        
    Print {number_of_paths} paths each containing a random number n (such that {min_num_points} <= n <= {max_num_points}) of random way-points to STDOUT:
        ./MapTools.py gen_paths {number_of_paths} {min_num_points} <= n <= {max_num_points} <input.map
        
    Clear map from disagonal obstacles:   NOT YET IMPLEMENTED
        ./MapTools.py clear_diags [policy] <input.map >output.map
        
    Remove closures:   NOT YET IMPLEMENTED
        ./MapTools.py del_closures <input.map >output.map
        
    Generate scene file:
        ./MapTools.py gen_scene {number_of_blocks} [{block_size}] [{display_map_path}] <input.map >output.map.scene
        
Map notation:
"""+map_notation


if __name__=="__main__":
    nargs = len(sys.argv)-1
    if len(sys.argv)<2:
        sys.stderr.write(doc)
        exit(1)
    else:
        action = sys.argv[1]
        if action=="help":
            sys.stdout.write(doc)
        elif action=="stats":
            mapFile = StringIO.StringIO(sys.stdin.read())
            grid = GridMap.fromText(mapFile)
            s = grid.calcStats()
            sys.stdout.write("Map statistics:\n    resolution: %(width)d x %(height)d\n" % s)
            sys.stdout.write("    cells:\n        total: %(total)d\n        free: %(free)d\n        occup: %(occup)d\n        occup/total: %(ratio)f\n" % s['cells'])
            sys.stdout.write("    verts:\n        total: %(total)d\n        unblocked: %(unblocked)d\n        blocked: %(blocked)d\n        blocked/total: %(ratio)f\n" % s['verts'])
        elif action=="img2map":
            imgFile = StringIO.StringIO(sys.stdin.read())
            intensity = int(sys.argv[2]) if len(sys.argv)>2 else 100
            grid = GridMap.fromImage(imgFile, intensity)
            output = sys.stdout
            grid.printAsText(output)
            sys.stderr.write("Finished. Resolution=%dx%d\n" % (grid.width, grid.height))
        elif action=="add_border":
            mapFile = StringIO.StringIO(sys.stdin.read())
            grid = GridMap.fromText(mapFile)
            grid.addBorder()
            grid.printAsText(sys.stdout)
            sys.stderr.write("Finished. Resolution=%dx%d\n" % (grid.width, grid.height))
        elif action=="find_diags":
            mapFile = StringIO.StringIO(sys.stdin.read())
            grid = GridMap.fromText(mapFile)
            diags = grid.findDiagObst(False)
            if diags:
                sys.stderr.write(str(diags)+"\n")
            else:
                sys.stderr.write("No diagonal occupied cells found\n")
        elif action=="gen_path":
            if len(sys.argv)<3:
                sys.stderr.write("ERROR: gen_path requires argument {number_of_point}. Run './MapTools.py help'\n\n")
                exit(1)
            npoints = int(sys.argv[2])
            mapFile = StringIO.StringIO(sys.stdin.read())
            grid = GridMap.fromText(mapFile)
            path = grid.genRandPath(npoints)
            sys.stderr.write(str(path)+"\n")
        elif action=="gen_paths":
            if len(sys.argv)<5:
                sys.stderr.write("ERROR: gen_paths requires 3 additional arguments {number_of_paths} {min_num_points} {max_num_points}. Run './MapTools.py help'\n\n")
                exit(1)
            npaths = int(sys.argv[2])
            minpts = int(sys.argv[3])
            maxpts = int(sys.argv[4])
            mapFile = StringIO.StringIO(sys.stdin.read())
            grid = GridMap.fromText(mapFile)
            sys.stdout.write("'paths': {")
            for k in range(npaths-1):
                npoints = randint(minpts,maxpts)  #randint(a, b)   a <= N <= b
                path = grid.genRandPath(npoints)
                sys.stdout.write("'%d': %s,\n" % (k, str(path)))
                #sys.stdout.write("%s,\n" % (str(path)))
            npoints = randint(minpts,maxpts)  #randint(a, b)   a <= N <= b
            path = grid.genRandPath(npoints)
            sys.stdout.write("'%d': %s\n}\n" % (k+1, str(path)))
            #sys.stdout.write("%s\n]\n" % (str(path)))
        elif action=="gen_scene":
            if len(sys.argv)<3:
                sys.stderr.write("ERROR: gen_scene requires minimum 1 additional parameters. Run './MapTools.py help'\n\n")
                exit(1)
            n_blocks = int(sys.argv[2])
            block_size = int(sys.argv[3]) if len(sys.argv)>=4 else 1
            path_to_show = sys.argv[4] if len(sys.argv)>=5 else "anonymous.map"
            map_file = StringIO.StringIO(sys.stdin.read())
            grid = GridMap.fromText(map_file)
            grid.genScene(sys.stdout, n_blocks, block_size, path_to_show)
        else:
            sys.stderr.write(doc)
            exit(1)
    
        #n = int(sys.argv[2])
        #exp_prefix = "e"
        #map_name = sys.argv[3] #"office2_1.map"
        #resolution =   float(sys.argv[4])#0.1
        #tries = int(sys.argv[5]) #3
        
        #print "experiments: %s" % str(["%s%d"%(exp_prefix,k) for k in range(n)])
        #print "experiment_set:"
        #for k in range(n):
            #print "    %s%d:" % (exp_prefix, k)
            #print "        map: \"%s\"" % map_name
            #print "        resolution: %f" % resolution
            #print "        tries: %d" % tries
            #print "        poses: %s" % str(gen.generatePair())
