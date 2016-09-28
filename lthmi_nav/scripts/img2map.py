#!/usr/bin/env python

import sys
from PIL import Image

fin = sys.argv[1]
fout = sys.argv[2]


im = Image.open(fin)
px = im.load()

nfree = 0
nocc  = 0


fh = open(fout, 'wb')
fh.write("type octile\n")
fh.write("height %d\n" % im.size[1])
fh.write("width %d\n" % im.size[0])
fh.write("map\n")

for y in range(im.size[1]):
    for x in range(im.size[0]):
        if px[x,y]>100:
            nfree +=1
        else:
            nocc +=1
    line = ''.join(('.' if px[x,y]>100 else '@') for x in range(im.size[0]))
    fh.write(line+"\n")

print "Resolution=%dx%d, number of free pixels=%d, number of occupied pixels=%d, total number of pixels=%d" % (im.size[0], im.size[1], nfree, nocc, im.size[0]*im.size[1])
print "Finished."