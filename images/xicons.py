#!/usr/bin/env python
from PIL import Image, ImageDraw, ImageChops
from scipy.misc import imsave

# get three screen captures with different wallpapers

i1 = Image.open('IMG_0023.PNG')
i2 = Image.open('IMG_0024.PNG')
i3 = Image.open('IMG_0025.PNG')

# run differences between each pair yielding black
# where they match

i12 = ImageChops.difference(i1,i2)
i23 = ImageChops.difference(i2,i3)
i13 = ImageChops.difference(i1,i3)

# run a filter against each of the differences
# which elevates all non-black pixels to white

def bw(p):
  if (p[0] != 0) or(p[1] != 0) or (p[2] != 0):
    return (255,255,255)
  return (0,0,0)

def wb(p):
  if (p[0] != 0) or(p[1] != 0) or (p[2] != 0):
    return (0,0,0)
  return (255,255,255)

def applyfilter(im, func):
    """ Applies a function to each pixel of an image."""
    width,height = im.size
    pixel = im.load()
    for y in range(0, height):
        for x in range(0, width):
            pixel[x,y] = func(pixel[x,y])

bw12 = i12
bw23 = i23
bw13 = i13

applyfilter(bw12, bw)
applyfilter(bw23, bw)
applyfilter(bw13, bw)

m123 = ImageChops.add(ImageChops.add(bw12,bw23),bw13)
applyfilter(m123, wb)

m1 = ImageChops.multiply(i1,m123)

def geticon(m1, i0, x0, y0, nx, ny):
  d1 = ImageDraw.Draw(m1)
  for iy in range(0, ny):
    y = y0 + (iy * 88)
    for ix in range(0, nx):
      x = x0 + (ix * 76)
      #icon = m1.crop((x, y, x+58, y+58)).save('icon%02d.png' % (i0))
      icon = m1.crop((x, y, x+60, y+80)).save('icon%02d.png' % (i0))
      i0 += 1

geticon(m1,  0, 16,  32, 4, 4)
geticon(m1, 16, 16, 400, 4, 1)
