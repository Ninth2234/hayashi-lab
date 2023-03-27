# Imports PIL module
from PIL import Image, ImageDraw

MAX_PIXEL = 1000
NUM_COL_DOT = 7
RADIUS = 10

# creating a image object (new image object) with
# RGB mode and size 200x200
im = Image.new(mode="RGB", size=(MAX_PIXEL, MAX_PIXEL), color=(255,255,255))

# create a draw object
draw = ImageDraw.Draw(im)

interval = MAX_PIXEL/(NUM_COL_DOT+1)

for i in range(NUM_COL_DOT):
    for j in range(NUM_COL_DOT):
        
        # set the coordinates and RADIUS of the circle
        x = interval+i*interval
        y = interval+j*interval
        

        # draw the circle
        draw.ellipse((x - RADIUS, y - RADIUS, x + RADIUS, y + RADIUS), outline=(0, 0, 0),fill=(0,0,0))

draw.line((interval/2,interval/2,interval/2+interval/4,interval/2),fill = (0,0,0))
draw.line((interval/2,interval/2,interval/2,interval/2+interval/4),fill = (0,0,0))



# This method will show image in any image viewer
im.show()

im.save("myCloth.png")