"""
Image add
=====================

"""


from PIL import Image, ImageChops

point_table = ([0] + ([255] * 255))

def black_or_b(a, b):
    diff = ImageChops.difference(a, b)
    diff = diff.convert('L')
    diff = diff.point(point_table)
    new = diff.convert('RGB')
    new.paste(b, mask=diff)
    return new

a = Image.open('octomapwarehouse9.pgm')
b = Image.open('octomapwarehouse8.pgm')
c = black_or_b(a, b)
c.save('c.pgm')
