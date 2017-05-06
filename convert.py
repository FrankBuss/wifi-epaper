#!/usr/bin/python

from PIL import Image
import sys

if len(sys.argv) != 3:
	print("usage: convert inputfile.png outputfile.bin")
	sys.exit(1)

im = Image.open(sys.argv[1])
im = im.convert('RGB')
width, height = im.size
print("width: " + str(width) + ", height: " + str(height))
data = bytearray(width * height / 8 * 2)
counter = 0

def createBits(cmp):
	global counter
	bits = 0
	bitCounter = 0
	byteCounter = 0
	for y in range(height):
		for x in range(width):
			bits <<= 1
			r, g, b = im.getpixel((x, y))
			if cmp(r, g, b):
				bits |= 1
			bitCounter += 1
			if bitCounter == 8:
				# sys.stdout.write(format(bits, '#04x') + ", ")
				data[counter] = bits
				counter += 1
				bitCounter = 0
				bits = 0
				byteCounter += 1
				if byteCounter == 33:
					byteCounter = 0
					# sys.stdout.write('\n')

def black(r, g, b):
	return r == 0 and g == 0 and b == 0

def red(r, g, b):
	return r == 255 and g == 0 and b == 0

createBits(black)
createBits(red)

sys.stdout.flush()


with open(sys.argv[2], "wb") as file:
	file.write(data)
  