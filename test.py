from PIL import Image
import sys

im = Image.open('test.png')
im = im.convert('RGB')
width, height = im.size
print("width: " + str(width) + ", height: " + str(height))
bits = 0
bitCounter = 0
byteCounter = 0
counter = 0
data = bytearray(width * height / 8)
for y in range(height):
	for x in range(width):
		bits <<= 1
		r, g, b = im.getpixel((x, y))
		if r != 0:
			bits |= 1
		bitCounter += 1
		if bitCounter == 8:
			sys.stdout.write(format(bits, '#04x') + ", ")
			data[counter] = bits
			counter += 1
			bitCounter = 0
			bits = 0
			byteCounter += 1
			if byteCounter == 33:
				byteCounter = 0
				sys.stdout.write('\n')
				
sys.stdout.flush()


with open("test.bin", "wb") as file:
  file.write(data)
  