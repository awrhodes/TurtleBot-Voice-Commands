import sys

text = open('destination.txt', 'w+')

for i in range(199,300):
    text.write('room ' + str(i) + '\n')

text.close()
