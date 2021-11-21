# Python 3 program to demonstrate working
# of method 1 and method 2.
import numpy as np
rows, cols = (28, 56)

# method 2a
arr = [[0]*cols]*rows

# lets change the first element of the
# first row to 1 and print the array


# outputs the following
#[1, 0, 0, 0, 0]
#[1, 0, 0, 0, 0]
#[1, 0, 0, 0, 0]
#[1, 0, 0, 0, 0]
#[1, 0, 0, 0, 0]

# method 2b
arr = [[1 for i in range(cols)] for j in range(rows)]

# again in this new array lets change
# the first element of the first row
# to 1 and print the array
arr[-2+13][0+27] = 'X'
for row in arr:
    for elem in row:
        if(elem == 1):
            print(' ', end = ' ')
        else:
            print(elem, end = ' ')
    print()

# outputs the following as expected
#[1, 0, 0, 0, 0]
#[0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0]