#!/bin/python3
import random

hit1 = 0
hit2 = 0
for t in range(1000):
    ans = random.randint(0,2)

    if ans == 0:
        hit1 += 1
    elif ans == 1:
        hit2 += 1
    elif ans == 2:
        hit2 += 1

