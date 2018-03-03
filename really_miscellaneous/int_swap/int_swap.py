# 180302
#
# the original code is from Irene's math professor
# it originally allows duplicate copies of element to swap
# 
from itertools import product as product
from itertools import combinations as comb

def fix(A_, B_):
    A = A_
    B = B_
    if sum(A_) == sum(B_):
        return A, B
    elif sum(A_) > sum(B_):
        A = B_
        B = A_
    sa, sb = sum(A), sum(B)
    s = sa + sb
    if s % 2 == 1:
        return False
    else:
        avg = s/2
        d = max(sa, sb) - avg
        done = False
        for i in range(min(len(A), len(B))):
            candidates = []
            table = {}
            for b in comb(B,i):
                table[sum(b) - d] = b 
            for a in comb(A,i):
                if sum(a) in table:
                    c = [a, table[sum(a)]]
                    candidates = candidates + [c]
            if len(candidates) != 0:
                done = True
                c = candidates[0]
                for j in range(len(c[0])):
                    A.remove(c[0][j])
                    B.remove(c[1][j])
                    A.append(c[1][j])
                    B.append(c[0][j])
                break
        if done:
            return A, B
        else:
            return False

A = [11,5,7,1,50,4]
B = [7,9,13,3,48]
print('original')
print('A: ', A, ' sum = ', sum(A))
print('B: ', B, ' sum = ', sum(B))

fix(A,B)

print('results')
print('A: ', A, ' sum = ', sum(A))
print('B: ', B, ' sum = ', sum(B))

