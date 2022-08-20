import numpy as np

a = 4

b = np.float32(a).tobytes()

l = list(b)

l = [int(i) for i in l]

h = [hex(i) for i in l]

print(hex(sum(l).to_bytes(2, 'little')[0]))

print(h)