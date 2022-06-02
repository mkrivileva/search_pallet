import random

ranges = [str(random.uniform(0.45, 5.0)) for x in range(20)]

with open('ranges', 'w') as fp:
    fp.write(', '.join(ranges))