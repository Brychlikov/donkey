from donkeycar.parts.datastore import Tub
from PIL import Image
import numpy as np


class LineFollower:

    def __init__(self, polling=5, width=3, middle=None, flip=False):
        self.polling = polling
        self.width = width
        self.middle = middle
        self.flip = flip

    def run(self, img):

        # def green_heuristic(v):
        #     v = v / np.linalg.norm(v)
        #     return v[1]

        def green_for_line(r):
            return  [green_heuristic(x) for x in r]

        def green_heuristic(v):
            r, g, b = v
            r, g, b = int(r), int(g), int(b)
            if r + g + b == 0:
                return 0
            return g / (r*r + g*g + b*b) ** 0.5

        def find_line(row):
            part = np.argpartition(row, -5)
            min_var = float('inf')
            min_var_i = 0
            for i in range(5 - 3):
                line = part[-3 - i:len(row) - i]
                v = np.var(line)
                if v < min_var:
                    min_var = v
                    min_var_i = i
            result = line
            return np.mean(result)


        #mask = np.apply_along_axis(green_heuristic, 2, img)

        mask = [green_for_line(r) for r in img]

        middle = self.middle if self.middle is not None else len(mask[0]) / 2
        line_coords = [find_line(row) for row in mask]
        deltas = [middle - lc for lc in line_coords]

        angle = 0
        for i, d in enumerate(deltas):
            angle += d / (i + 1)

        angle = angle / -150 if self.flip else angle / 150
        if angle > 1:
            angle = 1
        elif angle < -1:
            angle = -1

        return angle, repr(line_coords)


if __name__ == '__main__':

    t = Tub(path='~/fork_car/fork_car/tub')
    data = t.get_record(400)
    img = data['cam/image_array']
    img = img[:30, :, :]
    
    mask = np.apply_along_axis(green_heuristic, 2, img)
    mask = (mask ** 1 * 255).astype('uint8')
    
    found_line = np.zeros_like(mask)
    
    for i in range(30):
        l = find_line(mask[i, :])
        found_line[i, int(round(l))] = 255
        print(l)
    
    line = Image.fromarray(found_line, 'L')
    line.show()
    
    image = Image.fromarray(mask, 'L')
    image.show()
    orig = Image.fromarray(img, 'RGB')
    orig.show()
