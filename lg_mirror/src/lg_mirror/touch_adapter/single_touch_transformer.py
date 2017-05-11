from lg_mirror.fake_touchscreen import MIN_X, MAX_X, MIN_Y, MAX_Y


def mirror_one_dimension(val, middle):
    diff = val - middle
    return val - diff * 2


class SingleTouchTransformer:
    def __init__(self, min_x=MIN_X, max_x=MAX_X, min_y=MIN_Y, max_y=MAX_Y,
                 invert_x=False, invert_y=False):
        x_range = max_x - min_x
        y_range = max_y - min_y

        to_x_range = MAX_X - MIN_X
        to_y_range = MAX_Y - MIN_Y

        self.x_scale = to_x_range / x_range
        self.y_scale = to_y_range / y_range

        self.x_offset = MIN_X - min_x
        self.y_offset = MIN_Y - min_y

        self.x_median = MIN_X + to_x_range / 2
        self.y_median = MIN_Y + to_y_range / 2

        self.invert_x = invert_x
        self.invert_y = invert_y

    def transform(self, coord):
        coord.x *= self.x_scale
        coord.y *= self.y_scale
        coord.x += self.x_offset
        coord.y += self.y_offset

        if self.invert_x:
            coord.x = mirror_one_dimension(coord.x, self.x_median)
        if self.invert_y:
            coord.y = mirror_one_dimension(coord.y, self.y_median)
