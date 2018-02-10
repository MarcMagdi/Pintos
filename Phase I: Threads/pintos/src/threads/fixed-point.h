#define FIXED_FACTOR (1 << 14)

#define convert_to_fixed_point(x)               ((int) (x * FIXED_FACTOR))
#define convert_to_integer_towards_zero(x)      ((int) (x / FIXED_FACTOR))
#define convert_to_integer_towards_nearest(x)   (x >= 0) ? ((int) (x + (FIXED_FACTOR / 2)) / FIXED_FACTOR) : ((int) (x - (FIXED_FACTOR / 2)) / FIXED_FACTOR)

#define add_two_fixed_point(x,y)        ((int) (x + y))
#define subtract_two_fixed_point(x,y)     ((int) (x - y))
#define multiply_fixed_point_integer(x,y)       ((int) (x * y))
#define divide_fixed_point_integer(x,y)       ((int) (x / y))

#define add_fixed_point_integer(x,y)      ((int) (x + (y * FIXED_FACTOR)))
#define subtract_fixed_point_integer(x,y)     ((int) (x - (y * FIXED_FACTOR)))
#define multiply_two_fixed_point(x,y)       ((int) (((int64_t) x) * y / FIXED_FACTOR))
#define divide_two_fixed_point(x,y)       ((int) (((int64_t) x) * FIXED_FACTOR / y))
