from math import cos, sin, pi

SIN30 = sin(30 * pi / 180)  # 0.5
COS30 = cos(30 * pi / 180)  # 0.866
SIN45 = sin(45 * pi / 180)  # 0.7071
COS45 = cos(45 * pi / 180)  # 0.7071
SIN15 = sin(15 * pi / 180)  # 0.2588
COS15 = cos(15 * pi / 180)  # 0.9659
SIN10 = sin(10 * pi / 180)  # 0.0872
COS10 = cos(10 * pi / 180)  # 0.9962


class FastTransform:
    @staticmethod
    def rotate45(src_point3d):
        dest = [0.0, 0.0, 0.0]
        dest[0] = src_point3d[0] * COS45 - src_point3d[1] * SIN45
        dest[1] = src_point3d[0] * SIN45 + src_point3d[1] * COS45
        dest[2] = src_point3d[2]
        return dest

    @staticmethod
    def rotate135(src_point3d):
        dest = [0.0, 0.0, 0.0]
        dest[0] = src_point3d[0] * -COS45 - src_point3d[1] * SIN45
        dest[1] = src_point3d[0] * SIN45 + src_point3d[1] * -COS45
        dest[2] = src_point3d[2]
        return dest

    @staticmethod
    def rotate225(src_point3d):
        dest = [0.0, 0.0, 0.0]
        dest[0] = src_point3d[0] * -COS45 - src_point3d[1] * -SIN45
        dest[1] = src_point3d[0] * -SIN45 + src_point3d[1] * -COS45
        dest[2] = src_point3d[2]
        return dest

    @staticmethod
    def rotate315(src_point3d):
        dest = [0.0, 0.0, 0.0]
        dest[0] = src_point3d[0] * COS45 - src_point3d[1] * -SIN45
        dest[1] = src_point3d[0] * -SIN45 + src_point3d[1] * COS45
        dest[2] = src_point3d[2]
        return dest

    @staticmethod
    def rotate90(src_point3d):
        dest = [0.0, 0.0, 0.0]
        dest[0] = src_point3d[0] * 0 - src_point3d[1] * 1
        dest[1] = src_point3d[0] * 1 + src_point3d[1] * 0
        dest[2] = src_point3d[2]
        return dest

    @staticmethod
    def rotate270(src_point3d):
        dest = [0.0, 0.0, 0.0]
        dest[0] = src_point3d[0] * 0 - src_point3d[1] * -1
        dest[1] = src_point3d[0] * -1 + src_point3d[1] * 0
        dest[2] = src_point3d[2]
        return dest


def tran2pose(Tpt):
    pass


def pos2vec(pos):
    return list(pos) + [1,]


def pose2tran(pos, orn, inv_order=False):
    """pos to homogeneous transformation matrix"""
    Tpt = [0.0] * 16  # 4 * 4
    if inv_order is False:  # T*Rz*Ry*Rx
        # first row
        Tpt[0 * 4 + 0] = cos(orn[2]) * cos(orn[1])
        Tpt[0 * 4 + 1] = cos(orn[2]) * sin(orn[1]) * sin(orn[0]) - sin(orn[2]) * cos(orn[0])
        Tpt[0 * 4 + 2] = cos(orn[2]) * sin(orn[1]) * cos(orn[0]) + sin(orn[2]) * sin(orn[0])
        Tpt[0 * 4 + 3] = pos[0]
        # second row
        Tpt[1 * 4 + 0] = sin(orn[2]) * cos(orn[1])
        Tpt[1 * 4 + 1] = sin(orn[2]) * sin(orn[1]) * sin(orn[0]) + cos(orn[2]) * cos(orn[0])
        Tpt[1 * 4 + 2] = sin(orn[2]) * sin(orn[1]) * cos(orn[0]) - cos(orn[2]) * sin(orn[0])
        Tpt[1 * 4 + 3] = pos[1]
        # third row
        Tpt[2 * 4 + 0] = -sin(orn[1])
        Tpt[2 * 4 + 1] = cos(orn[1]) * sin(orn[0])
        Tpt[2 * 4 + 2] = cos(orn[1]) * cos(orn[0])
        Tpt[2 * 4 + 3] = pos[2]
        # forth row
        Tpt[3 * 4 + 0] = 0.0
        Tpt[3 * 4 + 1] = 0.0
        Tpt[3 * 4 + 2] = 0.0
        Tpt[3 * 4 + 3] = 1.0
    else:  # Rx*Ry*Rz*T = inverse of (T*Rz*Ry*Rx)
        # first row
        Tpt[0 * 4 + 0] = cos(orn[1]) * cos(orn[2])
        Tpt[0 * 4 + 1] = -cos(orn[1]) * sin(orn[2])
        Tpt[0 * 4 + 2] = sin(orn[1])
        Tpt[0 * 4 + 3] = (cos(orn[1]) * cos(orn[2])) * pos[0] + (-cos(orn[1]) * sin(orn[2])) * pos[1] + sin(orn[1]) * \
                         pos[2]
        # second row
        Tpt[1 * 4 + 0] = cos(orn[0]) * sin(orn[2]) + sin(orn[0]) * sin(orn[1]) * cos(orn[2])
        Tpt[1 * 4 + 1] = cos(orn[0]) * cos(orn[2]) - sin(orn[0]) * sin(orn[1]) * sin(orn[2])
        Tpt[1 * 4 + 2] = -sin(orn[0]) * cos(orn[1])
        Tpt[1 * 4 + 3] = (cos(orn[0]) * sin(orn[2]) + sin(orn[0]) * sin(orn[1]) * cos(orn[2])) * pos[0] + (
                    cos(orn[0]) * cos(orn[2]) - sin(orn[0]) * sin(orn[1]) * sin(orn[2])) * pos[1] + (
                                     -sin(orn[0]) * cos(orn[1])) * pos[2]
        # third row
        Tpt[2 * 4 + 0] = sin(orn[0]) * sin(orn[2]) - cos(orn[0]) * sin(orn[1]) * cos(orn[2])
        Tpt[2 * 4 + 1] = sin(orn[0]) * cos(orn[2]) + cos(orn[0]) * sin(orn[1]) * sin(orn[2])
        Tpt[2 * 4 + 2] = cos(orn[0]) * cos(orn[1])
        Tpt[2 * 4 + 3] = (sin(orn[0]) * sin(orn[2]) - cos(orn[0]) * sin(orn[1]) * cos(orn[2])) * pos[0] + (
                    sin(orn[0]) * cos(orn[2]) + cos(orn[0]) * sin(orn[1]) * sin(orn[2])) * pos[1] + cos(orn[0]) * cos(
            orn[1]) * pos[2]
        # forth row
        Tpt[3 * 4 + 0] = 0.0
        Tpt[3 * 4 + 1] = 0.0
        Tpt[3 * 4 + 2] = 0.0
        Tpt[3 * 4 + 3] = 1.0
    return Tpt


def inv_tran(Titi):
    """finding the inverse of the homogeneous transformation matrix"""
    Titf = [0.0] * 16  # 4 * 4
    # first row
    Titf[0 * 4 + 0] = Titi[0 * 4 + 0]
    Titf[0 * 4 + 1] = Titi[1 * 4 + 0]
    Titf[0 * 4 + 2] = Titi[2 * 4 + 0]
    Titf[0 * 4 + 3] = -Titi[0 * 4 + 0] * Titi[0 * 4 + 3] - Titi[1 * 4 + 0] * Titi[1 * 4 + 3] - Titi[2 * 4 + 0] * Titi[
        2 * 4 + 3]
    # second row
    Titf[1 * 4 + 0] = Titi[0 * 4 + 1]
    Titf[1 * 4 + 1] = Titi[1 * 4 + 1]
    Titf[1 * 4 + 2] = Titi[2 * 4 + 1]
    Titf[1 * 4 + 3] = -Titi[0 * 4 + 1] * Titi[0 * 4 + 3] - Titi[1 * 4 + 1] * Titi[1 * 4 + 3] - Titi[2 * 4 + 1] * Titi[
        2 * 4 + 3]
    # third row
    Titf[2 * 4 + 0] = Titi[0 * 4 + 2]
    Titf[2 * 4 + 1] = Titi[1 * 4 + 2]
    Titf[2 * 4 + 2] = Titi[2 * 4 + 2]
    Titf[2 * 4 + 3] = -Titi[0 * 4 + 2] * Titi[0 * 4 + 3] - Titi[1 * 4 + 2] * Titi[1 * 4 + 3] - Titi[2 * 4 + 2] * Titi[
        2 * 4 + 3]
    # forth row
    Titf[3 * 4 + 0] = 0.0
    Titf[3 * 4 + 1] = 0.0
    Titf[3 * 4 + 2] = 0.0
    Titf[3 * 4 + 3] = 1.0
    return Titf


def matrix_print(matrix1d, m, n):
    for i in range(m):
        for j in range(n):
            print(matrix1d[i * n + j], end=' ')
        print('\n')


def matrix_multiply(matrix1d_left, matrix1d_right, m, p, n):
    """Matrix Multiplication Routine"""
    matrix1d_res = [0] * m * n
    for i in range(m):
        for j in range(n):
            for k in range(p):
                matrix1d_res[n * i + j] += matrix1d_left[p * i + k] * matrix1d_right[n * k + j]
    return matrix1d_res


def matrix_add(matrix1d_left, matrix1d_right, m, n):
    matrix1d_res = [0] * m * n
    for i in range(m):
        for j in range(n):
            matrix1d_res[n * i + j] = matrix1d_left[n * i + j] + matrix1d_right[n * i + j]
    return matrix1d_res


def matrix_subtract(matrix1d_left, matrix1d_right, m, n):
    return matrix_add(matrix1d_left, matrix_scale(matrix1d_right, m, n, -1), m, n)


def matrix_inverse(matrix1d, m, n):
    """预留"""
    matrix1d_inv = [0] * m * n
    return matrix1d_inv


def matrix_transpose(matrix1d, m, n):
    matrix1d_res = [0] * m * n
    for i in range(m):
        for j in range(n):
            matrix1d_res[m * j + i] = matrix1d[n * i + j]


def matrix_scale(matrix1d, m, n, k):
    """不改变传入矩阵"""
    matrix1d_res = [0] * m * n
    for i in range(m):
        for j in range(n):
            matrix1d_res[n * i + j] = matrix1d[n * i + j] * k
    return matrix1d_res


if __name__ == '__main__':
    matrix_1d = [4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7]
    matrix_1d_2 = [5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8]
    print(matrix_subtract(matrix_1d, matrix_1d_2, 4, 4))



