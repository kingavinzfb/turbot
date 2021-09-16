#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math


# 赋值限定
def saturation(value, lower, upper):
    return min(max(value, lower), upper)


def limit(value_k, value_k1, lower, upper, limit_x):
    if (value_k - value_k1) > upper:
        value_k = value_k1 + limit_x
    if (value_k - value_k1) < lower:
        value_k = value_k1 - limit_x
    return value_k


def dmod(a, m):
    b = a - m * math.floor(a / m)
    return b


# 角度限定
def angle_adjust(angle):
    while angle <= -math.pi or angle >= math.pi:
        if angle > math.pi:
            angle = angle - 2 * math.pi
        if angle < -math.pi:
            angle = angle + 2 * math.pi
    return angle


# 极坐标转换为直角坐标
def polarToCartesian(x, y, flag):
    a = 0
    b = 0
    if flag == 1:
        a = x * math.cos(y)
        b = x * math.sin(y)
        return a, b
    if flag == 0:
        a = math.sqrt(x * x + y * y)
        b = math.atan2(y, x)
        return a, b

def dist(a, b):
    dist = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    return dist
