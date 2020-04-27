# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 13:14:10 2020

@author: tabor
"""

height = 3 #inch
width = 18 #inch 
length =18 #inch
mass = 8 #kg


hm = height *2.54 /100.0
wm = width *2.54 /100.0
lm = length *2.54 /100.0

xx = (1/12.0) + (hm*hm + lm*lm)
yy =  (1/12.0) + (lm*lm + wm*wm)
zz =  (1/12.0) + (hm*hm + wm*wm)
print('xx',xx)
print('yy',yy)
print('zz',zz)