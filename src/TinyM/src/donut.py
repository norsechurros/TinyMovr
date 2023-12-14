#! /usr/bin/env python3.10

from os import system
from math import sin , cos , pi , e 

def clear():
	system('clear')

class Osmium():

	def __init__(self,v):
		self.vector = v
	
	def matrix(self,q,n):
		mx = [
			[1,0,0],
			[0,cos(q),-sin(q)],
			[0,sin(q),cos(q)]
		
			]
	
		my = [
			[cos(q) , 0 , -sin(q)],
			[0,1,0],
			[sin(q),0,cos(q)]	
		
			]	

		mz = [
			[cos(q),-sin(q),0],
			[sin(q),cos(q),0],
			[0,0,1]		
		
			]

		return [mx,my,mz][n]


	def rotate(self,q,n):
		m = self.matrix(q,n)
		v = [0 for _ in range(len(m))]
		for y in range(len(m)):
			s = 0
			for x in range(len(m[y])):
				s += m[y][x] * self.vector[x]
			v[y] = s
		self.vector = v


	def mag(self):
		return sum(i**2 for i in self.vector) **0.5

	

class donut():
	def __init__(self):
		self.light = [0,0,1]

	
	def circle(self , r , n ,  w  , o):
		
		vx = [w,r,0]
		vy = [r,w,0]
		vz = [r,0,w]
	
		v = Osmium([vx,vy,vz][n])
		c = []
	
		for ø in range(360):	
			v.rotate(pi/180,n)
			c += [[o[i] + v.vector[i] for i in range(len(o))]]
		
		return c
			

	def donut(self,R,r,n):
		
		d = []

		v = [
			[r+R,0,0],
			[0,r+R,0],
			[0,0,r+R],
				]
		
		v = v[n-1]
		c = self.circle(r,n-1,0,(R,0,0))
		for ø in range(360):	
			d += c
			
			
				
			

		
def render():
	pass	
		
		
		
		
			
		
