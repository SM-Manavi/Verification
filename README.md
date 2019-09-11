# Verification

## Introduction

> We implemented a library using Computational Geometry Algorithms Library (CGAL) in order to automatically generate 
a discrete state space of a given robot system. This allows using existing model checking tools (e.g., CADP) to guarantee
the correctness of the desired requirements. We construct the state space of a number of robots, each move in four directions:
up, down, left and right within a rectangular area. This state space can then be used to verify the connectivity property.

## Build Requirements

* Windows 10
* CMake 3.13+
* CGAL 4.11+
* Boost 1.69
