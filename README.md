# physics_filter
Underworlds filter that use Bullet RT physics simulation to produce the near future of the intput world

## Installation

# Dependencies

To install Underworlds please refer to [installation documentation](http://underworlds.readthedocs.io/en/latest/installation.html?highlight=installation)

This package use Bullet as physics engine so you need first to install the C++ Bullet Physics SDK :

> cd
> git clone https://github.com/bulletphysics/bullet3
> cd bullet3
> python setup.py install --user

Then install pybullet with pip :

> pip install pybullet

# Package installation

> cd catkin_ws/src
> git clone https://github.com/YoanSallami/physics_filter.git
> cd ..
> catkin_make

## ROS Services
`physics_filter/pick`:

`phyics_filter/release`:


## Facts Produced
`hold(X,Y)`: where X is an agent and Y an object

## Event Produced
`pick(X,Y)`: where X is an agent and Y an object

`release(X,Y)`: where X is an agent and Y an object
