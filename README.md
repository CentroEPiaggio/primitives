# primitives
Primitive library for loco-manipulation tasks
# Description
At this stage we are working on ideas using tools like MPT toolbox and Yalmip.

# Installation step-by-step instructions
Download the package with:
```
git@github.com:CentroEPiaggio/primitives.git
```
if you have a registered ssh-key in your Github account, otherwise you can use:
```
git clone https://github.com/CentroEPiaggio/primitives.git
```

From a Matlab shell run:
```
cd primitives
install_mpt3.m
```

# Usage

To start using the package move in the src directory. From a Matlab shell:
```
cd /path/to/primitives/src
```
and run
```
init.m
```
to initialize the library.

Then the main script is
```
locomaniplanner.m
```
