# Landscape_Opt

Library for handling ecological landscape datas and optimize their connectivity according to the PC indicator and other indices.

[![Generic badge](https://img.shields.io/badge/C++-17-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
[![Generic badge](https://img.shields.io/badge/CMake-3.12+-blue.svg?style=flat&logo=cmake)](https://cmake.org/cmake/help/latest/release/3.12.html)

[![Generic badge](https://img.shields.io/badge/license-Boost%20Software%20License-blue)](https://www.boost.org/users/license.html)

## Dependencies

### Build process
The build process requires CMake 3.12 (https://cmake.org/) or more and the Conan C++ package manager (https://conan.io/).
#### Ubuntu
    sudo apt install cmake
    pip install conan
#### Manjaro
    sudo pamac install cmake
    pip install conan

##### Configure Conan for GCC >= 5.1
    conan profile update settings.compiler.libcxx=libstdc++11 default

### Libraries
The project uses COINOR libraries Clp Cbc and LEMON that are not currently available in Conan so get the binaries from your system packet manager or compile them (except LEMON that is header only) from sources

#### From package manager
##### Ubuntu
    sudo apt install coinor-libclp-dev coinor-libcbc-dev
    sudo apt install liblemon-dev
##### Manjaro (AUR)
    sudo pamac install coin-or-lemon

#### From sources
##### Lemon : http://lemon.cs.elte.hu/trac/lemon/wiki/Downloads
    cd lemon-x.y.z
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

##### Gurobi : https://www.gurobi.com/
    linux64/bin/grbgetkey <licence_key>
    cd linux64/src/build
    make
    mv libgurobi_c++.a ../../lib

add to .bashrc:

    export GUROBI_HOME="<path_to>/linux64"
    export PATH="$PATH:$GUROBI_HOME/bin"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$GUROBI_HOME/lib"

##### Clp and Cbc using coinbrew :
###### Ubuntu requirements
    sudo apt install build-essential git gcc-9 g++-9 cmake gfortran
###### Manjaro requirements
    pamac install base-devel git cmake gcc-fortran

###### Instructions
<!-- export OPT_CFLAGS="-pipe -flto -march=native"
    export OPT_CXXFLAGS="-pipe -flto -march=native"
    export LDFLAGS="-pipe -flto" -->

    mkdir coinor
    cd coinor
    git clone https://github.com/coin-or/coinbrew
    ./coinbrew/coinbrew fetch Cbc:releases/2.10.5
    ./coinbrew/coinbrew build Cbc:releases/2.10.5 --enable-cbc-parallel

add to .bashrc :

    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:<path_to>/coinor/dist/lib/"

## How to Compile
Just

    make

## Usage
making the project will product a static library "llandscape_opt.a" and an executable "solve"

    solve <landscape_file> <problem_file> <budget_value> <solver_name> [<option>=<value>]

A wrong call of "solve" will output propositions for available solvers or options.

"<landscape_file>" is the path to a csv file with columns "patches_file" and "links_file" giving paths to the csv files describing the set of patches and of links of the landscape.

"<problem_file>" is the path to a file describing the available options to improve elements of the landscape.

See the data repertory for examples.

## Acknowledgments
This work is a part of the phd thesis of François Hamonic which is funded by region SUD (https://www.maregionsud.fr/) and Natural Solutions (https://www.natural-solutions.eu/)