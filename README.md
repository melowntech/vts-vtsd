# VTS-VTSD

[VTS-VTSD](https://github.com/melown/vts-vtsd) is HTTP server for surface and
map configuration delivery.

## User documentation

VTS-VTSD user documentation is being created in [separate
project](https://github.com/melown/workshop) and can be seen
on [ReadTheDocs.io](https://melown.readthedocs.io)

## Download, build and install

You basically need just 2 steps to get VTS-VTSD installed: `git clone` the
source code from the repository and `make` it. But there are some tricky parts
of the process, so read carefully following compilation howto.

### Dependencies

#### Basic deps

Make sure, you have `cmake` and `g++` installed, before you try to compile
anything.

```
sudo apt-get update
sudo apt-get install cmake g++
```

#### VTS dependencies

Before you can run [VTS-VTSD](https://github.com/melown/vts-vtsd), you
need at least [VTS-Registry](https://github.com/melown/vts-registry) downloaded
and installed in your system. Please refer to related
[README.md](https://github.com/Melown/vts-registry/blob/master/README.md) file,
about how to install and compile VTS-Registry.

#### Unpackaged deps

[VTS-VTSD](https://github.com/melown/vts-vtsd) is using (among other
libraries) [OpenMesh](https://www.openmesh.org/). You have to download and
install OpenMesh library and this is, how you do it

```
git clone https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
cd OpenMesh
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

#### Adding UbuntuGIS repo

VTS-VTSD needs newer version of [GDAL](http://gdal.org) library, than it is
available in Ubuntu repos. Therefore you need to add [UbuntuGIS](https://wiki.ubuntu.com/UbuntuGIS)
repository to your `apt` sources:

```
sudo add-apt-repository ppa:ubuntugis/ubuntugis-unstable
sudo apt-get update
```

#### Installing packaged dependencies

Now we can download and install rest of the dependencies, which are needed to
get VTS-VTSD compiled:

```
sudo apt-get install \
    libboost-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-iostreams-dev\
    libboost-python-dev \
    libopencv-dev libopencv-core-dev libopencv-highgui-dev \
    libopencv-photo-dev libopencv-imgproc-dev libeigen3-dev libgdal-dev \
    libproj-dev libgeographic-dev libjsoncpp-dev \
    libprotobuf-dev protobuf-compiler libprocps-dev libmagic-dev gawk sqlite3
```

### Clone and Download

The source code can be downloaded from
[GitHub repository](https://github.com/melown/vts-vtsd), but since there are
external dependences, you have to use `--recursive` switch while cloning the
repo.


```
git clone --recursive https://github.com/Melown/vts-vtsd.git 
```

**NOTE:** If you did clone from GitHub previously without the `--recursive`
parameter, you should probably delete the `vts-vtsd` directory and clone
again. The build will not work otherwise.

### Build

For building VTS-VTSD, you just have to use ``make``

```
cd vtsd
make -j4 # to compile in 4 threads
```

You should see compilation progress. Depends, how many threads you allowed for
the compilation (the `-jNUMBER` parameter) it might take couple of minutes to an
hour of compilation time.

The binaries are then stored in `bin` directory.

## Install from Melown repository

Yes, we [are working on it](https://github.com/Melown/vts-vtsd/issues/3) ...
but till we do, you have to compile VTS-VTSD manually. 

## Run VTS-VTSD server

First you need to create `vtsd.conf` configuration file. You then can run

```
vtsd --help
vtsd --config vtsd.conf
```

**NOTE:** You might need to add also `--registry` parameter, and point it to
previously compiled [VTS-Registry](https://github.com/melown/vts-registry).

Description of the configuration file can be found in our [user documentation](http://melown.readthedocs.io/en/latest/server/vtsd.html).

The server is not intended to be exposed to the Internet as it is, instead it's
advised to hide it behind e.g. [NGINX](https://www.nginx.com/) server.

## How to contribute

Check the [CONTRIBUTING.md](CONTRIBUTING.md) file.

