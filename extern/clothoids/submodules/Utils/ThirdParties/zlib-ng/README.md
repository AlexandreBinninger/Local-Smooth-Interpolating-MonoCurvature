Precompiled library of the zlib library from:

- `https://github.com/zlib-ng/zlib-ng <https://github.com/zlib-ng/zlib-ng>`__.

Compiled with:

~~~
mkdir build
cd build
cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug -DZLIB_COMPAT=ON ..
nmake
~~~

and

~~~
mkdir build
cd build
cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DZLIB_COMPAT=ON ..
namke
~~~