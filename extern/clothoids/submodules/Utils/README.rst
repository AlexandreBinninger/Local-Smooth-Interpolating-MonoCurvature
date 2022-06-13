UTILS
=====

A collection of useful code for C++ applications:

- Terminal coloring use code from `https://github.com/agauniyal/rang` by Abhinav Gauniyal (license `http://unlicense.org`)
- Stream compression use code from `https://github.com/geromueller/zstream-cpp` by Jonathan de Halleux and Gero Müller
- Stream formatting use code from  `https://fmt.dev` by Victor Zverovich (MIT license)
- Table formatting use code from  `https://github.com/Bornageek/terminal-table` by Andreas Wilhelm (Apache License, Version 2.0) partially rewritten.

in addition a TreadPool class, TicToc class for timing, Malloc
class for easy allocation with traking of allocated memory.

- Online doc `here <https://ebertolazzi.github.io/Utils>`__.

COMPILE AND TEST
---------------

**On linux**

using rake

.. code-block:: bash

    rake build_linux

**On windows**

using Visual Studio

.. code-block:: bash

    rake build_win[2017,x64]
    rake build_win[2017,x86]

**On OSX use**

.. code-block:: bash

    rake build_osx
