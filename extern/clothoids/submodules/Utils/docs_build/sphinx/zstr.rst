Zstr
====

.. note::

    A C++ ZLib wrapper by `Matei David <https://github.com/mateidavid>`__,
    published on GITHUB
    `here <https://github.com/mateidavid/zstr>`__.

STL iostream implementation using the library zlib.
This means that you can easily manipulate zipped
streams like any other STL ostream/istream.

To give you an idea, consider following snippet
that create a gzipped files:

.. code-block:: cpp

    std::ofstream file("hello_world.txt.gz");
    zstr::ostream gzfile(file);
    gzfile << "Hello world\n";
    gzfile.flush(); // must be flushed before close stream
    file.close();

.. note:: From original documentation:

    As you can see adding zipped buffers into your existing applications
    is quite straightforward.
