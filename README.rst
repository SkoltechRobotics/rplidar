*******
RPLidar
*******

Simple and lightweight module for working with RPLidar rangefinder scanners

.. image:: https://readthedocs.org/projects/rplidar/badge/?version=latest
    :target: http://rplidar.readthedocs.org/en/latest/?badge=latest
    :alt: Documentation Status

.. image:: https://img.shields.io/pypi/v/rplidar.svg
    :target: https://pypi.python.org/pypi/rplidar
    :alt: PyPI version

.. image:: https://img.shields.io/github/license/mashape/apistatus.svg
    :target: https://github.com/SkRobo/rplidar/blob/master/LICENSE
    :alt: MIT License

This module aims to implement communication protocol with RPLidar
laser rangefinder scaners. It's Python 2 and 3 compatible but was mainly tested using Python 3.

For protocol specifications please refer to the following document:

- http://www.slamtec.com/download/lidar/documents/en-us/rplidar_interface_protocol_en.pdf

==========
Installing
==========

You can install rplidar using ``pip``::

    $ sudo pip install rplidar

Or for Python 3::

    $ sudo pip3 install rplidar

=============
Documentation
=============

View the latest rplidar documentation at http://rplidar.rtfd.org/.

=============
Usage example
=============

Simple example::

    >>> from rplidar import RPLidar
    >>> lidar = RPLidar('/dev/ttyUSB0')
    >>> info =lidar.get_info()
    >>> print('\n'.join('%s: %s' % (k, str(v)) for k, v in info.items()))
    firmware: (1, 15)
    model: 0
    hardware: 0
    serialnumber: 64E699F3C7E59AF0A2E69DF8F13735
    >>> lidar.get_health()
    ('Good', 0)
    >>> process_scan = lambda scan: None
    >>> for scan in lidar.iter_scans():
    ...  process_scan(scan)
    KeyboardInterrupt
    >>> lidar.stop()
    >>> lidar.stop_motor()

In addition to it you can view example applications inside
`examples <https://github.com/SkRobo/rplidar/tree/master/examples>`_ directory.
