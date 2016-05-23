.. PyRobotDHS documentation master file, created by
   sphinx-quickstart on Mon May 23 10:58:22 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

PyRobotDHS
==========

PyRobotDHS is an application to enable control of the sample mounting robots at
the Australian Synchrotron via the `Distributed Control System
<http://smb.slac.stanford.edu/research/developments/blu-ice/dcsAdmin4_1/dcsAdmin.html>`_.
developed at SLAC. It is a Distributed Hardware Server that translates requests
from the DCSS into API calls to the ASPyRobotMX server. Robot state changes
broadcast by the ASPyRobotMX server are relayed to the DCSS according to the DCS
protocol.

Setup
-----

.. code-block:: bash

   python3 -m venv .venv
   . .venv/bin activate
   pip install git+https://github.com/AustralianSynchrotron/pyrobotdhs


Running
-------

.. code-block:: bash

   . .venv/bin activate
   pyrobotdhs --config config.json --dcss 10.109.3.21


Contents:

.. toctree::
   :maxdepth: 2


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

