#!/usr/bin/env python3
# Example code that helps you convert between AMSL and ellipsoid height
# To run this code you need:
#
# 1) the egm96-5.pgm file from geographiclib.
# To get it on Ubuntu run:
# sudo apt install geographiclib-tools
# sudo geographiclib-get-geoids egm96-5
#
# 2) PyGeodesy
# To get it using pip:
# pip install PyGeodesy

from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

def geoid_height(lat, lon):
    """Calculates AMSL to ellipsoid conversion offset.
    Uses EGM96 data with 5' grid and cubic interpolation.
    The value returned can help you convert from meters 
    above mean sea level (AMSL) to meters above
    the WGS84 ellipsoid.

    If you want to go from AMSL to ellipsoid height, add the value.

    To go from ellipsoid height to AMSL, subtract this value.
    """
    return _egm96.height(lat, lon)
