# convert OnShape (our PDM) URDF exports to Stonefish XML
# TODO generalize this and add more if needed

import sys
import xml.etree.ElementTree as ET

# get the filename
if len(sys.argv != 2):
    raise RuntimeError("Plz specify URDF directory")
else:
    fname = sys.argv[1]

# TODO probably wanna add a bit more functionality

# read the Stonefish tree
og_tree = ET.parse(fname)

