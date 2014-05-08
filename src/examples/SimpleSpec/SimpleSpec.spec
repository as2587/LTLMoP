# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
RobotSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
SimpRegions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
move, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p3
r2 = p2
others = p1

Spec: # Specification in structured English
Go to r1
If you are sensing move then go to r2

