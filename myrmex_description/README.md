# Myrmex description

## parameters

foam_size_x/y is the size of the foam atop of each myrmex 16x16 module and default to 0.095 m, which is larger than the standard 0.08 x 0.08 m tactile sensor active zone.

This permits to physically glue the foam outside the border of the sensor, but in counter-part is not active on the full surface.

In simulation, this might become an issue if the an object is bigger than the active surface, since contacts points occur at the edges of the foam, and not at the edges of the active surface
You might want to specify foam size per myrmex to 0.08 to match the active size and have all the foam become an active surface.




