All available robot hardware instances are listed in robots.yaml specifying how to instantiate the corresponding URDF (which file + macro) as well as a variable list of properties (props) which are used by the macro.

Typically the macro is a wrapper (placed in robots/ instantiating macros from urdf/). This wrapper should translate (robot-specific) props to corresponding low-level macro parameters.

For each hardware instance, there is also a calibration file specifying the origin for mounting. For different setups (e.g. famula vs. sfb) there are different folders in calibration/ (because e.g. the shadow hands need different calibration settings in both setups).
Which calibration directory to choose is specified by the substitution arg CALIB.


