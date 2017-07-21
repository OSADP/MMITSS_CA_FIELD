# About

This directory includes C++11 source code which provide library APIs for
- MRP component configuration (i.e., cnfUtils);
- data logger configuration (i.e., logUtils);
- pack and unpack serialized data messages (i.e., msgUtils);
- UDP/TCP socket utilities (i.e., socketUtils); and
- timestamps utilities (i.e., timeUtils)

# Build and Install

This directory is included in the top-level (directory 'mrp') Makefile and does not need to build
manually. After the compilation process, a shared library ('libutils.so') is created in the
'utils/lib' subdirectory. User's application links the 'libutils.so' to access the aforementioned
library functions.
