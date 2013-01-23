four-point
==========

Four-point algorithm

When running the compiled code, some may have problems like this "symbol lookup error: /usr/lib/libgsl.so.0: undefined symbol: cblas\_dnrm2". 

This problem is caused by binutils-gold linker. To solve it, run apt-get remove binutils-gold in terminal. 

