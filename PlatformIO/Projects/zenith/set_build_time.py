import time, os
Import("env")
with open("src/build_timestamp.h", "w") as f:
    f.write("#pragma once\n")
    f.write("#define BUILD_UTC_TIMESTAMP {}UL\n".format(int(time.time())))
# Touch zenith.cpp so SCons always recompiles it with the fresh timestamp
os.utime("src/zenith.cpp", None)
