import time
Import("env")
env.Append(CPPDEFINES=[("BUILD_UTC_TIMESTAMP", int(time.time()))])
