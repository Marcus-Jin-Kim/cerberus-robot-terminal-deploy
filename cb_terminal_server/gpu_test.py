import ctypes
try:
    ctypes.CDLL("libtensorflowlite_gpu_delegate.so")
    print("TFLite GPU delegate available")
except OSError as e:
    print("No TFLite GPU delegate:", e)