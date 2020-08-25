import numpy as np

def decode_array(request):
    x = np.array(request.data)
    return x.reshape(request.shape)

def encode_array(array, response):
    response.shape = array.shape
    array_flatten = array.reshape(array.size)
    response.data = array_flatten.tolist()
    return response