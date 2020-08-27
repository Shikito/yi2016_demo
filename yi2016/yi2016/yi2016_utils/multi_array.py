import numpy as np

def decode_array(comm_data):
    x = np.array(comm_data.data)
    return x.reshape(comm_data.shape)


def encode_array(array, comm_data):
    comm_data.shape = array.shape
    array_flatten = array.reshape(array.size)
    comm_data.data = array_flatten.tolist()
    return comm_data
