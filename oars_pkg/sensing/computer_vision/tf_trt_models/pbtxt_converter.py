#!/usr/bin/env python

from tensorflow.core.framework import graph_pb2 as gpb
from google.protobuf import text_format as pbtf

# LABELS_PATH = '../data/inception_v2_class_labels.pbtxt'

def pbtxt_to_dic(path):

    """
    Converts a pbtxt file into a dictionary
    Argument: path to the pbtxt
    Returns: dictionary
    """
    # import the pbtxt file
    with open(path, 'r') as fid:
        graph_str = fid.read()

    length_graph_str = len(graph_str) - 1
    loop = len(graph_str)
    # n is used for the index of the dictionary
    n = 1
    dictionary = {}

    # find instances of '_name', after which the name of the id occurs
    for i in range(4, length_graph_str):
        if graph_str[i-4]+graph_str[i-3]+graph_str[i-2]+graph_str[i-1]+graph_str[i] == '_name':
            # 25 is a sufficiently large number that will capture the display_names that follow '_name'
            for j in range(25):
                # Prevents the program from not working with the last item in the list
                if i + j > length_graph_str:
                        break
                if graph_str[i + j] == '"':
                    # k = length of spaces the between 'display_name' and next indent
                    # does not include the indent itself (hence the -1)
                    k = j
            index_str = ''
            for l in range(k):
                if l > 3 and l != k:
                    # index_str captures the display_name
                    index_str = index_str + graph_str[i+l]
            dictionary[n] = index_str
            n += 1
    print(dictionary)
    return dictionary
