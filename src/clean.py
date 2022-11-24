#! 
import os 

WORKSPACE = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(WORKSPACE, 'data', 'img')

for file in os.listdir(path):
    os.remove(os.path.join(path, file))

print("All files in '{}' have been removed.".format(path))