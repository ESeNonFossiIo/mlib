from os.path import dirname, abspath, join
from sys import path as sys_path

# Make sure that the application source directory (this directory's parent) is
# on sys.path.

currentFolder = join(dirname(abspath(__file__)), "../../build/")
sys_path.insert(0, currentFolder)
