USAGE
#####

Linux and MacOS both have Python3 available, so just create a virtualenv,
activate it and install the required libraries:

```
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
./qt_measure.py
```

Windows has no native Python3 interpreter: install Python Install Manager from
MS Store, launch it, choose to add commands dir to PATH, and install CPython.
Then the commands are almost the same as above:

```
python3 -m venv venv
venv/Scripts/activate
pip install -r requirements.txt
py qt_measure.py
```
