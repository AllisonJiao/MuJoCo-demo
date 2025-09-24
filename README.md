1. Create a virtual environment with Python 3.9
```bash
conda create -n mujoco python=3.9
```
2. Activate the virtual environment, then:
   - [Build MuJoCo from source](https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source)
   - [Install the Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
3. Install Pygame library
```bash
pip install pygame
```
4. Run the MuJoCo simulation
```bash
mjpython main.py
```