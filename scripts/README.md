# gazeboSimulation/scripts
**Note: This probably is outdated**

## Files:
- getModels.py
    - sub to:
        - gazebo service
        - /field/models
        - /gazebo/fake
    - pub to:
        - /gazebo/get_field
    - purpose:
        - get models from gazebo and update them
        - get models from /field/models and use their names to find the gazebo models

- setModels.py
    - sub to:
        - /gazebo/set_field
    - pub to:
        - gazebo set model service
    - purpose:
        - set models position in gazebo