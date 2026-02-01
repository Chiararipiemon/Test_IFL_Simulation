This section is used to generate a single slice to validate the planner and simulate the behavior of the ultrasound machine.

*Input*: .csv with the pose of the probe_tip on the skin 

*Output*: synthetic single slice 

# To do:
1. open imfusion suite
2. open python console
3. run
    ``` 
    exec(open("/home/chiararipiemo/Hybrid_simulation_last_try/get_sweep_from_csv_quaternion.py", encoding="utf-8").read()); main()
    ```
4. run the hybrid simulation
   
With **get_sweep_from_csv_quaternion.py** you can only obtain a linear frame, not convex. This problem is due to ImFusion.
<img width="1618" height="737" alt="immagine" src="https://github.com/user-attachments/assets/02204663-aaf8-4ad3-8452-bd92c08dc1da" />

