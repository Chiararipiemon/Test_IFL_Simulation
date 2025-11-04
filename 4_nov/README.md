Il codice ***pre_to_pose_touch_and_sweep_new.py*** al momento fa qualcosa che si avvicina molto a quello che voglio ottenere. Ci sono ancora alcuni moimenti che non mi piacciono (tipo per raggiungere p0 dal pre-approach e il fatto che una volta che sceglie p0 ci si allontana e poi si riapproccia al punto)
Non mi sembra molto conforme alla realtà lo sweep che esegue ma in teoria segue il fatto che il probe sia sempre orienato normale a ciascun punto che tocca.
Vorrei un comportamento pù smooth sulla pelle.
I log sono:
[INFO] [1762267611.064550045]: Ready to take commands for planning group manipulator.
[INFO] [1762267611.092782]: tip_frame=probe_tip -> vettore EE->tip = [0.000, 0.000, 0.240] (frame iiwa_link_ee).
[INFO] [1762267614.188038]: Raggiunto pre_approach via planner.
[INFO] [1762267621.394815]: Raggiunto target_pose via planner.
[INFO] [1762267621.634081]: P0 scelto: [0.660, -0.122, 0.344] (idx 6447).
[INFO] [1762267621.834082]: Uso servizio IK: /iiwa/compute_ik
[INFO] [1762267621.997271]: Eseguo traiettoria touch con 65/65 punti.
[INFO] [1762267639.110510]: Eseguo sweep con 32/32 punti verso P_des.
[INFO] [1762267646.501325]: Eseguo retract con 14/14 punti.
[INFO] [1762267659.171596]: Raggiunto pre_approach via planner.
[INFO] [1762267659.174222]: Sequenza touch → sweep → retract → pre-approach completata.
