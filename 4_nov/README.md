Voglio in qualche modo ottenere un codice che faccia queste cose:

- legge la cloud (con normali) da /cloud_with_normals
- fa n modo che il robot si posizioni in una posa di pre-appproach con probe orientato verso la nuvola di punti
- prende P0 e Pdes (via parametri oppure cliccando due punti in RViz su /clicked_point),
- campiona la retta in N punti, per ogni punto prende il nearest sulla cloud (e la sua normale),
- costruisce le pose (pos = punto_surface − backoff·normale, orientamento: asse Z del tool allineato a −normale, asse X allineato alla direzione di scan proiettata sul piano tangente),
- pianifica ed esegue un percorso cartesiano con MoveIt.

Problemi riscontrati fino ad ora:
- fallisco ogni approccio perchè in qualche modo il robot va in collisione o fa dei movimenti strani tali per cui impazzisce e non riesce a raggiungere la posa desiderata.
- ho provato a lanciare banalmente un codice in cui semplicemente a partire dalla home pose il robot deve ragggiungere una sola posa di pre approach, riesco ad ottenere la posa pianificata ma poi fallisco ogni tentativo di approccio CORRETTO al punto p0 --> approfondire su **issues.md** e **README.md** nella cartella ***3_nov***.

Cosa sono riuscita ad ottenere almeno, aggiornamento **3 Novembre ore 17:36**, tutti i file py sono nella cartella 3_nov:
- robot che da home va in una posa di pre-approach e poi si approccia al cloudpoint ma senza consapevolezza del cloudpoint, impongo solo i valori in radian che ciascun joint dele assumere:
```
source ~/iiwa_stack_ws/devel/setup.bash
ROS_NAMESPACE=iiwa \
~/iiwa_stack_ws/src/iiwa_probe_utils/scripts/3_nov/move_pre_to_pose.py \
  _group_name:=manipulator _ee_link:=iiwa_link_ee \
  _speed_scale:=0.2 \
  _fallback_steps:=50 _fallback_dt:=0.20

# pre_to_pose_touch_and_sweep_new.py
Il codice ***pre_to_pose_touch_and_sweep_new.py*** al momento fa qualcosa che si avvicina molto a quello che voglio ottenere. Ci sono ancora alcuni moimenti che non mi piacciono (tipo per raggiungere p0 dal pre-approach e il fatto che una volta che sceglie p0 ci si allontana e poi si riapproccia al punto).

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

[INFO] [1762267659.174222]: Sequenza touch → sweep → retract → pre-approach completata

# pre_to_pose_touch_and_sweep_fixedZ.py
Ora il probe scorre sulla cludpoint toccndo i punti ma iene un orientamento fisso senza seguire tutte le normali locali per ciascun punto. Per ora mi fermo qui, eventualemente affinerò questa cosa dopo con un metodo ibrido per rendere più smooth la scansione
