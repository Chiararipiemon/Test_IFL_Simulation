Ultimo aggiornamento:

Per il momento il codice che si avvicina di più è questo in cui però non è la punta del probe a toccare il p0 ma è il link a cui si collega il probe holder:
<img width="1619" height="692" alt="immagine" src="https://github.com/user-attachments/assets/ea1c9157-90df-4b66-a5d8-f97793f559b6" />
Aggiornamento delle 17:28:
<img width="1778" height="753" alt="immagine" src="https://github.com/user-attachments/assets/342098cc-dab5-411e-88c7-08f74ed8fbe0" />
Non riesco a fare in modo che il planner sia consapevole della geometria del probe holder. Con quest'ultimo aggiornamento del codice il massimo che riesco ad ottenere è che il frame probe_tip tocca il punto p0. Non la punta fisica della mesh del probe.

Altro tentativo precedente:
<img width="1464" height="648" alt="immagine" src="https://github.com/user-attachments/assets/435541b8-50c4-4d9c-b3cc-f60aecfc5344" />
```
source ~/iiwa_stack_ws/devel/setup.bash
ROS_NAMESPACE=iiwa \
~/iiwa_stack_ws/src/iiwa_probe_utils/scripts/3_nov/move_pre_to_pose.py \
  _group_name:=manipulator _ee_link:=iiwa_link_ee \
  _speed_scale:=0.2 \
  _fallback_steps:=50 _fallback_dt:=0.20
```
Per il momento il  codice che più si avvicina al funzionamento/movimento che voglio è quello sotto il nome di ***move_pre_to_pose.py***. Ma semplicemnete il robot si sposta da una posa di pre approach a una con probe orientato sulla nuvola di punt. Non è consapevole della nuvola di punti e delle normali
Leggere file issues.md per capire tutti i tentativi fatti
