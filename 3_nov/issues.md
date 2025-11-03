Questi ile non funzionano ancora, per ora quello che noto è che il problema riiede principalmente nell'approccio al punto p0 scelto.
Nello script **pre_touch_p0.py** cerco semplicemente di far spostare il robot da ***home*** a un ***pre-pre-approach*** scelto manualemnte sulla GUI di Moveit!. Una vlta raggiunta quella posa, seleziono il punto P0 e da lì in poi il programma fallisce.
<img width="1846" height="707" alt="immagine" src="https://github.com/user-attachments/assets/f0e2717d-1b99-46ae-9ac2-84cfa68f73de" />
Dal log si vede chiaramente che il robot riese a raggiungere la posa di pre-preapproach, rivece il punto P0 selezionato ma poi fallisce la pianificazione perso P0.
