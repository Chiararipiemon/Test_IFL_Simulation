Questi ile non funzionano ancora, per ora quello che noto è che il problema riiede principalmente nell'approccio al punto p0 scelto.

**Primo tentativo**
Nello script **pre_touch_p0.py** cerco semplicemente di far spostare il robot da ***home*** a un ***pre-pre-approach*** scelto manualmente sulla GUI di Moveit!. Una volta raggiunta quella posa, seleziono il punto P0 tramite GUI e da lì in poi il programma fallisce.

<img width="1846" height="707" alt="immagine" src="https://github.com/user-attachments/assets/f0e2717d-1b99-46ae-9ac2-84cfa68f73de" />
Dal log si vede chiaramente che il robot riese a raggiungere la posa di pre-preapproach, rivece il punto P0 selezionato ma poi fallisce la pianificazione perso P0.

**domande/dubbi:**

- Non seleziono il punto p0 correttamente?
- Ci sono collisioni e quindi fallisco? in realtà ho provato anche a togliere le collisioni tra pedestal e link base0 e tra probe holder e link ee ma il problema rimane lo stesso
- Il punto p0 lo "indico" direttamente da codice una volta capito qual è un buon punto p0 che non fallice e non cliccandolo ogni volta? 

**Secondo tentativo**
Qui raggiungo in qualche modo p0 ma il probe holder penetra il cloudpoint del paziente e il robot esegue una traiettoria senza senso
<img width="1845" height="1004" alt="immagine" src="https://github.com/user-attachments/assets/fa570022-bfde-4851-9b1b-eede0c33a017" />
<img width="689" height="577" alt="immagine" src="https://github.com/user-attachments/assets/2bf773ed-ba42-4f8e-91ef-ba3e32af60bb" />


