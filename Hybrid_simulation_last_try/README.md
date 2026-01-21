Questa sezione è per generare la singola slice per validare il planner e simulare il comportamento della ultrasound machine.
# Comandi
1. open imfusion suite
2. aprire la python console
3. eseguire
    ``` 
    exec(open("/home/chiararipiemo/Hybrid_simulation_last_try/get_sweep_from_csv_quaternion.py", encoding="utf-8").read()); main()
    ```
4. runnare la hybrid
5. 
Con il file **get_sweep_from_csv_quaternion.py** ottengo solo immagine lineare, non convex
<img width="1618" height="737" alt="immagine" src="https://github.com/user-attachments/assets/02204663-aaf8-4ad3-8452-bd92c08dc1da" />
Il problema è che perdo ancora le informazioni della convex geometry e genero uno sweep sintetico anche se gli dico che lo voglio lineare
