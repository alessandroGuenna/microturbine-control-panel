# Microturbine Control Panel

Applicazione MATLAB interattiva che simula il **pannello di controllo di una
microturbina a gas cogenerativa (CHP) da ~80 kW**. Dietro l'interfaccia (MATLAB
App Designer) gira un modello termodinamico dinamico in Simulink del ciclo Brayton
rigenerativo a singolo albero, con recupero di calore per produzione di acqua calda.

> **Nota.** Il modello non riproduce una macchina reale specifica: è un progetto
> didattico/di esercizio. Con dati accurati di un costruttore potrebbe essere
> calibrato verso un *digital twin* di una microturbina esistente.

🔗 **Descrizione completa e approfondimento del modello:**
https://alessandroguenna.github.io/projects/microturbine/

## Contenuto

| File / cartella | Descrizione |
|---|---|
| `ControlPanel.mlapp` | Interfaccia dell'app (App Designer) |
| `microturbine_model_v2.slx` | Modello dinamico Simulink del ciclo |
| `data_v2.m` | Inizializzazione: parametri, punto di progetto, schedule di velocità |
| `MATLAB functions/` | Funzioni di supporto (mappe compressore, punto di progetto, recuperatore, controllo…) |
| `Allegati/` | Dati della mappa del compressore e risorse del modello |

## Come eseguirla

1. Aprire il progetto in **MATLAB** (con **Simulink** e **Simscape**).
2. Aprire `ControlPanel.mlapp` e premere **Run**.
3. Nell'app: impostare la temperatura ambiente → **Initialize** → **START**,
   poi variare il carico con lo slider *Set load*.

## Modello

Ciclo Brayton rigenerativo a singolo albero: compressore (mappa) → recuperatore →
camera di combustione → turbina (in choking) → recuperatore → caldaia a recupero (HRB).
Il controllo insegue la potenza richiesta con uno **schedule di velocità ottimale**
(massimo rendimento) e protezioni (margine al pompaggio, temperatura massima turbina,
load shedding proporzionale).
