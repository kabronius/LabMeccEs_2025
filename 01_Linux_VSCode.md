# Esercitazione 1 - Istallazione del'Ambiente Sviluppo: Ubuntu e VSCode
Questa guida descrive il processo di istalazione di la distribuzione di `Ubuntu 24.04 LTS`, un sistema operativo basato su Linux, e di `Visual Studio Code`, un editor di codice ampiamente utilizzato nello sviluppo software.

L'obbietivo di questa istallazione è configurare un ambiente adatto allo sviluppo di codice in `ROS2`. `Ubuntu` è il sistema operativo raccomandato per `ROS2` grazie alla sua compatibilità e supporto ufficiale, mentre `VSCode` offre un ambiente di sviluppo flessibile e potente per la scrittura e il debbuging.  

In questa sezione verrà illustrato il processo passo a passo per:
1) Scaricare e istallare `Ubuntu` su un computer.
2) Configurare `Ubuntu` con le opzioni esenziali per lo sviluppo.
3) Istallare e preparare `VSCode` con estenzioni utili per lavorare con `ROS2` (Python, C++, CMake, etc.).

## Istalazione di Ubuntu 24.04 LTS
### 1)  Scaricare un'immagine di Ubuntu 
Scaricare l'ISO di Ubuntu corrispondente all'architettura del portatile o PC da [questo link](https://ubuntu.com/download/desktop).

![Pagina di download](images/bb8147f0d63925b1b415c34c6afe154e68520940_2_800x415.png)

### 2) Creare una chiavetta USB 
Per installare Ubuntu Desktop, bisogna scrivere l'ISO scaricato su una chiavetta USB per creare il supporto di installazione. 

Questo tutorial usa [balenaEtcher](https://etcher.balena.io/) perché funziona su Linux, Windows e macOS. Scegliere la versione che corrisponde al attuale sistema operativo, scaricare e installare lo strumento.

![Pagina di download](images/create-a-bootable-usb-stick.png)

Selezionare l'ISO scaricato, scegliere l'unità flash USB, cliccare su **`Flash!`** per scrivere l'immagine.

![Selezzione dell'ISO](images/select-iso.png)

### 3) Avvio da unità flash USB

Inserire l'unità flash USB nel laptop o nel PC e avviare o riavviare il dispositivo. Dovrebbe riconoscere automaticamente il supporto d'installazione. In caso contrario, provare a tenere premuto F12 durante l'avvio e a selezionare il dispositivo USB dal menu di avvio specifico del sistema.

> 
> [!NOTE]\
> `F12` è il tasto più comune per richiamare il menu di avvio del sistema, ma `Escape`, `F2` e `F10` sono alternative comuni. Se non sicuro, cercare un breve messaggio all'avvio del sistema: spesso informa su quale tasto premere per accedere al menu di avvio.

Una volta inizializzato il programma di installazione, verrà chiesto di scegliere la lingua.

![Selezione della lingua](images/3a7fffa914037195d7242fc7edce07242756186f_2_800x496.jpeg)

E poi verrà presentata l'opzione di selezionare le impostazioni di accessibilità.

![Accessibilità](images/e0a488479c69165ce348d43b877ff131ab4b1d02_2_800x496.jpeg)

Il layout della tastiera.

![Layout tastiera](images/205b12c3c5c62477fc1975c5f55c133443efbb43_2_800x496.jpeg)

E la connessione alla rete. Ciò consente a Ubuntu di scaricare aggiornamenti e driver di terze parti (come i driver grafici NVIDIA) durante l'installazione.

![Conessione alla rete](images/99680bc1faae75b38af82bc83b1586bf2276eae2_2_800x496.jpeg)

Dopo di che verrà presentata l'opzione di provare oppure istallare Ubuntu.

![Provare o istallare Ubuntu](images/e03b2b35eb840b7bc31dcb3b21a2395ad4671713_2_800x496.jpeg)

> [!NOTE]\
> Cliccando su **`Prova Ubuntu`**, sarà possibile visualizzare l'anteprima di Ubuntu senza apportare modifiche al PC. Si può tornare al menu di installazione in qualsiasi momento cliccando sul collegamento **`Installa Ubuntu`** sul desktop.

Per procedere, cliccare  **`Installa Ubuntu`**.

> [!WARNING]\
> Alcuni PC utilizzano `Intel RST (Rapid Storage Technology)`, che non è supportato da `Ubuntu`. Se questo è il caso, non si sarà in grado di procedere oltre questo punto senza disabilitare `RST` nel menu `BIOS` della macchina.

### 5) Configurazione di istallazione 
In questo punto verrà chiesto di scegliere tra `Installazione interattiva` e `Installazione automatica`. L'opzione `interattiva` è la via standard, ma gli utenti più avanzati possono usare l'opzione di `installazione automatica` per importare un file di configurazione da un server web per standardizzare installazioni multiple e aggiungere ulteriori personalizzazioni.

![Configurazione di istallazione](images/f735c2969c6a39079e02bcf7245ad42a928dd7f7_2_800x496.jpeg)

Poi verrà chiesto di scegliere tra le opzioni `Selezione predefinita` e `Selezione estesa`. L'installazione `predefinita` include gli elementi essenziali di base per iniziare. La `selezione estesa` contiene strumenti e utilità per ufficio aggiuntivi, utili per le situazioni offline.

![Applicazioni](images/d1d320d5b877f4381179e75c5475124dca6ef812_2_800x496.jpeg)

Nella schermata successiva verrà chiesto di installare software di terze parti che potrebbero migliorare il supporto e le prestazioni del dispositivo (ad esempio, driver grafici Nvidia) e il supporto per formati multimediali aggiuntivi. Si consiglia di selezionare entrambe queste caselle.

![Software di terze parti](images/ff8b742c5b79efaaed137928080a48b7ad260ba7_2_800x496.jpeg)

### 6) Tipo di istallazione

Questa schermata consente di configurare il tipo di installazione. Se si desidera che Ubuntu sia l'unico sistema operativo nel disco rigido, selezionare **`Cancella disco e installa Ubuntu`**.

Se il dispositivo ha attualmente un altro sistema operativo installato (ad es. Windows), si riceveranno opzioni aggiuntive per installare Ubuntu insieme a quel sistema operativo anziché sostituirlo. 

![Tipo di istallazione](images/099483354b97a42b0e7e0f00ce88938aca58d215_2_800x496.jpeg)

#### 6a) Installare Ubuntu insieme a un altro sistema operativo

Se questa opzione viene selezionata verrà fornita un'interfaccia semplice che consente di selezionare l'unità su cui si vuole installare Ubuntu e un cursore per determinare la quantità di spazio su disco per l'utilizzo. Lo spazio disponibile è limitato dal contenuto esistente del disco ed è progettato per evitare di sovrascrivere i file esistenti.

![Istalazione insieme ad un altro SO](images/f7edda3590591d72273283dc9682d490a686abef_2_690x490.png)

Questa vista seleziona automaticamente la partizione più grande sull'unità. Per un controllo più dettagliato è possibile passare all'opzione di partizionamento manuale che è descritta in dettaglio più avanti.

#### 6b) Cancellare disco e istallare Ubuntu
Con questa opzione si cancella tutto lo spazio del disco nella unità selezionata.

![Cancellare disco](images/3cd054f6bf81483f0ef13a1717126f9c2428be7f_2_690x490.png)

#### 6c) Partizionamento manuale

Il partizionamento manuale è stato pensato per gli utenti avanzati che desiderano creare delle configurazioni specifiche per i loro casi d'uso. Qui gli utenti possono vedere tutte le unità e le partizioni e creare nuove tabelle di partizioni e configurazioni.

![Partizionamento manuale](images/e5101b968a2c7b8e626a34217179b6910db14649_2_690x490.png)

### 7) Creazione dei dati di login

In questa schermata saranno richiesti il nome utente, il nome del computer (anche noto come `nome host`) e la password. 

![Dati di login](images/7578560dea014f12ec591489b94be0dd93ec059b_2_800x496.jpeg)

### 8) Pronto per l'istallazione

Cliccando su **`Avanti`** verrà indirizzato a un riepilogo della configurazione dell'istalazione che darà la possibilità di controllare le opzioni scelte prima di fare click su **`Istalla`**.

![Riepilogo](images/756b120659d8316585aec66f2fe193252af876e1_2_800x496.jpeg)

Una volta proceduto, Ubuntu avvierà il processo di installazione.

![Installazione](images/3c59fffc42c4abbb309495512cff9986cb3fe911_2_800x496.jpeg)

### 9) Completare l'istallazione

Una volta finalizzato il processo d'istallazione, verrà richiesto il riavvio del dispositivo. Cliccare su **`Riavviare adesso`**.

![Istallazione finita](images/0f64f4d042f78dfebab66df19ea21e350b62b70f_2_800x496.jpeg)

Mentre si riavvia il dispositivo, verrà richiesta remozione del USB dal PC, una volta fatto ciò, premere **`↵ ENTER`**.

![Riavvio](images/329a4686895ead38fc4638f33fc998d1e395f474_2_800x450.png)

Al riavvio, segliere la voce Ubuntu nel menu che dobrevve comparire per avviare il sistema operativo. Attraverso questo menu poi si potrà anche scegliere avviare il resto di sistemi operativi che sono istallati nel PC.

![GRUB](images/Default-Grub-Screen-Ubuntu-24-04.webp)

Segue la schermata di login, dove si potrà inserire il nome utente e la password creati previamente. 

![Login](images/caa1cbbd80a4be6e3a504aedf69665461b18c0b7_2_800x450.png)

Finalmente, il desktop di Ubuntu compare ed è pronto al utilizzo.

![Desktop](images/f9921880e30c6dffb3d303fbab17d39ce5e8c1d3_2_800x410.jpeg)

## Istalazione di Visual Studio Code
### 1) Scaricare il pacchetto .deb
Il modo più semplice per installare Visual Studio Code per le distribuzioni basate su Debian/Ubuntu è scaricare il pachetto .deb (64 bit), tramite il Centro Software grafico, se disponibile, oppure da [questo link](https://go.microsoft.com/fwlink/?LinkID=760868).

![Centro software di Ubuntu](images/ubuntu-app-center-1536x864.jpg)

### 2) Istallare il pachetto dal terminale (se il Centro Software non è disponibile)
Per istallare il pachetto .deb dal terminale, basta scrivere:
```
sudo apt install ./<file>.deb
```
Quando si installa il pacchetto .deb, viene richiesto di installare il repository apt e la chiave di firma per abilitare l'aggiornamento automatico tramite il gestore pacchetti del sistema.

### 3) Istallazione di estensioni di base
VS Code ha un ricco ecosistema di estensioni che consentono di aggiungere linguaggi, debugger e strumenti alla installazione per supportare il flusso di lavoro di sviluppo specifico. Ci sono migliaia di estensioni disponibili nel Visual Studio Marketplace.

####  Installare un'estensione di linguaggio per aggiungere supporto per Python o qualsiasi altro linguaggio di programmazione d'interesse.
1) 
![Vista Estensioni](images/extensions-view.png)

2)
![Python](images/extensions-search-python.png)

