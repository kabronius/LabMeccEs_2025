# Esercitazione 3 - Python 3 Basico: Parte I

## Introduzione 

Questa guida fornisce una veloce panoramica a quelli che sono gli aspetti sintattici e le funzionalità principali del linguaggio di programazzione **Python 3**.

Python è un linguaggio di programmazione ad [alto livello](https://it.wikipedia.org/wiki/Linguaggio_di_programmazione_ad_alto_livello) che fa della leggibilità e della semplicità il suo punto di forza principale. Ecco alcune delle sue caratteristiche principali:
* **Sintassi chiara**: Struttura vicina al linguaggio umano, migliorando comprensibilità e leggibilità.
* **Indentazione obbligatoria**: Utilizza l'indentazione per separare i blocchi di codice invece delle parentesi.
* **Versatilità**: Può essere usato sia come linguaggio di programmazione convenzionale che come linguaggio di scripting.
* **Programmazione a oggetti**: Supporta classi, ereditarietà multipla e i principali costrutti [OOP](https://it.wikipedia.org/wiki/Programmazione_orientata_agli_oggetti).
* **Librerie standard**: Include moduli per interazione con sistema operativo, networking, multi-threading e altro.
* **Ampio ecosistema**: Disponibilità di numerose librerie aggiuntive facilmente scaricabili dal [repository ufficiale](https://pypi.org/).

Sebbene Python venga considerato un linguaggio **interpretato**, in realtà il codice sorgente non viene convertito direttamente in linguaggio macchina ma viene prima sottoposto a una fase di pre-compilazione in _bytecode_ (è un linguaggio intermedio - più astratto - tra il linguaggio macchina e il linguaggio di programmazione, usato per descrivere le operazioni che costituiscono un programma), che viene successivamente interpretato un comando per volta, traducendo ed eseguendo ogni singola istruzione del programma. 

Si può dunque dire che il  codice sorgente del programma viene eseguito senza la necessità di creare un file oggetto eseguibile. L'esecuzione è più lenta rispetto a un codice compilato, tuttavia il bytecode è quasi sempre riutilizzato dopo la prima esecuzione del programma, evitando così di reinterpretare ogni volta il sorgente e migliorando le prestazioni. Inoltre è possibile distribuire programmi Python direttamente in bytecode (file `.pyc`), saltando totalmente la fase di interpretazione da parte dell'utilizzatore finale e ottenendo programmi Python a sorgente chiuso.  

Ciò garantisce anche una più facile redistribuzione multi-piattaforma del codice, il quale può essere eseguito da un qualsiasi sistema (GNU/Linux, Windows, Mac OS, Android) che abbia installata una versione dell'interprete Python.  

## Interprete ed esecuzione

L'interprete Python 3 può essere scaricato dal [sito ufficiale](https://www.python.org/download/) selezionando la piattaforma che si sta utilizzando. La maggior parte delle distribuzioni Linux, tuttavia, forniscono già nativamente un'installazione dell'interprete Python 3, la cui versione dell'interprete può essere verificata digitando da terminale il comando 

```bash
$ python3 --version
```
In caso contrario, qualora non fosse installata nessuna versione dell'interprete, è possibile rimediare eseguendo da terminale il comando 
```bash
$ sudo apt get install python3
```

> [!WARNING]\
> Attenzione a **non digitare** il comando `sudo apt get install python` anziché `python3`.  
> `python` fa riferimento alla versione 2.x del linguaggio che oltre a differire nella sintassi da Python 3, ha raggiunto il suo _fine-vita_ il giorno 01/01/2020, ed è quindi consigliata la migrazione a Python 3.

Una volta installato l'interprete è possibile eseguire un programma Python in 3 modi diversi:
* Utilizzando l'`interprete interattivo`.
* Invocando l'`interprete da terminale` per eseguire uno script.
* Rendendo `eseguibile` uno script Python.

### Interprete interattivo

L'interprete interattivo può essere invocato da terminale digitando semplicemente

```python
$ python3
Python 3.8.5 (default, Jan 27 2021, 15:41:15) 
[GCC 9.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> 
```
che mostra le informazioni sulla versione correntemente installata dell'interprete e mette a disposizione un prompt dei comandi in cui è possibile eseguire i singoli comandi in real-time (l'analogo della `command window` di Matlab), mostrando immediatamente il risultato dell'operazione. L'interprete interattivo svolge un ruolo cruciale nel velocizzare la fase di testing iniziale in cui si vuole verificare la correttezza di una singola operazione o il funzionamento di un nuovo modulo o di una funzione. L'interprete interattivo mette inoltre a disposizione alcuni strumenti di supporto come il comando `help(<fun>)` per ricevere informazioni sull'utilizzo delle funzioni. 

> [!NOTE]\
> Grazie all'interprete interattivo si possono provare istantaneamente le operazioni base per entrare in confidenza col linguaggio o con nuovi moduli e funzioni.
>```python
>$ python3
>Python 3.6.9 (default, Nov  7 2019, 10:44:02) 
>[GCC 8.3.0] on linux
>Type "help", "copyright", "credits" or "license" for more information.
>>>> a = 2
>>>> b = 3
>>>> a+b
>5                          <-- PREVIOUS OPERATON RESULT
>>>> print("Hello World")
>Hello World                <-- PREVIOUS FUNCTION RESULT
>>>> 
>```

### Eseguire uno script dal terminale

Qualora le operazioni da eseguire siano più complesse e di numero tale per cui la loro esecuzione ripetitiva risulti onerosa dal punto di vista della riscrittura delle operazioni, può risultare conveniente creare uno `script` da eseguire periodicamente. 

Uno script non è altro che un file di testo con estensione `*.py` contenente istruzioni Python. Uno script non ha necessità di un metodo `main` da eseguire, i singoli comandi vengono eseguiti dal primo all'ultimo come se fossero inseriti manualmente nell'interprete interattivo. 

**Unica differenza**: le stampe a video sono esguite solo se fatte dalle apposite funzioni demandate a farlo (per intendersi: le `print`).
Il programma termina e rilascia tutte le risorse allocate dopo l'esecuzione dell'ultimo comando presente nello script. 
Uno script può essere lanciato da terminale mediante l'invocazione esplicita dell'interprete seguita dal nome del file `*.py`.

> [!NOTE]\
> I comandi Python possono essere raggruppati in un file di testo con apposita estensione (`test.py`) per poi essere eseguiti da terminale.
> ```python
> a = 2
> b = 3
> a+b                      <-- OUTPUT NOT DISPLAYED
> print("Hello World")
> ```
> 
> ```python
> $ python3 test.py
> Hello World              <-- SCRIPT OUTPUT
> ```

### Rendere uno script Python eseguibile

Un altro modo per eseguire un programma Python è rendere eseguibile il codice sorgente aggiungendo l'*hashbang* `#!/usr/bin/env python3` ad inizio file, per indicare al sistema operativo quale interprete utilizzare per eseguire lo script stesso. Oltre a tale operazione, è necessario conferire allo script i permessi di esecuzione affinché possa essere lanciato da terminale come un programma eseguibile qualsiasi.

> [!NOTE]\
> Per rendere eseguibile un file Python bisogna aggiungere l'hashbang come prima riga del file `test.py` e modificare i suoi permessi.
> ```python
> #!/usr/bin/env python3   <-- SPECIFY PYTHON INTERPRETER
> a = 2
> b = 3
> a+b                      <-- OUTPUT NOT DISPLAYED
> print("Hello World")
> ```
> 
> ```python
> $ chmod +x test.py       <-- ADD EXECUTION PERMITS
> $ ./test.py
> Hello World              <-- SCRIPT OUTPUT
> ```

## Sintassi - Variabili & Operatori

Python è un linguaggio con `tipizzazione dinamica forte` delle variabili. La tipizzazione dinamica consente di modificare a runtime il tipo delle variabili, senza richiedere una loro preventiva dichiarazione del tipo. L'aggettivo `forte` sta ad indicare che le operazioni aritmetiche sono ammesse `solo` tra variabili dello stesso tipo, sarà dunque necessario introdurre meccanismi di `casting` delle variabili per poter eseguire le varie operazioni.

### Variabili

Le variabili sono classificate nei seguenti tipi: 

> [!NOTE]\
> è interessante sottolineare come quasi tutto in Python sia realizzato per mezzo di classi, compresi i tipi di base che sono mappati attraverso le apposite classi `wrapper`

* **Numeric** 
I tipi primitivi raggruppati sotto tale categoria generica rispettano l'imposizione di tipizzazione forte e rientrano nella stesso tipo. Ne fanno parte:
    * **Integer** `<class 'int'>`: non vi è differenza tra interi a diversa precisione, gli interi in Python hanno precisione arbitraria.
    * **Float** `<class 'float'>`: rappresentano i numeri decimali in notazione a virgola mobile, con tutti i comuni problemi computazionali annessi.
    * **Complex** `<class 'complex'>`: introduce come tipo primitivo i numeri complessi, in cui l'unita immaginaria è espressa come `1j`.
* **String** `<class 'str'>`: non vi è differenza tra singoli caratteri (char) e stringhe. Le stringhe sono oggetti immutbili: una volta create non possono essere dunque modificate.
* **Boolean** `<class 'bool'>`: i valori che può assumere sono `True` o `False`.

### Operatori

Gli `operatori` elementari da usare fra variabili sono quelli classici di quasi tutti i linguaggi di programmazione, con qualche piccola eccezione - come l'elevazione a potenza ottenuta attraverso l'operatore `**` - o qualche novità - gli operatori di `membership` o di appartenenza ad insiemi-. Gli operatori elementari possono essere raggruppati in nel seguente modo:

* **Aritmetici**: `+  ,  -  ,  *  ,  /  ,  %  ,  //  ,  **`
* **Relzionali**: `==  ,  !=  ,  <  ,  >  ,  <=  ,  >= `
* **Assegnazione**: `+=  ,  -=  ,  *=  ,  /=  ,  %=  ,  //=  ,  **=`
* **Binari**: `&  ,  |  ,  ^  ,  ~  ,  <<  ,  >>`
* **Logici**: `and  ,  or  ,  not`
* **Appartenenza**: `in  ,  not in`
* **Uguaglianza**: `is  ,  is not`

Alcune funzioni  utili messe a disposizione dalla libreria base di python sono le funzioni di conversione di tipo, `casting`, per trasformare le variabili da un tipo all'altro (qualora il passaggio sia compatibile): 

* ` int()`
* `float()`
* `complex()`
* `str()`
* `bool()`

o alcune funzioni matematiche di base:
* `abs()`
*  `pow()`
* `round()`

> [!NOTE]\
> Tramite l'interprete interattivo è possibile verificare i concetti di tipizzazione dinamica forte delle variabili
> ```python
> >>> a = 2                                               <-- Integer
> >>> b = 3.5                                             <-- Float
> >>> a+b
> 5.5                                                     <-- Float
> >>> b = 'hi'                                            <-- String (b change type)
> >>> a+b
> Traceback (most recent call last):                      <-- Error (b and c have different type)
>   File "<stdin>", line 1, in <module>
> TypeError: unsupported operand type(s) for +:'int' and 'str'
> >>>
> ```
> e verificare il risultato prodotto dall'utilizzo degli operatori.
> ```python
> >>> a = '2'             <-- String
> >>> b = 3.5             <-- Float
> >>> int(a)+b
> 5.5                     <-- Float
> >>> a+str(b)
> '23.5'                  <-- String (concatenation)
> >>> c = 11              <-- Integer
> >>> c/3.5
> 3.142857142857143       <-- Float (division)
> >>> round(c/3.5,3)
> 3.143                   <-- Approximation with 3 decimal positions
> >>> c//3.5
> 3.0                     <-- Float  (integer part of division)
> >>> c%3.5
> 0.5                     <-- Float  (remainder of division)
> ```
 
## Sintassi - Costrutti di Controllo Flusso
Al fine di mantenere un'elevata leggibilità del codice, i fondatori di Python scelsero di imporre l'uso dell'indentazione del codice per separare i blocchi di controllo nel codice, anziché fare ricorso all'uso delle parentesi (`{` `}`) o di parole chiave (come `begin` o `end`) come di prassi accade nella maggior parte dei linguaggi di programmazione. Inoltre, come da raccomandazioni della [PEP 8 - Style Guide for Python Code](https://www.python.org/dev/peps/pep-0008), è consigliato utilizzare 4 spazi bianchi come tabulazione (ogni editor di testo decente, e degno di questo nome, permette di configurare tale parametro nella sezione `Peferenze`).

#### Blocchi di Alternativa

Il loro scopo e il loro utilizzo è alquanto triviale per chi ha una minima infarinatura di programmazione.

```python
if condition1:
    operation1
elif condition2:
    operation2
else:
    operation3
```

#### Blocchi Iterativi

I cicli `for` in Python utilizzano il concetto di _iteratore_ su strutture dati e prevede intrinsecamente la costruzione di una struttura contenente gli indici su cui iterare se si vuole fare ricorso a questi ultimi.
```python
for i in range(10):                # for i=0:9
    operation on the index
    
for i in range(2,10):              # for i=2:9
    operation on the index
    
for i in range(1,10,2):            # for i=1:2:9
    operation on the index
    
for e in l:
    operation on the element
```
Alternativa per iterare è data dal costrutto `while`:

```python    
while condition:
    operation
```

Entrambi i costrutti d'iterazione permettono l'interruzione del ciclo prima del raggiungimento naturale della fine attraverso l'istruzione di `break` (in caso di cicli innestati è interrotto il più interno), oppure l'interruzione dell'iterazione corrente mediante l'istruzione `continue`.

#### Blocchi di Intercettazione Eccezioni
Python supporta la gestione a runtime delle **eccezioni**, ovvero degli errori generati durante l'esecuzione del programma. Si tratta ovviamente di errori a runtime, quindi non di natura sintattica, ma di consistenza dei dati durante l'utilizzo di funzioni o l'esecuzione di operazioni. 

È possibile catturare eccezioni specifiche per gestire opportunamente il caso difforme o propagare l'eccezione a livello successivo mediante la parola chiave `raise`, oppure intercettare un'eccezione qualsiasi per evitare l'arresto anomalo del programma. 

È inoltre possibile utilizzare il blocco `finally` per eseguire operazioni delicate che devono essere eseguite qualsiasi sia l'esito di una determinata operazione, come ad esempio la chiusura di file aperti, rilasciare lucchetti per la gestione della concorrenza, chiusura socket e quant'altro.

```python
try:
    operations         <-- Operations that can be raise exception
except ErrorName:      <-- Cactch 'ErrorName' exception
    operations
except:                <-- Cactch others exception
    operations
finally:
    operations         <-- Operation executed in both cases
```