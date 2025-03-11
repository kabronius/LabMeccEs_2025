def ma_fir(x):
    '''(Questo è un esempio di docstring per commentare una funzione)
    
    Funzione per calcolare la media aritmetica:

    Parametri d'ingresso:    
    `x`- Vettore di'interesse

    Risultati:  
    `returns` - media aritmetica degli elementi del vettore d'ingresso
    '''
    N = len(x)
    s = 0
    for k in range(N):
        s += x[k]
    return round(s/N,2)

def wma_fir(x,w):
    N = len(x)
    s = 0
    for k in range(N):
        s += w[k]*x[k]
    return round(s,2)