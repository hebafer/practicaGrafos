/**
 *
 * Esta clase representa un Grafo sin tener en cuenta ninguna representacion concreta del mismo. Basicamente
 * mantiene el numero de vertices y de aristas que tiene el grafo. Es una clase abstracta que define la cabecera
 * de varios metodos (abstractos) que obligatoriamente se deberan implementar en las clases hijas. Este tipo de Grafo
 * se crea con un conjunto de vertices concreto (identificados del 0..numVertices-1) no dinamico (una vez creado el objeto
 * pueden incluirse y eliminarse aristas, pero el numero de vertices es estatico)
 */

public abstract class Grafo {

    public static final double SIN_ARISTA = Double.POSITIVE_INFINITY;

    protected int numVertices;
    protected int numAristas;

    /**
     * Crea un objeto Grafo con el numero de vertices y aristas que se indiquen como parametro
     * @param numVertices numero de vertices del grafo
     * @param numAristas numero de aristas del grafo
     */
    public Grafo(int numVertices, int numAristas){
        this.numVertices = numVertices;
        this.numAristas = numAristas;
    }

    /**
     * Devuelve el numero de vertices del grafo
     * @return numero de vertices del grafo
     */
    public int getNumVertices(){
        return this.numVertices;
    }

    /**
     * Devuelve el numero de aristas del grafo
     * @return numero de aristas del grafo
     */
    public int getNumAristas(){
        return this.numAristas;
    }

    /**
     * Incrementa en 1 el numero de aristas del grafo
     */
    public void incNumAristas(){
        this.numAristas++;
    }

    /**
     * Comprueba si una arista concreta se encuentra en el grafo
     * @param arista Arista a buscar en el grafo (vertices origen y destino)
     * @return true si en el grafo existe la arista que une los vertice origen y destino. False en caso contrario
     */
    public abstract boolean existeArista(Arista arista);

    /**
     * Devuelve el peso de la arista
     * @param arista Arista cuyo peso se desea buscar en el grafo (vertices origen y destino)
     * @return peso de la arista, en caso de que exista, o Double.POSITIVE_INFINITY en caso de que la arista no exista
     */
    public abstract double getPesoArista(Arista arista);

    /**
     * Inserta la arista (vertices origen y destino) en el grafo (si no existe). En caso de que la arista ya exista
     * este metodo no tiene ningun efecto en el grafo
     * @param arista Arista a insertar en el grafo (vertices origen y destino)
     */
    public abstract void insertaArista(Arista arista);

    /**
     * Inserta la arista (vertices origen y destino) en el grafo (si no existe) con el peso indicado. En caso de que
     * la arista ya exista este metodo no tiene ningun efecto en el grafo
     * @param arista Arista a insertar en el grafo (vertices origen y destino)
     * @param peso peso a asignar a la arista
     */
    public abstract void insertaArista(Arista arista, double peso);

}
