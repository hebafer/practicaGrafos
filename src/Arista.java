
/**
 * Modela una arista de un grafo. Define las propiedades para los vertices origen y destino, un constructor, los métodos
 * get de ambas propiedades y el método toString
 */

public class Arista {

    private int origen;
    private int destino;

    /**
     * Crea un objeto de tipo Arista con los vértices origen y destino indicados como parametro
     * @param origen indice del vertice origen (0..numeroVertice-1)
     * @param destino indice del vertice destino (0..numeroVertice-1)
     */
    public Arista(int origen, int destino){
        this.origen = origen;
        this.destino = destino;
    }

    /**
     * Obtiene el identificador del vertice origen
     * @return indice del vertice origen
     */
    public int getOrigen() {
        return this.origen;
    }

    /**
     * Obtiene el identificador del vertice destino
     * @return indice del vertice destino
     */
    public int getDestino(){
        return this.destino;
    }

    /**
     * Devuelve la representacion en formato String del objeto Arista
     * @return representacion en formato String del objeto Arista
     */
    public String toString(){
        return ("(" + this.origen + ", " + this.destino + ")");
    }
}
