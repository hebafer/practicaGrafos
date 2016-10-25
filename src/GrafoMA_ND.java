/**
 * Esta clase representa a un Grafo NO Dirigido con aristas ponderadas (con valores double) implementado con una
 * matriz de adyacencia. Este tipo de Grafo se crea con un conjunto de vertices concreto (identificados del 0..V-1)
 * no dinamico (una vez creado el objeto pueden incluirse y eliminarse aristas, pero el numero de vertices es estatico)
 */

import java.util.ArrayList;

public class GrafoMA_ND extends Grafo {

    private static final double YA_VISITADO = Double.MAX_VALUE;
    private static final int SIN_PREVIO = -1;
    protected double[][] grafo;


    public GrafoMA_ND(int numVertices) {
        super(numVertices, 0);
        grafo = new double[numVertices][numVertices];
        for (int v = 0; v < numVertices; v++)
            for (int vAd = 0; vAd < numVertices; vAd++)
                grafo[v][vAd] = SIN_ARISTA;
    }

    /**
     * Comprueba si una arista concreta se encuentra en el grafo
     *
     * @param arista Arista a buscar en el grafo (vertices origen y destino)
     * @return true si en el grafo existe la arista que une los vertice origen y destino. False en caso contrario
     */
    public boolean existeArista(Arista arista) {
        return grafo[arista.getOrigen()][arista.getDestino()] != SIN_ARISTA;
    }

    /**
     * Devuelve el peso de la arista
     *
     * @param arista Arista cuyo peso se desea buscar en el grafo (vertices origen y destino)
     * @return peso de la arista, en caso de que exista, o Double.POSITIVE_INFINITY en caso de que la arista no exista
     */
    public double getPesoArista(Arista arista) {
        return grafo[arista.getOrigen()][arista.getDestino()];
    }

    /**
     * Inserta la arista (vertices origen y destino) en el grafo (si no existe). En caso de que la arista ya exista
     * este metodo no tiene ningun efecto en el grafo. En caso de insertarse la arista, el peso asignado es 1.
     *
     * @param arista Arista a insertar en el grafo (vertices origen y destino)
     */
    public void insertaArista(Arista arista) {
        insertaArista(arista, 1);
    }

    /**
     * Inserta la arista (vertices origen y destino) en el grafo (si no existe) con el peso indicado. Al tratarse de un grafo
     * no dirigido, se añaden las aristas (origen, destino) y (destino, origen), aunque sólo cuentan como 1 arista. En caso
     * de que la arista ya exista este metodo no tiene ningun efecto en el grafo.
     *
     * @param arista Arista a insertar en el grafo (vertices origen y destino)
     * @param peso   peso a asignar a la arista
     */

    public void insertaArista(Arista arista, double peso) {
        if (!existeArista(arista)) {
            grafo[arista.getOrigen()][arista.getDestino()] = peso;
            grafo[arista.getDestino()][arista.getOrigen()] = peso;
            incNumAristas();

        }
    }

    /**
     * Devuelve el vector que indica qué vertices son adyacentes al vertice origen
     *
     * @param origen vertice origen
     * @return vetor de tamaño V qué indica qué vertices son adyacentes a origen y con qué peso y cuales
     * no (Double.POSITIVE_INFINITY)
     */
    public double[] getAdyacentesDe(int origen) {
        return grafo[origen];
    }


    /**
     * ESTE METODO DEBERA IMPLEMENTARSE OBLIGATORIAMENTE PARA QUE LA PRACTICA SEA ACEPTADA
     * Este metodo debera devolver el camino mas corto entre los vertices origen y destino, asi como la distancia de este camino
     *
     * @param origen    vertice origen (valor entre 0..numVertices-1)
     * @param destino   vertice destino (valor entre 0..numVertices-1)
     * @param distancia objeto de tipo Real donde se debera devolver la longitud del camino mas corto obtenido
     * @return ArrayList de identificadores de los vertices del grafo que recorre el camino mas corto obtenido. Este ArrayList debera
     * incluir como primer elemento el identificador origen y, como ultimo elemento el identificador destino
     */
    public ArrayList<Integer> caminoMasCorto(int origen, int destino, Real distancia) {
        ArrayList<Integer> camino = new ArrayList<Integer>();

        double[] menorCoste = new double[numVertices];
        int[] verticeMasCercano = new int[numVertices];

        Dijkstra(origen, menorCoste, verticeMasCercano);

        distancia.setValor(menorCoste[destino]);
        if (distancia.getValor() != YA_VISITADO) {
            int destinoTemp = destino;
            while (destinoTemp != origen) {
                camino.add(0, destinoTemp);
                destinoTemp = verticeMasCercano[destinoTemp];
            }
            camino.add(0, destinoTemp);
        }
        return camino;
    }

    public void Dijkstra(int origen, double[] menorCoste, int[] verticeMasCercano) {
        boolean[] visitados = new boolean[numVertices];
        int vertice;

        /*Inicializar Dijkstra*/
        inicializarDijkstra(origen, menorCoste, verticeMasCercano, visitados);

        for (int i = 0; i < numVertices; i++) {
            vertice = seleccionarVertice(menorCoste, visitados);
            visitados[vertice] = true;
            for (int j = 0; j < numVertices; j++) {
                if (!visitados[j])
                    if (menorCoste[j] > (menorCoste[vertice] + grafo[vertice][j])) {
                        menorCoste[j] = menorCoste[vertice] + grafo[vertice][j];
                        verticeMasCercano[j] = vertice;
                    }
            }
        }

    }

    private void inicializarDijkstra(int origen, double[] menorCoste, int[] verticeMasCercano, boolean[] visitados) {
        for (int i = 0; i < numVertices; i++) {
            visitados[i] = false;
            menorCoste[i] = grafo[origen][i];
            if (grafo[origen][i] != SIN_ARISTA) verticeMasCercano[i] = origen;
            else verticeMasCercano[i] = -1;
        }
        visitados[origen] = true;
    }

    private int seleccionarVertice(double[] menorCoste, boolean[] visitados) {
        int vertice = 0;
        double menor;
        menor = Integer.MAX_VALUE;

        for (int i = 0; i < numVertices; i++) {
            if (!visitados[i] && (menorCoste[i] < menor)) {
                menor = menorCoste[i];
                vertice = i;
            }
        }
        return vertice;
    }

    /**
     * ESTE METODO DEBERA IMPLEMENTARSE OBLIGATORIAMENTE PARA QUE LA PRACTICA SEA ACEPTADA
     * Obtiene los puentes del grafo (aristas cuya eliminacion haria que el grafo quedase fragmentado en mas de una componente conexa)
     *
     * @return ArrayList de aristas identificadas como puentes
     */
    public ArrayList<Arista> getPuentes() {
        ArrayList<Arista> puentes = new ArrayList<Arista>();

        for (int i = 0; i < numVertices; i++) {
            for (int j = i; j < numVertices; j++) {
                if (grafo[i][j] != SIN_ARISTA) {
                    Arista aristaEliminar = new Arista(i, j);
                    double pesoArista = grafo[i][j];
                    grafo[i][j] = SIN_ARISTA;
                    grafo[j][i] = SIN_ARISTA;
                    if (!esGrafoConexo()) {
                        puentes.add(aristaEliminar);
                    }
                    insertaArista(aristaEliminar, pesoArista);
                }
            }
        }
        return puentes;
    }

    /**
     * LA IMPLEMENTACION DE ESTE METODO ES OPCIONAL PARA PODER ENTREGAR LA PRACTICA (2 PUNTOS)
     * Obtiene las articulaciones del grafo (vertices cuya eliminacion haria que el grafo quedase fragmentado en mas de una componente conexa)
     *
     * @return ArrayList de identificadores de los vertices del grafo de tipo articulacion
     */
    public ArrayList<Integer> getArticulaciones() {
        ArrayList<Integer> articulaciones = new ArrayList<Integer>();
        for (int i = 0; i < numVertices; i++) {
           if(!esGrafoConexo(i)) {
               articulaciones.add(i);
           }
        }
        return articulaciones;
    }

    /**
     * LA IMPLEMENTACION DE ESTE METODO ES OPCIONAL PARA PODER ENTREGAR LA PRACTICA (2 PUNTOS)
     * Obtiene los centros del grafo (vertices que tienen la menor de las excentricidades del grafo, siendo la excentricidad de un vertice la maxima
     * de las distancias entre cualquiera de los vertices del grafo y el vertice en cuestion)
     *
     * @return ArrayList de identificadores de los vertices del grafo de tipo centro
     */
    public ArrayList<Integer> getCentros() {
        ArrayList<Integer> centros = new ArrayList<Integer>();
        double[][] caminosMinimos = new double[numVertices][numVertices];
        int[][] verticeMasCercano = new int[numVertices][numVertices];
        double[] excentricidades = new double[numVertices];
        double min = Double.MAX_VALUE;

        Floyd(caminosMinimos, verticeMasCercano);

        for (int j = 0; j < numVertices; j++) {
            double max = 0;
            for (int i = 0; i < numVertices; i++) {
                if (caminosMinimos[i][j] > max)
                    max = caminosMinimos[i][j];
            }
            excentricidades[j] = max;
            if (max < min) min = max;
        }
        for (int i = 0; i < numVertices; i++) {
            if (excentricidades[i] == min) centros.add(0, i);
        }

        return centros;
    }

    public void Floyd(double[][] caminosMinimos, int[][] verticeMasCercano) {
        inicializarFloyd(caminosMinimos, verticeMasCercano);
        for (int k = 0; k < numVertices; k++)
            for (int i = 0; i < numVertices; i++)
                if ((i != k) && (caminosMinimos[i][k] != SIN_ARISTA))
                    for (int j = 0; j < numVertices; j++)
                        if ((k != j) && (caminosMinimos[k][j] != SIN_ARISTA) &&
                                (caminosMinimos[i][k] + caminosMinimos[k][j] < caminosMinimos[i][j])) {
                            caminosMinimos[i][j] = caminosMinimos[i][k] + caminosMinimos[k][j];
                            verticeMasCercano[i][j] = k;
                        }
    }


    private void inicializarFloyd(double[][] caminosMinimos, int[][] verticeMasCercano) {
        for (int i = 0; i < numVertices; i++)
            for (int j = 0; j < numVertices; j++) {
                caminosMinimos[i][j] = grafo[i][j];
                if (grafo[i][j] != SIN_ARISTA) verticeMasCercano[i][j] = i;
                else verticeMasCercano[i][j] = SIN_PREVIO;
            }
        for (int i = 0; i < numVertices; i++) {
            caminosMinimos[i][i] = 0;
            verticeMasCercano[i][i] = SIN_PREVIO;
        }
    }

    public boolean esGrafoConexo() {
        boolean[] visitados = new boolean[numVertices];
        for (int i = 0; i < numVertices; i++) visitados[i] = false;
        recorrerConexo(visitados);
        return todosVisitados(visitados);
    }


    public boolean esGrafoConexo(int n) {
        boolean[] visitados = new boolean[numVertices];
        for (int i = 0; i < numVertices; i++) visitados[i] = false;
        visitados[n] = true;
        recorrerConexo(visitados);
        return todosVisitados(visitados);
    }

    public boolean todosVisitados(boolean[] visitados){
        boolean ok=true; int i=0;
        while ((i<visitados.length) && ok){
            ok = visitados[i];
            i++;
        }
        return ok;
    }

    public void recorrerConexo(boolean[] visitados) {
        visitar_DFS_Componente(0, visitados);
    }


    public void visitar_DFS_Componente(int origen, boolean[] visitados) {
        visitados[origen] = true;
        for (int ady = 0; ady < numVertices; ady++)
            if (grafo[origen][ady] != SIN_ARISTA && !visitados[ady] )
                visitar_DFS_Componente(ady, visitados);
    }
}