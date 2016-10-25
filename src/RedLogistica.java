
/**
 * Modela el grafo de la red de transporte de la empresa de servicios logísticos.
 * Básicamente su funcionalidad es la de dotar a los vértices de un nombre (una ciudad) y de implementar los métodos
 * que permitan traducir o transformar los nombres de los vértices (con los que trabaja la clase AplicacionLogistica)
 * en índices (con los que trabaja la clase GrafoMA_ND) y viceversa.
 */

import java.util.ArrayList;

public class RedLogistica {

    /**
     * Vector que define los nombres de las ciudades en los que la empresa tiene instalado un almacen
     */
    public static final String[] ciudades = {"Amsterdam", "Atenas", "Barcelona", "Belgrado", "Berlin", "Bruselas", "Bucarest",
                                             "Budapest", "Burdeaux", "Dublin", "Estocolmo", "Frankfurt", "Ginebra", "Glasgow",
                                             "Hamburg", "Innsbruck", "Kopenhage", "Lisboa", "Liverpool", "Lyon", "Londres", "Madrid",
                                             "Marseille", "Milan", "Nice", "Oslo", "Paris", "Praga", "Rome", "Sofia", "Turin",
                                             "Varsovia", "Zurich"};
    private GrafoMA_ND redTransporte;
    private ArrayList<String> listaCiudades;

    /**
     * Crea un objeto de tipo RedLogistica con las 33 ciudades Europeas donde la empresa tiene instalado un almacen, así como las rutas
     * que unen estos almacenes y la distancia en kilometros de cada ruta
     */
    public RedLogistica(){
        redTransporte = new GrafoMA_ND(ciudades.length);
        listaCiudades = new ArrayList<String>();
        for (int c=0; c<ciudades.length; c++){
            listaCiudades.add(ciudades[c]);
        }
        insertaTramo("Amsterdam", "Berlin", 655);
        insertaTramo("Amsterdam", "Bruselas", 201);
        insertaTramo("Atenas", "Belgrado", 1102);
        insertaTramo("Atenas", "Bucarest", 1158);
        insertaTramo("Atenas", "Sofia", 800);
        insertaTramo("Barcelona", "Burdeaux", 637);
        insertaTramo("Barcelona", "Lyon", 640);
        insertaTramo("Barcelona", "Madrid", 623);
        insertaTramo("Barcelona", "Marseille", 506);
        insertaTramo("Belgrado", "Bucarest", 592);
        insertaTramo("Belgrado", "Budapest", 378);
        insertaTramo("Belgrado", "Sofia", 394);
        insertaTramo("Berlin", "Frankfurt", 545);
        insertaTramo("Berlin", "Hamburg", 288);
        insertaTramo("Berlin", "Innsbruck", 749);
        insertaTramo("Berlin", "Kopenhage", 438);
        insertaTramo("Berlin", "Praga", 350);
        insertaTramo("Berlin", "Varsovia", 573);
        insertaTramo("Berlin", "Zurich", 843);
        insertaTramo("Bruselas", "Frankfurt", 400);
        insertaTramo("Bruselas", "Londres", 320);
        insertaTramo("Bruselas", "Paris", 312);
        insertaTramo("Bucarest", "Budapest", 816);
        insertaTramo("Bucarest", "Sofia", 358);
        insertaTramo("Budapest", "Praga", 525);
        insertaTramo("Budapest", "Varsovia", 869);
        insertaTramo("Burdeaux", "Lisboa", 1150);
        insertaTramo("Burdeaux", "Lyon", 555);
        insertaTramo("Burdeaux", "Madrid", 690);
        insertaTramo("Burdeaux", "Paris", 590);
        insertaTramo("Dublin", "Glasgow", 394);
        insertaTramo("Dublin", "Liverpool", 267);
        insertaTramo("Estocolmo", "Kopenhage", 657);
        insertaTramo("Estocolmo", "Oslo", 530);
        insertaTramo("Frankfurt", "Ginebra", 575);
        insertaTramo("Frankfurt", "Paris", 572);
        insertaTramo("Ginebra", "Lyon", 149);
        insertaTramo("Ginebra", "Turin", 248);
        insertaTramo("Glasgow", "Liverpool", 354);
        insertaTramo("Glasgow", "Londres", 662);
        insertaTramo("Hamburg", "Kopenhage", 336);
        insertaTramo("Hamburg", "Oslo", 812);
        insertaTramo("Innsbruck", "Milan", 407);
        insertaTramo("Innsbruck", "Rome", 762);
        insertaTramo("Innsbruck", "Zurich", 296);
        insertaTramo("Kopenhage", "Oslo", 606);
        insertaTramo("Lisboa", "Madrid", 626);
        insertaTramo("Liverpool", "Londres", 355);
        insertaTramo("Lyon", "Marseille", 313);
        insertaTramo("Lyon", "Paris", 465);
        insertaTramo("Lyon", "Turin", 313);
        insertaTramo("Marseille", "Nice", 198);
        insertaTramo("Milan", "Rome", 574);
        insertaTramo("Milan", "Turin", 143);
        insertaTramo("Nice", "Turin", 278);
        insertaTramo("Praga", "Varsovia", 675);
    }

    /**
     * Inserta un nuevo tramo en la red de transporte entre dos ciudades.
     * @param origen Nombre de la ciudad origen del tramo (deberá corresponderse con alguno de los nombres del vector ciudades)
     * @param destino Nombre de la ciudad destino del tramo (deberá corresponderse con alguno de los nombres del vector ciudades)
     * @param kilometros Kilometros que separan ambas ciudades en ese tramo
     */
    private void insertaTramo(String origen, String destino, double kilometros){
        int o,d;
        o = listaCiudades.indexOf(origen);
        d = listaCiudades.indexOf(destino);
        redTransporte.insertaArista(new Arista(o,d),kilometros);
    }

    /**
     * Devuelve la representación en formato String de este tipo de objeto.
     * @return String con la representacion del objeto. Basicamente incluye la informacion relativa al numero de ciudades y tramos, así como una lista
     * de las ciudades que son adyacentes a otras ciudades (mediante un tramo) y la distancia en kilometros que las separa
     */
    public String toString(){
        String respuesta = "";
        double[] adyacentes;
        int numAdyacentes;

        respuesta = "-----------------------------------------------------------------\n";
        respuesta = respuesta + "   Red de transporte con " + redTransporte.getNumVertices() + " ciudades (almacenes) y " + redTransporte.getNumAristas() + " conexiones\n";
        respuesta = respuesta + "-----------------------------------------------------------------\n";
        for (int v=0; v<redTransporte.getNumVertices(); v++){
            respuesta = respuesta + "  * " + ciudades[v];
            for (int i=0; i<(12-ciudades[v].length()); i++) respuesta = respuesta + " ";

            respuesta = respuesta + "* Ciudades adyacentes: ";
            adyacentes = redTransporte.getAdyacentesDe(v);
            numAdyacentes = 0;
            for (int a=0; a<adyacentes.length; a++){
                if (adyacentes[a] != Grafo.SIN_ARISTA) {
                    respuesta = respuesta + "[" + ciudades[a] + " ->  " + adyacentes[a] + " Km.]  ";
                    numAdyacentes++;
                }
            }
            if (numAdyacentes == 0) respuesta = respuesta + "ninguno";
            respuesta = respuesta + '\n';
        }
        return respuesta;
    }


    /**
     * Dadas dos ciudades origen y destino, este metodo devuelve las ciudades que debe recorrer el camion (ordenadas por orden de visita)
     * teniendo en cuenta que debera ser el camino más corto calculado para toda la red de transportes de la empresa. Ademas, devolvera
     * la distancia en kilometros calculada para este camino
     * @param origen Nombre de la ciudad origen (deberá corresponderse con alguno de los nombres del vector ciudades)
     * @param destino Nombre de la ciudad destino (deberá corresponderse con alguno de los nombres del vector ciudades)
     * @param distancia Objeto de tipo Real que debera estar creado antes de llamar a este metodo. Al final de la ejecucion del metodo,
     *                  en este parametro se devolvera la distancia en kilometros calculada para el camino mas corto entre origen y destino
     * @return Un ArrayList de String que contendra los nombres de las ciudades a visitar (en orden de visita) del camino más corto (en kilometros)
     * calculado entre las ciudades origen y destino (debera incluir ambas ciudades)
     */
    public ArrayList<String> caminoMasCorto(String origen, String destino, Real distancia){
        ArrayList<String> camino = new ArrayList<String>();
        ArrayList<Integer> verticesCamino;

        verticesCamino = redTransporte.caminoMasCorto(listaCiudades.indexOf(origen), listaCiudades.indexOf(destino), distancia);
        for (int i=0; i<verticesCamino.size(); i++){
            camino.add(ciudades[verticesCamino.get(i)]);
        }
        return camino;
    }

    /**
     * Obtiene los tramos (aristas) de la red de transporte cuya eliminacion haria que la red quedase fragmentada en mas de un trozo
     * @return String[2][*] Array de dos dimensiones (dos filas y * columnas, donde * será el número de puentes encontrado en la red
     * de transporte). Cada elemento [0][x] contiene el nombre de la ciudad origen de un puente y [1][x] el nombre de la ciudad destino de ese puente.
     */
    public String[][] getTramosCriticos(){
        ArrayList<Arista> puentes = redTransporte.getPuentes();

        String[][] tramosCriticos = new String[2][puentes.size()];
        for (int i=0; i<puentes.size(); i++){
            tramosCriticos[0][i] = ciudades[puentes.get(i).getOrigen()];
            tramosCriticos[1][i] = ciudades[puentes.get(i).getDestino()];
        }
        return tramosCriticos;
    }

    /**
     * Obtiene los almacenes que dejarían la red fragmentada en más de un trozo si temporalmente la ciudad en la que está ubicado se quedase aislada.
     * @return Vector de String, con tantos elementos como articulaciones se encuentren en la red de transporte. Cada elemento contendrá el nombre de la ciudad
     * que se ha detectado como articulacion de la red de transportes.
     */
    public String[] getAlmacenesCriticos(){
        ArrayList<Integer> articulaciones = redTransporte.getArticulaciones();

        String[] almacenesCriticos = new String[articulaciones.size()];
        for (int i=0; i<articulaciones.size(); i++){
            almacenesCriticos[i] = ciudades[articulaciones.get(i)];
        }
        return almacenesCriticos;
    }

    /**
     * Obtiene la lista de ciudades más cercanas al resto de ciudades de la red.
     * @return Vector de String, con tantos elementos como centros se encuentren en la red de transporte. Cada elemento contendrá el nombre de la ciudad
     * que se ha detectado como centro de la red de transportes.
     */
    public String[] getAlmacenesMasCercanos(){
        ArrayList<Integer> centros = redTransporte.getCentros();

        String[] almacenesCentros = new String[centros.size()];
        for (int i=0; i<centros.size(); i++){
            almacenesCentros[i] = ciudades[centros.get(i)];
        }
        return almacenesCentros;
    }

}
